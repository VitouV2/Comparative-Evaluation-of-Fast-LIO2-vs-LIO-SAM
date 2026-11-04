#!/usr/bin/env python3
"""Normalise a LiDAR-inertial odometry backend onto the team's frozen interface.

Air-Ground Collaborative Mapping, SLAM module (C. Vitou).

Why this node exists
--------------------
The frozen interface (G1, Nov 18) asks every robot to publish

    <robot>/odom       nav_msgs/Odometry       consumed by: everyone
    <robot>/map_cloud  sensor_msgs/PointCloud2 consumed by: N. Vitou, Chamreourn

with the frame tree ``map -> <robot>/odom -> <robot>/base_link``.

No LIO backend publishes that. FAST-LIO2 publishes ``/Odometry`` and
``/cloud_registered`` in hard-coded frames ``camera_init`` and ``body``,
reports the pose of the *IMU* rather than base_link, leaves ``twist``
entirely zero, and emits ``/cloud_registered`` as the current deskewed scan
rather than an accumulated map. LIO-SAM and Point-LIO each differ again.

This node adapts the backend instead of forking it, which matters because P2
benchmarks FAST-LIO2 against LIO-SAM and Point-LIO and then adds FAST-LIVO2:
swapping backends must not change what the rest of the team consumes. Every
backend-specific detail is a parameter, so a swap is a config change.

What it does
------------
1. Re-anchors the backend's world frame so ``<robot>/odom`` starts at the
   robot's initial base_link pose, which is the ROS convention and what
   Phumint's relocalisation and Yong's allocator expect.
2. Moves the pose from the backend's body frame onto base_link using the
   extrinsics in the calibration file.
3. Derives ``twist`` by finite-differencing pose, since FAST-LIO2 never fills
   it and a nav_msgs/Odometry with a zero twist is a trap for consumers.
4. Accumulates ``map_cloud`` into a voxel grid and publishes it downsampled at
   a low rate. This is what the consumers actually need (a map, not a scan)
   and it is the mitigation for the "map clouds too large for Wi-Fi" risk:
   ``voxel_size`` and ``map_publish_period`` are the two knobs to turn.
"""

import argparse
import math
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from agc_slam.transforms import (
    Calibration,
    invert,
    matrix_to_quat,
    pose_to_matrix,
    rotation_log,
)

# Voxel keys are packed into one int64 so the occupied set can be a sorted
# numpy array rather than a Python set. 2^20 indices per axis keeps the packed
# value below 2^60, which at the default 0.2 m leaf covers +/- 104 km.
_VOXEL_BITS = 20
_VOXEL_HALF = 1 << (_VOXEL_BITS - 1)
_VOXEL_SPAN = 1 << _VOXEL_BITS


class SlamInterface(Node):

    def __init__(self):
        super().__init__('slam_interface')

        self.declare_parameter('robot_name', 'ugv1')
        self.declare_parameter('backend', 'fast_lio2')
        self.declare_parameter('backend_odom_topic', '/Odometry')
        self.declare_parameter('backend_cloud_topic', '/cloud_registered')
        # Which calibrated sensor the backend's child_frame_id actually is.
        # FAST-LIO2 and Point-LIO report the IMU; LIO-SAM reports the lidar.
        self.declare_parameter('backend_body_sensor', 'imu')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', '')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_identity_map_to_odom', True)
        self.declare_parameter('zero_odom_at_start', True)
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('map_publish_period', 1.0)
        self.declare_parameter('max_map_points', 3000000)
        self.declare_parameter('max_twist_dt', 0.5)

        def param(name):
            return self.get_parameter(name).value

        robot = param('robot_name')
        self.backend = param('backend')
        self.backend_body_sensor = param('backend_body_sensor')
        self.map_frame = param('map_frame')
        self.odom_frame = param('odom_frame') or f'{robot}/odom'
        self.base_frame = param('base_frame') or f'{robot}/base_link'
        self.publish_tf = param('publish_tf')
        self.zero_odom_at_start = param('zero_odom_at_start')
        self.voxel_size = float(param('voxel_size'))
        self.max_map_points = int(param('max_map_points'))
        self.max_twist_dt = float(param('max_twist_dt'))

        if self.voxel_size <= 0.0:
            raise ValueError('voxel_size must be > 0')

        # --- extrinsics -----------------------------------------------------
        calib_file = param('calibration_file')
        if calib_file:
            self.calib = Calibration.from_file(calib_file)
            self.get_logger().info(
                f"calibration '{calib_file}' -> sensors {self.calib.names()}")
        else:
            self.calib = Calibration.identity()
            self.get_logger().warn(
                'no calibration_file given: assuming the backend already '
                'reports base_link. Poses will be offset by the IMU mount '
                'height if that is not true.')

        if calib_file and not self.calib.has(self.backend_body_sensor):
            self.get_logger().error(
                f"calibration has no sensor '{self.backend_body_sensor}' "
                f"(has {self.calib.names()}); treating the backend body frame "
                'as base_link')

        # The backend reports the pose of its body frame; we want base_link.
        self.T_body_base = invert(self.calib.T_base_sensor(self.backend_body_sensor))

        # Set from the first odometry message; see _odom_anchor().
        self.T_odom_world = None

        # --- rolling state for the finite-difference twist ------------------
        self._prev_T = None
        self._prev_stamp = None

        # --- voxel map ------------------------------------------------------
        self._keys = np.empty(0, dtype=np.int64)
        self._points = np.empty((0, 4), dtype=np.float32)
        self._map_capped = False
        self._cloud_seen = 0

        # --- ROS wiring -----------------------------------------------------
        # map_cloud is transient_local so a consumer that starts late (the
        # merger, the frontier detector) gets the map immediately instead of
        # waiting a whole publish period.
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_odom = self.create_publisher(Odometry, 'odom', 20)
        self.pub_map = self.create_publisher(PointCloud2, 'map_cloud', map_qos)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(
            Odometry, param('backend_odom_topic'), self.on_backend_odom, 20)
        self.create_subscription(
            PointCloud2, param('backend_cloud_topic'), self.on_backend_cloud, 10)

        period = float(param('map_publish_period'))
        self.create_timer(period, self.publish_map)

        if param('publish_identity_map_to_odom'):
            # Placeholder until Yong's coordinator owns the global alignment.
            # Without it the tree is disconnected and RViz/consumers that fix
            # on `map` see nothing, which looks like a SLAM failure at a demo.
            self._static_tf = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame
            t.transform.rotation.w = 1.0
            self._static_tf.sendTransform(t)
            self.get_logger().info(
                f'publishing placeholder identity {self.map_frame} -> '
                f'{self.odom_frame}; set publish_identity_map_to_odom:=false '
                'once the coordinator owns this transform')

        self.create_timer(5.0, self._report_health)

        self.get_logger().info(
            f'agc_slam interface up | robot={robot} backend={self.backend} '
            f'body_sensor={self.backend_body_sensor} '
            f'frames: {self.map_frame} -> {self.odom_frame} -> {self.base_frame} '
            f'| voxel={self.voxel_size} m, map every {period} s')

    # ------------------------------------------------------------------ odom

    def _odom_anchor(self, T_world_body):
        """Choose the odom frame origin from the first backend pose.

        The backend's world frame is wherever its body frame happened to be at
        startup (``camera_init`` for FAST-LIO2), which sits at the IMU mount
        height rather than on the ground. Anchoring on the first base_link
        pose makes ``<robot>/odom`` start at identity, which is what the ROS
        convention and the downstream consumers assume.
        """
        if not self.zero_odom_at_start:
            return np.eye(4)
        return invert(T_world_body @ self.T_body_base)

    def on_backend_odom(self, msg):
        T_world_body = pose_to_matrix(msg.pose.pose)
        if self.T_odom_world is None:
            self.T_odom_world = self._odom_anchor(T_world_body)
        T = self.T_odom_world @ T_world_body @ self.T_body_base

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = float(T[0, 3])
        out.pose.pose.position.y = float(T[1, 3])
        out.pose.pose.position.z = float(T[2, 3])
        qx, qy, qz, qw = matrix_to_quat(T[:3, :3])
        out.pose.pose.orientation.x = float(qx)
        out.pose.pose.orientation.y = float(qy)
        out.pose.pose.orientation.z = float(qz)
        out.pose.pose.orientation.w = float(qw)
        # The backend's pose covariance is for its own body frame. Re-expressing
        # it about base_link needs the full adjoint; the lever arm here is a few
        # centimetres, so it is passed through and flagged rather than faked.
        out.pose.covariance = msg.pose.covariance

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._fill_twist(out, T, stamp)
        self._prev_T, self._prev_stamp = T, stamp

        self.pub_odom.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def _fill_twist(self, out, T, stamp):
        """Finite-difference the pose into a body-frame twist.

        nav_msgs/Odometry defines twist in ``child_frame_id``, so both the
        linear and angular parts are rotated into base_link.
        """
        if self._prev_T is None or self._prev_stamp is None:
            return
        dt = stamp - self._prev_stamp
        if not 1e-4 < dt < self.max_twist_dt:
            # Backend restart, bag loop or a dropped run of messages: a
            # velocity differenced across that gap is worse than none.
            return
        R = T[:3, :3]
        v_world = (T[:3, 3] - self._prev_T[:3, 3]) / dt
        v_body = R.T @ v_world
        w_body = rotation_log(self._prev_T[:3, :3].T @ R) / dt
        out.twist.twist.linear.x = float(v_body[0])
        out.twist.twist.linear.y = float(v_body[1])
        out.twist.twist.linear.z = float(v_body[2])
        out.twist.twist.angular.x = float(w_body[0])
        out.twist.twist.angular.y = float(w_body[1])
        out.twist.twist.angular.z = float(w_body[2])

    # ----------------------------------------------------------------- cloud

    def on_backend_cloud(self, msg):
        self._cloud_seen += 1
        if self.T_odom_world is None:
            # Points arriving before any pose cannot be placed in odom yet.
            return
        if self._map_capped:
            return
        try:
            pts = pc2.read_points_numpy(
                msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        except Exception:
            # Not every backend publishes intensity (kiss-icp does not).
            pts = pc2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.hstack([pts, np.zeros((len(pts), 1), dtype=pts.dtype)])
        if pts.size == 0:
            return
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 4)

        # The backend publishes the registered cloud in its own world frame;
        # move it into odom so it agrees with the pose we publish.
        xyz = pts[:, :3] @ self.T_odom_world[:3, :3].T + self.T_odom_world[:3, 3]
        finite = np.isfinite(xyz).all(axis=1)
        xyz, inten = xyz[finite], pts[finite, 3]
        if len(xyz) == 0:
            return

        idx = np.floor(xyz / self.voxel_size).astype(np.int64) + _VOXEL_HALF
        in_range = ((idx >= 0) & (idx < _VOXEL_SPAN)).all(axis=1)
        if not in_range.all():
            self.get_logger().warn(
                f'{int((~in_range).sum())} points outside the voxel grid '
                'extent were dropped', once=True)
            idx, xyz, inten = idx[in_range], xyz[in_range], inten[in_range]
            if len(xyz) == 0:
                return

        keys = (idx[:, 0] << (2 * _VOXEL_BITS)) | (idx[:, 1] << _VOXEL_BITS) | idx[:, 2]

        # One representative point per voxel within this scan, ...
        keys, first = np.unique(keys, return_index=True)
        # ... then keep only the voxels the map has not seen before.
        if len(self._keys):
            slot = np.searchsorted(self._keys, keys)
            slot_clipped = np.minimum(slot, len(self._keys) - 1)
            already = self._keys[slot_clipped] == keys
            keys, first = keys[~already], first[~already]
        if len(keys) == 0:
            return

        new = np.empty((len(keys), 4), dtype=np.float32)
        new[:, :3] = xyz[first]
        new[:, 3] = inten[first]

        room = self.max_map_points - len(self._points)
        if len(new) > room:
            new, keys = new[:room], keys[:room]
            self._map_capped = True
            self.get_logger().warn(
                f'map_cloud reached max_map_points={self.max_map_points}; '
                'no longer growing. Raise max_map_points or voxel_size.')

        if len(keys):
            merged = np.concatenate([self._keys, keys])
            order = np.argsort(merged, kind='stable')
            self._keys = merged[order]
            self._points = np.concatenate([self._points, new])[order]

    def publish_map(self):
        if len(self._points) == 0:
            return
        self.pub_map.publish(self._to_pointcloud2(self._points))

    def _to_pointcloud2(self, points):
        """Pack an (N, 4) float32 array into PointCloud2 without a Python loop."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.odom_frame
        fields = [
            PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
            for n, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))
        ]
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(points)
        msg.is_dense = True
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        return msg

    # ---------------------------------------------------------------- health

    def _report_health(self):
        """Say plainly whether the interface is actually producing output.

        At an integration demo the useful question is not "did the node
        start" but "is it publishing", so this distinguishes the two common
        failure modes: no backend pose at all, and poses but no cloud.
        """
        if self.T_odom_world is None:
            self.get_logger().warn(
                'no backend odometry received yet -- check that the LIO '
                'backend is running and that backend_odom_topic matches it',
                throttle_duration_sec=20.0)
            return
        if self._cloud_seen == 0:
            self.get_logger().warn(
                'odometry is flowing but no backend cloud has arrived -- '
                'check backend_cloud_topic',
                throttle_duration_sec=20.0)
            return
        self.get_logger().info(
            f'map_cloud: {len(self._points)} voxels from {self._cloud_seen} '
            f'scans at {self.voxel_size} m',
            throttle_duration_sec=20.0)


def _print_calibration(argv):
    """Dump the derived transforms for a calibration file and exit.

    Used to regenerate the `derived_for_reference` block in the calibration
    YAML, and to sanity-check a freshly measured calibration before a run.
    """
    parser = argparse.ArgumentParser(prog='slam_interface --print-calibration')
    parser.add_argument('--print-calibration', dest='path', required=True)
    args, _ = parser.parse_known_args(argv[1:])
    calib = Calibration.from_file(args.path)
    np.set_printoptions(suppress=True, precision=6)
    print(f'robot: {calib.robot}  base_frame: {calib.base_frame}')
    print(f'sensors: {calib.names()}\n')
    for name in calib.names():
        t = calib.T_base_sensor(name)
        print(f'T_base_{name}: xyz={t[:3, 3]} frame_id={calib.frame_id(name)}')
    print()
    for a, b in (('imu', 'lidar'), ('lidar', 'camera_optical'),
                 ('lidar', 'imu'), ('camera_optical', 'lidar')):
        if calib.has(a) and calib.has(b):
            t = calib.T_a_b(a, b)
            r = t[:3, :3]
            pitch = math.asin(-max(-1.0, min(1.0, r[2, 0])))
            roll = math.atan2(r[2, 1], r[2, 2])
            yaw = math.atan2(r[1, 0], r[0, 0])
            print(f'T_{a}_{b}: xyz={t[:3, 3]} '
                  f'rpy=[{roll:.4f}, {pitch:.4f}, {yaw:.4f}]')
    return 0


def main(argv=None):
    argv = sys.argv if argv is None else argv
    if '--print-calibration' in argv:
        return _print_calibration(argv)
    rclpy.init(args=argv)
    node = None
    try:
        node = SlamInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
