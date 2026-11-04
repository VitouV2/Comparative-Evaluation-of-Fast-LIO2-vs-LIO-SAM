"""Regression test for the frozen team interface.

Drives agc_slam's slam_interface with synthetic FAST-LIO2-shaped input on a
known trajectory and asserts that what comes out is exactly what the G1
interface spec promises the rest of the team:

    <robot>/odom       nav_msgs/Odometry, frames <robot>/odom -> <robot>/base_link
    <robot>/map_cloud  sensor_msgs/PointCloud2, accumulated and voxel-downsampled
    TF: map -> <robot>/odom -> <robot>/base_link

Run with:  colcon test --packages-select agc_slam
or stand-alone:  python3 -m pytest src/agc_slam/test/test_slam_interface.py -s

This exists because P2 swaps the backend (LIO-SAM, Point-LIO, FAST-LIVO2).
A backend swap must not change anything a consumer sees, and the quiet way to
break that is to get `backend_body_sensor` wrong, which offsets every pose by
a sensor mount height without any error message.
"""

import math
import os
import sys

# Run on a private ROS domain with localhost-only discovery. The team shares
# one domain on the lab Wi-Fi (per the interface spec), so without this a
# teammate's running SLAM node -- or a stray one of our own -- publishes onto
# /ugv1/odom alongside the node under test and the assertions below compare
# against someone else's poses. Must be set before rclpy is imported.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('AGC_TEST_DOMAIN_ID', '91')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('ROS_STATIC_PEERS', None)

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

from agc_slam.slam_interface import SlamInterface
from agc_slam.transforms import matrix_to_quat, rpy_to_matrix

from ament_index_python.packages import get_package_share_directory
import os

CALIB = os.path.join(
    get_package_share_directory('agc_slam'),
    'config', 'calibration', 'ugv1_husky_a200.yaml')
IMU_Z = 0.5265
V, WZ, DT = 1.0, 0.1, 0.1   # 1 m/s forward, 0.1 rad/s yaw, 10 Hz

class Driver(Node):
    """Pretends to be FAST-LIO2: /Odometry + /cloud_registered in camera_init."""
    def __init__(self):
        super().__init__('driver')
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 20)
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        self.k = 0
        self.poses = []

    def body_pose(self, t):
        yaw = WZ * t
        # constant body-forward speed, integrated
        if abs(WZ) > 1e-9:
            x = V / WZ * math.sin(yaw); y = V / WZ * (1 - math.cos(yaw))
        else:
            x, y = V * t, 0.0
        T = np.eye(4); T[:3, :3] = rpy_to_matrix(0, 0, yaw); T[:3, 3] = [x, y, 0.0]
        return T

    def step(self):
        t = self.k * DT
        T = self.body_pose(t); self.poses.append((t, T))
        stamp = rclpy.time.Time(seconds=t + 100.0).to_msg()
        m = Odometry()
        m.header.stamp = stamp; m.header.frame_id = 'camera_init'; m.child_frame_id = 'body'
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = T[:3, 3]
        q = matrix_to_quat(T[:3, :3])
        m.pose.pose.orientation.x, m.pose.pose.orientation.y = q[0], q[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = q[2], q[3]
        self.odom_pub.publish(m)

        # a 4x4 m patch of "ground" under the robot, in camera_init.
        # Ground is IMU_Z below the IMU start, i.e. world z = -IMU_Z.
        g = np.mgrid[-2:2:0.25, -2:2:0.25].reshape(2, -1).T
        pts = np.zeros((len(g), 4), dtype=np.float32)
        pts[:, 0] = g[:, 0] + T[0, 3]; pts[:, 1] = g[:, 1] + T[1, 3]
        pts[:, 2] = -IMU_Z; pts[:, 3] = 42.0
        h = Header(); h.stamp = stamp; h.frame_id = 'camera_init'
        c = PointCloud2(); c.header = h; c.height = 1; c.width = len(pts)
        c.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                    for n, o in (('x',0),('y',4),('z',8),('intensity',12))]
        c.is_bigendian = False; c.point_step = 16; c.row_step = 16*len(pts)
        c.is_dense = True; c.data = pts.tobytes()
        self.cloud_pub.publish(c)
        self.k += 1

class Sink(Node):
    def __init__(self):
        super().__init__('sink')
        self.odoms, self.clouds = [], []
        self.create_subscription(Odometry, '/ugv1/odom', self.odoms.append, 50)
        self.create_subscription(PointCloud2, '/ugv1/map_cloud', self.clouds.append, 5)
        self.buf = Buffer(); self.listener = TransformListener(self.buf, self)

def main():  # noqa: C901
    rclpy.init(args=[
        '--ros-args',
        '-r', 'slam_interface:__ns:=/ugv1',
        '-p', 'robot_name:=ugv1',
        '-p', f'calibration_file:={CALIB}',
        '-p', 'backend_body_sensor:=imu',
        '-p', 'voxel_size:=0.25',
        '-p', 'map_publish_period:=0.3',
        '-p', 'publish_identity_map_to_odom:=true',
    ])
    iface = SlamInterface()
    drv, sink = Driver(), Sink()
    ex = SingleThreadedExecutor()
    for n in (iface, drv, sink): ex.add_node(n)

    for _ in range(60):
        drv.step()
        for _ in range(12): ex.spin_once(timeout_sec=0.01)
    for _ in range(80): ex.spin_once(timeout_sec=0.01)

    fails = []
    def chk(ok, label, extra=''):
        print(('  PASS  ' if ok else '  FAIL  ') + label + (f'   {extra}' if extra else ''))
        if not ok: fails.append(label)

    print(f'\n=== received {len(sink.odoms)} odom, {len(sink.clouds)} map_cloud ===')
    chk(len(sink.odoms) >= 50, 'odom messages relayed', f'{len(sink.odoms)}')
    chk(len(sink.clouds) >= 1, 'map_cloud published', f'{len(sink.clouds)}')
    if not sink.odoms: 
        print('no odom, aborting'); return 1

    o0, oN = sink.odoms[0], sink.odoms[-1]
    chk(o0.header.frame_id == 'ugv1/odom', 'header.frame_id', o0.header.frame_id)
    chk(o0.child_frame_id == 'ugv1/base_link', 'child_frame_id', o0.child_frame_id)

    p0 = o0.pose.pose.position
    chk(abs(p0.x) < 1e-6 and abs(p0.y) < 1e-6 and abs(p0.z) < 1e-6,
        'odom starts at identity (zero_odom_at_start)',
        f'({p0.x:.2e}, {p0.y:.2e}, {p0.z:.2e})')

    # base_link must track the IMU trajectory shifted down by the mount height,
    # which for identity mount rotation means the SAME x,y and z = 0.
    if len(sink.odoms) > len(drv.poses):
        print(f'  FAIL  more odom messages ({len(sink.odoms)}) than poses '
              f'published ({len(drv.poses)}) -- another slam_interface is '
              'publishing onto /ugv1/odom. Check: pgrep -af slam_interface')
        fails.append('duplicate publisher on /ugv1/odom')
        return 1
    tN, TN = drv.poses[len(sink.odoms)-1]
    pN = oN.pose.pose.position
    exp = TN[:3, 3].copy()   # imu xy; z stays 0 because base is straight below
    chk(abs(pN.x-exp[0]) < 1e-5 and abs(pN.y-exp[1]) < 1e-5 and abs(pN.z) < 1e-5,
        'base_link pose matches IMU trajectory minus mount height',
        f'got ({pN.x:.4f},{pN.y:.4f},{pN.z:.4f}) want ({exp[0]:.4f},{exp[1]:.4f},0.0000)')

    # derived twist: body-frame forward speed V, yaw rate WZ
    mids = sink.odoms[len(sink.odoms)//2]
    tw = mids.twist.twist
    chk(abs(tw.linear.x - V) < 0.02, 'derived twist linear.x == forward speed',
        f'{tw.linear.x:.4f} vs {V}')
    chk(abs(tw.linear.y) < 0.02 and abs(tw.linear.z) < 1e-3,
        'derived twist has no spurious lateral/vertical speed',
        f'y={tw.linear.y:.2e} z={tw.linear.z:.2e}')
    chk(abs(tw.angular.z - WZ) < 0.01, 'derived twist angular.z == yaw rate',
        f'{tw.angular.z:.4f} vs {WZ}')

    # map_cloud
    c = sink.clouds[-1]
    chk(c.header.frame_id == 'ugv1/odom', 'map_cloud frame_id', c.header.frame_id)
    arr = pc2.read_points_numpy(c, field_names=('x','y','z','intensity'))
    chk(len(arr) > 0, 'map_cloud non-empty', f'{len(arr)} points')
    chk(np.allclose(arr[:,2], 0.0, atol=1e-4),
        'ground plane lands at z=0 in odom (world->odom anchor applied)',
        f'z range [{arr[:,2].min():.4f}, {arr[:,2].max():.4f}]')
    chk(np.allclose(arr[:,3], 42.0), 'intensity preserved')

    # voxel dedup: 60 scans x 256 pts = 15360 raw; dedup must cut it hard
    raw = 60*256
    chk(len(arr) < raw*0.5, 'voxel grid deduplicated overlapping scans',
        f'{len(arr)} unique voxels from {raw} raw points')
    k = np.floor(arr[:,:3]/0.25).astype(np.int64)
    chk(len(np.unique(k, axis=0)) == len(arr), 'exactly one point per voxel')

    # growth is monotone and bounded
    if len(sink.clouds) >= 2:
        ws = [cc.width for cc in sink.clouds]
        chk(all(b >= a for a, b in zip(ws, ws[1:])), 'map grows monotonically', f'{ws}')

    # TF tree
    import time
    for _ in range(40): ex.spin_once(timeout_sec=0.01)
    for parent, child in (('map','ugv1/odom'), ('ugv1/odom','ugv1/base_link'), ('map','ugv1/base_link')):
        ok = sink.buf.can_transform(parent, child, rclpy.time.Time())
        chk(ok, f'TF {parent} -> {child} resolves')

    for n in (iface, drv, sink): n.destroy_node()
    rclpy.shutdown()
    print(f'\n=== {len(fails)} failure(s) ===')
    for f in fails: print('   -', f)
    return 1 if fails else 0

def test_frozen_interface():
    assert main() == 0, 'slam_interface does not match the frozen G1 spec'


if __name__ == '__main__':
    sys.exit(main())
