"""Minimal SE(3) helpers and calibration loading.

Deliberately depends on nothing but numpy and PyYAML so the package builds
unchanged on ROS 2 Humble and Jazzy (tf_transformations / transforms3d are
not packaged consistently across the two distros).

Convention used throughout: ``T_a_b`` is the 4x4 homogeneous transform that
takes a point expressed in frame ``b`` and returns it in frame ``a``.  It is
equivalently the pose of frame ``b`` expressed in frame ``a``.
"""

import numpy as np
import yaml


def rpy_to_matrix(roll, pitch, yaw):
    """Build a rotation matrix from URDF/ROS fixed-axis RPY (Rz @ Ry @ Rx)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quat_to_matrix(x, y, z, w):
    """Rotation matrix from a ROS (x, y, z, w) quaternion."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(r):
    """ROS (x, y, z, w) quaternion from a rotation matrix (Shepperd's method)."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def make_transform(xyz, rpy=None, quat=None):
    """Assemble a 4x4 transform from a translation plus RPY or a quaternion."""
    t = np.eye(4)
    t[:3, 3] = np.asarray(xyz, dtype=float)
    if quat is not None:
        t[:3, :3] = quat_to_matrix(*quat)
    elif rpy is not None:
        t[:3, :3] = rpy_to_matrix(*np.asarray(rpy, dtype=float))
    return t


def invert(t):
    """Invert a rigid transform without a general matrix inverse."""
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def pose_to_matrix(pose):
    """4x4 transform from a geometry_msgs/Pose."""
    q = pose.orientation
    p = pose.position
    return make_transform([p.x, p.y, p.z], quat=[q.x, q.y, q.z, q.w])


def rotation_log(r):
    """Rotation-vector (axis * angle) of a rotation matrix; used for angular rate."""
    cos_theta = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-9:
        # Small-angle: the skew part is already the rotation vector.
        return np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) * 0.5
    if np.pi - theta < 1e-6:
        # Near pi, the skew part degenerates; recover the axis from R + I.
        a = (r + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(a), 0.0, None))
        idx = int(np.argmax(axis))
        if axis[idx] > 1e-9:
            axis = a[:, idx] / axis[idx]
        return axis / max(np.linalg.norm(axis), 1e-12) * theta
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return axis / (2.0 * np.sin(theta)) * theta


class Calibration:
    """Sensor extrinsics for one robot, loaded from a calibration YAML.

    The YAML stores every sensor pose relative to ``base_link``, which is the
    one convention the whole team can agree on: it is what the URDF publishes
    and what the frozen frame tree is anchored to.  Sensor-to-sensor
    transforms are derived here rather than stored, so there is exactly one
    source of truth per sensor.
    """

    def __init__(self, data):
        self.robot = data.get('robot', 'unknown')
        self.base_frame = data.get('base_frame', 'base_link')
        self._sensors = {}
        for name, entry in (data.get('sensors') or {}).items():
            self._sensors[name] = {
                'frame_id': entry.get('frame_id', name),
                'T_base_sensor': make_transform(
                    entry.get('xyz', [0.0, 0.0, 0.0]),
                    rpy=entry.get('rpy', [0.0, 0.0, 0.0]),
                ),
            }

    @classmethod
    def from_file(cls, path):
        with open(path, 'r') as handle:
            return cls(yaml.safe_load(handle) or {})

    @classmethod
    def identity(cls):
        return cls({})

    def has(self, sensor):
        return sensor in self._sensors

    def names(self):
        return sorted(self._sensors)

    def frame_id(self, sensor):
        return self._sensors[sensor]['frame_id']

    def T_base_sensor(self, sensor):
        """Pose of ``sensor`` in base_link. Identity if the sensor is unknown."""
        if sensor not in self._sensors:
            return np.eye(4)
        return self._sensors[sensor]['T_base_sensor'].copy()

    def T_sensor_base(self, sensor):
        return invert(self.T_base_sensor(sensor))

    def T_a_b(self, sensor_a, sensor_b):
        """Transform between two calibrated sensors, via base_link."""
        return self.T_sensor_base(sensor_a) @ self.T_base_sensor(sensor_b)
