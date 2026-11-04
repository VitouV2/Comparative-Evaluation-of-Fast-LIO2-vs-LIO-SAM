# agc_slam — SLAM module

Air–Ground Collaborative Mapping, 3-month plan. **Module owner: C. Vitou.**

This package is the SLAM module's half of the frozen team interface. It wraps a
LiDAR-inertial odometry backend and publishes, for one robot:

| Topic | Type | Consumed by |
|---|---|---|
| `<robot>/odom` | `nav_msgs/Odometry` | everyone |
| `<robot>/map_cloud` | `sensor_msgs/PointCloud2` | N. Vitou (merging), Chamreourn (frontiers) |

Frame tree: `map → <robot>/odom → <robot>/base_link`.

## Why a wrapper instead of patching the backend

No LIO backend publishes the frozen interface. FAST-LIO2 publishes `/Odometry`
and `/cloud_registered` in hard-coded frames `camera_init` / `body`, reports the
pose of the **IMU** rather than `base_link`, leaves `twist` entirely zero, and
emits `/cloud_registered` as the current deskewed scan rather than an
accumulated map.

P2 benchmarks FAST-LIO2 against LIO-SAM and Point-LIO, then adds FAST-LIVO2 for
colour. Those swaps must not change anything a consumer sees. So the backend
stays stock and un-namespaced on its own private topics, and this node adapts
it. Swapping backend is a `backend:=` argument, not a code change.

## Quick start

```bash
# 1. simulation (or: ros2 bag play <bag> --clock)
ros2 launch warthog_gazebo multi_robot_warehouse.launch.py

# 2. SLAM module
ros2 launch agc_slam slam.launch.py robot_name:=ugv1 use_sim_time:=true rviz:=true

# 3. drive the robot, then check the interface against the spec
$(ros2 pkg prefix agc_slam)/share/agc_slam/scripts/verify_interface.sh ugv1
```

`verify_interface.sh` is the pre-demo gate check: it asserts the topics exist
with the right types, are actually publishing, carry the right `frame_id` and
`child_frame_id`, that `map_cloud` is non-empty, and that the frame tree
resolves end to end. Exit code 0 means ready to merge.

The frame-tree hop uses `ros2 run agc_slam check_frames <robot>` rather than
`tf2_echo` scraping: `tf2_echo`'s buffer is wall-clock driven, so replaying a
bag recorded months ago makes it spin on "jump back in time" and report a
healthy frame tree as broken. `check_frames` asks tf2 the question consumers
actually ask — can the latest transform be resolved — and is time-domain
agnostic.

## What the interface node does

1. **Re-anchors the odom frame.** FAST-LIO2's world frame is wherever the IMU
   sat at startup — about 0.53 m above the ground on the Husky's sensor arch.
   `<robot>/odom` is re-anchored so it starts at the robot's initial
   `base_link` pose, which is the ROS convention and what Phumint's
   relocalisation and Yong's allocator assume.
2. **Moves the pose onto `base_link`** using the extrinsics in the calibration
   file. `backend_body_sensor` says which sensor the backend's pose belongs to
   (`imu` for FAST-LIO2 and Point-LIO, `lidar` for LIO-SAM). Getting this wrong
   offsets every pose by a sensor mount height with no error message — which is
   why the regression test checks it.
3. **Derives `twist`** by finite-differencing pose into the `base_link` frame.
   FAST-LIO2 never fills it, and a `nav_msgs/Odometry` carrying a zero twist is
   a trap for consumers.
4. **Accumulates `map_cloud`** into a voxel grid, published downsampled at a low
   rate. This is what the consumers need (a map, not a scan) and it is the
   mitigation for the plan's *"map clouds too large for Wi-Fi"* risk.
   On the `slam_run` bag: **13 655 voxels ≈ 0.22 MB per publish** at a 0.2 m
   leaf over a 30 × 50 m warehouse. Turn `voxel_size` up and
   `map_publish_period` down if the link saturates.

## Calibration

`config/calibration/ugv1_husky_a200.yaml` stores every sensor pose relative to
`base_link`; sensor-to-sensor transforms are **derived** from those, so each
sensor has exactly one source of truth and the chains cannot disagree.

```bash
ros2 run agc_slam slam_interface --print-calibration \
  $(ros2 pkg prefix agc_slam)/share/agc_slam/config/calibration/ugv1_husky_a200.yaml
```

Values come from composing the Husky's fixed-joint URDF chain, cross-checked
against the independent `tf2_echo` capture in `~/calib_ground_truth.txt`
(July 2026). The two agree to under 1 mm.

> **Status: sim-exact, not field-measured.** The physical Husky carries no
> camera, so the camera entries are simulation-only and must be re-measured on
> hardware with a ChArUco target before the P4 field runs. The lidar and IMU
> entries are mechanical mount values and should hold on hardware.
> `camera_intrinsics.K` is deliberately left empty so nobody silently uses a
> wrong `K` — capture it from the live `camera_info`.

## Shared dataset

```bash
$(ros2 pkg prefix agc_slam)/share/agc_slam/scripts/record_shared_dataset.sh ugv1_warehouse_01
```

Records **raw** sensor streams plus ground truth and `/clock` — never SLAM
output. If the bags contained `/ugv1/odom` instead of raw lidar and IMU, nobody
could re-run a different backend on them, which is exactly what the P2
benchmark has to do. The script records only topics that are actually
publishing (rosbag2 otherwise waits forever on a topic that never appears),
reports the ones that are missing, and writes a manifest next to the bag with
the calibration in effect at record time.

## Tests

```bash
colcon test --packages-select agc_slam
```

The regression test drives the interface with synthetic FAST-LIO2-shaped input
on a known trajectory and asserts the full frozen contract: topics, types,
frames, the odom anchor, the pose offset onto `base_link`, the derived twist,
voxel deduplication, and the TF tree. It runs on a private `ROS_DOMAIN_ID`
(default 91) because the team shares one domain on the lab Wi-Fi.

## Known gaps

- **ROS distro.** The plan freezes the team on Jazzy + Gazebo Harmonic. This
  machine is Ubuntu 22.04 / Humble / Ignition Fortress, and Jazzy needs Ubuntu
  24.04. This package avoids `tf_transformations` and other distro-sensitive
  deps so it builds unchanged on both, but the Jazzy Docker image is still
  outstanding work before G1.
- **Pose covariance** is passed through from the backend rather than
  re-expressed about `base_link`; the lever arm is a few centimetres. Flagged
  rather than faked.
- **Backends other than FAST-LIO2** are wired in `BACKENDS` in the launch file
  but not launched by it yet — start them yourself and this node adapts their
  output. They get launched as part of the P2 benchmark.
