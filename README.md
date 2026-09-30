# Quantifying the Cost of Missing Per-Point Timestamps in Simulated LiDAR

A comparative evaluation of **Fast-LIO2**, **Point-LIO**, **LIO-SAM** and **KISS-ICP** in ROS 2 Humble.

Undergraduate thesis — Chea Vitou (ID 6023010001)
BSc Robotics and Automation Engineering, Cambodia University of Technology and Science, 2026

---

## What this measures

The Ignition Gazebo lidar sensor publishes `PointCloud2` messages containing `x`, `y`, `z`, `intensity` and `ring` — but **no per-point `time` field**. Physical LiDAR drivers supply that field, and SLAM algorithms use it to deskew scans, correcting for the robot's motion during the ~50 ms a sweep takes to complete.

The omission is already known to the Gazebo community ([gz-sensors #507](https://github.com/gazebosim/gz-sensors/issues/507)). What was not known is what it costs.

This repository contains the framework used to measure that. Each algorithm runs twice over an identical route on a Clearpath Husky A200 in a simulated warehouse:

| Condition | Topic |
|---|---|
| **Raw** | `/a200_0001/sensors/lidar3d_0/points` — no `time` field |
| **Reconstructed** | `/a200_0001/sensors/lidar3d_0/points_timed` — timing added by `cloud_timestamper.py` |

The difference between an algorithm's two runs is what the missing field costs it.

---

## Results

Absolute Trajectory Error, RMSE in metres. Route: three laps of a 2×3 m rectangle, 37.8 m total, 12 turns, 0.2 m/s.

| Algorithm | Raw | Reconstructed | Factor |
|---|---|---|---|
| LIO-SAM | 0.007 | 0.011 | — |
| KISS-ICP | 0.051 | 0.056 | — |
| Point-LIO | 0.953 | **0.005** | 191× |
| Fast-LIO2 | 17.756 | **0.021** | 846× |

Relative Position Error over 1 m segments agrees with ATE on every cell.

**The cost ranges from nothing to a factor of 846, and architecture does not predict it.** Three of the four algorithms are IMU-coupled; two are devastated by the missing field and one is untouched.

- **KISS-ICP** uses no IMU and performs its own motion compensation, so it should be indifferent to the field. It is. This is the control.
- **LIO-SAM** is unaffected here despite being IMU-coupled. It reads the `ring` field, which this sensor supplies, and appears to take a code path that does not require `time`.
- **Point-LIO** and **Fast-LIO2** fail in different ways. Point-LIO stays approximately correct but becomes extremely noisy. Fast-LIO2 produces a smooth trajectory that is grossly wrong, consistent with registration errors compounding through its own map.

### Note on an earlier phase

An earlier phase of this project used a Clearpath Warthog W200 in an outdoor environment. On that robot the sensor supplied **neither** `time` **nor** `ring`, and LIO-SAM was unusable — 21.9 m of reported displacement over 20 s with the robot stationary. The contrast between the two robots is what suggests `ring` rather than `time` is the field LIO-SAM actually depends on. That remains an inference, not a controlled result; see Future Work.

---

## Repository contents

```
src/
  FAST_LIO/              Fast-LIO2, Husky config
  point_lio_ros2/        Point-LIO (ROS 2 port), Husky config
  LIO-SAM/               LIO-SAM (ros2 branch), Husky config
  kiss-icp/              KISS-ICP
  cloud_tools/
    cloud_timestamper.py Reconstructs per-point timing from azimuth
    gt_pose.py           Extracts world-frame ground truth from Gazebo
    route_driver.py      Drives a fixed, calibrated route
bags/                    Recorded trajectories
```

---

## Setup

Ubuntu 22.04, ROS 2 Humble, Ignition Gazebo Fortress.

```bash
cd ~/warthog_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**KISS-ICP build note.** It requires CMake ≥ 3.24 while Ubuntu 22.04 ships 3.22. Installing a newer CMake via pip may overshoot — 4.x rejects a bundled Sophus dependency for declaring too *low* a minimum. This works:

```bash
colcon build --packages-select kiss_icp \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

---

## Running an experiment

Three nodes stay up for the whole session:

```bash
# 1. Simulation — then press PLAY in the Gazebo window
ros2 launch clearpath_gz simulation.launch.py

# 2. Timestamp reconstruction
python3 src/cloud_tools/cloud_timestamper.py --ros-args \
  -p use_sim_time:=true -p reverse:=true \
  -p input_topic:=/a200_0001/sensors/lidar3d_0/points \
  -p output_topic:=/a200_0001/sensors/lidar3d_0/points_timed

# 3. Ground truth
ros2 run ros_gz_bridge parameter_bridge \
  /model/a200_0001/robot/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V &
python3 src/cloud_tools/gt_pose.py --ros-args -p use_sim_time:=true -p index:=5
```

Index 5 is the model root in the world frame. Earlier entries in the pose array are the four wheels in robot-local coordinates and are **not** usable as ground truth.

Then launch one algorithm:

| Algorithm | Launch | Odometry topic | Node |
|---|---|---|---|
| Fast-LIO2 | `ros2 launch fast_lio mapping.launch.py config_file:=husky_vlp16.yaml use_sim_time:=true` | `/Odometry` | `/laser_mapping` |
| Point-LIO | `ros2 launch point_lio mapping_velody16.launch.py` | `/aft_mapped_to_init` | `/laserMapping` |
| LIO-SAM | `ros2 launch lio_sam run.launch.py` | `/lio_sam/mapping/odometry` | `/lio_sam_imageProjection` |
| KISS-ICP | `ros2 launch kiss_icp odometry.launch.py topic:=<TOPIC> visualize:=false` | `/kiss/odometry` | — |

Record, drive, evaluate:

```bash
cd bags
ros2 bag record <ALGO_ODOM> /ground_truth/odom /clock -o <name>
# wait for "All requested topics are subscribed", then:
python3 ../src/cloud_tools/route_driver.py --ros-args \
  -p use_sim_time:=true -p route:=lap3

evo_ape bag2 <name> /ground_truth/odom <ALGO_ODOM> --align
evo_rpe bag2 <name> /ground_truth/odom <ALGO_ODOM> --delta 1.0 --delta_unit m --align
```

---

## Things that cost me time

Documented here because several fail **silently** rather than producing an error.

**Verify topics before recording.** `ros2 bag record` does not error on a topic that does not exist yet — it waits, and you get an empty bag. Always check `ros2 topic hz` on both the algorithm output and ground truth first.

**`ros2 param get` is the only reliable check.** After editing a config: rebuild if needed, **kill the node**, relaunch, then confirm with `ros2 param get <node> <key>`. A running node keeps its loaded parameters regardless of what is on disk.

**Fast-LIO2 and Point-LIO copy their configs at build time.** Editing the file under `src/` does nothing until you `colcon build`. LIO-SAM's config is symlinked; KISS-ICP takes its topic as a launch argument.

**YAML failures are silent.** A tab instead of spaces, a stray `#`, or a duplicate key later in the file will cause the parser to drop that key. The algorithm then falls back to a built-in default topic, subscribes to nothing, and publishes nothing — with no error anywhere.

**Sort the reconstructed cloud by timestamp.** Ignition emits points grouped by beam, not in sweep order. Downstream code reads the sweep duration from the *last* array element, so an unsorted output reports roughly half the true duration — which is worse than supplying no timing at all.

**Calibrate open-loop motion.** Commanded durations fall short through actuator ramp-up. Measured against ground truth: a 90° turn at 0.3 rad/s achieved 87.3°, and 2 m commanded at 0.2 m/s achieved 1.62 m. Uncalibrated, the rectangular route ended 2.56 m from its start; calibrated, it closes within 0.04 m.

---

## Limitations

- Each cell is a **single run**. No confidence intervals.
- One robot, one environment, a 37.8 m route at 0.2 m/s.
- The reconstruction infers timing from azimuth assuming constant rotation rate and one sweep per message. Both hold in simulation; neither is guaranteed on hardware.
- LIO-SAM publishes at ~6 Hz against 49–80 Hz for the others, so its error figures come from a sparser trajectory and are not directly comparable.
- The explanation for the two failure modes is inferred from behaviour, not demonstrated by controlled test.

## Future work

1. **Isolate the `ring` field** — supply `time` while withholding `ring`, and run LIO-SAM against it. This converts the strongest inference here into a measured result.
2. **Test the map-corruption hypothesis** — run Fast-LIO2 raw on a straight-line route. Minimal turning should mean minimal smearing and a much smaller error.
3. **Repeat each cell** for confidence intervals. The automated route driver makes repeat runs genuinely comparable.
4. **Vary speed and environment.** Scan distortion scales with velocity.
5. **Validate on hardware**, where a physical Velodyne supplies both fields.
6. **Contribute upstream** to [gz-sensors #507](https://github.com/gazebosim/gz-sensors/issues/507).

---

## References

- Xu et al., *Fast-LIO2: Fast Direct LiDAR-Inertial Odometry*, IEEE T-RO 2022
- He et al., *Point-LIO: Robust High-Bandwidth LiDAR-Inertial Odometry*, Adv. Intell. Syst. 2023
- Shan et al., *LIO-SAM: Tightly-Coupled LiDAR Inertial Odometry via Smoothing and Mapping*, IROS 2020
- Vizzo et al., *KISS-ICP: In Defense of Point-to-Point ICP*, IEEE RA-L 2023
- Grupp, *evo: Python package for the evaluation of odometry and SLAM*, 2017

## Acknowledgements

Supervised by Dr. Doyun Lee (project supervisor), Mr. Prum Lipheng (CamTech mentor) and Mr. Mel Sokkheng (industry supervisor, Tribal Education Group).
