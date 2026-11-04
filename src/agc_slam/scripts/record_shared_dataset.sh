#!/usr/bin/env bash
# ============================================================================
# Record the shared dataset for the Air-Ground Collaborative Mapping project.
#   Deliverable: P1 (Oct 26 - Nov 18), gate G1 -- "shared dataset recorded"
#   Owner: C. Vitou
#
# The whole team develops against these bags, so the recording must be
# reproducible and must capture the RAW sensor streams plus ground truth --
# never the SLAM output. If we record /ugv1/odom instead of the raw lidar and
# IMU, nobody can re-run a different backend on the data, which is exactly
# what the P2 benchmark (FAST-LIO2 vs LIO-SAM vs Point-LIO) needs to do.
#
# Usage:
#   ./record_shared_dataset.sh ugv1_warehouse_01            # record
#   ./record_shared_dataset.sh ugv1_warehouse_01 --dry-run  # list topics only
# ============================================================================
set -euo pipefail

NAME="${1:-shared_$(date +%Y%m%d_%H%M%S)}"
DRY_RUN="${2:-}"
OUT_DIR="${AGC_DATASET_DIR:-$HOME/agc_datasets}"
ROBOT_NS="${ROBOT_NS:-/a200_0001}"
DEST="$OUT_DIR/$NAME"

# Raw sensor streams + ground truth + clock. Deliberately NOT /Odometry,
# /cloud_registered or /ugv1/*: those are derived and must stay reproducible.
TOPICS=(
  "$ROBOT_NS/sensors/lidar3d_0/points_timed"   # VLP-16, per-point timestamps
  "$ROBOT_NS/sensors/imu_0/data"               # Microstrain IMU @ 400 Hz
  "$ROBOT_NS/sensors/imu_0/data_raw"
  "$ROBOT_NS/platform/odom"                    # wheel odometry
  "$ROBOT_NS/platform/joint_states"
  "$ROBOT_NS/cmd_vel"
  "$ROBOT_NS/sensors/camera_0/color/image_raw"
  "$ROBOT_NS/sensors/camera_0/color/camera_info"
  "$ROBOT_NS/sensors/camera_0/depth/image_rect_raw"
  "$ROBOT_NS/sensors/camera_0/depth/camera_info"
  "/tf"
  "/tf_static"
  "/clock"
)

echo "=== agc shared dataset recording ==="
echo "name       : $NAME"
echo "destination: $DEST"
echo "namespace  : $ROBOT_NS"
echo

# Record only what is actually publishing. rosbag2 will sit and wait forever
# on a topic that never appears, and a half-dead recording discovered after
# the run is worse than a short topic list known up front.
AVAILABLE=()
MISSING=()
LIVE="$(ros2 topic list 2>/dev/null || true)"
if [[ -z "$LIVE" ]]; then
  echo "ERROR: no ROS graph visible. Start the simulation first." >&2
  exit 1
fi
for t in "${TOPICS[@]}"; do
  if grep -Fxq "$t" <<<"$LIVE"; then AVAILABLE+=("$t"); else MISSING+=("$t"); fi
done

printf 'recording (%d):\n' "${#AVAILABLE[@]}"
printf '  + %s\n' "${AVAILABLE[@]}"
if ((${#MISSING[@]})); then
  printf 'NOT PUBLISHING, will be absent from the bag (%d):\n' "${#MISSING[@]}"
  printf '  - %s\n' "${MISSING[@]}"
  echo
  echo "NOTE: the camera topics are expected to be missing on the physical"
  echo "Husky, which currently carries no camera. They should be present in"
  echo "simulation -- if they are not, the RealSense is not being launched"
  echo "and the dataset will not support the P2 colourised map."
fi

if [[ "$DRY_RUN" == "--dry-run" ]]; then
  echo; echo "dry run: nothing recorded."; exit 0
fi
if ((${#AVAILABLE[@]} == 0)); then
  echo "ERROR: none of the expected topics are publishing." >&2
  exit 1
fi
if [[ -e "$DEST" ]]; then
  echo "ERROR: $DEST already exists; pick another name." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

# Record the calibration and the topic list alongside the bag. A bag whose
# extrinsics are unknown six weeks later cannot be used for a benchmark.
SIDECAR="$DEST.manifest.txt"

echo
echo "recording -- Ctrl-C to stop"
ros2 bag record -o "$DEST" --storage mcap "${AVAILABLE[@]}" || true

{
  echo "dataset: $NAME"
  echo "recorded: $(date -Iseconds)"
  echo "recorded_by: C. Vitou"
  echo "namespace: $ROBOT_NS"
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
  echo
  echo "topics:"
  printf '  %s\n' "${AVAILABLE[@]}"
  echo
  echo "calibration (copied below from the file in effect at record time):"
  CALIB="$(ros2 pkg prefix agc_slam 2>/dev/null)/share/agc_slam/config/calibration/ugv1_husky_a200.yaml"
  if [[ -f "$CALIB" ]]; then sed 's/^/  /' "$CALIB"; else echo "  MISSING: $CALIB"; fi
} > "$SIDECAR"

echo
echo "bag     : $DEST"
echo "manifest: $SIDECAR"
echo
echo "Next: verify it replays and drives the SLAM module:"
echo "  ros2 bag play $DEST --clock"
echo "  ros2 launch agc_slam slam.launch.py robot_name:=ugv1 use_sim_time:=true"
echo "  \$(ros2 pkg prefix agc_slam)/share/agc_slam/scripts/verify_interface.sh ugv1"
