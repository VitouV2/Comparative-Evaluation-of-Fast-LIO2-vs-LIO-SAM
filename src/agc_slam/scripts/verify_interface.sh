#!/usr/bin/env bash
# ============================================================================
# Gate check for the SLAM module's frozen interface.
#   Owner: C. Vitou.  Run this before every Wednesday integration demo.
#
# Checks what G1 actually asks for -- that the module publishes ITS topics,
# with the right types, in the right frames, with the frame tree connected.
# "The node starts" is not the gate; this is.
#
# Usage: ./verify_interface.sh [robot_name]      (default: ugv1)
# Exits 0 if every check passes, 1 otherwise.
# ============================================================================
set -uo pipefail

ROBOT="${1:-ugv1}"
PASS=0
FAIL=0

green() { printf '  \033[32mPASS\033[0m  %s\n' "$1"; PASS=$((PASS+1)); }
red()   { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FAIL=$((FAIL+1)); }
info()  { printf '        %s\n' "$1"; }

# `ros2 topic echo` mixes diagnostics into STDOUT -- "WARNING: topic ... does
# not appear to be published yet" and "A message was lost!!!" among them. A
# naive capture stores that text as the field value, so comparisons fail for
# the wrong reason, and a bare non-empty check passes on a diagnostic. Strip
# them and return only a single clean value.
field() {
  local topic="$1" path="$2" budget="${3:-5}"
  timeout "$budget" ros2 topic echo "$topic" --field "$path" --once 2>/dev/null \
    | tr -d '\r' \
    | grep -v -e '^WARNING' -e '^A message was lost' -e '^---$' -e '^$' \
    | head -1
}

echo "=== agc_slam interface check -- robot '$ROBOT' ==="
echo

# --- 1. topics exist with the right type ------------------------------------
echo "[1] frozen topics"
check_topic() {
  local topic="$1" want="$2"
  local got
  got="$(ros2 topic type "$topic" 2>/dev/null | head -1)"
  if [[ -z "$got" ]]; then
    red "$topic does not exist"
    info "is the module running? ros2 launch agc_slam slam.launch.py robot_name:=$ROBOT"
    return 1
  fi
  if [[ "$got" != "$want" ]]; then
    red "$topic has type $got, the spec says $want"
    return 1
  fi
  green "$topic : $got"
}
check_topic "/$ROBOT/odom" "nav_msgs/msg/Odometry"
check_topic "/$ROBOT/map_cloud" "sensor_msgs/msg/PointCloud2"
echo

# --- 2. they are actually publishing ----------------------------------------
echo "[2] data is flowing"
# A topic that exists because a subscriber created it, with no publisher, is
# the most common way a demo "works" on one machine and not the next.
for pair in "odom:5" "map_cloud:15"; do
  topic="${pair%%:*}"; budget="${pair##*:}"
  if timeout "$budget" ros2 topic echo "/$ROBOT/$topic" --once >/dev/null 2>&1; then
    green "/$ROBOT/$topic published a message within ${budget}s"
  else
    red "/$ROBOT/$topic produced nothing in ${budget}s"
    info "check the backend: ros2 topic hz /Odometry"
  fi
done
echo

# --- 3. frames match the frozen spec ----------------------------------------
echo "[3] frame ids on /$ROBOT/odom"
odom_frame="$(field "/$ROBOT/odom" header.frame_id 5)"
child_frame="$(field "/$ROBOT/odom" child_frame_id 5)"
[[ "$odom_frame"  == "$ROBOT/odom"      ]] && green "header.frame_id = $odom_frame" \
                                           || red "header.frame_id = '${odom_frame:-<none>}', expected '$ROBOT/odom'"
[[ "$child_frame" == "$ROBOT/base_link" ]] && green "child_frame_id = $child_frame" \
                                           || red "child_frame_id = '${child_frame:-<none>}', expected '$ROBOT/base_link'"
echo

# --- 4. map_cloud is a map, and not empty -----------------------------------
echo "[4] map_cloud contents"
cloud_frame="$(field "/$ROBOT/map_cloud" header.frame_id 15)"
width="$(field "/$ROBOT/map_cloud" width 15)"
if [[ "$cloud_frame" == "$ROBOT/odom" ]]; then
  green "frame_id = $cloud_frame"
elif [[ "$cloud_frame" =~ ^[A-Za-z_][A-Za-z0-9_/]*$ ]]; then
  # Not fatal: the coordinator may legitimately move this to `map` later.
  green "frame_id = $cloud_frame"
  info "expected '$ROBOT/odom'; make sure consumers agree on this frame"
else
  red "no usable frame_id on map_cloud (got '${cloud_frame:-<none>}')"
fi
if [[ "${width:-0}" =~ ^[0-9]+$ ]] && (( width > 0 )); then
  green "map_cloud has $width points"
  (( width < 100 )) && info "that is very few -- has the robot moved yet?"
else
  red "map_cloud is empty (width=${width:-<none>})"
  info "the robot must move before there is a map; drive it and re-run"
fi
echo

# --- 5. the frame tree resolves ---------------------------------------------
echo "[5] frame tree  map -> $ROBOT/odom -> $ROBOT/base_link"
# Delegated to a real tf2 lookup (ros2 run agc_slam check_frames) rather than
# scraping tf2_echo, which is wall-clock driven and spins on "jump back in
# time" when a bag recorded months ago is replayed.
tf_out="$(ros2 run agc_slam check_frames "$ROBOT" --timeout 10 2>/dev/null || true)"
if [[ -z "$tf_out" ]]; then
  red "frame check produced no output -- is agc_slam built and sourced?"
else
  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    if [[ "$line" == *"PASS"* ]]; then
      green "${line#*PASS  }"
    elif [[ "$line" == *"FAIL"* ]]; then
      red "${line#*FAIL  }"
    else
      info "$line"
    fi
  done <<< "$tf_out"
fi
echo

# --- summary ----------------------------------------------------------------
echo "=== $PASS passed, $FAIL failed ==="
if (( FAIL == 0 )); then
  echo "Interface matches the frozen spec. Ready to merge."
  exit 0
fi
echo "Interface does NOT match the frozen spec -- do not merge before the demo."
exit 1
