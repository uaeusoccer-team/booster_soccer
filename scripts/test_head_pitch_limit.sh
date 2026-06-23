#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"
PITCH_DEG="${1:--18}"
YAW_RAD="${2:-0.0}"

cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

PITCH_RAD="$(awk "BEGIN { printf \"%.6f\", ${PITCH_DEG} * 3.141592653589793 / 180.0 }")"
BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/head_pitch_limit_test.xml"

./scripts/stop.sh || true
sleep 2

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence name="head_pitch_limit_test">
      <MoveHead pitch="${PITCH_RAD}" yaw="${YAW_RAD}" />
      <SetVelocity x="0" y="0" theta="0" />
    </ReactiveSequence>
  </BehaviorTree>
</root>
XML

echo "Testing head pitch: ${PITCH_DEG} deg (${PITCH_RAD} rad), yaw ${YAW_RAD} rad."
echo "This is head-only; keep the robot in DAMP or safely standing still."

ros2 launch brain launch.py \
  tree:=head_pitch_limit_test.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &

sleep 6

echo "Latest measured head joints from /low_state:"
timeout 3 ros2 topic echo --once /low_state | sed -n '/motor_state_serial:/,/tau_est:/p' | head -40 || true
echo
echo "Stop with: ./scripts/stop.sh"
