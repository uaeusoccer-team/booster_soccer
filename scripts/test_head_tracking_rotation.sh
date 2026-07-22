#!/usr/bin/env bash
set -Eeuo pipefail

if (( $# != 0 )); then
  echo "This fixed-value safety test does not accept arguments." >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE}"

deactivate 2>/dev/null || true
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${FASTRTPS_DEFAULT_PROFILES_FILE}}"

CLEANING_UP=0

send_game_stop() {
  for _ in 1 2 3; do
    timeout 2 ros2 topic pub --once /booster_agent/soccer_game_control \
      std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
  done
}

game_controller_is_end() {
  local game_state
  game_state="$(
    timeout 1 ros2 topic echo /booster_soccer/game_controller \
      --once --field state 2>/dev/null || true
  )"
  printf '%s\n' "${game_state}" | grep -qE '^[[:space:]]*4[[:space:]]*$'
}

cleanup() {
  if (( CLEANING_UP )); then
    return
  fi
  CLEANING_UP=1
  set +e
  echo
  echo "Stopping rotation test..."
  send_game_stop
  ./scripts/stop.sh
  echo "Rotation test stopped."
}

trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

./scripts/stop.sh || true

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/head_tracking_rotation_test.xml"

tee "${TREE_PATH}" >/dev/null <<'XML'
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="fixed head tracking rotation test">
      <IfThenElse>
        <ScriptCondition name="GameController END?" code="gc_game_state=='END'" />
        <Script code="tracking_theta=0.0" />
        <CamTrackBall theta="{tracking_theta}" />
      </IfThenElse>
      <SetVelocity x="0.0" y="0.0" theta="{tracking_theta}" />
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "Starting immediately with fixed values: deadband ±0.40 rad, turn ±0.40 rad/s, head step 0.04 rad."
echo "The test never invokes Ball search. Press s or Ctrl-C to stop."

ros2 launch vision launch.py > vision.log 2>&1 &
VISION_PID=$!
ros2 launch brain launch.py \
  tree:=head_tracking_rotation_test.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=false \
  disable_com:=true \
  > brain.log 2>&1 &
BRAIN_PID=$!
ros2 launch game_controller launch.py > game_controller.log 2>&1 &
GAME_CONTROLLER_PID=$!

echo "Logs: vision.log, brain.log, game_controller.log"
while kill -0 "${BRAIN_PID}" 2>/dev/null; do
  key=""
  if IFS= read -rsn1 -t 1 key; then
    if [[ "${key}" == "s" || "${key}" == "S" ]]; then
      exit 0
    fi
  else
    read_status=$?
    if (( read_status == 1 )); then
      echo "Terminal input closed; stopping the test." >&2
      exit 1
    fi
  fi

  if game_controller_is_end; then
    echo "GameController END received; stopping the test."
    exit 0
  fi

  if ! kill -0 "${VISION_PID}" 2>/dev/null; then
    echo "Vision exited; stopping the test." >&2
    exit 1
  fi
  if ! kill -0 "${GAME_CONTROLLER_PID}" 2>/dev/null; then
    echo "GameController receiver exited; stopping the test." >&2
    exit 1
  fi
done

echo "Brain exited; stopping the test." >&2
exit 1
