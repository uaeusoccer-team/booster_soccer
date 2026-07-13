#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

BODY_TURN_SPEED="1.20"
HEAD_TURN_START_RATIO="0.75"
HEAD_TURN_STOP_RATIO="0.65"
STOP_ANGLE="0.10"
USE_BALL_YAW_FALLBACK="false"
TURN_BODY_ON_LOSS="false"
LOST_TURN_MSEC="800"
LOST_TURN_SPEED="0.20"
LOST_TURN_MIN_YAW="0.08"
LOW_PITCH="1.00"
HIGH_PITCH="0.45"
YAW_LIMIT="1.10"
SWEEP_MSEC="3000"
PITCH_CYCLE_MSEC="6000"
CMD_INTERVAL_MSEC="100"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/test_head_tracking_rotation.sh [setting=value ...]

Settings:
  body_turn_speed=1.20
  head_turn_start_ratio=0.75
  head_turn_stop_ratio=0.65
  stop_angle=0.10
  use_ball_yaw_fallback=false
  turn_body_on_loss=false
  lost_turn_msec=800
  lost_turn_speed=0.20
  lost_turn_min_yaw=0.08
  low_pitch=1.00
  high_pitch=0.45
  yaw_limit=1.10
  sweep_msec=3000
  pitch_cycle_msec=6000
  cmd_interval_msec=100

Examples:
  # Isolate head-edge rotation (recommended first test):
  ./scripts/test_head_tracking_rotation.sh

  # Also rotate from robot-relative ball yaw before the head reaches its edge:
  ./scripts/test_head_tracking_rotation.sh use_ball_yaw_fallback=true

  # Also test brief recent-memory rotation after vision loss:
  ./scripts/test_head_tracking_rotation.sh turn_body_on_loss=true

The script always commands vx=0 and vy=0. Rotation is enabled only while the
GameController state is PLAY. Press s or Ctrl-C to stop the test stack.
USAGE
}

clean_value() {
  local value="$1"
  value="${value//\"/}"
  value="${value//\'/}"
  value="${value%,}"
  value="${value%;}"
  printf '%s' "$value"
}

normalize_bool() {
  local key="$1"
  local value="${2,,}"
  case "$value" in
    true|1) printf 'true' ;;
    false|0) printf 'false' ;;
    *) echo "Invalid boolean for ${key}: ${value}" >&2; exit 2 ;;
  esac
}

set_value() {
  local key="$1"
  local value
  value="$(clean_value "$2")"

  case "$key" in
    use_ball_yaw_fallback|turn_body_on_loss)
      value="$(normalize_bool "$key" "$value")"
      ;;
    *)
      if ! [[ "$value" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]]; then
        echo "Invalid numeric value for ${key}: ${value}" >&2
        exit 2
      fi
      ;;
  esac

  case "$key" in
    body_turn_speed) BODY_TURN_SPEED="$value" ;;
    head_turn_start_ratio) HEAD_TURN_START_RATIO="$value" ;;
    head_turn_stop_ratio) HEAD_TURN_STOP_RATIO="$value" ;;
    stop_angle) STOP_ANGLE="$value" ;;
    use_ball_yaw_fallback) USE_BALL_YAW_FALLBACK="$value" ;;
    turn_body_on_loss) TURN_BODY_ON_LOSS="$value" ;;
    lost_turn_msec) LOST_TURN_MSEC="$value" ;;
    lost_turn_speed) LOST_TURN_SPEED="$value" ;;
    lost_turn_min_yaw) LOST_TURN_MIN_YAW="$value" ;;
    low_pitch) LOW_PITCH="$value" ;;
    high_pitch) HIGH_PITCH="$value" ;;
    yaw_limit) YAW_LIMIT="$value" ;;
    sweep_msec) SWEEP_MSEC="$value" ;;
    pitch_cycle_msec) PITCH_CYCLE_MSEC="$value" ;;
    cmd_interval_msec) CMD_INTERVAL_MSEC="$value" ;;
    *) echo "Unknown setting: ${key}" >&2; exit 2 ;;
  esac
}

parse_args() {
  while (($#)); do
    case "$1" in
      -h|--help)
        usage
        exit 0
        ;;
      body_turn_speed=*|head_turn_start_ratio=*|head_turn_stop_ratio=*|stop_angle=*|use_ball_yaw_fallback=*|turn_body_on_loss=*|lost_turn_msec=*|lost_turn_speed=*|lost_turn_min_yaw=*|low_pitch=*|high_pitch=*|yaw_limit=*|sweep_msec=*|pitch_cycle_msec=*|cmd_interval_msec=*)
        set_value "${1%%=*}" "${1#*=}"
        shift
        ;;
      *)
        echo "Unknown argument: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done
}

send_game_stop() {
  for _ in 1 2 3; do
    ros2 topic pub --once /booster_agent/soccer_game_control \
      std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
    sleep 0.1
  done
}

controlled_stop() {
  local reason="${1:-manual}"
  trap - HUP INT TERM

  echo
  echo "---- controlled stop requested: ${reason} ----"
  echo "Sending stop while brain is still alive..."
  send_game_stop
  sleep 0.5
  ./scripts/stop.sh || true
  echo "Stopped."
  exit 0
}

parse_args "$@"

cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$FASTRTPS_DEFAULT_PROFILES_FILE}"

trap 'controlled_stop "Ctrl-C"' INT
trap 'controlled_stop "termination signal"' TERM
trap 'controlled_stop "terminal disconnected"' HUP

echo "Head tracking + body rotation test settings:"
echo "  vx=0 (fixed)"
echo "  vy=0 (fixed)"
echo "  body_turn_speed=${BODY_TURN_SPEED}"
echo "  head_turn_start_ratio=${HEAD_TURN_START_RATIO}"
echo "  head_turn_stop_ratio=${HEAD_TURN_STOP_RATIO}"
echo "  stop_angle=${STOP_ANGLE}"
echo "  use_ball_yaw_fallback=${USE_BALL_YAW_FALLBACK}"
echo "  turn_body_on_loss=${TURN_BODY_ON_LOSS}"
echo "  lost_turn_msec=${LOST_TURN_MSEC}"
echo "  lost_turn_speed=${LOST_TURN_SPEED}"
echo "  lost_turn_min_yaw=${LOST_TURN_MIN_YAW}"
echo "  low_pitch=${LOW_PITCH}"
echo "  high_pitch=${HIGH_PITCH}"
echo "  yaw_limit=${YAW_LIMIT}"
echo "  sweep_msec=${SWEEP_MSEC}"
echo "  pitch_cycle_msec=${PITCH_CYCLE_MSEC}"
echo "  cmd_interval_msec=${CMD_INTERVAL_MSEC}"
echo "Rotation will follow GameController PLAY/stop state."

# Avoid multiple brain nodes publishing conflicting movement commands.
./scripts/stop.sh || true

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/head_tracking_rotation_test.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence name="GameController-controlled head tracking rotation">
      <IfThenElse>
        <ScriptCondition name="GameController PLAY?" code="gc_game_state == 'PLAY'" />
        <Sequence name="Track and rotate during PLAY">
          <CheckAndStandUp />
          <IfThenElse>
            <ScriptCondition name="Ball visible?" code="ball_visible" />
            <CamTrackBall body_turn_speed="${BODY_TURN_SPEED}"
                          head_turn_start_ratio="${HEAD_TURN_START_RATIO}"
                          head_turn_stop_ratio="${HEAD_TURN_STOP_RATIO}"
                          stop_angle="${STOP_ANGLE}"
                          use_ball_yaw_fallback="${USE_BALL_YAW_FALLBACK}"
                          theta="{tracking_theta}" />
            <CamFindBall low_pitch="${LOW_PITCH}"
                         high_pitch="${HIGH_PITCH}"
                         yaw_limit="${YAW_LIMIT}"
                         sweep_msec="${SWEEP_MSEC}"
                         pitch_cycle_msec="${PITCH_CYCLE_MSEC}"
                         cmd_interval_msec="${CMD_INTERVAL_MSEC}"
                         turn_body_on_loss="${TURN_BODY_ON_LOSS}"
                         lost_turn_msec="${LOST_TURN_MSEC}"
                         lost_turn_speed="${LOST_TURN_SPEED}"
                         lost_turn_min_yaw="${LOST_TURN_MIN_YAW}"
                         theta="{tracking_theta}" />
          </IfThenElse>
          <SetVelocity x="0" y="0" theta="{tracking_theta}" />
        </Sequence>
        <SetVelocity x="0" y="0" theta="0" />
      </IfThenElse>
    </ReactiveSequence>
  </BehaviorTree>
</root>
XML

echo "Wrote ${TREE_PATH}"
echo "Starting vision, brain, and GameController receiver..."
ros2 launch vision launch.py > vision.log 2>&1 &
ros2 launch brain launch.py \
  tree:=head_tracking_rotation_test.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=false \
  disable_com:=true \
  > brain.log 2>&1 &
ros2 launch game_controller launch.py > game_controller.log 2>&1 &

echo "Ready. Use GameController PLAY to enable rotation; other states command zero."
echo "Press s or Ctrl-C to stop."
echo "Diagnostics: tail -f brain.log | grep -E 'CamTrackBall/direct_pixel|CamFindBall/search'"

while true; do
  if ! read -rsn1 key; then
    controlled_stop "input closed"
  fi
  if [[ "$key" == "s" || "$key" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
