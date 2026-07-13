#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

BODY_TURN_SPEED="0.20"
HEAD_TURN_START_RATIO="0.75"
HEAD_TURN_STOP_RATIO="0.65"
STOP_ANGLE="0.10"
USE_BALL_YAW_FALLBACK="false"
TURN_BODY_ON_LOSS="false"
LOST_TURN_MSEC="800"
LOST_TURN_SPEED="0.20"
LOST_TURN_MIN_YAW="0.08"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/test_head_tracking_rotation.sh [setting=value ...]

Settings:
  body_turn_speed=0.20
  head_turn_start_ratio=0.75
  head_turn_stop_ratio=0.65
  stop_angle=0.10
  use_ball_yaw_fallback=false
  turn_body_on_loss=false
  lost_turn_msec=800
  lost_turn_speed=0.20
  lost_turn_min_yaw=0.08

Examples:
  # Isolate head-edge rotation (recommended first test):
  ./scripts/test_head_tracking_rotation.sh

  # Also rotate from robot-relative ball yaw before the head reaches its edge:
  ./scripts/test_head_tracking_rotation.sh use_ball_yaw_fallback=true

  # Also test brief recent-memory rotation after vision loss:
  ./scripts/test_head_tracking_rotation.sh turn_body_on_loss=true

The script always commands vx=0 and vy=0. The robot can still rotate, so it must
be balanced on the floor in open space. Press s or Ctrl-C to stop safely.
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
      body_turn_speed=*|head_turn_start_ratio=*|head_turn_stop_ratio=*|stop_angle=*|use_ball_yaw_fallback=*|turn_body_on_loss=*|lost_turn_msec=*|lost_turn_speed=*|lost_turn_min_yaw=*)
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
  for _ in 1 2 3 4 5 6 7 8; do
    ros2 topic pub --once /booster_agent/soccer_game_control \
      std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
    sleep 0.4
  done
}

controlled_stop() {
  local reason="${1:-manual}"
  trap - HUP INT TERM

  echo
  echo "---- controlled stop requested: ${reason} ----"
  echo "Sending stop while brain is still alive..."
  send_game_stop
  echo "Holding a zero command for 3 seconds before stopping nodes..."
  sleep 3
  ./scripts/stop.sh || true
  echo "Stopped safely."
  exit 0
}

wait_for_ball() {
  echo "Waiting for a ball detection. Press s or Ctrl-C to stop."

  while true; do
    local detection_file
    detection_file="$(mktemp /tmp/head_rotation_detection.XXXXXX)"
    timeout 5 ros2 topic echo /booster_soccer/detection --once > "$detection_file" || true

    if grep -q 'label: Ball' "$detection_file"; then
      echo "Ball detected."
      grep -E 'label:|confidence:|xmin:|ymin:|xmax:|ymax:' "$detection_file" || true
      rm -f "$detection_file"
      return 0
    fi

    rm -f "$detection_file"
    echo "No ball yet. Still waiting..."
    read -rsn1 -t 1 key || true
    if [[ "${key:-}" == "s" || "${key:-}" == "S" ]]; then
      controlled_stop "operator s while waiting"
    fi
  done
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
echo
echo "SAFETY: Put the robot on the floor in open space and be ready to press s."
read -r -p "Type ROTATE to arm this test: " confirmation
if [[ "$confirmation" != "ROTATE" ]]; then
  echo "Not armed; exiting without starting robot nodes."
  exit 1
fi

./scripts/stop.sh || true
sleep 3

ros2 daemon stop || true
sleep 2
ros2 daemon start
sleep 2

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/head_tracking_rotation_test.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="gc_game_state=='END'" name="manual controlled stop">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <ReactiveSequence _while="gc_game_state!='END'" name="head tracking with body rotation">
        <CheckAndStandUp />
        <IfThenElse>
          <ScriptCondition name="Ball visible?" code="ball_visible" />
          <CamTrackBall body_turn_speed="${BODY_TURN_SPEED}"
                        head_turn_start_ratio="${HEAD_TURN_START_RATIO}"
                        head_turn_stop_ratio="${HEAD_TURN_STOP_RATIO}"
                        stop_angle="${STOP_ANGLE}"
                        use_ball_yaw_fallback="${USE_BALL_YAW_FALLBACK}"
                        theta="{tracking_theta}" />
          <CamFindBall turn_body_on_loss="${TURN_BODY_ON_LOSS}"
                       lost_turn_msec="${LOST_TURN_MSEC}"
                       lost_turn_speed="${LOST_TURN_SPEED}"
                       lost_turn_min_yaw="${LOST_TURN_MIN_YAW}"
                       theta="{tracking_theta}" />
        </IfThenElse>
        <SetVelocity x="0" y="0" theta="{tracking_theta}" />
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "Wrote ${TREE_PATH}"
echo "Starting vision..."
ros2 launch vision launch.py > vision.log 2>&1 &
sleep 8

wait_for_ball

echo "Starting rotation-only head tracking. Press s or Ctrl-C to stop."
echo "Diagnostics: tail -f brain.log | grep -E 'CamTrackBall/direct_pixel|CamFindBall/search'"
ros2 launch brain launch.py \
  tree:=head_tracking_rotation_test.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &

while true; do
  if ! read -rsn1 key; then
    controlled_stop "input closed"
  fi
  if [[ "$key" == "s" || "$key" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
