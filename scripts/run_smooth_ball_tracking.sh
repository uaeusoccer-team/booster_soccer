#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_smooth_ball_tracking.sh [options]

Starts vision and a ball tracking behavior tree that uses CamFindBall's smooth
search mode. When the ball is visible, the head tracks it and body velocity is
held at zero. When the ball is lost, CamFindBall can optionally rotate the body
toward the last seen ball yaw using turn_body_on_loss.

Options:
      --turn-body-on-loss true|false   Enable body yaw after losing the ball, default true
      --no-turn-body-on-loss           Same as --turn-body-on-loss false
      --lost-turn-msec MSEC            How long to turn after ball loss, default 1200
      --lost-turn-speed RAD_PER_SEC    Body yaw speed while turning, default 0.18
      --lost-turn-min-yaw RAD          Minimum remembered yaw before turning, default 0.08
      --pitches LIST                   Smooth search pitch rows, default 0.75,0.50,0.35
      --min-yaw RAD                    Smooth search minimum yaw, default -1.0
      --max-yaw RAD                    Smooth search maximum yaw, default 1.0
      --yaw-speed RAD_PER_SEC          Smooth search yaw speed, default 0.75
      --pitch-speed RAD_PER_SEC        Smooth search pitch speed, default 0.35
      --command-hz HZ                  Head command rate, default 25
      --dwell-msec MSEC                Pause at each search target, default 80
  -h, --help                           Show this help

Press s to stop after launch, or Ctrl-C at any time.
EOF
}

normalize_bool() {
  case "${1,,}" in
    1|true|yes|y|on) echo "true" ;;
    0|false|no|n|off) echo "false" ;;
    *)
      echo "Invalid boolean value: $1" >&2
      exit 2
      ;;
  esac
}

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"
TURN_BODY_ON_LOSS="${TURN_BODY_ON_LOSS:-true}"
LOST_TURN_MSEC="${LOST_TURN_MSEC:-1200}"
LOST_TURN_SPEED="${LOST_TURN_SPEED:-0.18}"
LOST_TURN_MIN_YAW="${LOST_TURN_MIN_YAW:-0.08}"
SMOOTH_PITCHES="${SMOOTH_PITCHES:-0.75,0.50,0.35}"
MIN_YAW="${HEAD_MIN_YAW:--1.0}"
MAX_YAW="${HEAD_MAX_YAW:-1.0}"
YAW_SPEED="${HEAD_YAW_SPEED:-0.75}"
PITCH_SPEED="${HEAD_PITCH_SPEED:-0.35}"
COMMAND_HZ="${HEAD_COMMAND_HZ:-25}"
DWELL_MSEC="${HEAD_DWELL_MSEC:-80}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --turn-body-on-loss)
      TURN_BODY_ON_LOSS="${2:?missing value for $1}"
      shift 2
      ;;
    --no-turn-body-on-loss)
      TURN_BODY_ON_LOSS=false
      shift
      ;;
    --lost-turn-msec)
      LOST_TURN_MSEC="${2:?missing value for $1}"
      shift 2
      ;;
    --lost-turn-speed)
      LOST_TURN_SPEED="${2:?missing value for $1}"
      shift 2
      ;;
    --lost-turn-min-yaw)
      LOST_TURN_MIN_YAW="${2:?missing value for $1}"
      shift 2
      ;;
    --pitches)
      SMOOTH_PITCHES="${2:?missing value for $1}"
      shift 2
      ;;
    --min-yaw)
      MIN_YAW="${2:?missing value for $1}"
      shift 2
      ;;
    --max-yaw)
      MAX_YAW="${2:?missing value for $1}"
      shift 2
      ;;
    --yaw-speed)
      YAW_SPEED="${2:?missing value for $1}"
      shift 2
      ;;
    --pitch-speed)
      PITCH_SPEED="${2:?missing value for $1}"
      shift 2
      ;;
    --command-hz)
      COMMAND_HZ="${2:?missing value for $1}"
      shift 2
      ;;
    --dwell-msec)
      DWELL_MSEC="${2:?missing value for $1}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

TURN_BODY_ON_LOSS="$(normalize_bool "$TURN_BODY_ON_LOSS")"

cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"

safe_stop() {
  timeout 2 ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
  ./scripts/stop.sh || true
}

controlled_stop() {
  local reason="${1:-manual}"
  trap - INT TERM
  echo
  echo "---- controlled stop requested: ${reason} ----"
  safe_stop
  echo "Stopped."
  exit 0
}

trap 'controlled_stop "Ctrl-C"' INT
trap 'controlled_stop "TERM"' TERM

./scripts/stop.sh || true
sleep 3

ros2 daemon stop || true
sleep 2
ros2 daemon start
sleep 2

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/smooth_ball_tracking.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence name="smooth_ball_tracking">
      <IfThenElse>
        <ScriptCondition name="Ball location known?" code="ball_location_known || tm_ball_pos_reliable" />

        <Sequence name="[Yes] track ball and hold body still">
          <CamTrackBall />
          <SetVelocity x="0" y="0" theta="0" />
        </Sequence>

        <Sequence name="[No] smooth find ball">
          <CamFindBall search_mode="smooth"
                       smooth_pitches="${SMOOTH_PITCHES}"
                       min_yaw="${MIN_YAW}"
                       max_yaw="${MAX_YAW}"
                       yaw_speed="${YAW_SPEED}"
                       pitch_speed="${PITCH_SPEED}"
                       command_hz="${COMMAND_HZ}"
                       dwell_msec="${DWELL_MSEC}"
                       turn_body_on_loss="${TURN_BODY_ON_LOSS}"
                       lost_turn_msec="${LOST_TURN_MSEC}"
                       lost_turn_speed="${LOST_TURN_SPEED}"
                       lost_turn_min_yaw="${LOST_TURN_MIN_YAW}" />
        </Sequence>
      </IfThenElse>
    </ReactiveSequence>
  </BehaviorTree>
</root>
XML

echo "---- smooth ball tracking config ----"
echo "tree: ${TREE_PATH}"
echo "turn_body_on_loss: ${TURN_BODY_ON_LOSS}"
echo "lost_turn_msec: ${LOST_TURN_MSEC}"
echo "lost_turn_speed: ${LOST_TURN_SPEED}"
echo "smooth_pitches: ${SMOOTH_PITCHES}"
echo "yaw range: ${MIN_YAW} to ${MAX_YAW}"
echo

echo "---- start vision ----"
ros2 launch vision launch.py > vision.log 2>&1 &
sleep 8

echo "---- check detection once ----"
timeout 5 ros2 topic echo /booster_soccer/detection --once \
  | grep -E 'label:|confidence:|xmin:|ymin:|xmax:|ymax:|position_projection|detected_objects' || true

echo "---- start smooth ball tracking ----"
if [[ "$TURN_BODY_ON_LOSS" == "true" ]]; then
  echo "Body yaw can move briefly after ball loss. Use only on the floor with space and an operator ready to stop."
else
  echo "Body yaw on ball loss is disabled; head search and head tracking only."
fi

ros2 launch brain launch.py \
  tree:=smooth_ball_tracking.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &

echo "Press s to stop. Press Ctrl-C to exit."
while true; do
  read -rsn1 -t 1 key || true
  if [[ "${key:-}" == "s" || "${key:-}" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
