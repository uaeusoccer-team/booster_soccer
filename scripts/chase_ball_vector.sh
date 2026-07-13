#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

VX_LIMIT="0.60"
VY_LIMIT="0.20"
STOP_DIST="1.00"
Y_TOLERANCE="0.03"

BODY_TURN_SPEED="0.25"
HEAD_TURN_START_RATIO="0.75"
HEAD_TURN_STOP_RATIO="0.65"
STOP_ANGLE="0.10"
USE_BALL_YAW_FALLBACK="true"
REQUIRE_PLAY="false"

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
  ./scripts/chase_ball_vector.sh [setting=value ...]

Chase settings:
  vx_limit=0.60
  vy_limit=0.20
  stop_dist=1.00
  y_tolerance=0.03

Tracking and rotation settings:
  body_turn_speed=0.25
  head_turn_start_ratio=0.75
  head_turn_stop_ratio=0.65
  stop_angle=0.10
  use_ball_yaw_fallback=true
  require_play=false

Ball-search settings:
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
  # Start chasing immediately with ballYaw x 4 rotation fallback:
  ./scripts/chase_ball_vector.sh

  # Change chase limits and stop distance:
  ./scripts/chase_ball_vector.sh vx_limit=0.40 vy_limit=0.15 stop_dist=0.80

  # Tune rotation and require referee GameController PLAY:
  ./scripts/chase_ball_vector.sh stop_angle=0.20 body_turn_speed=0.50 require_play=true

SimpleChase calculates only vx and vy. CamTrackBall/CamFindBall calculate theta.
SetVelocity publishes the combined vx, vy, and theta once per active tick.
Press s or Ctrl-C to stop the test stack.
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
  local value="$2"
  case "$value" in
    [Tt][Rr][Uu][Ee]|1) printf 'true' ;;
    [Ff][Aa][Ll][Ss][Ee]|0) printf 'false' ;;
    *) echo "Invalid boolean for ${key}: ${value}" >&2; exit 2 ;;
  esac
}

set_value() {
  local key="$1"
  local value
  value="$(clean_value "$2")"

  case "$key" in
    use_ball_yaw_fallback|require_play|turn_body_on_loss)
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
    vx_limit) VX_LIMIT="$value" ;;
    vy_limit) VY_LIMIT="$value" ;;
    stop_dist) STOP_DIST="$value" ;;
    y_tolerance) Y_TOLERANCE="$value" ;;
    body_turn_speed) BODY_TURN_SPEED="$value" ;;
    head_turn_start_ratio) HEAD_TURN_START_RATIO="$value" ;;
    head_turn_stop_ratio) HEAD_TURN_STOP_RATIO="$value" ;;
    stop_angle) STOP_ANGLE="$value" ;;
    use_ball_yaw_fallback) USE_BALL_YAW_FALLBACK="$value" ;;
    require_play) REQUIRE_PLAY="$value" ;;
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
      vx_limit=*|vy_limit=*|stop_dist=*|y_tolerance=*|body_turn_speed=*|head_turn_start_ratio=*|head_turn_stop_ratio=*|stop_angle=*|use_ball_yaw_fallback=*|require_play=*|turn_body_on_loss=*|lost_turn_msec=*|lost_turn_speed=*|lost_turn_min_yaw=*|low_pitch=*|high_pitch=*|yaw_limit=*|sweep_msec=*|pitch_cycle_msec=*|cmd_interval_msec=*)
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

echo "Ball vector chase settings:"
echo "  vx_limit=${VX_LIMIT}"
echo "  vy_limit=${VY_LIMIT}"
echo "  stop_dist=${STOP_DIST}"
echo "  y_tolerance=${Y_TOLERANCE}"
echo "  body_turn_speed=${BODY_TURN_SPEED}"
echo "  head_turn_start_ratio=${HEAD_TURN_START_RATIO}"
echo "  head_turn_stop_ratio=${HEAD_TURN_STOP_RATIO}"
echo "  stop_angle=${STOP_ANGLE}"
echo "  use_ball_yaw_fallback=${USE_BALL_YAW_FALLBACK}"
echo "  require_play=${REQUIRE_PLAY}"
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

if [[ "$REQUIRE_PLAY" == "true" ]]; then
  STOP_CONDITION="gc_game_state!='PLAY'"
  RUN_CONDITION="gc_game_state=='PLAY'"
  echo "Chase waits for GameController PLAY."
else
  STOP_CONDITION="gc_game_state=='END'"
  RUN_CONDITION="gc_game_state!='END'"
  echo "Chase starts immediately; GameController END or app stop disables it."
fi

# Avoid multiple brain nodes publishing conflicting movement commands.
./scripts/stop.sh || true

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/chase_ball_vector.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="${STOP_CONDITION}" name="chase disabled">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <ReactiveSequence _while="${RUN_CONDITION}" name="track rotate and chase ball">
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
        <SimpleChase vx_limit="${VX_LIMIT}"
                     vy_limit="${VY_LIMIT}"
                     stop_dist="${STOP_DIST}"
                     y_tolerance="${Y_TOLERANCE}"
                     vx="{chase_vx}"
                     vy="{chase_vy}" />
        <SetVelocity x="{chase_vx}"
                     y="{chase_vy}"
                     theta="{tracking_theta}" />
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "Wrote ${TREE_PATH}"
echo "Starting vision, vector chase, and GameController receiver..."
ros2 launch vision launch.py > vision.log 2>&1 &
ros2 launch brain launch.py \
  tree:=chase_ball_vector.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=false \
  disable_com:=true \
  > brain.log 2>&1 &
ros2 launch game_controller launch.py > game_controller.log 2>&1 &

if [[ "$REQUIRE_PLAY" == "true" ]]; then
  echo "Ready. GameController PLAY runs the chase; non-PLAY states command zero."
else
  echo "Chasing immediately."
fi
echo "Press s or Ctrl-C to stop."
echo "Diagnostics: tail -f brain.log | grep -E 'CamTrackBall/direct_pixel|CamFindBall/search|SimpleChase/vector'"

while true; do
  if ! read -rsn1 key; then
    controlled_stop "input closed"
  fi
  if [[ "$key" == "s" || "$key" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
