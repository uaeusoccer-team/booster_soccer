#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

STOP_ANGLE="0.10"
BALL_YAW_GAIN="4.0"
PITCH_TURN_GAIN="1.0"
REQUIRE_PLAY="false"
HEAD_SEARCH_SPEED="0.20"
BODY_SEARCH_SPEED="0.25"
YAW_LIMIT="1.10"
CMD_INTERVAL_MSEC="100"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/test_head_tracking_rotation.sh [setting=value ...]

Settings:
  stop_angle=0.10
  ball_yaw_gain=4.0
  pitch_turn_gain=1.0
  require_play=false
  head_search_speed=0.20
  body_search_speed=0.25
  yaw_limit=1.10
  cmd_interval_msec=100

Examples:
  # Run immediately with base ballYaw x 4 body rotation:
  ./scripts/test_head_tracking_rotation.sh

  # Tune visible-ball rotation and the extra speed while looking down:
  ./scripts/test_head_tracking_rotation.sh stop_angle=0.20 ball_yaw_gain=5.0 pitch_turn_gain=1.5

  # Require referee GameController PLAY instead of starting immediately:
  ./scripts/test_head_tracking_rotation.sh require_play=true

  # Tune the slow head look and following body rotation after ball loss:
  ./scripts/test_head_tracking_rotation.sh head_search_speed=0.15 body_search_speed=0.35

The script always commands vx=0 and vy=0. With require_play=false (the default),
tracking and rotation begin as soon as the brain starts. Set require_play=true
if GameController PLAY should gate the test.
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
    require_play)
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
    stop_angle) STOP_ANGLE="$value" ;;
    ball_yaw_gain) BALL_YAW_GAIN="$value" ;;
    pitch_turn_gain) PITCH_TURN_GAIN="$value" ;;
    require_play) REQUIRE_PLAY="$value" ;;
    head_search_speed) HEAD_SEARCH_SPEED="$value" ;;
    body_search_speed) BODY_SEARCH_SPEED="$value" ;;
    yaw_limit) YAW_LIMIT="$value" ;;
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
      stop_angle=*|ball_yaw_gain=*|pitch_turn_gain=*|require_play=*|head_search_speed=*|body_search_speed=*|yaw_limit=*|cmd_interval_msec=*)
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
echo "  stop_angle=${STOP_ANGLE}"
echo "  ball_yaw_gain=${BALL_YAW_GAIN}"
echo "  pitch_turn_gain=${PITCH_TURN_GAIN}"
echo "  require_play=${REQUIRE_PLAY}"
echo "  head_search_speed=${HEAD_SEARCH_SPEED}"
echo "  body_search_speed=${BODY_SEARCH_SPEED}"
echo "  yaw_limit=${YAW_LIMIT}"
echo "  cmd_interval_msec=${CMD_INTERVAL_MSEC}"
if [[ "$REQUIRE_PLAY" == "true" ]]; then
  STOP_CONDITION="gc_game_state!='PLAY'"
  RUN_CONDITION="gc_game_state=='PLAY'"
  echo "Tracking waits for GameController PLAY."
else
  STOP_CONDITION="gc_game_state=='END'"
  RUN_CONDITION="gc_game_state!='END'"
  echo "Tracking starts immediately; GameController END or app stop disables it."
fi

# Avoid multiple brain nodes publishing conflicting movement commands.
./scripts/stop.sh || true

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/head_tracking_rotation_test.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="${STOP_CONDITION}" name="rotation disabled">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <ReactiveSequence _while="${RUN_CONDITION}" name="head tracking with body rotation">
          <CheckAndStandUp />
          <IfThenElse>
            <ScriptCondition name="Ball visible?" code="ball_visible" />
            <CamTrackBall stop_angle="${STOP_ANGLE}"
                          ball_yaw_gain="${BALL_YAW_GAIN}"
                          pitch_turn_gain="${PITCH_TURN_GAIN}"
                          theta="{tracking_theta}" />
            <CamFindBall yaw_limit="${YAW_LIMIT}"
                         head_search_speed="${HEAD_SEARCH_SPEED}"
                         body_search_speed="${BODY_SEARCH_SPEED}"
                         cmd_interval_msec="${CMD_INTERVAL_MSEC}"
                         theta="{tracking_theta}" />
          </IfThenElse>
          <SetVelocity x="0" y="0" theta="{tracking_theta}" />
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "Wrote ${TREE_PATH}"
echo "Starting vision, immediate rotation test, and GameController receiver..."
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

if [[ "$REQUIRE_PLAY" == "true" ]]; then
  echo "Ready. GameController PLAY runs the test; non-PLAY states command zero."
else
  echo "Running immediately."
fi
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
