#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

STOP_ANGLE="0.10"
BALL_YAW_GAIN="4.0"
PITCH_TURN_GAIN="1.0"
REQUIRE_PLAY="false"
HEAD_SEARCH_SPEED="0.20"
BODY_SEARCH_SPEED="0.45"
TRACK_TURN_YAW_LIMIT="0.75"
LOSS_TURN_PITCH_LIMIT="0.70"
SEARCH_YAW_LIMIT="1.15"
CMD_INTERVAL_MSEC="100"
HEAD_STEP_RAD="0.04"
HEAD_SETTLE_STEP_RAD="0.02"
HEAD_DEADBAND_X_PX="35"
HEAD_DEADBAND_Y_PX="35"
LEGACY_YAW_LIMIT_USED="false"
SEARCH_YAW_LIMIT_EXPLICIT="false"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/test_head_tracking_rotation.sh [setting=value ...]

Settings:
  stop_angle=0.10
  ball_yaw_gain=4.0
  pitch_turn_gain=1.0
  require_play=false
  head_search_speed=0.20          # head-only scan speed
  body_search_speed=0.45          # exact fast body turn after qualifying RGB loss
  track_turn_yaw_limit=0.75       # last head-yaw threshold for a fast loss turn
  loss_turn_pitch_limit=0.70      # downward head-pitch threshold for a fast loss turn
  search_yaw_limit=1.15           # full head scan endpoint, radians
  yaw_limit=1.15                  # compatibility alias for search_yaw_limit
  cmd_interval_msec=100
  head_step_rad=0.04              # normal head yaw/pitch step per vision frame
  head_settle_step_rad=0.02       # step within 20 px of the pixel deadband
  head_deadband_x_px=35            # horizontal head deadband
  head_deadband_y_px=35            # vertical head deadband

Examples:
  # Run immediately with base ballYaw x 4 body rotation:
  ./scripts/test_head_tracking_rotation.sh

  # Tune visible-ball rotation and the extra speed while looking down:
  ./scripts/test_head_tracking_rotation.sh stop_angle=0.20 ball_yaw_gain=5.0 pitch_turn_gain=1.5

  # Require referee GameController PLAY instead of starting immediately:
  ./scripts/test_head_tracking_rotation.sh require_play=true

  # Tune the head scan and continued body turn after ball loss:
  ./scripts/test_head_tracking_rotation.sh head_search_speed=0.15 body_search_speed=0.35

The script always commands vx=0 and vy=0. With require_play=false (the default),
tracking and rotation begin as soon as the brain starts. Set require_play=true
if GameController PLAY should gate the test.

After RGB ball loss, the last RGB/head direction selects the search side. If
the last head yaw passed track_turn_yaw_limit or the last downward head pitch
passed loss_turn_pitch_limit, the body turns in that direction at the exact
body_search_speed. Otherwise the body stays still and the head scans between
search_yaw_limit edges. Before the first observed ball, search holds still.
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
    track_turn_yaw_limit) TRACK_TURN_YAW_LIMIT="$value" ;;
    loss_turn_pitch_limit) LOSS_TURN_PITCH_LIMIT="$value" ;;
    search_yaw_limit)
      SEARCH_YAW_LIMIT="$value"
      SEARCH_YAW_LIMIT_EXPLICIT="true"
      ;;
    yaw_limit)
      SEARCH_YAW_LIMIT="$value"
      LEGACY_YAW_LIMIT_USED="true"
      ;;
    cmd_interval_msec) CMD_INTERVAL_MSEC="$value" ;;
    head_step_rad) HEAD_STEP_RAD="$value" ;;
    head_settle_step_rad) HEAD_SETTLE_STEP_RAD="$value" ;;
    head_deadband_x_px) HEAD_DEADBAND_X_PX="$value" ;;
    head_deadband_y_px) HEAD_DEADBAND_Y_PX="$value" ;;
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
      stop_angle=*|ball_yaw_gain=*|pitch_turn_gain=*|require_play=*|head_search_speed=*|body_search_speed=*|track_turn_yaw_limit=*|loss_turn_pitch_limit=*|search_yaw_limit=*|yaw_limit=*|cmd_interval_msec=*|head_step_rad=*|head_settle_step_rad=*|head_deadband_x_px=*|head_deadband_y_px=*)
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

if [[ "$LEGACY_YAW_LIMIT_USED" == "true" && "$SEARCH_YAW_LIMIT_EXPLICIT" == "true" ]]; then
  echo "Do not set both yaw_limit and search_yaw_limit; yaw_limit is only a compatibility alias." >&2
  exit 2
fi

if [[ "$LEGACY_YAW_LIMIT_USED" == "true" ]]; then
  echo "Warning: yaw_limit is deprecated; use search_yaw_limit." >&2
fi

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
echo "  track_turn_yaw_limit=${TRACK_TURN_YAW_LIMIT}"
echo "  loss_turn_pitch_limit=${LOSS_TURN_PITCH_LIMIT}"
echo "  search_yaw_limit=${SEARCH_YAW_LIMIT}"
echo "  cmd_interval_msec=${CMD_INTERVAL_MSEC}"
echo "  head_step_rad=${HEAD_STEP_RAD}"
echo "  head_settle_step_rad=${HEAD_SETTLE_STEP_RAD}"
echo "  head_deadband_x_px=${HEAD_DEADBAND_X_PX}"
echo "  head_deadband_y_px=${HEAD_DEADBAND_Y_PX}"
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
            <ScriptCondition name="Depth-confirmed RGB acquisition?" code="ball_visible &amp;&amp; ball_depth_acquired" />
            <Sequence>
              <CamTrackBall stop_angle="${STOP_ANGLE}"
                            ball_yaw_gain="${BALL_YAW_GAIN}"
                            pitch_turn_gain="${PITCH_TURN_GAIN}"
                            track_turn_yaw_limit="${TRACK_TURN_YAW_LIMIT}"
                            loss_turn_pitch_limit="${LOSS_TURN_PITCH_LIMIT}"
                            head_step_rad="${HEAD_STEP_RAD}"
                            head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                            head_deadband_x_px="${HEAD_DEADBAND_X_PX}"
                            head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                            theta="{tracking_theta}" />
              <Script code="tracking_apply_min_theta=true" />
            </Sequence>
            <Sequence>
              <CamFindBall yaw_limit="${SEARCH_YAW_LIMIT}"
                           track_turn_yaw_limit="${TRACK_TURN_YAW_LIMIT}"
                           loss_turn_pitch_limit="${LOSS_TURN_PITCH_LIMIT}"
                           head_search_speed="${HEAD_SEARCH_SPEED}"
                           body_search_speed="${BODY_SEARCH_SPEED}"
                           cmd_interval_msec="${CMD_INTERVAL_MSEC}"
                           theta="{tracking_theta}" />
              <Script code="tracking_apply_min_theta=false" />
            </Sequence>
          </IfThenElse>
          <SetVelocity x="0"
                       y="0"
                       theta="{tracking_theta}"
                       apply_min_theta="{tracking_apply_min_theta}" />
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
echo "Diagnostics: tail -f brain.log | grep -E 'CamTrackBall/direct_pixel|CamFindBall/(wait_observation|start|search)'"

while true; do
  if ! read -rsn1 key; then
    controlled_stop "input closed"
  fi
  if [[ "$key" == "s" || "$key" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
