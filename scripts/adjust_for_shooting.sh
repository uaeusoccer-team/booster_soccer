#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

TARGET_RANGE="0.40"
TARGET_Y_OFFSET="0.00"
THETA_OFFSET="0.00"
RANGE_TOLERANCE="0.06"
Y_TOLERANCE="0.05"
STOP_ANGLE="0.10"
RANGE_GAIN="1.0"
Y_GAIN="1.0"
BALL_YAW_GAIN="4.0"
VX_LIMIT="0.40"
VY_LIMIT="0.40"
VTHETA_LIMIT="0.80"
TURN_FIRST_THRESHOLD="0.50"
FIXED_HEAD_YAW="0.00"
MAX_BALL_RANGE="1.20"
REQUIRE_PLAY="false"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/adjust_for_shooting.sh [setting=value ...]

This is a standalone shooting-pose adjustment test. It starts its own brain
tree, so do not run it beside chase_ball_vector.sh. Later it can become a
higher-priority branch inside that same chase tree without restarting the brain.

Desired shooting pose:
  target_range=0.40               # desired forward ball position, metres
  target_y_offset=0.00            # desired robot-relative ball Y, metres; signed
  theta_offset=0.00               # desired ball yaw, radians; signed
  range_tolerance=0.06            # metres
  y_tolerance=0.05                # metres
  stop_angle=0.10                 # yaw deadband, radians

Robot motion:
  range_gain=1.0
  y_gain=1.0
  ball_yaw_gain=4.0
  vx_limit=0.40                   # m/s
  vy_limit=0.40                   # m/s
  vtheta_limit=0.80               # rad/s
  turn_first_threshold=0.50       # radians; zero disables turn-first

Fixed head:
  fixed_head_yaw=0.00             # commanded once per ball acquisition, radians

Test control:
  max_ball_range=1.20             # metres; zero outside this range
  require_play=false

Examples:
  # Start immediately and hold the head at zero yaw while the ball is visible:
  ./scripts/adjust_for_shooting.sh

  # Leave the ball on positive robot Y and use a slight positive yaw offset:
  ./scripts/adjust_for_shooting.sh target_y_offset=0.06 theta_offset=0.05

Positive target_y_offset is positive robot Y (normally robot-left). Positive
theta_offset is counter-clockwise/left ball yaw. The node uses errors:
ballX-target_range, ballY-target_y_offset, and ballYaw-theta_offset.

Body theta is the offset-adjusted ball yaw, capped by vtheta_limit and zero
inside stop_angle. On each ball acquisition, the node commands fixed_head_yaw
once while preserving the measured pitch. It sends no more head commands while
the ball remains visible. If the ball is lost, body motion stops and CamFindBall
performs head-only recovery; reacquisition commands fixed_head_yaw once again.

RobotClient raises smaller nonzero requests to vx/vy=0.30 m/s and theta=0.25
rad/s. Tolerances prevent those minimum speeds from causing oscillation.

The script commands zero body motion if the ball is unavailable or too far. It
uses head-only search after a previously seen ball is lost; it does not chase
or kick. Run only with the robot on the floor in open space; press s or Ctrl-C
to stop.
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

is_signed_number() {
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]]
}

is_nonnegative_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]]
}

set_value() {
  local key="$1"
  local value
  value="$(clean_value "$2")"

  case "$key" in
    require_play)
      value="$(normalize_bool "$key" "$value")"
      ;;
    target_y_offset|theta_offset|fixed_head_yaw)
      if ! is_signed_number "$value"; then
        echo "Invalid signed numeric value for ${key}: ${value}" >&2
        exit 2
      fi
      ;;
    *)
      if ! is_nonnegative_number "$value"; then
        echo "Invalid nonnegative numeric value for ${key}: ${value}" >&2
        exit 2
      fi
      ;;
  esac

  case "$key" in
    target_range) TARGET_RANGE="$value" ;;
    target_y_offset) TARGET_Y_OFFSET="$value" ;;
    theta_offset) THETA_OFFSET="$value" ;;
    range_tolerance) RANGE_TOLERANCE="$value" ;;
    y_tolerance) Y_TOLERANCE="$value" ;;
    stop_angle) STOP_ANGLE="$value" ;;
    range_gain) RANGE_GAIN="$value" ;;
    y_gain) Y_GAIN="$value" ;;
    ball_yaw_gain) BALL_YAW_GAIN="$value" ;;
    vx_limit) VX_LIMIT="$value" ;;
    vy_limit) VY_LIMIT="$value" ;;
    vtheta_limit) VTHETA_LIMIT="$value" ;;
    turn_first_threshold) TURN_FIRST_THRESHOLD="$value" ;;
    fixed_head_yaw) FIXED_HEAD_YAW="$value" ;;
    max_ball_range) MAX_BALL_RANGE="$value" ;;
    require_play) REQUIRE_PLAY="$value" ;;
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
      target_range=*|target_y_offset=*|theta_offset=*|range_tolerance=*|y_tolerance=*|stop_angle=*|range_gain=*|y_gain=*|ball_yaw_gain=*|vx_limit=*|vy_limit=*|vtheta_limit=*|turn_first_threshold=*|fixed_head_yaw=*|max_ball_range=*|require_play=*)
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

echo "Shooting adjustment settings:"
echo "  target_range=${TARGET_RANGE}"
echo "  target_y_offset=${TARGET_Y_OFFSET}"
echo "  theta_offset=${THETA_OFFSET}"
echo "  range_tolerance=${RANGE_TOLERANCE}"
echo "  y_tolerance=${Y_TOLERANCE}"
echo "  stop_angle=${STOP_ANGLE}"
echo "  range_gain=${RANGE_GAIN}"
echo "  y_gain=${Y_GAIN}"
echo "  ball_yaw_gain=${BALL_YAW_GAIN}"
echo "  vx_limit=${VX_LIMIT}"
echo "  vy_limit=${VY_LIMIT}"
echo "  vtheta_limit=${VTHETA_LIMIT}"
echo "  turn_first_threshold=${TURN_FIRST_THRESHOLD}"
echo "  fixed_head_yaw=${FIXED_HEAD_YAW}"
echo "  max_ball_range=${MAX_BALL_RANGE}"
echo "  require_play=${REQUIRE_PLAY}"

if [[ "$REQUIRE_PLAY" == "true" ]]; then
  STOP_CONDITION="gc_game_state!='PLAY'"
  RUN_CONDITION="gc_game_state=='PLAY'"
  echo "Shooting adjustment waits for GameController PLAY."
else
  STOP_CONDITION="gc_game_state=='END'"
  RUN_CONDITION="gc_game_state!='END'"
  echo "Shooting adjustment starts immediately for a usable nearby ball."
fi

# This standalone test owns the only brain process and movement command.
./scripts/stop.sh || true

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/adjust_for_shooting.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="${STOP_CONDITION}" name="shooting adjustment disabled">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <ReactiveSequence _while="${RUN_CONDITION}" name="shooting adjustment">
        <CheckAndStandUp />
        <IfThenElse>
          <ScriptCondition name="Ball visible?" code="ball_visible" />
          <Sequence name="track and position for shot">
            <ShootingAdjust target_range="${TARGET_RANGE}"
                            target_y_offset="${TARGET_Y_OFFSET}"
                            theta_offset="${THETA_OFFSET}"
                            range_tolerance="${RANGE_TOLERANCE}"
                            y_tolerance="${Y_TOLERANCE}"
                            stop_angle="${STOP_ANGLE}"
                            range_gain="${RANGE_GAIN}"
                            y_gain="${Y_GAIN}"
                            ball_yaw_gain="${BALL_YAW_GAIN}"
                            vx_limit="${VX_LIMIT}"
                            vy_limit="${VY_LIMIT}"
                            vtheta_limit="${VTHETA_LIMIT}"
                            turn_first_threshold="${TURN_FIRST_THRESHOLD}"
                            fixed_head_yaw="${FIXED_HEAD_YAW}"
                            max_ball_range="${MAX_BALL_RANGE}"
                            vx="{shoot_vx}"
                            vy="{shoot_vy}"
                            theta="{shoot_theta}" />
            <SetVelocity x="{shoot_vx}" y="{shoot_vy}" theta="{shoot_theta}" />
          </Sequence>
          <Sequence name="recover missing ball with head only">
            <SetVelocity x="0" y="0" theta="0" />
            <CamFindBall body_search_speed="0.0" theta="{recovery_theta}" />
          </Sequence>
        </IfThenElse>
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "Wrote ${TREE_PATH}"
echo "Starting vision, shooting-adjustment brain, and GameController receiver..."
ros2 launch vision launch.py > vision.log 2>&1 &
ros2 launch brain launch.py \
  tree:=adjust_for_shooting.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=false \
  disable_com:=true \
  > brain.log 2>&1 &
ros2 launch game_controller launch.py > game_controller.log 2>&1 &

if [[ "$REQUIRE_PLAY" == "true" ]]; then
  echo "Ready. GameController PLAY enables shooting adjustment; other states command zero."
else
  echo "Running. The robot moves only for a usable ball within max_ball_range."
fi
echo "Press s or Ctrl-C to stop."
echo "Diagnostics: tail -f brain.log | grep -E 'ShootingAdjust/vector|CamFindBall/(start|search)|RobotClient/setVelocity_(in|out)'"

while true; do
  if ! read -rsn1 key; then
    controlled_stop "input closed"
  fi
  if [[ "$key" == "s" || "$key" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
