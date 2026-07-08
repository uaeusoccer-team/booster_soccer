#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

VX_LIMIT="0.18"
VY_LIMIT="0.06"
STOP_DIST="0.8"
STOP_ANGLE="0.2"
Y_TOLERANCE="0.03"
BODY_TURN_SPEED="0.25"
HEAD_TURN_START_RATIO="0.75"
HEAD_TURN_STOP_RATIO="0.65"
FINAL_ALIGN_TURN_SPEED="0.12"
FINAL_TURN_PULSE_MSEC="250"
FINAL_SETTLE_MSEC="600"
FINAL_HEAD_YAW_MIN=""
FINAL_HEAD_YAW_MAX=""
FINAL_BALL_YAW_MIN=""
FINAL_BALL_YAW_MAX=""

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/safe_chase_vector.sh [vx_limit=0.18] [vy_limit=0.06] [stop_dist=0.8] [stop_angle=0.2] [y_tolerance=0.03] [body_turn_speed=0.25] [head_turn_start_ratio=0.75] [head_turn_stop_ratio=0.65] [final_align_turn_speed=0.12] [final_turn_pulse_msec=250] [final_settle_msec=600] [final_head_yaw_min=-0.08] [final_head_yaw_max=0.08] [final_ball_yaw_min=-0.10] [final_ball_yaw_max=0.10]

Examples:
  ./scripts/safe_chase_vector.sh vx_limit=0.20 vy_limit=0.06 stop_dist=1.8 stop_angle=0.1 y_tolerance=0.05 body_turn_speed=0.25 head_turn_start_ratio=0.75 head_turn_stop_ratio=0.65 final_align_turn_speed=0.12 final_turn_pulse_msec=250 final_settle_msec=600 final_head_yaw_min=-0.08 final_head_yaw_max=0.08 final_ball_yaw_min=0.0 final_ball_yaw_max=0.10

  ./scripts/safe_chase_vector.sh --vx-limit 0.20 --vy-limit 0.06 --stop-dist 1.8 --stop-angle 0.1 --y-tolerance 0.05 --body-turn-speed 0.25 --head-turn-start-ratio 0.75 --head-turn-stop-ratio 0.65 --final-align-turn-speed 0.12 --final-turn-pulse-msec 250 --final-settle-msec 600 --final-head-yaw-min -0.08 --final-head-yaw-max 0.08 --final-ball-yaw-min 0.0 --final-ball-yaw-max 0.10

  ./scripts/safe_chase_vector.sh 'SimpleChase {
    vx_limit="0.20"
    vy_limit="0.06"
    stop_dist="1.8"
    stop_angle="0.1"
    y_tolerance="0.05"
    body_turn_speed="0.25"
    head_turn_start_ratio="0.75"
    head_turn_stop_ratio="0.65"
    final_align_turn_speed="0.12"
    final_turn_pulse_msec="250"
    final_settle_msec="600"
    final_head_yaw_min="-0.08"
    final_head_yaw_max="0.08"
    final_ball_yaw_min="0.0"
    final_ball_yaw_max="0.10"
  }'

Press s while the script is running to stop safely.
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

set_simple_chase_value() {
  local key="$1"
  local value
  value="$(clean_value "$2")"

  local value_pattern='^([0-9]+([.][0-9]+)?|[.][0-9]+)$'
  if [[ "$key" == "final_head_yaw_min" || "$key" == "final_head_yaw_max" || "$key" == "final_ball_yaw_min" || "$key" == "final_ball_yaw_max" ]]; then
    value_pattern='^-?([0-9]+([.][0-9]+)?|[.][0-9]+)$'
  fi

  if ! [[ "$value" =~ $value_pattern ]]; then
    echo "Invalid value for ${key}: ${value}" >&2
    exit 2
  fi

  case "$key" in
    vx_limit) VX_LIMIT="$value" ;;
    vy_limit) VY_LIMIT="$value" ;;
    stop_dist) STOP_DIST="$value" ;;
    stop_angle) STOP_ANGLE="$value" ;;
    y_tolerance) Y_TOLERANCE="$value" ;;
    body_turn_speed) BODY_TURN_SPEED="$value" ;;
    head_turn_start_ratio) HEAD_TURN_START_RATIO="$value" ;;
    head_turn_stop_ratio) HEAD_TURN_STOP_RATIO="$value" ;;
    final_align_turn_speed) FINAL_ALIGN_TURN_SPEED="$value" ;;
    final_turn_pulse_msec) FINAL_TURN_PULSE_MSEC="$value" ;;
    final_settle_msec) FINAL_SETTLE_MSEC="$value" ;;
    final_head_yaw_min) FINAL_HEAD_YAW_MIN="$value" ;;
    final_head_yaw_max) FINAL_HEAD_YAW_MAX="$value" ;;
    final_ball_yaw_min) FINAL_BALL_YAW_MIN="$value" ;;
    final_ball_yaw_max) FINAL_BALL_YAW_MAX="$value" ;;
    *)
      echo "Unknown SimpleChase setting: ${key}" >&2
      exit 2
      ;;
  esac
}

parse_block_text() {
  local text="$1"
  local matched=1
  local key

  for key in vx_limit vy_limit stop_dist stop_angle y_tolerance body_turn_speed head_turn_start_ratio head_turn_stop_ratio final_align_turn_speed final_turn_pulse_msec final_settle_msec final_head_yaw_min final_head_yaw_max final_ball_yaw_min final_ball_yaw_max; do
    if [[ "$text" =~ (^|[[:space:]\{])${key}[[:space:]]*=[[:space:]]*\"?(-?[0-9]+([.][0-9]+)?|-?[.][0-9]+)\"? ]]; then
      set_simple_chase_value "$key" "${BASH_REMATCH[2]}"
      matched=0
    fi
  done

  return "$matched"
}

parse_args() {
  while (($#)); do
    case "$1" in
      -h|--help)
        usage
        exit 0
        ;;
      --vx-limit|--vx_limit)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value vx_limit "$2"
        shift 2
        ;;
      --vy-limit|--vy_limit)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value vy_limit "$2"
        shift 2
        ;;
      --stop-dist|--stop_dist)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value stop_dist "$2"
        shift 2
        ;;
      --stop-angle|--stop_angle)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value stop_angle "$2"
        shift 2
        ;;
      --y-tolerance|--y_tolerance)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value y_tolerance "$2"
        shift 2
        ;;
      --body-turn-speed|--body_turn_speed)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value body_turn_speed "$2"
        shift 2
        ;;
      --head-turn-start-ratio|--head_turn_start_ratio)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value head_turn_start_ratio "$2"
        shift 2
        ;;
      --head-turn-stop-ratio|--head_turn_stop_ratio)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value head_turn_stop_ratio "$2"
        shift 2
        ;;
      --final-align-turn-speed|--final_align_turn_speed)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value final_align_turn_speed "$2"
        shift 2
        ;;
      --final-turn-pulse-msec|--final_turn_pulse_msec)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value final_turn_pulse_msec "$2"
        shift 2
        ;;
      --final-settle-msec|--final_settle_msec)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value final_settle_msec "$2"
        shift 2
        ;;
      --final-head-yaw-min|--final_head_yaw_min)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value final_head_yaw_min "$2"
        shift 2
        ;;
      --final-head-yaw-max|--final_head_yaw_max)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value final_head_yaw_max "$2"
        shift 2
        ;;
      --final-ball-yaw-min|--final_ball_yaw_min)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value final_ball_yaw_min "$2"
        shift 2
        ;;
      --final-ball-yaw-max|--final_ball_yaw_max)
        [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
        set_simple_chase_value final_ball_yaw_max "$2"
        shift 2
        ;;
      --vx-limit=*|--vx_limit=*)
        set_simple_chase_value vx_limit "${1#*=}"
        shift
        ;;
      --vy-limit=*|--vy_limit=*)
        set_simple_chase_value vy_limit "${1#*=}"
        shift
        ;;
      --stop-dist=*|--stop_dist=*)
        set_simple_chase_value stop_dist "${1#*=}"
        shift
        ;;
      --stop-angle=*|--stop_angle=*)
        set_simple_chase_value stop_angle "${1#*=}"
        shift
        ;;
      --y-tolerance=*|--y_tolerance=*)
        set_simple_chase_value y_tolerance "${1#*=}"
        shift
        ;;
      --body-turn-speed=*|--body_turn_speed=*)
        set_simple_chase_value body_turn_speed "${1#*=}"
        shift
        ;;
      --head-turn-start-ratio=*|--head_turn_start_ratio=*)
        set_simple_chase_value head_turn_start_ratio "${1#*=}"
        shift
        ;;
      --head-turn-stop-ratio=*|--head_turn_stop_ratio=*)
        set_simple_chase_value head_turn_stop_ratio "${1#*=}"
        shift
        ;;
      --final-align-turn-speed=*|--final_align_turn_speed=*)
        set_simple_chase_value final_align_turn_speed "${1#*=}"
        shift
        ;;
      --final-turn-pulse-msec=*|--final_turn_pulse_msec=*)
        set_simple_chase_value final_turn_pulse_msec "${1#*=}"
        shift
        ;;
      --final-settle-msec=*|--final_settle_msec=*)
        set_simple_chase_value final_settle_msec "${1#*=}"
        shift
        ;;
      --final-head-yaw-min=*|--final_head_yaw_min=*)
        set_simple_chase_value final_head_yaw_min "${1#*=}"
        shift
        ;;
      --final-head-yaw-max=*|--final_head_yaw_max=*)
        set_simple_chase_value final_head_yaw_max "${1#*=}"
        shift
        ;;
      --final-ball-yaw-min=*|--final_ball_yaw_min=*)
        set_simple_chase_value final_ball_yaw_min "${1#*=}"
        shift
        ;;
      --final-ball-yaw-max=*|--final_ball_yaw_max=*)
        set_simple_chase_value final_ball_yaw_max "${1#*=}"
        shift
        ;;
      vx_limit=*|vy_limit=*|stop_dist=*|stop_angle=*|y_tolerance=*|body_turn_speed=*|head_turn_start_ratio=*|head_turn_stop_ratio=*|final_align_turn_speed=*|final_turn_pulse_msec=*|final_settle_msec=*|final_head_yaw_min=*|final_head_yaw_max=*|final_ball_yaw_min=*|final_ball_yaw_max=*)
        set_simple_chase_value "${1%%=*}" "${1#*=}"
        shift
        ;;
      SimpleChase|"{"|"}")
        shift
        ;;
      *)
        if parse_block_text "$1"; then
          shift
        else
          echo "Unknown argument: $1" >&2
          usage >&2
          exit 2
        fi
        ;;
    esac
  done
}

send_game_stop() {
  for _ in 1 2 3 4 5 6 7 8; do
    ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
    sleep 0.4
  done
}

controlled_stop() {
  local reason="${1:-manual}"
  trap - INT TERM

  echo
  echo "---- controlled stop requested: ${reason} ----"
  echo "Sending stop while brain is still alive..."

  send_game_stop

  echo "Holding zero command for 3 seconds before killing nodes..."
  sleep 3

  ./scripts/stop.sh || true

  echo "Stopped safely."
  exit 0
}

quick_exit() {
  controlled_stop "Ctrl-C"
}

wait_for_ball() {
  echo "Waiting for ball detection. Press s to stop, Ctrl-C to exit."

  while true; do
    timeout 5 ros2 topic echo /booster_soccer/detection --once > /tmp/detection_once.txt || true

    if grep -q 'label: Ball' /tmp/detection_once.txt; then
      echo "Ball detected."
      grep -E 'label:|confidence:|xmin:|ymin:|xmax:|ymax:' /tmp/detection_once.txt || true
      return 0
    fi

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

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$FASTRTPS_DEFAULT_PROFILES_FILE}"

trap 'quick_exit' INT
trap 'quick_exit' TERM

echo "SimpleChase settings:"
echo "  vx_limit=${VX_LIMIT}"
echo "  vy_limit=${VY_LIMIT}"
echo "  stop_dist=${STOP_DIST}"
echo "  stop_angle=${STOP_ANGLE}"
echo "  y_tolerance=${Y_TOLERANCE}"
echo "  body_turn_speed=${BODY_TURN_SPEED}"
echo "  head_turn_start_ratio=${HEAD_TURN_START_RATIO}"
echo "  head_turn_stop_ratio=${HEAD_TURN_STOP_RATIO}"
echo "  final_align_turn_speed=${FINAL_ALIGN_TURN_SPEED}"
echo "  final_turn_pulse_msec=${FINAL_TURN_PULSE_MSEC}"
echo "  final_settle_msec=${FINAL_SETTLE_MSEC}"
echo "  final_head_yaw_min=${FINAL_HEAD_YAW_MIN:-disabled}"
echo "  final_head_yaw_max=${FINAL_HEAD_YAW_MAX:-disabled}"
echo "  final_ball_yaw_min=${FINAL_BALL_YAW_MIN:-default(-stop_angle)}"
echo "  final_ball_yaw_max=${FINAL_BALL_YAW_MAX:-default(stop_angle)}"

./scripts/stop.sh || true
sleep 3

ros2 daemon stop || true
sleep 2
ros2 daemon start
sleep 2

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/chase_vector_safe.xml"
FINAL_YAW_XML=""
if [[ -n "$FINAL_HEAD_YAW_MIN" ]]; then
  FINAL_YAW_XML="${FINAL_YAW_XML}
                     final_head_yaw_min=\"${FINAL_HEAD_YAW_MIN}\""
fi
if [[ -n "$FINAL_HEAD_YAW_MAX" ]]; then
  FINAL_YAW_XML="${FINAL_YAW_XML}
                     final_head_yaw_max=\"${FINAL_HEAD_YAW_MAX}\""
fi
if [[ -n "$FINAL_BALL_YAW_MIN" ]]; then
  FINAL_YAW_XML="${FINAL_YAW_XML}
                     final_ball_yaw_min=\"${FINAL_BALL_YAW_MIN}\""
fi
if [[ -n "$FINAL_BALL_YAW_MAX" ]]; then
  FINAL_YAW_XML="${FINAL_YAW_XML}
                     final_ball_yaw_max=\"${FINAL_BALL_YAW_MAX}\""
fi

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <include path="./subtrees/subtree_cam_find_and_track_ball.xml" />

  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="gc_game_state=='END'" name="manual controlled stop">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <ReactiveSequence _while="gc_game_state!='END'" name="safe chase behind ball">
        <CheckAndStandUp />
        <SetVelocity _while="!ball_location_known" x="0" y="0" theta="0" />
        <SubTree ID="CamFindAndTrackBall" _autoremap="true" />

        <SimpleChase _while="ball_location_known"
                     vx_limit="${VX_LIMIT}"
                     vy_limit="${VY_LIMIT}"
                     stop_dist="${STOP_DIST}"
                     stop_angle="${STOP_ANGLE}"
                     y_tolerance="${Y_TOLERANCE}"
                     body_turn_speed="${BODY_TURN_SPEED}"
                     head_turn_start_ratio="${HEAD_TURN_START_RATIO}"
                     head_turn_stop_ratio="${HEAD_TURN_STOP_RATIO}"
                     final_align_turn_speed="${FINAL_ALIGN_TURN_SPEED}"
                     final_turn_pulse_msec="${FINAL_TURN_PULSE_MSEC}"
                     final_settle_msec="${FINAL_SETTLE_MSEC}"${FINAL_YAW_XML} />
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

echo "Starting safe behind-ball chase. Press s to stop. Press Ctrl-C to exit."
ros2 launch brain launch.py \
  tree:=chase_vector_safe.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &

while true; do
  read -rsn1 key
  if [[ "$key" == "s" || "$key" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
