#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

VX_LIMIT="0.18"
VY_LIMIT="0.06"
STOP_DIST="0.8"
STOP_ANGLE="0.2"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/safe_chase_vector.sh [vx_limit=0.18] [vy_limit=0.06] [stop_dist=0.8] [stop_angle=0.2]

Examples:
  ./scripts/safe_chase_vector.sh vx_limit=0.20 vy_limit=0.06 stop_dist=1.4 stop_angle=0.1

  ./scripts/safe_chase_vector.sh --vx-limit 0.20 --vy-limit 0.06 --stop-dist 1.4 --stop-angle 0.1

  ./scripts/safe_chase_vector.sh 'SimpleChase {
    vx_limit="0.20"
    vy_limit="0.06"
    stop_dist="1.4"
    stop_angle="0.1"
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

  if ! [[ "$value" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]]; then
    echo "Invalid value for ${key}: ${value}" >&2
    exit 2
  fi

  case "$key" in
    vx_limit) VX_LIMIT="$value" ;;
    vy_limit) VY_LIMIT="$value" ;;
    stop_dist) STOP_DIST="$value" ;;
    stop_angle) STOP_ANGLE="$value" ;;
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

  for key in vx_limit vy_limit stop_dist stop_angle; do
    if [[ "$text" =~ (^|[[:space:]\{])${key}[[:space:]]*=[[:space:]]*\"?([0-9]+([.][0-9]+)?|[.][0-9]+)\"? ]]; then
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
      vx_limit=*|vy_limit=*|stop_dist=*|stop_angle=*)
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

./scripts/stop.sh || true
sleep 3

ros2 daemon stop || true
sleep 2
ros2 daemon start
sleep 2

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/chase_vector_safe.xml"

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
                     stop_angle="${STOP_ANGLE}" />
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
