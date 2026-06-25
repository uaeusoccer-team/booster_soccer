#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  test_turn_degrees_only.sh --degrees DEG [options]

Runs a no-ball body turn test through the normal brain TurnOnSpot node.
This script does not change robot mode. Put the robot in the desired mode from
the Booster app, and keep the app open as the trusted override. Press s to stop
motion while brain stays alive; press Ctrl-C to stop motion and kill the stack.

Options:
      --degrees DEG       Target yaw angle. Positive and negative are allowed.
      --left DEG          Turn left by DEG degrees.
      --right DEG         Turn right by DEG degrees.
      --tolerance DEG     Stop tolerance, default 2.
      --speed RAD_SEC     Maximum yaw velocity command, default 0.25.
      --kp GAIN           Proportional gain, default 1.4.
      --timeout-msec MS   Safety timeout, default 10000.
  -h, --help              Show this help

Examples:
  ./scripts/test_turn_degrees_only.sh --degrees 90 --speed 0.25 --tolerance 2
  ./scripts/test_turn_degrees_only.sh --right 90 --speed 0.25 --tolerance 2
EOF
}

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"
DEGREES=""
TOLERANCE="${TURN_TOLERANCE_DEG:-2}"
SPEED="${TURN_MAX_VTHETA:-0.25}"
KP="${TURN_KP:-1.4}"
TIMEOUT_MSEC="${TURN_TIMEOUT_MSEC:-10000}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --degrees)
      DEGREES="${2:?missing value for $1}"
      shift 2
      ;;
    --left)
      DEGREES="${2:?missing value for $1}"
      shift 2
      ;;
    --right)
      DEGREES="-${2#-}"
      shift 2
      ;;
    --tolerance)
      TOLERANCE="${2:?missing value for $1}"
      shift 2
      ;;
    --speed)
      SPEED="${2:?missing value for $1}"
      shift 2
      ;;
    --kp)
      KP="${2:?missing value for $1}"
      shift 2
      ;;
    --timeout-msec)
      TIMEOUT_MSEC="${2:?missing value for $1}"
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

if [[ -z "$DEGREES" ]]; then
  echo "Missing --degrees, --left, or --right." >&2
  usage >&2
  exit 2
fi

RAD="$(python3 - "$DEGREES" <<'PY'
import math
import sys
print(math.radians(float(sys.argv[1])))
PY
)"

cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml}"

STOP_REQUESTED=0
BRAIN_PID=""
LOG_REPORTED=0

send_game_stop() {
  for _ in 1 2 3 4 5 6 7 8; do
    ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
    sleep 0.4
  done
}

stop_motion() {
  local reason="${1:-manual}"
  trap - INT TERM

  echo
  echo "---- stop motion requested: ${reason} ----"
  echo "Sending stop while brain is still alive..."
  send_game_stop
  echo "Holding zero command for 3 seconds."
  sleep 3
  STOP_REQUESTED=1
  trap 'quick_exit' INT
  trap 'quick_exit' TERM
}

quick_exit() {
  stop_motion "Ctrl-C"
  ./scripts/stop.sh || true
  echo "Stopped stack."
  exit 0
}

print_status() {
  local brain_status="not-started"
  if [[ -n "$BRAIN_PID" ]]; then
    if kill -0 "$BRAIN_PID" 2>/dev/null; then
      brain_status="launch-running"
    else
      brain_status="launch-exited"
    fi
  fi

  echo "status $(date +%H:%M:%S) | brain=${brain_status} | target=${DEGREES}deg | tolerance=${TOLERANCE}deg | speed=${SPEED}rad/s | stopped=${STOP_REQUESTED}"

  if [[ "$brain_status" == "launch-exited" && "$LOG_REPORTED" == "0" ]]; then
    echo
    echo "---- brain.log tail ----"
    tail -n 120 brain.log || true
    echo "------------------------"
    LOG_REPORTED=1
  fi
}

trap 'quick_exit' INT
trap 'quick_exit' TERM

./scripts/stop.sh || true
sleep 2

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/turn_degrees_only_test.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="gc_game_state=='END'" name="manual controlled stop">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <Sequence _while="gc_game_state!='END'" name="turn degrees then hold">
        <RunOnce>
          <TurnOnSpot rad="${RAD}"
                      tolerance_deg="${TOLERANCE}"
                      max_vtheta="${SPEED}"
                      kp="${KP}"
                      timeout_msec="${TIMEOUT_MSEC}" />
        </RunOnce>
        <SetVelocity x="0" y="0" theta="0" />
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "---- turn degrees only test ----"
echo "tree: ${TREE_PATH}"
echo "target_degrees: ${DEGREES}"
echo "target_radians: ${RAD}"
echo "tolerance_degrees: ${TOLERANCE}"
echo "max_vtheta: ${SPEED}"
echo "kp: ${KP}"
echo "timeout_msec: ${TIMEOUT_MSEC}"
echo "robot mode: unchanged by this script"
echo
echo "Keep the Booster app open as override."
echo "The script stays alive and prints status. Press s to stop motion; Ctrl-C stops motion and kills the stack."

ros2 launch brain launch.py \
  tree:=turn_degrees_only_test.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &
BRAIN_PID=$!

while true; do
  read -rsn1 -t 1 key || true
  if [[ "${key:-}" == "s" || "${key:-}" == "S" ]]; then
    stop_motion "operator s"
  fi

  print_status
done
