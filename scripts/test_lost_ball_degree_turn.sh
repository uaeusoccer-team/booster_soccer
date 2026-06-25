#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  test_lost_ball_degree_turn.sh [options]

Starts vision and a temporary brain tree to test CamFindBall's degree-limited
lost-ball body turn. This script does not change robot mode. Put the robot in
the desired mode from the Booster app.

Test flow:
  1. Start this script.
  2. Show the ball so the brain tracks it.
  3. Hide the ball.
  4. After ball memory expires, CamFindBall should turn the body by the
     configured degrees toward the last seen ball side, then stop.

Options:
      --degrees DEG          Body yaw target after ball loss, default 25
      --speed RAD_PER_SEC    Max yaw speed command, default 0.25
      --tolerance DEG        Stop tolerance, default 4
      --recent-msec MSEC     Last-ball memory window, default 4000
      --timeout-msec MSEC    Safety timeout for one turn, default 3000
      --pitches LIST         Smooth search pitch rows, default 0.75,0.50,0.35
      --min-yaw RAD          Smooth search minimum yaw, default -1.0
      --max-yaw RAD          Smooth search maximum yaw, default 1.0
      --yaw-speed RAD_SEC    Smooth search yaw speed, default 0.75
      --pitch-speed RAD_SEC  Smooth search pitch speed, default 0.35
      --command-hz HZ        Head command rate, default 25
      --dwell-msec MSEC      Search dwell at each target, default 80
  -h, --help                 Show this help

Press s to stop motion while brain stays alive. Press Ctrl-C to stop motion and
kill the stack. Keep the Booster app open as the trusted override.
EOF
}

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"
DEGREES="${LOST_TURN_DEGREES:-25}"
TURN_SPEED="${LOST_TURN_SPEED:-0.25}"
TOLERANCE="${LOST_TURN_TOLERANCE_DEGREES:-4}"
RECENT_MSEC="${LOST_TURN_RECENT_MSEC:-4000}"
TIMEOUT_MSEC="${LOST_TURN_TIMEOUT_MSEC:-3000}"
PITCHES="${HEAD_SCAN_PITCHES:-0.75,0.50,0.35}"
MIN_YAW="${HEAD_MIN_YAW:--1.0}"
MAX_YAW="${HEAD_MAX_YAW:-1.0}"
YAW_SPEED="${HEAD_YAW_SPEED:-0.75}"
PITCH_SPEED="${HEAD_PITCH_SPEED:-0.35}"
COMMAND_HZ="${HEAD_COMMAND_HZ:-25}"
DWELL_MSEC="${HEAD_DWELL_MSEC:-80}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --degrees)
      DEGREES="${2:?missing value for $1}"
      shift 2
      ;;
    --speed)
      TURN_SPEED="${2:?missing value for $1}"
      shift 2
      ;;
    --tolerance)
      TOLERANCE="${2:?missing value for $1}"
      shift 2
      ;;
    --recent-msec)
      RECENT_MSEC="${2:?missing value for $1}"
      shift 2
      ;;
    --timeout-msec)
      TIMEOUT_MSEC="${2:?missing value for $1}"
      shift 2
      ;;
    --pitches)
      PITCHES="${2:?missing value for $1}"
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

cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml}"

STOP_REQUESTED=0
BRAIN_PID=""
VISION_PID=""
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
  local vision_status="not-started"
  if [[ -n "$BRAIN_PID" ]]; then
    if kill -0 "$BRAIN_PID" 2>/dev/null; then
      brain_status="launch-running"
    else
      brain_status="launch-exited"
    fi
  fi
  if [[ -n "$VISION_PID" ]]; then
    if kill -0 "$VISION_PID" 2>/dev/null; then
      vision_status="launch-running"
    else
      vision_status="launch-exited"
    fi
  fi

  echo "status $(date +%H:%M:%S) | brain=${brain_status} | vision=${vision_status} | lost_turn=${DEGREES}deg | speed=${TURN_SPEED}rad/s | stopped=${STOP_REQUESTED}"

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
sleep 3

ros2 daemon stop || true
sleep 2
ros2 daemon start
sleep 2

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/lost_ball_degree_turn_test.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="gc_game_state=='END'" name="manual controlled stop">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <ReactiveSequence _while="gc_game_state!='END'" name="lost ball degree turn test">
        <IfThenElse>
          <ScriptCondition name="Ball location known?" code="ball_location_known || tm_ball_pos_reliable" />

          <Sequence name="[Yes] track ball and hold body still">
            <CamTrackBall />
            <SetVelocity x="0" y="0" theta="0" />
          </Sequence>

          <Sequence name="[No] smooth search and degree-limited lost turn">
            <CamFindBall search_mode="smooth"
                         smooth_pitches="${PITCHES}"
                         min_yaw="${MIN_YAW}"
                         max_yaw="${MAX_YAW}"
                         yaw_speed="${YAW_SPEED}"
                         pitch_speed="${PITCH_SPEED}"
                         command_hz="${COMMAND_HZ}"
                         dwell_msec="${DWELL_MSEC}"
                         turn_body_on_loss="true"
                         lost_turn_msec="${RECENT_MSEC}"
                         lost_turn_speed="${TURN_SPEED}"
                         lost_turn_min_yaw="0.08"
                         lost_turn_degrees="${DEGREES}"
                         lost_turn_tolerance_degrees="${TOLERANCE}"
                         lost_turn_timeout_msec="${TIMEOUT_MSEC}" />
          </Sequence>
        </IfThenElse>
      </ReactiveSequence>

      <ReactiveSequence _while="gc_game_state=='END'" name="hold after stop">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "---- lost-ball degree turn test ----"
echo "tree: ${TREE_PATH}"
echo "lost_turn_degrees: ${DEGREES}"
echo "lost_turn_speed: ${TURN_SPEED}"
echo "lost_turn_tolerance_degrees: ${TOLERANCE}"
echo "lost_turn_recent_msec: ${RECENT_MSEC}"
echo "lost_turn_timeout_msec: ${TIMEOUT_MSEC}"
echo "robot mode: unchanged by this script"
echo

echo "---- start vision ----"
ros2 launch vision launch.py > vision.log 2>&1 &
VISION_PID=$!
sleep 8

echo "---- check detection once ----"
timeout 5 ros2 topic echo /booster_soccer/detection --once \
  | grep -E 'label:|confidence:|xmin:|ymin:|xmax:|ymax:|position_projection|detected_objects' || true

echo "---- start brain ----"
ros2 launch brain launch.py \
  tree:=lost_ball_degree_turn_test.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &
BRAIN_PID=$!

echo
echo "Show the ball until tracking starts, then hide it."
echo "The body should turn about ${DEGREES} degrees toward the last seen side, then stop."
echo "The script stays alive and prints status. Press s to stop motion; Ctrl-C stops motion and kills the stack."

while true; do
  read -rsn1 -t 1 key || true
  if [[ "${key:-}" == "s" || "${key:-}" == "S" ]]; then
    stop_motion "operator s"
  fi

  print_status
done
