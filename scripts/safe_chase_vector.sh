#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"
cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

send_game_stop() {
  for i in 1 2 3 4 5 6 7 8; do
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

trap 'quick_exit' INT
trap 'quick_exit' TERM

./scripts/stop.sh || true
sleep 3

ros2 daemon stop || true
sleep 2
ros2 daemon start
sleep 2

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/chase_vector_safe.xml"

cat > "$TREE_PATH" <<'XML'
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
                     vx_limit="0.18"
                     vy_limit="0.06"
                     stop_dist="0.8"
                     stop_angle="0.2" />
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

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
