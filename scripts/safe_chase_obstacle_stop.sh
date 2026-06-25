#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"
cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

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

wait_for_ball() {
  echo "Waiting for ball detection. Press s to stop, Ctrl-C to exit."

  while true; do
    timeout 5 ros2 topic echo /booster_soccer/detection --once > /tmp/detection_once.txt || true

    if grep -q 'label: Ball' /tmp/detection_once.txt; then
      echo "Ball detected."
      grep -E 'label:|confidence:|xmin:|ymin:|xmax:|ymax:|position_projection' /tmp/detection_once.txt || true
      return 0
    fi

    echo "No ball yet. Still waiting..."
    read -rsn1 -t 1 key || true
    if [[ "${key:-}" == "s" || "${key:-}" == "S" ]]; then
      controlled_stop "operator s while waiting"
    fi
  done
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
TREE_PATH="${BRAIN_SHARE}/behavior_trees/chase_obstacle_avoid.xml"

cat > "$TREE_PATH" <<'XML'
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="gc_game_state=='END'" name="manual controlled stop">
        <SetVelocity x="0" y="0" theta="0" />
      </ReactiveSequence>

      <ReactiveSequence _while="gc_game_state!='END'" name="slow chase with obstacle avoidance">
        <CheckAndStandUp />
        <SetVelocity _while="!ball_location_known" x="0" y="0" theta="0" />

        <IfThenElse>
          <ScriptCondition name="Ball location known?" code="ball_location_known || tm_ball_pos_reliable" />
          <CamTrackBall />
          <CamFindBall />
        </IfThenElse>

        <ReactiveSequence _while="ball_location_known" name="no-kick obstacle-avoid chase">
          <Chase vx_limit="0.22"
                 vy_limit="0.08"
                 vtheta_limit="0.7"
                 dist="0.50"
                 safe_dist="0.5" />
        </ReactiveSequence>
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "Starting vision..."
ros2 launch vision launch.py > vision.log 2>&1 &
sleep 8

wait_for_ball

echo "Starting no-kick Chase with obstacle avoidance. Press s to stop. Press Ctrl-C to exit."
echo "Obstacle debug topics:"
echo "  /booster_soccer/visualization_markers"
echo "  /booster_soccer/visualization_obstacle_grid"
echo "  /booster_soccer/visualization_point_cloud"

ros2 launch brain launch.py \
  tree:=chase_obstacle_avoid.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &

for _ in 1 2 3 4 5; do
  if ros2 node list | grep -qx '/brain_node'; then
    break
  fi
  sleep 1
done

ros2 param set /brain_node obstacle_avoidance.avoid_during_chase true >/dev/null 2>&1 || true
ros2 param set /brain_node obstacle_avoidance.occupancy_threshold 5.0 >/dev/null 2>&1 || true
ros2 param set /brain_node obstacle_avoidance.chase_ao_safe_dist 1.4 >/dev/null 2>&1 || true
ros2 param set /brain_node obstacle_avoidance.collision_threshold 0.35 >/dev/null 2>&1 || true
ros2 param set /brain_node game.treat_person_as_robot true >/dev/null 2>&1 || true
ros2 param set /brain_node robot.head_pitch_limit_up 0.75 >/dev/null 2>&1 || true
ros2 param set /brain_node robot.head_pitch_limit_down -0.314 >/dev/null 2>&1 || true
ros2 param set /brain_node robot.min_vx 0.05 >/dev/null 2>&1 || true
ros2 param set /brain_node robot.min_vy 0.04 >/dev/null 2>&1 || true
ros2 param set /brain_node robot.min_vtheta 0.06 >/dev/null 2>&1 || true

while true; do
  read -rsn1 key
  if [[ "$key" == "s" || "$key" == "S" ]]; then
    controlled_stop "operator s"
  fi
done
