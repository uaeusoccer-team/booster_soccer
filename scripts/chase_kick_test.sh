#!/usr/bin/env bash
set -Eeo pipefail
set +H

VX_LIMIT=0.12
VY_LIMIT=0.04
VTHETA_LIMIT=0.45
KICK_DIST=0.55
STOP_ANGLE=0.08
KICK_SPEED=0.45
MIN_MSEC_KICK=650
AUTO_PLAY=false

for arg in "$@"; do
  key="${arg%%=*}"
  val="${arg#*=}"
  case "$key" in
    vx_limit) VX_LIMIT="$val" ;;
    vy_limit) VY_LIMIT="$val" ;;
    vtheta_limit) VTHETA_LIMIT="$val" ;;
    kick_dist|stop_dist) KICK_DIST="$val" ;;
    stop_angle) STOP_ANGLE="$val" ;;
    kick_speed) KICK_SPEED="$val" ;;
    min_msec_kick) MIN_MSEC_KICK="$val" ;;
    auto_play) AUTO_PLAY="$val" ;;
  esac
done

cd "${WORKSPACE:-$HOME/booster_soccer}"
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash

if [[ ! -f install/setup.bash ]]; then
  echo "install/setup.bash is missing. Build first:"
  echo "  cd ~/booster_soccer"
  echo "  deactivate 2>/dev/null || true"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  ./scripts/build.sh"
  exit 1
fi

source install/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="$BRAIN_SHARE/behavior_trees/chase_kick_test.xml"

cat > "$TREE_PATH" <<XML
<root BTCPP_format="4">
  <include path="./subtrees/subtree_cam_find_and_track_ball.xml" />

  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ReactiveSequence _while="gc_game_state!='PLAY'" name="wait for play">
        <SubTree ID="CamFindAndTrackBall" _autoremap="true" />
        <SetVelocity />
      </ReactiveSequence>

      <ReactiveSequence _while="gc_game_state=='PLAY'" name="chase and kick">
        <CheckAndStandUp />
        <SubTree ID="CamFindAndTrackBall" _autoremap="true" />
        <SimpleChase _while="ball_location_known &amp;&amp; ball_range &gt; ${KICK_DIST}"
                     vx_limit="${VX_LIMIT}"
                     vy_limit="${VY_LIMIT}"
                     vtheta_limit="${VTHETA_LIMIT}"
                     stop_dist="${KICK_DIST}"
                     stop_angle="${STOP_ANGLE}" />
        <Kick _while="ball_location_known &amp;&amp; ball_range &lt;= ${KICK_DIST}"
              speed_limit="${KICK_SPEED}"
              min_msec_kick="${MIN_MSEC_KICK}"
              msecs_stablize="0" />
        <SetVelocity _while="!ball_location_known" />
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML

stop_safe() {
  ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
  ./scripts/stop.sh || true
  exit 0
}

trap stop_safe INT TERM

./scripts/stop.sh || true
sleep 3

ros2 launch vision launch.py > vision.log 2>&1 &
sleep 8

timeout 5 ros2 topic echo --once /booster_soccer/detection | grep -A 30 "label: Ball" || true

ros2 launch brain launch.py \
  tree:=chase_kick_test.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &
sleep 5

echo "Press p to PLAY, s to STOP."
if [[ "$AUTO_PLAY" == "true" ]]; then
  ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: play}" || true
fi

while true; do
  read -rsn1 key
  if [[ "$key" == "p" || "$key" == "P" ]]; then
    ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: play}" || true
  elif [[ "$key" == "s" || "$key" == "S" ]]; then
    stop_safe
  fi
done
