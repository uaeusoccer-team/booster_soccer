# CLAUDE.md

Claude should follow the same project instructions as `AGENTS.md`. This file repeats the critical operating context because this project only runs on the physical Booster T1 robot.

## Project Model

This repo is the Booster T1 RoboCup autonomy workspace:

```text
vision -> brain behavior tree -> RobotClient -> LocoApiTopicReq -> robot motion
```

Use the existing RoboCup stack first. Do not write a separate SDK-only control loop unless the user asks for a diagnostic demo.

## Robot-Only Runtime

The code can only be built, launched, and verified on the robot via SSH:

```bash
ssh booster@192.168.68.103
```

Robot repo path:

```bash
~/booster_soccer
```

Do not put passwords in this repo. Ask the user for SSH auth if needed.

## Required Reminder After Code Changes

Whenever you modify code, remind the user:

1. Push the working branch.
2. Merge that branch into `main`.
3. SSH into the robot.
4. Pull `main` in `~/booster_soccer`.
5. Rebuild and run on the robot.

Command template:

```bash
# Development machine
git status
git add <changed-files>
git commit -m "<message>"
git push origin <your-branch>

# Merge via PR, or locally if appropriate
git switch main
git pull origin main
git merge <your-branch>
git push origin main

# Robot
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
git switch main
git pull origin main
git submodule update --init --recursive
chmod +x scripts/*.sh
```

## Safety

- Use `DAMP` for SSH, topic checks, builds, and head-only tests.
- Never run walking, chasing, kicking, `WALK`, or `CUSTOM` behavior on the stand.
- Run locomotion only with the robot on the ground, balanced, in open space, and with an operator ready to stop it.
- Stop before starting a new run.

Safe stop:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 2 ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: stop}" || true
./scripts/stop.sh
```

## Robot Environment

For ROS work on the robot:

```bash
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
which python3
```

`which python3` should be `/usr/bin/python3`. Do not build ROS while `.venv-booster-sdk` is active.

## Build Commands

Normal robot build:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
./scripts/build.sh
```

Clean robot build:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
rm -rf build install log
./scripts/build.sh
```

Package-only iteration:

```bash
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src --packages-select brain
```

## Run Commands

Run all commands on the robot over SSH.

Full stack in agent mode:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/stop.sh
./scripts/start.sh role:=striker team_id:=5 player_id:=1 agent_mode:=true disable_com:=true
```

Tournament-style stack:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/stop.sh
./scripts/start.sh role:=striker team_id:=5 player_id:=1
```

Head-only ball tracking:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/stop.sh
sleep 3
ros2 daemon stop || true
sleep 2
ros2 daemon start
sleep 2

ros2 launch vision launch.py > vision.log 2>&1 &
sleep 8

timeout 5 ros2 topic echo /booster_soccer/detection --once | grep -E 'label:|confidence:|position_projection|xmin:|ymin:|xmax:|ymax:' || true

ros2 launch brain launch.py \
  tree:=head_track_only.xml \
  role:=striker \
  team_id:=5 \
  player_id:=1 \
  agent_mode:=true \
  disable_com:=true \
  > brain.log 2>&1 &
```

Logs:

```bash
cd ~/booster_soccer
tail -f vision.log
tail -f brain.log
tail -f game_controller.log
```

## Perception And Topic Notes

`brain` expects:

```text
/booster_soccer/detection
/booster_soccer/line_segments
```

The robot may also show packaged vision topics:

```text
/booster_vision/ball
/booster_vision/detection
```

If `/booster_vision/ball` is active but `/booster_soccer/detection` is missing, use the bridge:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run perception_bridge ball_to_detection_bridge \
  --ros-args \
  -p input_topic:=/booster_vision/ball \
  -p output_topic:=/booster_soccer/detection \
  -p min_confidence:=0.5
```

Useful checks:

```bash
ros2 node list | sort | grep -E 'vision|brain|game|yolo'
ros2 topic list | sort | grep -E 'booster_soccer|booster_vision|detection|ball|line|kick|Loco'
ros2 topic info -v /booster_soccer/detection
ros2 topic echo --once /booster_soccer/detection | grep -A 30 "label: Ball"
ros2 topic hz /boostercamera/head/depth
```

## Key Files

```text
src/brain/config/config.yaml
src/brain/launch/launch.py
src/brain/behavior_trees/game.xml
src/brain/behavior_trees/head_track_only.xml
src/brain/behavior_trees/subtrees/subtree_cam_find_and_track_ball.xml
src/brain/behavior_trees/subtrees/subtree_find_ball.xml
src/brain/behavior_trees/subtrees/subtree_striker_play.xml
src/brain/src/brain.cpp
src/brain/src/brain_tree.cpp
src/brain/src/robot_client.cpp
src/perception_bridge/perception_bridge/ball_to_detection_bridge.py
```

Notes:

- `head_track_only.xml` should keep body velocity zero.
- `subtree_find_ball.xml` can rotate/walk and is not stand-safe.
- `StrikerPlay` already has search, chase, adjust, kick, and visual-kick logic.
- T1 config uses `robot_height: 1.12`, `odom_factor: 1.2`, and disables `RLVisionKick.enableAutoVisualKick`.
- Depth topics are `/boostercamera/head/depth` and `/boostercamera/head/depth/camera_info`.

## GameController

Robot-side topic:

```text
/booster_soccer/game_controller
```

Verify on robot:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /booster_soccer/game_controller
```

Run referee UI on the referee/laptop side when needed:

```bash
cd ~/GameController
LIBGL_ALWAYS_SOFTWARE=1 cargo run
```

