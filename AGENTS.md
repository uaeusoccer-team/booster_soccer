# AGENTS.md

This repo is the Booster T1 RoboCup autonomy workspace (`booster_soccer`, derived from Booster Robotics `robocup_demo`). Agents should treat it as the main contest stack:

```text
vision -> brain behavior tree -> RobotClient -> LocoApiTopicReq -> robot motion
```

Do not replace this with ad hoc SDK-only demos unless the user explicitly asks for a diagnostic test.

## Robot-Only Runtime

The code can only be built, launched, and verified on the physical robot over SSH. Laptop-to-robot DDS over Wi-Fi is not the normal workflow because the robot Fast DDS profile is scoped for local robot communication.

Robot SSH target:

```bash
ssh booster@192.168.68.103
```

Do not commit passwords or other credentials into this repo. Ask the user if SSH authentication is needed.

Robot repo path:

```bash
~/booster_soccer
```

Development checkouts may live anywhere on each teammate's machine. Refer to this project by the repo directory name, `booster_soccer`, unless a command is explicitly meant to run on the robot.

## Critical User Reminder After Changes

Before making any code changes, the agent must stay on the current team member's dedicated branch and pull `main` into that branch:

```bash
git status --short --branch
git branch --show-current
git pull origin main
```

If the GitHub CLI is available and the team's workflow uses it, the agent may use the equivalent GitHub-assisted sync flow. If GitHub CLI is not available, use `git pull origin main`.

Do not create new branches, delete branches, rename branches, or switch to a new work branch. Each team member already has a dedicated branch. If the working tree already has uncommitted user changes, do not overwrite them; ask the user before pulling if a conflict or merge would affect those changes.

When an agent changes code, remind the user that robot testing requires moving the changes to the robot:

1. Push the current working branch.
2. Merge that branch into `main`.
3. SSH into the robot and pull `main` in `~/booster_soccer`.
4. Rebuild on the robot before running.

Use commands like these, staying on the current dedicated branch:

```bash
# On the development machine
git status
git add <changed-files>
git commit -m "<message>"
git push origin HEAD

# Merge the current branch into main via PR or the team's existing merge workflow.
# Do not create, delete, or rename branches.

# On the robot
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
git switch main
git pull origin main
git submodule update --init --recursive
chmod +x scripts/*.sh
```

## Safety Rules

- Use the Booster app/mode controls safely before running robot code.
- Keep the robot in `DAMP` for SSH setup, firmware/software operations, topic inspection, and head-only tests.
- Do not run walking, chasing, kicking, `WALK`, or `CUSTOM` behavior while the robot is on the stand.
- Only run walking/chasing/kicking when the robot is off the stand, balanced on the floor, in open space, and the operator is ready to stop it.
- Always know how to stop the stack before starting it.
- Avoid low-level SDK publishers initially. Prefer the high-level `booster_soccer` brain and RobotClient path.

Safe stop commands:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 2 ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: stop}" || true
./scripts/stop.sh
```

## Robot Environment

The robot runs Ubuntu/ROS 2 Humble and has the Booster software stack installed. Use bash on the robot.

Before ROS builds or launches:

```bash
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
which python3
```

Expected Python for ROS work:

```text
/usr/bin/python3
```

Do not build ROS while the Booster SDK Python venv is active (`.venv-booster-sdk`), because it can break ROS message generation.

Use the SDK venv only for SDK Python examples:

```bash
boostersdk
cd ~/booster_robotics_sdk
python -c "import booster_robotics_sdk_python; print('SDK OK')"
```

## Build On Robot

Normal build:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
./scripts/build.sh
```

Clean build when generated state may be stale:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
rm -rf build install log
./scripts/build.sh
```

Build only selected packages when iterating:

```bash
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src --packages-select brain
```

## Run Commands For The Robot

All run commands below must be executed on the robot over SSH.

### Stop Everything

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/stop.sh
```

### Full RoboCup Stack, Agent Mode

Use this for early autonomous testing without the referee GameController:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/stop.sh
./scripts/start.sh role:=striker team_id:=5 player_id:=1 agent_mode:=true disable_com:=true
```

This starts `vision`, `brain`, and `game_controller`, with logs in:

```bash
tail -f vision.log
tail -f brain.log
tail -f game_controller.log
```

### Full Tournament-Style Stack

Use this after simplified behavior is proven and GameController packets are verified:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/stop.sh
./scripts/start.sh role:=striker team_id:=5 player_id:=1
```

### Head-Only Ball Tracking

This is the safest first behavior because it keeps body velocity at zero through `SetVelocity`.

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

Stop head-only tracking:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/stop.sh
```

### Built-In Packaged Vision Bridge, If Needed

The public brain subscribes to:

```text
/booster_soccer/detection
/booster_soccer/line_segments
```

The robot has sometimes shown built-in packaged vision on:

```text
/booster_vision/ball
/booster_vision/detection
```

If `/booster_vision/ball` is active but `/booster_soccer/detection` is missing, the repo includes a bridge:

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

Then start the brain in another SSH session.

### Optional Slow Chase

Only run chase behavior with the robot on the ground in open space. If the robot has the helper script installed:

```bash
ssh booster@192.168.68.103
~/safe_chase_vector.sh
```

Press `s` in that script to stop if supported by the running helper. Otherwise stop with the safe stop commands above.

## Perception Facts

- Official `vision` publishes `/booster_soccer/detection`, `/booster_soccer/line_segments`, and `/booster_soccer/ball`.
- `brain` primarily consumes `/booster_soccer/detection`, not `/booster_soccer/ball`.
- A ball detection includes `position_projection: [x, y, z]`.
- For the ball, `x` is forward distance, `y` is lateral offset, and `z` is near zero on the ground plane.
- Depth is available on `/boostercamera/head/depth` and `/boostercamera/head/depth/camera_info`.
- T1 camera config in this repo uses `/boostercamera/head/rgb`, `/boostercamera/head/rgb/camera_info`, and `/boostercamera/head/depth`.

Useful checks:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | sort | grep -E 'vision|brain|game|yolo'
ros2 topic list | sort | grep -E 'booster_soccer|booster_vision|detection|ball|line|kick|Loco'
ros2 topic info -v /booster_soccer/detection
ros2 topic echo --once /booster_soccer/detection | grep -A 30 "label: Ball"
ros2 topic hz /boostercamera/head/depth
```

## Brain And Behavior Trees

Important files:

```text
src/brain/behavior_trees/game.xml
src/brain/behavior_trees/head_track_only.xml
src/brain/behavior_trees/head_track_no_search.xml
src/brain/behavior_trees/subtrees/subtree_cam_find_and_track_ball.xml
src/brain/behavior_trees/subtrees/subtree_find_ball.xml
src/brain/behavior_trees/subtrees/subtree_striker_play.xml
src/brain/config/config.yaml
src/brain/src/brain.cpp
src/brain/src/brain_tree.cpp
src/brain/src/robot_client.cpp
```

Behavior notes:

- `head_track_only.xml` runs `CamFindAndTrackBall` and then `SetVelocity`, so the body should remain still.
- `subtree_find_ball.xml` can rotate/walk (`TurnOnSpot`, `GoToReadyPosition`), so do not treat it as stand-safe.
- `StrikerPlay` already includes ball tracking, decision making, chase, adjust, kick, and visual kick nodes.
- `SimpleChase` is the first-choice primitive for a slow chase test.
- `RLVisionKick.enableAutoVisualKick` is disabled in T1 config and should usually stay disabled.

T1 config already set in `src/brain/config/config.yaml`:

```yaml
robot_height: 1.12
odom_factor: 1.2
RLVisionKick:
  enableAutoVisualKick: false
vision:
  image_camera_info_topic: "/boostercamera/head/rgb/camera_info"
  depth_image_topic: "/boostercamera/head/depth"
  depth_camera_info_topic: "/boostercamera/head/depth/camera_info"
```

## GameController / Tournament Rules

Robot-side brain listens for GameController data on:

```text
/booster_soccer/game_controller
```

Verified game states in the brain include:

```text
INITIAL, READY, SET, PLAY, END
```

GameController/referee UI is run from the referee/laptop side, not inside this repo's normal robot launch path:

```bash
cd ~/GameController
LIBGL_ALWAYS_SOFTWARE=1 cargo run
```

Robot verification:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /booster_soccer/game_controller
```

If state changes are not received, inspect UDP traffic on the correct interface:

```bash
sudo tcpdump -ni any udp
```

## Development Rules

- Prefer existing behavior tree nodes and `RobotClient` functions over new motion pathways.
- Keep changes focused; avoid unrelated refactors and generated artifact churn.
- Do not copy `build/`, `install/`, or `log/` from laptop to robot.
- Do not edit `/opt/booster/BoosterRos2/fastdds_profile.xml` unless the user explicitly approves it.
- Use `rg` for searching.
- Use `./scripts/build.sh` or `colcon build --symlink-install --base-paths src ...` on the robot for verification.
- If you cannot run or verify because the robot is not reachable, say that clearly and give the exact SSH commands for the user.
