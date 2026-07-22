# AGENTS.md

This repo is the Booster T1 RoboCup autonomy workspace (`booster_soccer`, derived from Booster Robotics `robocup_demo`). Agents should treat it as the main contest stack:

```text
vision -> brain behavior tree -> RobotClient -> LocoApiTopicReq -> robot motion
```

Do not replace this with ad hoc SDK-only demos unless the user explicitly asks for a diagnostic test.

## Robot-Only Runtime

The code can only be built, launched, and verified on the physical robot over SSH. Laptop-to-robot DDS over Wi-Fi is not the normal workflow because the robot Fast DDS profile is scoped for local robot communication.

Robot SSH target: `booster@192.168.68.105`

When giving the user commands to run on the robot, assume they are already inside an SSH session unless they explicitly ask how to connect. Do not put `ssh booster@...` inside the same copyable command block as robot commands. If a connection reminder is useful, mention the SSH target separately in prose, then provide a copyable robot command block that starts with `cd ~/booster_soccer`.

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

For the normal integration workflow, unless the user invokes the explicit `commit++` shortcut below, remind the user that robot testing requires moving changes through `main`:

1. Push the current working branch.
2. Merge that branch into `main`.
3. Connect to the robot and pull `main` in `~/booster_soccer`.
4. Fix/verify the robot clock, then rebuild on the robot before running.

Use commands like these, staying on the current dedicated branch:

```bash
# On the development machine
git status
git add <changed-files>
git commit -m "<message>"
git push origin HEAD

# Merge the current branch into main via PR or the team's existing merge workflow.
# Do not create, delete, or rename branches.

# On the robot after connecting over SSH
cd ~/booster_soccer
deactivate 2>/dev/null || true
git switch main
git pull origin main
git submodule update --init --recursive
chmod +x scripts/*.sh
```

## `commit++` Shortcut

When the user sends `commit++`, treat it as shorthand for the complete local-commit and robot-transfer handoff. Do not make the user separately ask for `commit`, `push pull commands`, and `build command`.

For `commit++`:

1. Inspect the status and diff and run checks appropriate to the changed files. If a required check fails, stop and report it instead of committing.
2. Stage only the files belonging to the current task, preserve unrelated user changes, review the staged diff, and run `git diff --cached --check`.
3. Create one local commit with a clear message and report its hash. Do not create an empty commit if there are no changes.
4. Do **not** automatically push, merge, connect to the robot, discard robot changes, change the robot clock, or start a build. Instead, return the commands below for the user to run.
5. Replace the development path and branch placeholders with the actual checkout path and exact current dedicated branch. The final response must not leave placeholders for the user to edit.
6. Keep development-machine commands and robot commands in separate, clearly labeled copyable blocks. Assume the user is already in the robot SSH session for robot blocks.

First provide the development-machine push block:

```bash
cd <actual-development-checkout>
git status --short --branch
git push origin <current-dedicated-branch>
```

Then provide the immediate dedicated-branch test pull block for the robot:

```bash
cd ~/booster_soccer
git status --short --branch
git switch <current-dedicated-branch>
git pull --ff-only origin <current-dedicated-branch>
git log -1 --oneline
```

If robot changes prevent switching or pulling, tell the user to stop and share `git status`; never add an automatic stash, reset, or clean operation to this shortcut.

Then provide the clock command, explicitly labeled to run from the development machine:

```bash
ssh -t booster@192.168.68.105 "sudo timedatectl set-ntp false; sudo date -s '@$(date +%s)'; sudo hwclock --systohc 2>/dev/null || true; date"
```

Finally provide this clean-build block, explicitly labeled to run inside the robot SSH session. Keep `./scripts/build.sh` as the final command and remind the user to wait for its summary and shell prompt:

```bash
cd ~/booster_soccer
date
timedatectl status
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
rm -rf build install log
./scripts/build.sh
```

For `commit++` only, the dedicated-branch pull above replaces the normal `main` pull for immediate robot testing. It does not replace the normal merge into `main` before shared or tournament use.

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

## Fix Robot Clock Before Every Build

The robot can boot with its clock reset to 1970, and `systemd-timesyncd` may be masked. A build with the wrong clock emits `Clock skew detected` and cannot be trusted. Before every build, verify `date` on the robot. Do not change the clock while a build is running.

The reusable fix is to copy the development machine's current epoch time to the robot. Run this command from the development machine, not from inside the robot SSH session:

```bash
ssh -t booster@192.168.68.105 "sudo timedatectl set-ntp false; sudo date -s '@$(date +%s)'; sudo hwclock --systohc 2>/dev/null || true; date"
```

Then, after connecting to the robot, verify that the year and timezone are correct before building:

```bash
cd ~/booster_soccer
date
timedatectl status
```

If the date still shows 1970, stop and fix it before continuing. Do not try to restart `systemd-timesyncd` while its service is masked.

## Build On Robot

Run the clock procedure above before every normal, clean, or selected-package build. Keep the build command as the final command in its copyable block. Do not append `source install/setup.bash`, a launch command, or any other command after it; wait for the build summary and the shell prompt first.

Normal build, run on the robot after connecting over SSH:

```bash
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
./scripts/build.sh
```

Clean build when generated state may be stale, run on the robot after connecting over SSH:

```bash
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

timeout 5 ros2 topic echo /booster_soccer/detection --once | grep -E 'label:|confidence:|position:|position_confidence:|xmin:|ymin:|xmax:|ymax:' || true

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
cd ~/booster_soccer
~/safe_chase_vector.sh
```

Press `s` in that script to stop if supported by the running helper. Otherwise stop with the safe stop commands above.

## Perception Facts

- Official `vision` publishes `/booster_soccer/detection`, `/booster_soccer/line_segments`, and `/booster_soccer/ball`.
- `brain` primarily consumes `/booster_soccer/detection`, not `/booster_soccer/ball`.
- A motion-valid ball detection includes canonical `position: [x, y, z]` with positive `position_confidence`; an RGB-only Ball keeps its bounding box but has no usable position.
- `position_projection` remains only as a temporary compatibility mirror for non-Ball objects while their canonical coordinates are verified on the robot.
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
- Do not edit C++ source or header files (`*.cpp`, `*.h`, `*.hpp`, `*.cc`, `*.cxx`) unless the user explicitly approves the C++ change in that turn. Reading, explaining, or reviewing C++ files is allowed without approval.
- Do not copy `build/`, `install/`, or `log/` from laptop to robot.
- Do not edit `/opt/booster/BoosterRos2/fastdds_profile.xml` unless the user explicitly approves it.
- Use `rg` for searching.
- Use `./scripts/build.sh` or `colcon build --symlink-install --base-paths src ...` on the robot for verification.
- If you cannot run or verify because the robot is not reachable, say that clearly and give the SSH target separately from any copyable robot command block.
