# Chase Vector Quick Reference

Task:

> Plan the chase vector: calculate forward speed (`vx`), lateral slide (`vy`), and turning rate (`vtheta`) so the striker approaches the ball safely.

This is a documentation-only file. It does not change robot behavior, scripts, XML, config, or C++ code.

## Chase Flow

```text
game.xml
-> StrikerPlay
-> StrikerDecide chooses "chase"
-> Chase::tick() calculates vx, vy, vtheta
-> RobotClient::setVelocity() sends command
-> robot moves
```

## Main Files

`src/brain/behavior_trees/game.xml`
Checks when `StrikerPlay` runs.

`src/brain/behavior_trees/subtrees/subtree_striker_play.xml`
Checks when `Chase` runs and what XML limits it receives.

`src/brain/src/brain_tree.cpp`
Main chase-vector math in `Chase::tick()`.

`src/brain/include/brain_tree.h`
Chase input definitions.

`src/brain/src/robot_client.cpp`
Final velocity command and global caps.

`src/brain/config/config.yaml`
Robot-wide tuning and safety config.



## Fast Diagnosis

Chases too fast

- First look: `subtree_striker_play.xml`
- Likely fix: lower `vx_limit`

Turns too hard

- First look: `subtree_striker_play.xml`, then `Chase::tick()`
- Likely fix: lower `vtheta_limit` or scale turn math

Does not slide sideways

- First look: `Chase::tick()`
- Likely fix: add/use `vy` math

Approaches from wrong side

- First look: `Chase::tick()`
- Likely fix: fix target point or direct/circle-back logic

Circles too much

- First look: `Chase::tick()`
- Likely fix: tune circle-back conditions

Pushes ball away

- First look: `Chase::tick()`, `dist`, `kickDir`
- Likely fix: improve approach point behind ball

Slows too much near ball

- First look: `config.yaml`, `Chase::tick()`
- Likely fix: tune near-ball speed limit

Motion feels jerky

- First look: `Chase::tick()`
- Likely fix: use/tune smoothing

Command seems capped

- First look: `RobotClient::setVelocity()`
- Likely fix: check global caps

Loses ball

- First look: tracking/vision subtree
- Likely fix: fix ball tracking first

Leaves chase too early

- First look: `StrikerDecide`
- Likely fix: tune decision thresholds

Never enters chase

- First look: `game.xml`, `StrikerPlay`, `StrikerDecide`
- Likely fix: check game state, role, and decision

## Current Chase XML

In `subtree_striker_play.xml`:

```xml
<Chase
  _while="decision == 'chase'"
  vx_limit="0.9"
  vy_limit="0.2"
  vtheta_limit="1.0"
  dist="0.1"
  safe_dist="0.5" />
```

What this means:

- XML decides when `Chase` runs.
- XML gives `Chase` limits.
- C++ decides the actual vector math.
- `RobotClient` sends the final command.




## Scenario: Robot Chases Too Fast

Symptom:

- Robot rushes toward the ball.
- Robot overshoots.
- Robot looks unstable.

Look here first:

1. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`
2. `src/brain/config/config.yaml`
3. `src/brain/src/robot_client.cpp`

Check:

```xml
vx_limit="0.9"
```

Fix path:

```text
Lower vx_limit
-> test slowly
-> if still too fast, check config speed caps
-> if command differs from motion, inspect RobotClient::setVelocity()
```

For the chase-vector task:

- Speed-only problems are usually XML/config first.
- Math problems are usually `Chase::tick()`.




## Scenario: Robot Turns Too Hard

Symptom:

- Robot spins sharply.
- Robot oscillates left/right.
- Robot turns more than it walks.

Look here first:

1. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`
2. `src/brain/src/brain_tree.cpp`

Check:

```xml
vtheta_limit="1.0"
```

In `Chase::tick()`:

```cpp
vtheta = targetDir;
vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
```

Fix path:

```text
Lower vtheta_limit
-> test
-> if still oscillating, scale vtheta in C++
-> consider smoothing
```

For the chase-vector task:

- If direction is right but turning is too strong, tune XML.
- If turn direction keeps flipping, inspect C++ math.




## Scenario: Robot Does Not Slide Sideways

Symptom:

- Robot only walks forward and turns.
- Robot does not strafe around the ball.
- Changing `vy_limit` does not help.

Look here first:

1. `src/brain/src/brain_tree.cpp`
2. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`

Check:

```xml
vy_limit="0.2"
```

In `Chase::tick()`:

```cpp
vy = 0;
```

Fix path:

```text
Confirm vy_limit exists
-> add lateral velocity math in Chase::tick()
-> cap with vy_limit
-> test with low speed first
```

For the chase-vector task:

- XML allows sideways movement.
- Current normal C++ chase mostly does not use it.
- Real lateral sliding needs a C++ change.




## Scenario: Robot Approaches From The Wrong Side

Symptom:

- Robot reaches the ball from a bad angle.
- Robot gets between the ball and desired kick direction.
- Robot pushes the ball away from target.

Look here first:

1. `src/brain/src/brain_tree.cpp`
2. `CalcKickDir`
3. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`

Check target point in `Chase::tick()`:

```cpp
target_f.x = ballPos.x - dist * cos(kickDir);
target_f.y = ballPos.y - dist * sin(kickDir);
```

Check:

```xml
dist="0.1"
safe_dist="0.5"
```

Fix path:

```text
Verify kickDir
-> inspect target point behind ball
-> tune dist/safe_dist
-> fix direct vs circle_back logic if needed
```

For the chase-vector task:

- This is usually target math, not speed tuning.
- Chase should approach a useful point near the ball, not just the ball center.




## Scenario: Robot Circles Too Much

Symptom:

- Robot keeps going around the ball.
- Robot wastes time circling.
- Robot cannot commit to approaching.

Look here first:

1. `src/brain/src/brain_tree.cpp`
2. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`

Check in `Chase::tick()`:

```cpp
static string targetType = "direct";
static double circleBackDir = 1.0;
double dirThreshold = M_PI / 2;
```

Also check:

```xml
safe_dist="0.5"
```

Fix path:

```text
Check direct vs circle_back condition
-> inspect dirThreshold
-> tune safe_dist
-> test approach from both sides of ball
```

For the chase-vector task:

- Circle-back is useful only when the robot is on the wrong side.
- Too much circle-back means target-selection logic needs tuning.




## Scenario: Robot Pushes The Ball Too Early

Symptom:

- Robot contacts the ball before lining up.
- Robot drives into the ball center.
- Robot does not switch to adjust early enough.

Look here first:

1. `src/brain/src/brain_tree.cpp`
2. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`
3. `StrikerDecide`

Check:

```cpp
vx = min(vxLimit, brain->data->ball.range);
```

And:

```xml
dist="0.1"
```

Fix path:

```text
Increase/tune target distance behind ball
-> reduce near-ball vx
-> check chase -> adjust threshold
-> test slowly
```

For the chase-vector task:

- The goal is controlled approach, not fastest contact.
- If contact happens before alignment, inspect both chase target and decision thresholds.




## Scenario: Robot Slows Too Much Near Ball

Symptom:

- Robot approaches, then stalls.
- Robot never reaches adjust/kick position.
- Robot looks too cautious near the ball.

Look here first:

1. `src/brain/config/config.yaml`
2. `src/brain/src/brain_tree.cpp`
3. `StrikerDecide`

Check in `Chase::tick()`:

```cpp
if (
    brain->config->get_limit_near_ball_speed()
    && brain->data->ball.range < brain->config->get_near_ball_range()
) {
    vxLimit = min(brain->config->get_near_ball_speed_limit(), vxLimit);
}
```

Fix path:

```text
Check near-ball speed config
-> check ball range accuracy
-> check chase -> adjust threshold
-> tune carefully
```

For the chase-vector task:

- Near-ball speed limiting is safety logic.
- Tune it; do not remove it blindly.




## Scenario: Motion Feels Jerky

Symptom:

- Velocity changes sharply.
- Robot starts/stops roughly.
- Path looks unstable.

Look here first:

1. `src/brain/src/brain_tree.cpp`
2. `src/brain/src/robot_client.cpp`

Check in `Chase::tick()`:

```cpp
smoothVx = smoothVx * 0.7 + vx * 0.3;
smoothVy = smoothVy * 0.7 + vy * 0.3;
smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

brain->client->setVelocity(vx, vy, vtheta);
```

Fix path:

```text
Confirm jerk is from Chase output
-> consider sending smoothed values
-> test with conservative limits
```

For the chase-vector task:

- Smoothing exists but is not currently used in the final command.
- Using smoothing can help jerk, but may make response slower.




## Scenario: Command Seems Capped

Symptom:

- Chase requests one speed, but robot moves differently.
- XML changes seem ignored.
- Final motion feels limited.

Look here first:

1. `src/brain/src/robot_client.cpp`
2. `src/brain/config/config.yaml`

Check in `RobotClient::setVelocity()`:

```cpp
x = cap(x, brain->config->get_vx_limit(), -brain->config->get_vx_limit());
```

Fix path:

```text
Log Chase output
-> log RobotClient input/output
-> compare requested vs sent velocity
-> inspect global config caps
```

For the chase-vector task:

- Separate "what Chase requested" from "what RobotClient sent."
- If RobotClient caps it, Chase math may not be the problem.




## Scenario: Robot Loses Ball During Chase

Symptom:

- Robot starts chasing, then searches.
- Ball direction becomes stale.
- Head/camera does not keep ball visible.

Look here first:

1. `src/brain/behavior_trees/subtrees/subtree_cam_find_and_track_ball.xml`
2. `src/brain/behavior_trees/subtrees/subtree_find_ball.xml`
3. Vision topics on robot

Check on robot:

```bash
ros2 topic echo --once /booster_soccer/detection
```

Fix path:

```text
Confirm Ball detections
-> check ball position projection
-> inspect CamFindAndTrackBall
-> inspect FindBall if ball is actually lost
```

For the chase-vector task:

- Bad ball data creates bad chase vectors.
- Fix perception/tracking before tuning chase math.




## Scenario: Robot Leaves Chase Too Early

Symptom:

- Robot quickly switches from chase to adjust/kick/find.
- Chase does not run long enough to evaluate.
- Decision changes too often.

Look here first:

1. `StrikerDecide` in `src/brain/src/brain_tree.cpp`
2. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`

Check:

```xml
<StrikerDecide decision_out="{decision}" decision_in="{decision}" chase_threshold="1.0" />
```

Fix path:

```text
Log decision changes
-> check chase_threshold
-> inspect adjust/kick conditions
-> tune thresholds
```

For the chase-vector task:

- If Chase is not active, Chase math is not the first issue.
- Decision logic controls when Chase gets to run.




## Scenario: Robot Never Enters Chase

Symptom:

- Robot tracks ball but does not chase.
- Chase logs do not appear.
- Robot stays stopped, finding, adjusting, or kicking.

Look here first:

1. `src/brain/behavior_trees/game.xml`
2. `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`
3. `StrikerDecide`

Check in `game.xml`:

```xml
<SubTree ID="StrikerPlay" _while="player_role == 'striker'" />
```

Check in `subtree_striker_play.xml`:

```xml
<Chase _while="decision == 'chase'" ... />
```

Fix path:

```text
Confirm game state is PLAY
-> confirm player_role is striker
-> log decision value
-> inspect StrikerDecide
```

For the chase-vector task:

- Chase vector math only matters after Chase actually runs.




## Scenario: Obstacle Avoidance Changes Chase

Symptom:

- Robot detours strangely.
- Robot avoids too aggressively.
- Robot chases through obstacles.

Look here first:

1. `src/brain/src/brain_tree.cpp`
2. `src/brain/config/config.yaml`

Check in `Chase::tick()`:

```cpp
bool avoidObstacle = brain->config->get_avoid_during_chase();
double oaSafeDist = brain->config->get_chase_ao_safe_dist();
double distToObstacle = brain->distToObstacle(targetDir);
```

Fix path:

```text
Check avoid_during_chase
-> check chase_ao_safe_dist
-> verify obstacle data
-> tune safe distance
```

For the chase-vector task:

- Obstacle avoidance can override or alter the chase vector.
- Do not tune ball approach using bad obstacle data.




## Scenario: Robot Does Not Match Your Code Changes

Symptom:

- Local code changed, robot behavior did not.
- Robot logs look old.
- Changes work locally in theory but not on robot.

Look here first:

1. Local git branch/status.
2. Robot git branch/status.
3. Robot build status.

Fix path:

```text
Push branch
-> merge to main using team workflow
-> SSH into robot
-> pull main in ~/booster_soccer
-> rebuild on robot
-> test
```

For the chase-vector task:

- The robot runs its own checkout at `~/booster_soccer`.
- Laptop changes do not affect robot behavior until merged, pulled, and rebuilt.




## Safe Git Workflow

Before edits:

```bash
git status --short --branch
git branch --show-current
git pull origin main
```

Rules:

- Stay on your current teammate branch.
- Do not create, delete, rename, or switch branches unless the team asks.
- If there are uncommitted changes you do not understand, stop and ask.
- Commit only files related to your task.

After edits:

```bash
git status --short
git diff -- <changed-file>
```

Commit:

```bash
git add <changed-file>
git commit -m "<clear message>"
```

Push current branch:

```bash
git push origin HEAD
```

## Safe Robot Workflow

After your branch is merged into `main`, update robot:

```bash
ssh booster@192.168.68.103
cd ~/booster_soccer
deactivate 2>/dev/null || true
git switch main
git pull origin main
git submodule update --init --recursive
chmod +x scripts/*.sh
source /opt/ros/humble/setup.bash
./scripts/build.sh
```

Safety:

- Keep robot in `DAMP` for SSH/setup/topic checks.
- Do not run walking/chase/kick while robot is on the stand.
- Only test chase on the floor, balanced, in open space, with operator ready to stop.

Safe stop:

```bash
cd ~/booster_soccer
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 2 ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: stop}" || true
./scripts/stop.sh
```

## Final Rule

```text
Behavior tree decides when.
StrikerDecide decides what.
Chase::tick() decides where and how fast.
RobotClient sends the final command.
Vision provides ball data.
Robot testing proves the result.
```
