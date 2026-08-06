# Depth obstacle avoidance

The obstacle stack keeps perception and motion ownership separate:

```text
head depth + head pose + ball detections
  -> obstacle_perception/depth_obstacle_node
  -> /booster_soccer/obstacle_state
  -> brain ObstacleVelocityFilter
  -> SetVelocity
  -> RobotClient hard-stop guard
```

`obstacle_perception` projects sampled depth into the robot frame, removes the
robot body and the depth-confirmed ball, and publishes an angular clearance
scan. It never publishes a velocity command. The behavior tree is the only
owner of directional avoidance, while `RobotClient` can only cancel unsafe
translation.

The brain's old in-process depth grid remains available as a fallback through
`obstacle_avoidance.use_external_state:=false`, but both depth implementations
must never be active together.

## Safety behavior

- Stale perception stops translation; yaw remains available so search/tracking
  can recover camera coverage.
- A command outside the camera-observed angular sector stops translation.
- Chase may select a nearby clear direction and reduce speed.
- Shooting adjustment only slows/stops; it does not detour away from alignment.
- The final `RobotClient` guard stops translation inside `hard_stop_distance`.

Primary tuning is in:

```text
src/obstacle_perception/config/obstacle_perception.yaml
src/brain/config/config.yaml
```

## Passive robot check

Keep the robot in `DAMP`. This check does not start the brain or command body
motion:

```bash
cd ~/booster_soccer
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/stop.sh
ros2 launch obstacle_perception launch.py > obstacle_perception.log 2>&1 &
sleep 3
ros2 topic echo /booster_soccer/obstacle_state --once
```

Confirm that `valid` is true, the forward direction is `observed`, and placing
a large object in front reduces its clearance. If `valid` remains false, check
`obstacle_perception.log`, `/head_pose`, depth, and camera-info topics.

Do not tune walking avoidance while the robot is on the stand. After passive
validation, use the slow chase workflow only with the robot balanced on the
floor in open space and an operator ready to run `./scripts/stop.sh`.
