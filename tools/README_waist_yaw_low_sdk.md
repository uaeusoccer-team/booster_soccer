# Waist Yaw Low SDK Test

`test_waist_yaw_motion.py` is only a dry-run/high-level payload checker. It does not move the robot because the robot SDK does not expose a high-level waist `LocoApiId`.

The real waist joint is low-level SDK joint index `10`:

```text
JointIndex::kWaist = 10
topic = rt/joint_ctrl
```

Use `waist_yaw_low_sdk.cpp` only on the robot, with the Booster low-level SDK available.

## Robot Prep

Stay on the correct branch first:

```bash
cd ~/booster_soccer
git merge --abort 2>/dev/null || true
git switch abdallah
git pull origin abdallah
```

The SDK example says low-level control requires:

1. Put the robot in `Prepare` mode.
2. Start the low-level test program.
3. When ready, switch the robot to `Custom` mode.

Do not do this while the robot is on the stand. Keep the area around the waist clear and be ready to stop immediately.

## Build

Build this on the robot, not on the laptop. The exact link flags depend on the robot SDK install. If the SDK CMake project is already built, the usual pattern is to add or compile this file against `~/booster_robotics_sdk`.

The source file is:

```bash
~/booster_soccer/tools/waist_yaw_low_sdk.cpp
```

## Run

Start with small angles:

```bash
./waist_yaw_low_sdk <network_interface> 10 0 -10 0
```

Only after small motion works:

```bash
./waist_yaw_low_sdk <network_interface> 50 0 -50 0 --hold 2
```

Use the robot network interface used by the SDK examples, for example `eth0` or the active robot DDS interface.
