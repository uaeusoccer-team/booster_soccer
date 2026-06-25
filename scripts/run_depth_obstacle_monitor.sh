#!/usr/bin/env bash
# Run the passive depth obstacle monitor on the robot.
#
# This script only reads ROS topics and prints obstacle status. It does not
# start walking, kicking, chasing, or any behavior tree.

set -euo pipefail

cd "$(dirname "$0")/.."

deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"

python3 scripts/depth_obstacle_monitor.py "$@"
