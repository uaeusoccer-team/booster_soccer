#!/usr/bin/env bash
set -Eeuo pipefail

ROBOT="${ROBOT:-booster@192.168.68.105}"
ROBOT_REPO="${ROBOT_REPO:-/home/booster/booster_soccer}"
GAME_CONTROLLER_DIR="${GAME_CONTROLLER_DIR:-$HOME/GameController}"
RECEIVER="auto"
MONITOR="true"
SOFTWARE_GL="true"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/open_game_controller_ui.sh [setting=value ...]

Run this script on the development machine, not inside the robot SSH session.
It opens one reusable SSH connection, ensures the robot UDP-to-ROS receiver is
available, optionally mirrors /booster_soccer/game_controller in this terminal,
and runs the local Rust referee UI.

Settings:
  robot=booster@192.168.68.105
  robot_repo=/home/booster/booster_soccer
  game_controller_dir=~/GameController
  receiver=auto       # auto: reuse or start; existing: require one already running
  monitor=true        # print the robot ROS GameController topic beside the UI
  software_gl=true    # run the UI with LIBGL_ALWAYS_SOFTWARE=1

Examples:
  ./scripts/open_game_controller_ui.sh
  ./scripts/open_game_controller_ui.sh receiver=existing
  ./scripts/open_game_controller_ui.sh game_controller_dir=/path/to/GameController monitor=false

The script does not store passwords and does not stop the robot receiver when
the UI closes. A chase/adjust script that started the receiver keeps ownership
of it. If this script starts the receiver, it remains available until the robot
stack is stopped normally.
USAGE
}

normalize_bool() {
  local key="$1"
  local value="${2,,}"
  case "$value" in
    true|1) printf 'true' ;;
    false|0) printf 'false' ;;
    *) echo "Invalid boolean for ${key}: $2" >&2; exit 2 ;;
  esac
}

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    robot=*) ROBOT="${1#*=}" ;;
    robot_repo=*) ROBOT_REPO="${1#*=}" ;;
    game_controller_dir=*) GAME_CONTROLLER_DIR="${1#*=}" ;;
    receiver=*) RECEIVER="${1#*=}" ;;
    monitor=*) MONITOR="$(normalize_bool monitor "${1#*=}")" ;;
    software_gl=*) SOFTWARE_GL="$(normalize_bool software_gl "${1#*=}")" ;;
    *)
      echo "Unknown setting: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

case "$RECEIVER" in
  auto|existing) ;;
  *) echo "Invalid receiver mode: ${RECEIVER} (expected auto or existing)" >&2; exit 2 ;;
esac

if [[ ! -d "$GAME_CONTROLLER_DIR" || ! -f "$GAME_CONTROLLER_DIR/Cargo.toml" ]]; then
  echo "GameController checkout not found at: ${GAME_CONTROLLER_DIR}" >&2
  echo "Set game_controller_dir=/path/to/GameController." >&2
  exit 1
fi

if ! command -v cargo >/dev/null 2>&1; then
  if [[ -f "$HOME/.cargo/env" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/.cargo/env"
  fi
fi

if ! command -v cargo >/dev/null 2>&1; then
  echo "cargo is not available. Install Rust or source \$HOME/.cargo/env." >&2
  exit 1
fi

TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/booster-game-controller.XXXXXX")"
CONTROL_PATH="${TMP_DIR}/ssh-control"
MONITOR_PID=""
MASTER_OPEN="false"

cleanup() {
  trap - EXIT HUP INT TERM

  if [[ -n "$MONITOR_PID" ]]; then
    kill "$MONITOR_PID" >/dev/null 2>&1 || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi

  if [[ "$MASTER_OPEN" == "true" ]]; then
    ssh -o ControlPath="$CONTROL_PATH" -O exit "$ROBOT" >/dev/null 2>&1 || true
  fi

  rm -rf "$TMP_DIR"
}
trap cleanup EXIT HUP INT TERM

echo "Connecting to ${ROBOT}. SSH may request its password once."
ssh \
  -o ControlMaster=yes \
  -o ControlPath="$CONTROL_PATH" \
  -o ControlPersist=60 \
  -MNf \
  "$ROBOT"
MASTER_OPEN="true"

SSH=(ssh -o ControlMaster=auto -o ControlPath="$CONTROL_PATH")

receiver_result="$(
  "${SSH[@]}" "$ROBOT" bash -s -- "$ROBOT_REPO" "$RECEIVER" <<'REMOTE'
set -Eeuo pipefail

repo="$1"
receiver_mode="$2"

if [[ "$repo" == "~/"* ]]; then
  repo="$HOME/${repo:2}"
fi

cd "$repo"
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"

if pgrep -f '[g]ame_controller_node' >/dev/null 2>&1; then
  printf 'reused'
  exit 0
fi

if [[ "$receiver_mode" == "existing" ]]; then
  echo "No robot game_controller receiver is running." >&2
  exit 3
fi

nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 </dev/null &
for _ in 1 2 3 4 5; do
  sleep 1
  if pgrep -f '[g]ame_controller_node' >/dev/null 2>&1; then
    printf 'started'
    exit 0
  fi
done

echo "The robot game_controller receiver failed to start:" >&2
tail -n 40 game_controller.log >&2 || true
exit 4
REMOTE
)"

echo "Robot UDP-to-ROS receiver: ${receiver_result}"
echo "The receiver listens on UDP 0.0.0.0:3838 and publishes /booster_soccer/game_controller."

if [[ "$MONITOR" == "true" ]]; then
  echo "Robot ROS monitor enabled. Incoming referee messages will appear below."
  (
    "${SSH[@]}" "$ROBOT" bash -s -- "$ROBOT_REPO" <<'REMOTE'
set -Eeuo pipefail
repo="$1"
if [[ "$repo" == "~/"* ]]; then
  repo="$HOME/${repo:2}"
fi
cd "$repo"
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
exec ros2 topic echo /booster_soccer/game_controller
REMOTE
  ) 2>&1 | while IFS= read -r line; do
    printf '[robot GameController] %s\n' "$line"
  done &
  MONITOR_PID="$!"
fi

echo "Opening referee UI from ${GAME_CONTROLLER_DIR}."
echo "The development machine and robot must be on the same LAN, with UDP broadcast port 3838 allowed."

cd "$GAME_CONTROLLER_DIR"
if [[ "$SOFTWARE_GL" == "true" ]]; then
  LIBGL_ALWAYS_SOFTWARE=1 cargo run
else
  cargo run
fi
