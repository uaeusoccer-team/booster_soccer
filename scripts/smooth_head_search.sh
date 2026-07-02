#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  smooth_head_search.sh [networkInterface] [options]

Smoothly sweeps the robot head for ball search using the SDK's high-level
B1LocoClient.RotateHead(pitch, yaw) API.

Options:
  -i, --interface IFACE        DDS network interface/IP passed to ChannelFactory
      --robot-name NAME        Optional robot name passed to B1LocoClient.Init
      --min-yaw RAD            Left/right sweep minimum yaw, default -0.785
      --max-yaw RAD            Left/right sweep maximum yaw, default 0.785
      --pitches LIST           Comma-separated pitch rows, default 0.25,0.50,0.75
      --yaw-speed RAD_PER_SEC  Nominal yaw speed, default 0.55
      --pitch-speed RAD_PER_SEC
                               Nominal pitch speed, default 0.35
      --hz HZ                  RotateHead command rate, default 12
      --dwell SEC              Pause at each target, default 0.08
      --cycles N               Number of scan cycles, default 0 for forever
      --center-pitch RAD       Exit/initial center pitch, default 0.0
      --center-yaw RAD         Exit/initial center yaw, default 0.0
      --no-center              Do not return head to center when stopped
  -h, --help                   Show this help

Examples:
  ./smooth_head_search.sh eth0
  ./smooth_head_search.sh eth0 --pitches 0.35,0.60 --min-yaw -0.7 --max-yaw 0.7

From search_ball.sh:
  ./smooth_head_search.sh "$NETWORK_INTERFACE" &
  HEAD_SCAN_PID=$!
  # ...run ball detection...
  kill "$HEAD_SCAN_PID"
  wait "$HEAD_SCAN_PID" 2>/dev/null || true
EOF
}

NETWORK_INTERFACE="${ROBOT_NET_IFACE:-}"
ROBOT_NAME="${ROBOT_NAME:-}"
MIN_YAW="${HEAD_MIN_YAW:--1.0}"
MAX_YAW="${HEAD_MAX_YAW:-1.0}"
PITCHES="${HEAD_SCAN_PITCHES:-0.75,0.50,0.35}"
YAW_SPEED="${HEAD_YAW_SPEED:-0.75}"
PITCH_SPEED="${HEAD_PITCH_SPEED:-0.35}"
HZ="${HEAD_COMMAND_HZ:-25}"
DWELL="${HEAD_DWELL_SECONDS:-0.08}"
CYCLES="${HEAD_SCAN_CYCLES:-0}"
CENTER_PITCH="${HEAD_CENTER_PITCH:-0.75}"
CENTER_YAW="${HEAD_CENTER_YAW:-0.0}"
CENTER_ON_EXIT="${HEAD_CENTER_ON_EXIT:-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--interface)
      NETWORK_INTERFACE="${2:?missing value for $1}"
      shift 2
      ;;
    --robot-name)
      ROBOT_NAME="${2:?missing value for $1}"
      shift 2
      ;;
    --min-yaw)
      MIN_YAW="${2:?missing value for $1}"
      shift 2
      ;;
    --max-yaw)
      MAX_YAW="${2:?missing value for $1}"
      shift 2
      ;;
    --pitches)
      PITCHES="${2:?missing value for $1}"
      shift 2
      ;;
    --yaw-speed)
      YAW_SPEED="${2:?missing value for $1}"
      shift 2
      ;;
    --pitch-speed)
      PITCH_SPEED="${2:?missing value for $1}"
      shift 2
      ;;
    --hz)
      HZ="${2:?missing value for $1}"
      shift 2
      ;;
    --dwell)
      DWELL="${2:?missing value for $1}"
      shift 2
      ;;
    --cycles)
      CYCLES="${2:?missing value for $1}"
      shift 2
      ;;
    --center-pitch)
      CENTER_PITCH="${2:?missing value for $1}"
      shift 2
      ;;
    --center-yaw)
      CENTER_YAW="${2:?missing value for $1}"
      shift 2
      ;;
    --no-center)
      CENTER_ON_EXIT=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -z "$NETWORK_INTERFACE" ]]; then
        NETWORK_INTERFACE="$1"
        shift
      else
        echo "Unexpected argument: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
  esac
done

PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN=python3
  elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN=python
  else
    echo "Could not find python3 or python. Set PYTHON_BIN=/path/to/python." >&2
    exit 127
  fi
fi

exec "$PYTHON_BIN" - \
  "$NETWORK_INTERFACE" \
  "$ROBOT_NAME" \
  "$MIN_YAW" \
  "$MAX_YAW" \
  "$PITCHES" \
  "$YAW_SPEED" \
  "$PITCH_SPEED" \
  "$HZ" \
  "$DWELL" \
  "$CYCLES" \
  "$CENTER_PITCH" \
  "$CENTER_YAW" \
  "$CENTER_ON_EXIT" <<'PY'
import math
import signal
import sys
import time

try:
    from booster_robotics_sdk_python import B1LocoClient, ChannelFactory
except ImportError as exc:
    print(
        "Failed to import booster_robotics_sdk_python. Install the SDK package "
        "or run from an environment where the Python binding is available.",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc


def parse_float(name, value):
    try:
        return float(value)
    except ValueError as exc:
        raise SystemExit(f"{name} must be a number, got {value!r}") from exc


def parse_int(name, value):
    try:
        return int(value)
    except ValueError as exc:
        raise SystemExit(f"{name} must be an integer, got {value!r}") from exc


network_interface = sys.argv[1]
robot_name = sys.argv[2]
min_yaw = parse_float("min-yaw", sys.argv[3])
max_yaw = parse_float("max-yaw", sys.argv[4])
pitches = [
    parse_float("pitches", item.strip())
    for item in sys.argv[5].split(",")
    if item.strip()
]
yaw_speed = parse_float("yaw-speed", sys.argv[6])
pitch_speed = parse_float("pitch-speed", sys.argv[7])
hz = parse_float("hz", sys.argv[8])
dwell = parse_float("dwell", sys.argv[9])
cycles = parse_int("cycles", sys.argv[10])
center_pitch = parse_float("center-pitch", sys.argv[11])
center_yaw = parse_float("center-yaw", sys.argv[12])
center_on_exit = sys.argv[13] != "0"

if not pitches:
    raise SystemExit("At least one pitch value is required.")
if max_yaw <= min_yaw:
    raise SystemExit("max-yaw must be greater than min-yaw.")
if yaw_speed <= 0.0 or pitch_speed <= 0.0:
    raise SystemExit("yaw-speed and pitch-speed must be positive.")
if hz <= 0.0:
    raise SystemExit("hz must be positive.")
if dwell < 0.0:
    raise SystemExit("dwell must be non-negative.")
if cycles < 0:
    raise SystemExit("cycles must be zero or positive.")

stop_requested = False


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)

ChannelFactory.Instance().Init(0, network_interface)
client = B1LocoClient()
init_ret = client.Init(robot_name) if robot_name else client.Init()
if init_ret not in (None, 0):
    raise SystemExit(f"B1LocoClient.Init failed: {init_ret}")


def rotate_head(pitch, yaw):
    ret = client.RotateHead(float(pitch), float(yaw))
    if ret != 0:
        print(
            f"RotateHead failed: ret={ret}, pitch={pitch:.3f}, yaw={yaw:.3f}",
            file=sys.stderr,
        )
    return ret


def ease_in_out(t):
    return 0.5 - 0.5 * math.cos(math.pi * t)


current_pitch = center_pitch
current_yaw = center_yaw


def sleep_interruptibly(seconds):
    deadline = time.monotonic() + seconds
    while not stop_requested:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        time.sleep(min(remaining, 0.05))


def move_to(target_pitch, target_yaw):
    global current_pitch, current_yaw

    delta_pitch = target_pitch - current_pitch
    delta_yaw = target_yaw - current_yaw
    duration = max(
        abs(delta_pitch) / pitch_speed,
        abs(delta_yaw) / yaw_speed,
        1.0 / hz,
    )
    steps = max(1, int(math.ceil(duration * hz)))
    start_pitch = current_pitch
    start_yaw = current_yaw
    step_period = 1.0 / hz
    next_tick = time.monotonic()

    for step in range(1, steps + 1):
        if stop_requested:
            return
        u = ease_in_out(step / steps)
        pitch = start_pitch + delta_pitch * u
        yaw = start_yaw + delta_yaw * u
        rotate_head(pitch, yaw)
        current_pitch = pitch
        current_yaw = yaw

        next_tick += step_period
        remaining = next_tick - time.monotonic()
        if remaining > 0.0:
            sleep_interruptibly(remaining)

    if dwell > 0.0:
        sleep_interruptibly(dwell)


def scan_targets():
    direction = 1
    while True:
        for pitch in pitches:
            if direction > 0:
                yield pitch, min_yaw
                yield pitch, max_yaw
            else:
                yield pitch, max_yaw
                yield pitch, min_yaw
            direction *= -1


try:
    rotate_head(current_pitch, current_yaw)
    sleep_interruptibly(dwell)

    completed_cycles = 0
    targets_per_cycle = 2 * len(pitches)
    target_count = 0

    for pitch, yaw in scan_targets():
        if stop_requested:
            break
        move_to(pitch, yaw)
        target_count += 1
        if target_count % targets_per_cycle == 0:
            completed_cycles += 1
            if cycles and completed_cycles >= cycles:
                break
finally:
    if center_on_exit:
        stop_requested = False
        move_to(center_pitch, center_yaw)
PY
