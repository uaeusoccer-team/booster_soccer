#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

SWITCH_MODE="manual"
START_MODE="track"
REQUIRE_PLAY="false"
ROLE="striker"
TEAM_ID="5"
PLAYER_ID="1"

VX_LIMIT="0.60"
VY_LIMIT="0.20"
STOP_DIST="1.00"
Y_TOLERANCE="0.03"

STOP_ANGLE="0.10"
BALL_YAW_GAIN="4.0"
PITCH_TURN_GAIN="1.0"
HEAD_SEARCH_SPEED="0.20"
BODY_SEARCH_SPEED="0.25"
YAW_LIMIT="1.10"
CMD_INTERVAL_MSEC="100"
HEAD_STEP_RAD="0.04"
HEAD_SETTLE_STEP_RAD="0.02"
HEAD_DEADBAND_X_PX="35"
HEAD_DEADBAND_Y_PX="35"

ADJUST_TARGET_RANGE="0.40"
ADJUST_TARGET_Y_OFFSET="0.00"
ADJUST_THETA_OFFSET="0.00"
ADJUST_RANGE_TOLERANCE="0.06"
ADJUST_Y_TOLERANCE="0.05"
ADJUST_STOP_ANGLE="0.10"
ADJUST_RANGE_GAIN="1.0"
ADJUST_Y_GAIN="1.0"
ADJUST_BALL_YAW_GAIN="4.0"
ADJUST_VX_LIMIT="0.40"
ADJUST_VY_LIMIT="0.40"
ADJUST_VTHETA_LIMIT="0.80"
ADJUST_TURN_FIRST_THRESHOLD="0.50"
ADJUST_FIXED_HEAD_YAW="0.00"
ADJUST_MAX_BALL_RANGE="1.20"

READY_SAMPLES="10"
READY_MIN_MSEC="1000"
READY_DATA_MAX_AGE_MSEC="500"
SHOOT_HOST="127.0.0.1"
SHOOT_PORT="6868"

DRY_RUN="false"
STOPPING="false"

RUNTIME_SWITCH="$SWITCH_MODE"
RUNTIME_MANUAL_MODE="$START_MODE"
RUNTIME_AUTO_PHASE="$START_MODE"
ACTIVE_BEHAVIOR="starting"
BALL_VISIBLE="unknown"
BALL_DEPTH_USABLE="unknown"
LAST_BALL_RANGE="unknown"
LAST_VX="0.0"
LAST_VY="0.0"
LAST_THETA="0.0"
LAST_ADJUST_SOURCE="none"

READY_COUNT=0
READY_START_MSEC=0
LAST_READY_EVIDENCE_MSEC=0
SHOOT_READY="false"
READY_PROMPTED="false"
CHASE_STOP_PROMPTED="false"
MONITOR_LINE=0

VISION_PID=""
BRAIN_PID=""
GAME_CONTROLLER_PID=""

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/Autonomous_run.sh [setting=value ...]

One holistic controller combines the proven tracking/search, vector chase, and
shooting-adjustment behavior-tree logic. It does not launch or switch between
the three older test scripts.

Control:
  switch=manual                       # auto | manual
  start_mode=track                    # track | chase | adjust
  require_play=false                  # require GameController PLAY
  role=striker                        # striker | goal_keeper
  team_id=5
  player_id=1

Tracking and rotation:
  stop_angle=0.10
  ball_yaw_gain=4.0
  pitch_turn_gain=1.0
  head_search_speed=0.20
  body_search_speed=0.25
  yaw_limit=1.10
  cmd_interval_msec=100
  head_step_rad=0.04
  head_settle_step_rad=0.02
  head_deadband_x_px=35
  head_deadband_y_px=35

Chase:
  vx_limit=0.60
  vy_limit=0.20
  stop_dist=1.00
  y_tolerance=0.03

Shooting adjustment:
  adjust_target_range=0.40
  adjust_target_y_offset=0.00
  adjust_theta_offset=0.00
  adjust_range_tolerance=0.06
  adjust_y_tolerance=0.05
  adjust_stop_angle=0.10
  adjust_range_gain=1.0
  adjust_y_gain=1.0
  adjust_ball_yaw_gain=4.0
  adjust_vx_limit=0.40
  adjust_vy_limit=0.40
  adjust_vtheta_limit=0.80
  adjust_turn_first_threshold=0.50
  adjust_fixed_head_yaw=0.00
  adjust_max_ball_range=1.20

Shoot readiness:
  ready_samples=10
  ready_min_msec=1000
  ready_data_max_age_msec=500
  shoot_host=127.0.0.1
  shoot_port=6868

Utility:
  --help                               # show this help
  --dry-run                            # validate and print settings; do not run ROS

Runtime commands:
  auto      Enable automatic chase/adjust transitions.
  manual    Disable automatic transitions and preserve the saved phase.
  track     Select manual tracking/search/rotation (vx=0, vy=0).
  chase     Select manual tracking plus SimpleChase.
  adjust    Select manual ShootingAdjust.
  status    Show selected, active, perception, velocity, and process state.
  help      Show runtime commands.
  shoot     Shoot only after a fresh READY TO SHOOT state.
  stop      Send a controlled stop and exit.

Ball loss is a temporary fallback. RGB-only detections use CamTrackBall with
zero translation; a missing ball uses CamFindBall. Valid depth restores the
same selected manual mode or saved automatic phase. In automatic mode, normal
range rules are then applied.

Automatic transitions:
  chase -> adjust when ball range <= stop_dist
  adjust -> chase when ball range > adjust_max_ball_range

Example:
  ./scripts/Autonomous_run.sh \
    switch=auto \
    start_mode=chase \
    vx_limit=0.18 \
    vy_limit=0.06 \
    stop_dist=1.20 \
    y_tolerance=0.05 \
    stop_angle=0.12 \
    ball_yaw_gain=3.0 \
    pitch_turn_gain=1.0 \
    require_play=false \
    head_search_speed=0.15 \
    body_search_speed=0.20 \
    yaw_limit=1.10 \
    cmd_interval_msec=100 \
    head_step_rad=0.04 \
    head_settle_step_rad=0.01 \
    head_deadband_x_px=35 \
    head_deadband_y_px=50 \
    adjust_target_range=1.70 \
    adjust_target_y_offset=0.00 \
    adjust_theta_offset=0.00 \
    adjust_range_tolerance=0.06 \
    adjust_y_tolerance=0.05 \
    adjust_stop_angle=0.10 \
    adjust_range_gain=1.0 \
    adjust_y_gain=1.0 \
    adjust_ball_yaw_gain=4.0 \
    adjust_vx_limit=0.40 \
    adjust_vy_limit=0.40 \
    adjust_vtheta_limit=0.80 \
    adjust_turn_first_threshold=0.50 \
    adjust_fixed_head_yaw=0.00 \
    adjust_max_ball_range=2.20 \
    ready_samples=10 \
    ready_min_msec=1000

Run body-moving modes only with the robot off the stand, balanced on the floor,
in open space, and with the operator ready to use the safe stop.
USAGE
}

runtime_help() {
  cat <<'HELP'
Runtime commands:
  auto    manual    track    chase    adjust
  status  help      shoot    stop

Typing track, chase, or adjust selects that manual mode immediately. The exact
lowercase word "shoot" is accepted only while fresh adjustment readiness is
true. No automatic event can call the shoot command.
HELP
}

clean_value() {
  local value="$1"
  value="${value//\"/}"
  value="${value//\'/}"
  value="${value%,}"
  value="${value%;}"
  printf '%s' "$value"
}

normalize_bool() {
  local key="$1"
  local value="$2"
  case "$value" in
    [Tt][Rr][Uu][Ee]|1) printf 'true' ;;
    [Ff][Aa][Ll][Ss][Ee]|0) printf 'false' ;;
    *) echo "Invalid boolean for ${key}: ${2}" >&2; exit 2 ;;
  esac
}

is_signed_number() {
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]]
}

is_nonnegative_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]]
}

is_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

set_value() {
  local key="$1"
  local value
  value="$(clean_value "$2")"

  case "$key" in
    switch)
      [[ "$value" == "auto" || "$value" == "manual" ]] ||
        { echo "Invalid switch: ${value}; expected auto or manual" >&2; exit 2; }
      SWITCH_MODE="$value"
      return
      ;;
    start_mode)
      [[ "$value" == "track" || "$value" == "chase" || "$value" == "adjust" ]] ||
        { echo "Invalid start_mode: ${value}; expected track, chase, or adjust" >&2; exit 2; }
      START_MODE="$value"
      return
      ;;
    require_play)
      REQUIRE_PLAY="$(normalize_bool "$key" "$value")"
      return
      ;;
    role)
      [[ "$value" == "striker" || "$value" == "goal_keeper" ]] ||
        { echo "Invalid role: ${value}; expected striker or goal_keeper" >&2; exit 2; }
      ROLE="$value"
      return
      ;;
    team_id|player_id|cmd_interval_msec|head_deadband_x_px|head_deadband_y_px|ready_samples|ready_min_msec|ready_data_max_age_msec|shoot_port)
      is_positive_integer "$value" ||
        { echo "Invalid positive integer for ${key}: ${value}" >&2; exit 2; }
      ;;
    adjust_target_y_offset|adjust_theta_offset|adjust_fixed_head_yaw)
      is_signed_number "$value" ||
        { echo "Invalid signed number for ${key}: ${value}" >&2; exit 2; }
      ;;
    *)
      is_nonnegative_number "$value" ||
        { echo "Invalid nonnegative number for ${key}: ${value}" >&2; exit 2; }
      ;;
  esac

  case "$key" in
    team_id) TEAM_ID="$value" ;;
    player_id) PLAYER_ID="$value" ;;
    vx_limit) VX_LIMIT="$value" ;;
    vy_limit) VY_LIMIT="$value" ;;
    stop_dist) STOP_DIST="$value" ;;
    y_tolerance) Y_TOLERANCE="$value" ;;
    stop_angle) STOP_ANGLE="$value" ;;
    ball_yaw_gain) BALL_YAW_GAIN="$value" ;;
    pitch_turn_gain) PITCH_TURN_GAIN="$value" ;;
    head_search_speed) HEAD_SEARCH_SPEED="$value" ;;
    body_search_speed) BODY_SEARCH_SPEED="$value" ;;
    yaw_limit) YAW_LIMIT="$value" ;;
    cmd_interval_msec) CMD_INTERVAL_MSEC="$value" ;;
    head_step_rad) HEAD_STEP_RAD="$value" ;;
    head_settle_step_rad) HEAD_SETTLE_STEP_RAD="$value" ;;
    head_deadband_x_px) HEAD_DEADBAND_X_PX="$value" ;;
    head_deadband_y_px) HEAD_DEADBAND_Y_PX="$value" ;;
    adjust_target_range) ADJUST_TARGET_RANGE="$value" ;;
    adjust_target_y_offset) ADJUST_TARGET_Y_OFFSET="$value" ;;
    adjust_theta_offset) ADJUST_THETA_OFFSET="$value" ;;
    adjust_range_tolerance) ADJUST_RANGE_TOLERANCE="$value" ;;
    adjust_y_tolerance) ADJUST_Y_TOLERANCE="$value" ;;
    adjust_stop_angle) ADJUST_STOP_ANGLE="$value" ;;
    adjust_range_gain) ADJUST_RANGE_GAIN="$value" ;;
    adjust_y_gain) ADJUST_Y_GAIN="$value" ;;
    adjust_ball_yaw_gain) ADJUST_BALL_YAW_GAIN="$value" ;;
    adjust_vx_limit) ADJUST_VX_LIMIT="$value" ;;
    adjust_vy_limit) ADJUST_VY_LIMIT="$value" ;;
    adjust_vtheta_limit) ADJUST_VTHETA_LIMIT="$value" ;;
    adjust_turn_first_threshold) ADJUST_TURN_FIRST_THRESHOLD="$value" ;;
    adjust_fixed_head_yaw) ADJUST_FIXED_HEAD_YAW="$value" ;;
    adjust_max_ball_range) ADJUST_MAX_BALL_RANGE="$value" ;;
    ready_samples) READY_SAMPLES="$value" ;;
    ready_min_msec) READY_MIN_MSEC="$value" ;;
    ready_data_max_age_msec) READY_DATA_MAX_AGE_MSEC="$value" ;;
    shoot_port) SHOOT_PORT="$value" ;;
    *) echo "Unknown setting: ${key}" >&2; exit 2 ;;
  esac
}

parse_args() {
  while (($#)); do
    case "$1" in
      -h|--help)
        usage
        exit 0
        ;;
      --dry-run)
        DRY_RUN="true"
        shift
        ;;
      shoot_host=*)
        SHOOT_HOST="$(clean_value "${1#*=}")"
        [[ -n "$SHOOT_HOST" ]] || { echo "shoot_host cannot be empty" >&2; exit 2; }
        shift
        ;;
      *=*)
        set_value "${1%%=*}" "${1#*=}"
        shift
        ;;
      *)
        echo "Unknown argument: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done
}

float_lt() {
  awk -v left="$1" -v right="$2" 'BEGIN { exit !(left < right) }'
}

float_le() {
  awk -v left="$1" -v right="$2" 'BEGIN { exit !(left <= right) }'
}

validate_settings() {
  float_lt "$STOP_DIST" "$ADJUST_MAX_BALL_RANGE" || {
    echo "Invalid automatic hysteresis: stop_dist (${STOP_DIST}) must be less than adjust_max_ball_range (${ADJUST_MAX_BALL_RANGE})." >&2
    exit 2
  }

  float_le "$ADJUST_TARGET_RANGE" "$ADJUST_MAX_BALL_RANGE" || {
    echo "Invalid adjustment target: adjust_target_range (${ADJUST_TARGET_RANGE}) must not exceed adjust_max_ball_range (${ADJUST_MAX_BALL_RANGE})." >&2
    exit 2
  }

  ((SHOOT_PORT >= 1 && SHOOT_PORT <= 65535)) || {
    echo "Invalid shoot_port: ${SHOOT_PORT}; expected 1-65535" >&2
    exit 2
  }
}

print_settings() {
  cat <<SETTINGS
Autonomous run settings:
  switch=${SWITCH_MODE}
  start_mode=${START_MODE}
  require_play=${REQUIRE_PLAY}
  role=${ROLE}
  team_id=${TEAM_ID}
  player_id=${PLAYER_ID}

  vx_limit=${VX_LIMIT}
  vy_limit=${VY_LIMIT}
  stop_dist=${STOP_DIST}
  y_tolerance=${Y_TOLERANCE}

  stop_angle=${STOP_ANGLE}
  ball_yaw_gain=${BALL_YAW_GAIN}
  pitch_turn_gain=${PITCH_TURN_GAIN}
  head_search_speed=${HEAD_SEARCH_SPEED}
  body_search_speed=${BODY_SEARCH_SPEED}
  yaw_limit=${YAW_LIMIT}
  cmd_interval_msec=${CMD_INTERVAL_MSEC}
  head_step_rad=${HEAD_STEP_RAD}
  head_settle_step_rad=${HEAD_SETTLE_STEP_RAD}
  head_deadband_x_px=${HEAD_DEADBAND_X_PX}
  head_deadband_y_px=${HEAD_DEADBAND_Y_PX}

  adjust_target_range=${ADJUST_TARGET_RANGE}
  adjust_target_y_offset=${ADJUST_TARGET_Y_OFFSET}
  adjust_theta_offset=${ADJUST_THETA_OFFSET}
  adjust_range_tolerance=${ADJUST_RANGE_TOLERANCE}
  adjust_y_tolerance=${ADJUST_Y_TOLERANCE}
  adjust_stop_angle=${ADJUST_STOP_ANGLE}
  adjust_range_gain=${ADJUST_RANGE_GAIN}
  adjust_y_gain=${ADJUST_Y_GAIN}
  adjust_ball_yaw_gain=${ADJUST_BALL_YAW_GAIN}
  adjust_vx_limit=${ADJUST_VX_LIMIT}
  adjust_vy_limit=${ADJUST_VY_LIMIT}
  adjust_vtheta_limit=${ADJUST_VTHETA_LIMIT}
  adjust_turn_first_threshold=${ADJUST_TURN_FIRST_THRESHOLD}
  adjust_fixed_head_yaw=${ADJUST_FIXED_HEAD_YAW}
  adjust_max_ball_range=${ADJUST_MAX_BALL_RANGE}

  ready_samples=${READY_SAMPLES}
  ready_min_msec=${READY_MIN_MSEC}
  ready_data_max_age_msec=${READY_DATA_MAX_AGE_MSEC}
  shoot_host=${SHOOT_HOST}
  shoot_port=${SHOOT_PORT}
SETTINGS

  if float_lt "$STOP_DIST" "$ADJUST_TARGET_RANGE"; then
    echo
    echo "Note: adjust_target_range is farther than stop_dist; adjustment may back the robot away after chase stops."
  fi
}

now_msec() {
  local value
  value="$(date +%s%3N)"
  if [[ "$value" =~ ^[0-9]+$ ]]; then
    printf '%s\n' "$value"
  else
    python3 -c 'import time; print(time.time_ns() // 1_000_000)'
  fi
}

send_agent_command() {
  local command="$1"
  timeout 2 ros2 topic pub --once /booster_agent/soccer_game_control \
    std_msgs/msg/String "{data: ${command}}" >/dev/null 2>&1
}

send_game_stop() {
  local attempt
  for attempt in 1 2 3; do
    send_agent_command autonomy_stop || true
    send_agent_command stop || true
    sleep 0.15
  done
}

controlled_stop() {
  local reason="${1:-manual}"
  local exit_code="${2:-0}"

  if [[ "$STOPPING" == "true" ]]; then
    exit "$exit_code"
  fi
  STOPPING="true"
  trap - ERR HUP INT TERM

  echo
  echo "---- controlled stop requested: ${reason} ----"
  echo "Sending zero/stop while the brain is still alive..."
  send_game_stop
  sleep 0.5
  ./scripts/stop.sh || true
  echo "Stopped."
  exit "$exit_code"
}

reset_readiness() {
  if [[ "$SHOOT_READY" == "true" ]]; then
    echo
    echo "Shoot readiness revoked: the ball pose or active behavior changed."
  fi
  READY_COUNT=0
  READY_START_MSEC=0
  LAST_READY_EVIDENCE_MSEC=0
  SHOOT_READY="false"
  READY_PROMPTED="false"
}

evaluate_adjust_readiness() {
  local range_error="$1"
  local y_error="$2"
  local theta_error="$3"
  local vx="$4"
  local vy="$5"
  local theta="$6"
  local source="$7"
  local now

  if awk \
    -v range_error="$range_error" \
    -v range_tolerance="$ADJUST_RANGE_TOLERANCE" \
    -v y_error="$y_error" \
    -v y_tolerance="$ADJUST_Y_TOLERANCE" \
    -v theta_error="$theta_error" \
    -v theta_tolerance="$ADJUST_STOP_ANGLE" \
    -v vx="$vx" \
    -v vy="$vy" \
    -v theta="$theta" \
    -v source="$source" '
      function abs(v) { return v < 0 ? -v : v }
      BEGIN {
        valid_source = source != "NO_DEPTH" && source != "OUT_OF_RANGE"
        ready = valid_source &&
                abs(range_error) <= range_tolerance &&
                abs(y_error) <= y_tolerance &&
                abs(theta_error) <= theta_tolerance &&
                abs(vx) <= 0.0005 &&
                abs(vy) <= 0.0005 &&
                abs(theta) <= 0.0005
        exit !ready
      }
    '; then
    now="$(now_msec)"
    if ((READY_COUNT == 0)); then
      READY_START_MSEC="$now"
    fi
    READY_COUNT=$((READY_COUNT + 1))
    LAST_READY_EVIDENCE_MSEC="$now"

    if ((READY_COUNT >= READY_SAMPLES && now - READY_START_MSEC >= READY_MIN_MSEC)); then
      SHOOT_READY="true"
      if [[ "$READY_PROMPTED" != "true" ]]; then
        READY_PROMPTED="true"
        echo
        echo "========================================"
        echo "READY TO SHOOT"
        echo "Type shoot to fire."
        echo "Type chase, adjust, track, auto, or stop."
        echo "========================================"
      fi
    fi
  else
    reset_readiness
  fi
}

process_log_line() {
  local line="$1"
  local number_re='[-+]?[0-9]*[.]?[0-9]+'
  local pattern

  pattern='Autonomy switch => auto [(]phase: (track|chase|adjust)[)]'
  if [[ "$line" =~ $pattern ]]; then
    RUNTIME_SWITCH="auto"
    RUNTIME_AUTO_PHASE="${BASH_REMATCH[1]}"
  else
    pattern='Autonomy saved phase => (track|chase|adjust)'
    if [[ "$line" =~ $pattern ]]; then
      RUNTIME_AUTO_PHASE="${BASH_REMATCH[1]}"
    else
      pattern='Autonomy switch => manual [(]mode: (track|chase|adjust)[)]'
      if [[ "$line" =~ $pattern ]]; then
        RUNTIME_SWITCH="manual"
        RUNTIME_MANUAL_MODE="${BASH_REMATCH[1]}"
      else
        pattern='Autonomy manual mode => (track|chase|adjust)'
        if [[ "$line" =~ $pattern ]]; then
          RUNTIME_SWITCH="manual"
          RUNTIME_MANUAL_MODE="${BASH_REMATCH[1]}"
        fi
      fi
    fi
  fi

  if [[ "$line" == *"CamFindBall/"* ]]; then
    ACTIVE_BEHAVIOR="search"
    BALL_VISIBLE="false"
    BALL_DEPTH_USABLE="false"
    reset_readiness
  elif [[ "$line" == *"CamTrackBall/direct_pixel"* ]]; then
    ACTIVE_BEHAVIOR="track"
    BALL_VISIBLE="true"
    BALL_DEPTH_USABLE="false"
    reset_readiness
  elif [[ "$line" == *"SimpleChase/vector"* ]]; then
    ACTIVE_BEHAVIOR="chase"
    BALL_VISIBLE="true"
    BALL_DEPTH_USABLE="true"
    reset_readiness
    if [[ "$RUNTIME_SWITCH" == "auto" ]]; then
      RUNTIME_AUTO_PHASE="chase"
    fi

    pattern="range: (${number_re}).*vx: (${number_re}).*vy: (${number_re}).*source: ([A-Z_]+)"
    if [[ "$line" =~ $pattern ]]; then
      LAST_BALL_RANGE="${BASH_REMATCH[1]}"
      LAST_VX="${BASH_REMATCH[2]}"
      LAST_VY="${BASH_REMATCH[3]}"
      if [[ "${BASH_REMATCH[4]}" == "STOP_DIST" &&
            "$RUNTIME_SWITCH" == "manual" &&
            "$RUNTIME_MANUAL_MODE" == "chase" ]]; then
        if [[ "$CHASE_STOP_PROMPTED" != "true" ]]; then
          CHASE_STOP_PROMPTED="true"
          echo
          echo "CHASE STOPPED AT stop_dist."
          echo "Type adjust to begin shooting-position adjustment."
        fi
      else
        CHASE_STOP_PROMPTED="false"
      fi
    fi
  elif [[ "$line" == *"ShootingAdjust/vector"* ]]; then
    ACTIVE_BEHAVIOR="adjust"
    BALL_VISIBLE="true"
    BALL_DEPTH_USABLE="true"
    CHASE_STOP_PROMPTED="false"
    if [[ "$RUNTIME_SWITCH" == "auto" ]]; then
      RUNTIME_AUTO_PHASE="adjust"
    fi

    pattern="ballRange: (${number_re}).*rangeError: (${number_re}).*yError: (${number_re}).*thetaError: (${number_re}).*vx: (${number_re}).*vy: (${number_re}).*theta: (${number_re}).*source: ([A-Z_]+)"
    if [[ "$line" =~ $pattern ]]; then
      LAST_BALL_RANGE="${BASH_REMATCH[1]}"
      LAST_VX="${BASH_REMATCH[5]}"
      LAST_VY="${BASH_REMATCH[6]}"
      LAST_THETA="${BASH_REMATCH[7]}"
      LAST_ADJUST_SOURCE="${BASH_REMATCH[8]}"
      evaluate_adjust_readiness \
        "${BASH_REMATCH[2]}" \
        "${BASH_REMATCH[3]}" \
        "${BASH_REMATCH[4]}" \
        "${BASH_REMATCH[5]}" \
        "${BASH_REMATCH[6]}" \
        "${BASH_REMATCH[7]}" \
        "${BASH_REMATCH[8]}"
    else
      reset_readiness
    fi
  elif [[ "$line" == *"RobotClient/setVelocity_out"* ]]; then
    pattern="vx: (${number_re}).*vy: (${number_re}).*vtheta: (${number_re})"
    if [[ "$line" =~ $pattern ]]; then
      LAST_VX="${BASH_REMATCH[1]}"
      LAST_VY="${BASH_REMATCH[2]}"
      LAST_THETA="${BASH_REMATCH[3]}"
    fi
  fi
}

monitor_brain_log() {
  local total_lines
  local first_line

  [[ -f brain.log ]] || return 0
  total_lines="$(wc -l < brain.log | tr -d ' ')"
  [[ "$total_lines" =~ ^[0-9]+$ ]] || return 0
  ((total_lines > MONITOR_LINE)) || return 0

  first_line=$((MONITOR_LINE + 1))
  while IFS= read -r line; do
    process_log_line "$line"
  done < <(sed -n "${first_line},${total_lines}p" brain.log)
  MONITOR_LINE="$total_lines"
}

expire_readiness() {
  local now
  [[ "$SHOOT_READY" == "true" ]] || return 0
  now="$(now_msec)"
  if ((now - LAST_READY_EVIDENCE_MSEC > READY_DATA_MAX_AGE_MSEC)); then
    reset_readiness
  fi
}

process_state() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    printf 'running'
  else
    printf 'stopped'
  fi
}

show_status() {
  local resume_mode
  if [[ "$RUNTIME_SWITCH" == "auto" ]]; then
    resume_mode="$RUNTIME_AUTO_PHASE"
  else
    resume_mode="$RUNTIME_MANUAL_MODE"
  fi

  cat <<STATUS
Autonomous status:
  switch: ${RUNTIME_SWITCH}
  manual mode: ${RUNTIME_MANUAL_MODE}
  automatic phase: ${RUNTIME_AUTO_PHASE}
  active behavior: ${ACTIVE_BEHAVIOR}
  resume after ball loss: ${resume_mode}
  ball visible: ${BALL_VISIBLE}
  depth usable: ${BALL_DEPTH_USABLE}
  last ball range: ${LAST_BALL_RANGE}
  last adjustment source: ${LAST_ADJUST_SOURCE}
  last velocity: vx=${LAST_VX} vy=${LAST_VY} theta=${LAST_THETA}
  shoot ready: ${SHOOT_READY}
  vision: $(process_state "$VISION_PID")
  brain: $(process_state "$BRAIN_PID")
  game controller: $(process_state "$GAME_CONTROLLER_PID")
STATUS
}

publish_runtime_command() {
  local command="$1"
  if ! send_agent_command "$command"; then
    echo "Failed to publish ${command}; keeping the previous selection." >&2
    return 1
  fi
}

select_manual_mode() {
  local mode="$1"
  publish_runtime_command "autonomy_${mode}" || return 0
  RUNTIME_SWITCH="manual"
  RUNTIME_MANUAL_MODE="$mode"
  ACTIVE_BEHAVIOR="${mode} pending"
  CHASE_STOP_PROMPTED="false"
  reset_readiness
  echo "Manual mode selected: ${mode}"
}

select_manual_switch() {
  publish_runtime_command autonomy_manual || return 0
  if [[ "$RUNTIME_SWITCH" == "auto" ]]; then
    RUNTIME_MANUAL_MODE="$RUNTIME_AUTO_PHASE"
  fi
  RUNTIME_SWITCH="manual"
  ACTIVE_BEHAVIOR="${RUNTIME_MANUAL_MODE} pending"
  reset_readiness
  echo "Automatic transitions disabled; manual mode: ${RUNTIME_MANUAL_MODE}"
}

select_auto_switch() {
  publish_runtime_command autonomy_auto || return 0
  if [[ "$RUNTIME_MANUAL_MODE" == "chase" || "$RUNTIME_MANUAL_MODE" == "adjust" ]]; then
    RUNTIME_AUTO_PHASE="$RUNTIME_MANUAL_MODE"
  fi
  RUNTIME_SWITCH="auto"
  ACTIVE_BEHAVIOR="${RUNTIME_AUTO_PHASE} pending"
  reset_readiness
  echo "Automatic transitions enabled; saved phase: ${RUNTIME_AUTO_PHASE}"
}

shoot_is_fresh() {
  local now
  monitor_brain_log
  expire_readiness
  [[ "$SHOOT_READY" == "true" ]] || return 1
  [[ "$ACTIVE_BEHAVIOR" == "adjust" ]] || return 1
  now="$(now_msec)"
  ((now - LAST_READY_EVIDENCE_MSEC <= READY_DATA_MAX_AGE_MSEC))
}

send_shoot() {
  local shoot_result

  if ! shoot_is_fresh; then
    echo "Shoot rejected: adjustment readiness is not currently valid and fresh."
    return
  fi

  STOPPING="true"
  trap - ERR HUP INT TERM
  echo "Fresh readiness confirmed. Stopping this brain's motion publisher..."
  send_game_stop
  sleep 0.5
  ./scripts/stop.sh || true

  echo "Sending the operator-approved shoot command once..."
  set +e
  python3 -c 'import socket,struct,sys; p=b"{\"request\":\"send_to_agent\",\"app_api_level\":130,\"params\":{\"agent_id\":\"com.boosterobotics.default\"},\"agent_req\":{\"event\":\"on_component_click\",\"component_id\":\"shoot\",\"state\":0},\"app_platform\":\"iOS\"}"; print("payload length:",len(p)); s=socket.create_connection((sys.argv[1],int(sys.argv[2])),3); s.sendall(struct.pack("<I",len(p))+p); print("sent"); s.close()' "$SHOOT_HOST" "$SHOOT_PORT"
  shoot_result=$?
  set -e

  if ((shoot_result == 0)); then
    echo "Shoot command sent once. Autonomous_run is exiting."
  else
    echo "Shoot command failed with exit code ${shoot_result}; it was not retried." >&2
  fi
  exit "$shoot_result"
}

check_processes() {
  if ! kill -0 "$BRAIN_PID" 2>/dev/null; then
    controlled_stop "brain process exited unexpectedly" 1
  fi
  if ! kill -0 "$VISION_PID" 2>/dev/null; then
    controlled_stop "vision process exited unexpectedly" 1
  fi
  if ! kill -0 "$GAME_CONTROLLER_PID" 2>/dev/null; then
    controlled_stop "GameController receiver exited unexpectedly" 1
  fi
}

handle_runtime_command() {
  local command="$1"
  command="${command#"${command%%[![:space:]]*}"}"
  command="${command%"${command##*[![:space:]]}"}"

  case "$command" in
    auto) select_auto_switch ;;
    manual) select_manual_switch ;;
    track|chase|adjust) select_manual_mode "$command" ;;
    status) show_status ;;
    help) runtime_help ;;
    shoot) send_shoot ;;
    stop) controlled_stop "operator stop" ;;
    "") ;;
    *) echo "Unknown runtime command: ${command}. Type help for commands." ;;
  esac
}

write_tree() {
  local tree_path="$1"
  local stop_condition
  local run_condition

  if [[ "$REQUIRE_PLAY" == "true" ]]; then
    stop_condition="!autonomy_enabled || gc_game_state!='PLAY'"
    run_condition="autonomy_enabled &amp;&amp; gc_game_state=='PLAY'"
  else
    stop_condition="!autonomy_enabled || gc_game_state=='END'"
    run_condition="autonomy_enabled &amp;&amp; gc_game_state!='END'"
  fi

  cat > "$tree_path" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="TrackVisible">
    <Sequence>
      <CamTrackBall stop_angle="${STOP_ANGLE}"
                    ball_yaw_gain="${BALL_YAW_GAIN}"
                    pitch_turn_gain="${PITCH_TURN_GAIN}"
                    head_step_rad="${HEAD_STEP_RAD}"
                    head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                    head_deadband_x_px="${HEAD_DEADBAND_X_PX}"
                    head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                    theta="{tracking_theta}" />
      <Script code="autonomy_command_vx=0.0; autonomy_command_vy=0.0; autonomy_command_theta=tracking_theta" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="SearchBall">
    <Sequence>
      <CamFindBall yaw_limit="${YAW_LIMIT}"
                   head_search_speed="${HEAD_SEARCH_SPEED}"
                   body_search_speed="${BODY_SEARCH_SPEED}"
                   cmd_interval_msec="${CMD_INTERVAL_MSEC}"
                   theta="{tracking_theta}" />
      <Script code="autonomy_command_vx=0.0; autonomy_command_vy=0.0; autonomy_command_theta=tracking_theta" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="ChaseBall">
    <Sequence>
      <CamTrackBall stop_angle="${STOP_ANGLE}"
                    ball_yaw_gain="${BALL_YAW_GAIN}"
                    pitch_turn_gain="${PITCH_TURN_GAIN}"
                    head_step_rad="${HEAD_STEP_RAD}"
                    head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                    head_deadband_x_px="${HEAD_DEADBAND_X_PX}"
                    head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                    theta="{tracking_theta}" />
      <SimpleChase vx_limit="${VX_LIMIT}"
                   vy_limit="${VY_LIMIT}"
                   stop_dist="${STOP_DIST}"
                   y_tolerance="${Y_TOLERANCE}"
                   vx="{chase_vx}"
                   vy="{chase_vy}" />
      <Script code="autonomy_command_vx=chase_vx; autonomy_command_vy=chase_vy; autonomy_command_theta=tracking_theta" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="AdjustForShot">
    <Sequence>
      <ShootingAdjust target_range="${ADJUST_TARGET_RANGE}"
                      target_y_offset="${ADJUST_TARGET_Y_OFFSET}"
                      theta_offset="${ADJUST_THETA_OFFSET}"
                      range_tolerance="${ADJUST_RANGE_TOLERANCE}"
                      y_tolerance="${ADJUST_Y_TOLERANCE}"
                      stop_angle="${ADJUST_STOP_ANGLE}"
                      range_gain="${ADJUST_RANGE_GAIN}"
                      y_gain="${ADJUST_Y_GAIN}"
                      ball_yaw_gain="${ADJUST_BALL_YAW_GAIN}"
                      vx_limit="${ADJUST_VX_LIMIT}"
                      vy_limit="${ADJUST_VY_LIMIT}"
                      vtheta_limit="${ADJUST_VTHETA_LIMIT}"
                      turn_first_threshold="${ADJUST_TURN_FIRST_THRESHOLD}"
                      fixed_head_yaw="${ADJUST_FIXED_HEAD_YAW}"
                      max_ball_range="${ADJUST_MAX_BALL_RANGE}"
                      vx="{autonomy_adjust_vx}"
                      vy="{autonomy_adjust_vy}"
                      theta="{autonomy_adjust_theta}" />
      <Script code="autonomy_command_vx=autonomy_adjust_vx; autonomy_command_vy=autonomy_adjust_vy; autonomy_command_theta=autonomy_adjust_theta" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="MainTree">
    <Sequence name="autonomous run root">
      <ReactiveSequence _while="${stop_condition}" name="autonomous run disabled">
        <Script code="autonomy_command_vx=0.0; autonomy_command_vy=0.0; autonomy_command_theta=0.0" />
        <SetVelocity x="{autonomy_command_vx}"
                     y="{autonomy_command_vy}"
                     theta="{autonomy_command_theta}" />
      </ReactiveSequence>

      <ReactiveSequence _while="${run_condition}" name="holistic autonomous run">
        <CheckAndStandUp />
        <IfThenElse>
          <ScriptCondition name="Usable ball depth?" code="ball_visible &amp;&amp; ball_location_known" />
          <IfThenElse>
            <ScriptCondition name="Manual switch?" code="autonomy_switch=='manual'" />
            <IfThenElse>
              <ScriptCondition name="Manual track?" code="autonomy_manual_mode=='track'" />
              <SubTree ID="TrackVisible" _autoremap="true" />
              <IfThenElse>
                <ScriptCondition name="Manual chase?" code="autonomy_manual_mode=='chase'" />
                <SubTree ID="ChaseBall" _autoremap="true" />
                <SubTree ID="AdjustForShot" _autoremap="true" />
              </IfThenElse>
            </IfThenElse>
            <Sequence name="automatic phase">
              <IfThenElse>
                <ScriptCondition code="autonomy_auto_phase=='track'" />
                <IfThenElse>
                  <ScriptCondition code="ball_range&lt;=${STOP_DIST}" />
                  <Script code="autonomy_auto_phase='adjust'" />
                  <Script code="autonomy_auto_phase='chase'" />
                </IfThenElse>
                <IfThenElse>
                  <ScriptCondition code="autonomy_auto_phase=='chase' &amp;&amp; ball_range&lt;=${STOP_DIST}" />
                  <Script code="autonomy_auto_phase='adjust'" />
                  <IfThenElse>
                    <ScriptCondition code="autonomy_auto_phase=='adjust' &amp;&amp; ball_range&gt;${ADJUST_MAX_BALL_RANGE}" />
                    <Script code="autonomy_auto_phase='chase'" />
                    <Script code="autonomy_auto_phase=autonomy_auto_phase" />
                  </IfThenElse>
                </IfThenElse>
              </IfThenElse>
              <IfThenElse>
                <ScriptCondition code="autonomy_auto_phase=='chase'" />
                <SubTree ID="ChaseBall" _autoremap="true" />
                <SubTree ID="AdjustForShot" _autoremap="true" />
              </IfThenElse>
            </Sequence>
          </IfThenElse>
          <IfThenElse>
            <ScriptCondition name="RGB ball visible?" code="ball_visible" />
            <SubTree ID="TrackVisible" _autoremap="true" />
            <SubTree ID="SearchBall" _autoremap="true" />
          </IfThenElse>
        </IfThenElse>
        <SetVelocity x="{autonomy_command_vx}"
                     y="{autonomy_command_vy}"
                     theta="{autonomy_command_theta}" />
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
XML
}

parse_args "$@"
validate_settings
print_settings

if [[ "$DRY_RUN" == "true" ]]; then
  echo
  echo "Dry run passed: settings are valid; ROS was not started."
  exit 0
fi

cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/booster/BoosterRos2/fastdds_profile.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$FASTRTPS_DEFAULT_PROFILES_FILE}"

trap 'controlled_stop "Ctrl-C"' INT
trap 'controlled_stop "termination signal"' TERM
trap 'controlled_stop "terminal disconnected"' HUP
trap 'controlled_stop "unexpected script error" 1' ERR

echo
echo "Stopping any existing soccer stack before creating the single-owner tree..."
./scripts/stop.sh || true

BRAIN_SHARE="$(ros2 pkg prefix brain)/share/brain"
TREE_PATH="${BRAIN_SHARE}/behavior_trees/autonomous_run.xml"
write_tree "$TREE_PATH"
echo "Wrote ${TREE_PATH}"

echo "Starting vision, holistic brain tree, and GameController receiver..."
ros2 launch vision launch.py > vision.log 2>&1 &
VISION_PID=$!
ros2 launch brain launch.py \
  tree:=autonomous_run.xml \
  role:="$ROLE" \
  team_id:="$TEAM_ID" \
  player_id:="$PLAYER_ID" \
  agent_mode:=false \
  disable_com:=true \
  > brain.log 2>&1 &
BRAIN_PID=$!
ros2 launch game_controller launch.py > game_controller.log 2>&1 &
GAME_CONTROLLER_PID=$!

sleep 3
check_processes

RUNTIME_SWITCH="$SWITCH_MODE"
RUNTIME_MANUAL_MODE="$START_MODE"
RUNTIME_AUTO_PHASE="$START_MODE"

if [[ "$SWITCH_MODE" == "auto" ]]; then
  publish_runtime_command "autonomy_auto_${START_MODE}" ||
    controlled_stop "failed to initialize automatic mode" 1
else
  publish_runtime_command "autonomy_save_${START_MODE}" ||
    controlled_stop "failed to initialize the saved automatic phase" 1
  publish_runtime_command "autonomy_${START_MODE}" ||
    controlled_stop "failed to initialize manual mode" 1
fi

echo
echo "Autonomous_run is active."
if [[ "$REQUIRE_PLAY" == "true" ]]; then
  echo "Movement is gated by GameController PLAY."
else
  echo "Movement is enabled immediately unless END/app stop is received."
fi
echo "Selected switch=${RUNTIME_SWITCH}, start_mode=${START_MODE}."
runtime_help
echo "Diagnostics: tail -f brain.log | grep -E 'CamTrackBall/direct_pixel|CamFindBall/|SimpleChase/vector|ShootingAdjust/vector|RobotClient/setVelocity_out'"

while true; do
  check_processes
  monitor_brain_log
  expire_readiness

  command=""
  if read -r -t 0.2 command; then
    handle_runtime_command "$command"
  elif [[ ! -t 0 ]]; then
    controlled_stop "input closed" 1
  fi
done
