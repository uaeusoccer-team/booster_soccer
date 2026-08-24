#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="${WORKSPACE:-$HOME/booster_soccer}"

SWITCH_MODE="manual"
START_MODE="track"
REQUIRE_PLAY="false"
ROLE="striker"
TEAM_ID="5"
PLAYER_ID="1"

OBJECT_AVOIDANCE="true"
OBJECT_AVOID_DISTANCE="1.40"
OBJECT_STOP="0.50"

VX_LIMIT="0.30"
VY_LIMIT="0.30"
STOP_DIST="1.00"
Y_TOLERANCE="0.05"

STOP_ANGLE="0.12"
BALL_YAW_GAIN="2.0"
PITCH_TURN_GAIN="1.0"
HEAD_SEARCH_SPEED="0.15"
BODY_SEARCH_SPEED="0.00"
TRACK_TURN_YAW_LIMIT="0.35"
LOSS_TURN_PITCH_LIMIT="0.70"
SEARCH_YAW_LIMIT="1.15"
CMD_INTERVAL_MSEC="100"
HEAD_STEP_RAD="0.04"
HEAD_SETTLE_STEP_RAD="0.02"
HEAD_DEADBAND_X_PX="35"
HEAD_DEADBAND_Y_PX="35"
BALL_TARGET_Y_RATIO="0.70"
BALL_BOTTOM_MARGIN="30"
HEAD_PITCH_LIMIT_DOWN="0.90"
BOTTOM_EDGE_SEARCH_MARGIN="10"
BOTTOM_EDGE_SEARCH_PITCH_STEP="0.06"
BOTTOM_EDGE_SEARCH_PITCH_LIMIT="0.85"
ADJUST_HEAD_STEP_RAD="0.02"

ADJUST_TARGET_RANGE="1.70"
ADJUST_TARGET_Y_OFFSET="0.00"
ADJUST_THETA_OFFSET="0.00"
ADJUST_RANGE_TOLERANCE="0.10"
ADJUST_Y_TOLERANCE="0.05"
ADJUST_STOP_ANGLE="0.12"
ADJUST_RANGE_GAIN="1.0"
ADJUST_Y_GAIN="1.0"
ADJUST_GOAL_ALIGNMENT_GAIN="1.0"
ADJUST_GOAL_ALIGNMENT_TOLERANCE_PX="50"
ADJUST_GOAL_ALIGNMENT_HYSTERESIS_PX="20"
ADJUST_GOAL_MIN_POST_SEPARATION_PX="40"
ADJUST_GOAL_MAX_AGE_MSEC="300"
ADJUST_GOAL_BALL_MAX_SKEW_MSEC="100"
ADJUST_BALL_MAX_AGE_MSEC="300"
ADJUST_BALL_YAW_GAIN="2.0"
ADJUST_VX_LIMIT="0.30"
ADJUST_VY_LIMIT="0.30"
ADJUST_VTHETA_LIMIT="0.35"
ADJUST_TURN_FIRST_THRESHOLD="0.50"
ADJUST_FIXED_HEAD_YAW="0.00"
ADJUST_MAX_BALL_RANGE="2.20"

STRIKER_STOP_DIST="1.30"
STRIKER_ADJUST_TARGET_RANGE="1.30"
STRIKER_ADJUST_MAX_BALL_RANGE="1.70"

SCORE_VX="1.00"
SCORE_START_MAX_RANGE="1.70"
SCORE_BALL_MAX_AGE_MSEC="300"
SCORE_BALL_LOST_GRACE_MSEC="800"
SCORE_MAX_DURATION_MSEC="2500"
SCORE_HEAD_PITCH="0.45"
SCORE_HEAD_YAW="0.00"

READY_SAMPLES="10"
READY_MIN_MSEC="1000"
READY_DATA_MAX_AGE_MSEC="500"
SHOOT_PATH_CLEAR_MIN_MSEC="500"

DRY_RUN="false"
STOPPING="false"
LEGACY_YAW_LIMIT_USED="false"
SEARCH_YAW_LIMIT_EXPLICIT="false"
LEGACY_OBJECT_AVOIDNCE_USED="false"
OBJECT_STOP_DISTANCE_ALIAS_USED="false"

RUNTIME_SWITCH="$SWITCH_MODE"
RUNTIME_MANUAL_MODE="$START_MODE"
RUNTIME_AUTO_PHASE="$START_MODE"
RUNTIME_STRIKER_PHASE="track"
ACTIVE_BEHAVIOR="starting"
BALL_VISIBLE="unknown"
BALL_DEPTH_USABLE="unknown"
LAST_BALL_RANGE="unknown"
LAST_VX="0.0"
LAST_VY="0.0"
LAST_THETA="0.0"
LAST_ADJUST_SOURCE="none"
LAST_GOAL_COUNT="unknown"
LAST_GOAL_CENTER_X="unknown"
LAST_ALIGNMENT_ERROR_PX="unknown"
LAST_GOAL_ALIGNED="unknown"
LAST_LATERAL_SOURCE="none"
SCORE_STATE="INACTIVE"

OBSTACLE_STATE="WAITING_FOR_DEPTH"
OBSTACLE_MODE="unknown"
NEAREST_OBSTACLE="unknown"
OBSTACLE_SIDE="none"
OBSTACLE_DEPTH_AGE_MSEC="unknown"
OBSTACLE_BALL_STATE="unknown"
EXPECTED_BALL_X="unknown"
EXPECTED_BALL_Y="unknown"
SHOOT_PATH_CLEAR="unknown"
SHOOT_PATH_CLEAR_SINCE_MSEC=0
SHOOT_BLOCKED_PROMPTED="false"

READY_COUNT=0
LAST_READY_EVIDENCE_MSEC=0
SHOOT_READY="false"
READY_PROMPTED="false"
LAST_READINESS_REASON="not evaluated"
CHASE_STOP_PROMPTED="false"
MONITOR_LINE=0
# Drain every pending line each loop. A fixed small batch falls behind the
# 100 Hz brain diagnostics and can leave shooting-path readiness stale.
MONITOR_BATCH_LINES=0

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
  switch=manual                       # auto | manual | striker
  start_mode=track                    # track | chase | adjust | score
  require_play=false                  # require GameController PLAY
  role=striker                        # striker | goal_keeper
  team_id=5
  player_id=1

Obstacle avoidance:
  object_avoidance=true               # true | false
  object_avoid_distance=1.40          # begin detouring at this distance (m)
  object_stop=0.50                    # emergency/blocked-path stop distance (m)
  object_avoidnce=true                # deprecated typo alias
  object_stop_distance=0.50           # compatibility alias for object_stop

Tracking and rotation:
  stop_angle=0.12
  ball_yaw_gain=2.0
  pitch_turn_gain=1.0
  head_search_speed=0.15
  body_search_speed=0.00
  track_turn_yaw_limit=0.35
  loss_turn_pitch_limit=0.70
  search_yaw_limit=1.15
  yaw_limit=1.15                    # compatibility alias for search_yaw_limit
  cmd_interval_msec=100
  head_step_rad=0.04
  head_settle_step_rad=0.02
  head_deadband_x_px=35
  head_deadband_y_px=35
  ball_target_y_ratio=0.70             # place ball centre below image centre
  ball_bottom_margin=30                # preserve bbox-to-bottom safety pixels
  head_pitch_limit_down=0.90
  bottom_edge_search_margin=10         # identify a loss at the true bottom edge
  bottom_edge_search_pitch_step=0.06   # one-time downward reacquisition nudge
  bottom_edge_search_pitch_limit=0.85
  adjust_head_step_rad=0.02            # slower pitch motion during adjustment

Chase:
  vx_limit=0.30
  vy_limit=0.30
  stop_dist=1.00
  y_tolerance=0.05

Shooting adjustment:
  adjust_target_range=1.70
  adjust_target_y_offset=0.00
  adjust_theta_offset=0.00
  adjust_range_tolerance=0.10
  adjust_y_tolerance=0.05
  adjust_stop_angle=0.12
  adjust_range_gain=1.0
  adjust_y_gain=1.0
  adjust_goal_alignment_gain=1.0
  adjust_goal_alignment_tolerance_px=50
  adjust_goal_alignment_hysteresis_px=20
  adjust_goal_min_post_separation_px=40
  adjust_goal_max_age_msec=300
  adjust_goal_ball_max_skew_msec=100
  adjust_ball_max_age_msec=300
  adjust_ball_yaw_gain=2.0
  adjust_vx_limit=0.30
  adjust_vy_limit=0.30
  adjust_vtheta_limit=0.35
  adjust_turn_first_threshold=0.50
  adjust_fixed_head_yaw=0.00
  adjust_max_ball_range=2.20

Striker state machine:
  striker_stop_dist=1.30
  striker_adjust_target_range=1.30
  striker_adjust_max_ball_range=1.70

Score operation:
  score_vx=1.00
  score_start_max_range=1.70
  # Entry also requires |ball yaw| <= adjust_stop_angle (default 0.12 rad).
  score_ball_max_age_msec=300
  score_ball_lost_grace_msec=800
  score_max_duration_msec=2500
  score_head_pitch=0.45
  score_head_yaw=0.00

Shoot readiness:
  ready_samples=10
  ready_min_msec=1000
  ready_data_max_age_msec=500

Utility:
  --help                               # show this help
  --dry-run                            # validate and print settings; do not run ROS

Runtime commands:
  auto      Enable automatic chase/adjust transitions.
  striker   Start the track -> chase -> adjust -> score striker sequence.
  manual    Disable automatic transitions and preserve the saved phase.
  track     Select manual tracking/search/rotation (vx=0, vy=0).
  chase     Select manual tracking plus SimpleChase.
  adjust    Select manual ShootingAdjust.
  score     Run once, straight through the aligned ball, then return to track.
  status    Show selected, active, perception, velocity, and process state.
  help      Show runtime commands.
  shoot     Immediately send the standalone operator shoot command once.
  stop      Send a controlled stop and exit.

Each new RGB acquisition must contain usable depth once before CamTrackBall is
allowed. After that first depth-confirmed frame, CamTrackBall may continue
through temporary depth loss with zero translation. A complete RGB loss resets
the latch and returns to CamFindBall. Valid current depth restores the same
selected manual mode or saved automatic phase.

Automatic transitions:
  chase -> adjust when ball range <= stop_dist
  adjust -> chase when ball range > adjust_max_ball_range

Striker transitions:
  track -> chase when a depth-confirmed ball is found
  chase -> adjust at striker_stop_dist (default 1.30 m)
  adjust -> chase above striker_adjust_max_ball_range (default 1.70 m)
  stable adjust -> score; score timeout/loss -> track

During adjustment, vy aligns the midpoint of one fresh OL+OR opponent-goal
pair with the ball's horizontal image position while theta continues to face
the ball. A goal midpoint left of the ball commands a right strafe; a midpoint
right of the ball commands a left strafe. Missing, ambiguous, unlabeled, or
unsynchronized goalposts stop vy.
A stale ball observation disables all adjustment motion: vx, vy, and theta.
OL/OR identity comes from the brain's current field pose; verify localization
and the opponent-goal direction before enabling body motion.

Example:
  ./scripts/Autonomous_run.sh \
    switch=manual \
    start_mode=track \
    require_play=false \
    role=striker \
    team_id=5 \
    player_id=1 \
    vx_limit=0.30 \
    vy_limit=0.30 \
    stop_dist=1.00 \
    y_tolerance=0.05 \
    stop_angle=0.12 \
    ball_yaw_gain=2.0 \
    pitch_turn_gain=1.0 \
    head_search_speed=0.15 \
    body_search_speed=0.00 \
    track_turn_yaw_limit=0.35 \
    loss_turn_pitch_limit=0.70 \
    search_yaw_limit=1.15 \
    cmd_interval_msec=100 \
    head_step_rad=0.04 \
    head_settle_step_rad=0.02 \
    head_deadband_x_px=35 \
    head_deadband_y_px=35 \
    ball_target_y_ratio=0.70 \
    ball_bottom_margin=30 \
    head_pitch_limit_down=0.90 \
    bottom_edge_search_margin=10 \
    bottom_edge_search_pitch_step=0.06 \
    bottom_edge_search_pitch_limit=0.85 \
    adjust_head_step_rad=0.02 \
    adjust_target_range=1.70 \
    adjust_target_y_offset=0.00 \
    adjust_theta_offset=0.00 \
    adjust_range_tolerance=0.10 \
    adjust_y_tolerance=0.05 \
    adjust_stop_angle=0.12 \
    adjust_range_gain=1.0 \
    adjust_y_gain=1.0 \
    adjust_goal_alignment_gain=1.0 \
    adjust_goal_alignment_tolerance_px=50 \
    adjust_goal_alignment_hysteresis_px=20 \
    adjust_goal_min_post_separation_px=40 \
    adjust_goal_max_age_msec=300 \
    adjust_goal_ball_max_skew_msec=100 \
    adjust_ball_max_age_msec=300 \
    adjust_ball_yaw_gain=2.0 \
    adjust_vx_limit=0.30 \
    adjust_vy_limit=0.30 \
    adjust_vtheta_limit=0.35 \
    adjust_turn_first_threshold=0.50 \
    adjust_fixed_head_yaw=0.00 \
    adjust_max_ball_range=2.20 \
    object_avoidance=true \
    object_avoid_distance=1.40 \
    object_stop=0.50 \
    ready_samples=10 \
    ready_min_msec=1000 \
    ready_data_max_age_msec=500

Run body-moving modes only with the robot off the stand, balanced on the floor,
in open space, and with the operator ready to use the safe stop.
USAGE
}

runtime_help() {
  cat <<'HELP'
Runtime commands:
  auto    striker  manual   track    chase    adjust    score
  status  help      shoot   stop

Typing track, chase, adjust, or score selects that manual operation immediately.
The exact word "striker" starts its track/chase/adjust/score state machine. The
exact lowercase word "shoot" immediately runs shoot_once.sh once without stopping or
restarting the autonomy stack. `score` is a bounded run; `shoot` is the existing
standalone kick command.
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
      [[ "$value" == "auto" || "$value" == "manual" || "$value" == "striker" ]] ||
        { echo "Invalid switch: ${value}; expected auto, manual, or striker" >&2; exit 2; }
      SWITCH_MODE="$value"
      return
      ;;
    start_mode)
      [[ "$value" == "track" || "$value" == "chase" || "$value" == "adjust" || "$value" == "score" ]] ||
        { echo "Invalid start_mode: ${value}; expected track, chase, adjust, or score" >&2; exit 2; }
      START_MODE="$value"
      return
      ;;
    require_play)
      REQUIRE_PLAY="$(normalize_bool "$key" "$value")"
      return
      ;;
    object_avoidance)
      OBJECT_AVOIDANCE="$(normalize_bool "$key" "$value")"
      return
      ;;
    object_avoidnce)
      OBJECT_AVOIDANCE="$(normalize_bool "$key" "$value")"
      LEGACY_OBJECT_AVOIDNCE_USED="true"
      return
      ;;
    role)
      [[ "$value" == "striker" || "$value" == "goal_keeper" ]] ||
        { echo "Invalid role: ${value}; expected striker or goal_keeper" >&2; exit 2; }
      ROLE="$value"
      return
      ;;
    team_id|player_id|cmd_interval_msec|head_deadband_x_px|head_deadband_y_px|adjust_goal_max_age_msec|adjust_goal_ball_max_skew_msec|adjust_ball_max_age_msec|score_ball_max_age_msec|score_ball_lost_grace_msec|score_max_duration_msec|ready_samples|ready_min_msec|ready_data_max_age_msec)
      is_positive_integer "$value" ||
        { echo "Invalid positive integer for ${key}: ${value}" >&2; exit 2; }
      ;;
    adjust_target_y_offset|adjust_theta_offset|adjust_fixed_head_yaw|score_head_yaw)
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
    track_turn_yaw_limit) TRACK_TURN_YAW_LIMIT="$value" ;;
    loss_turn_pitch_limit) LOSS_TURN_PITCH_LIMIT="$value" ;;
    search_yaw_limit)
      SEARCH_YAW_LIMIT="$value"
      SEARCH_YAW_LIMIT_EXPLICIT="true"
      ;;
    yaw_limit)
      SEARCH_YAW_LIMIT="$value"
      LEGACY_YAW_LIMIT_USED="true"
      ;;
    cmd_interval_msec) CMD_INTERVAL_MSEC="$value" ;;
    head_step_rad) HEAD_STEP_RAD="$value" ;;
    head_settle_step_rad) HEAD_SETTLE_STEP_RAD="$value" ;;
    head_deadband_x_px) HEAD_DEADBAND_X_PX="$value" ;;
    head_deadband_y_px) HEAD_DEADBAND_Y_PX="$value" ;;
    ball_target_y_ratio) BALL_TARGET_Y_RATIO="$value" ;;
    ball_bottom_margin|ball_bottom_margin_px) BALL_BOTTOM_MARGIN="$value" ;;
    head_pitch_limit_down) HEAD_PITCH_LIMIT_DOWN="$value" ;;
    bottom_edge_search_margin) BOTTOM_EDGE_SEARCH_MARGIN="$value" ;;
    bottom_edge_search_pitch_step) BOTTOM_EDGE_SEARCH_PITCH_STEP="$value" ;;
    bottom_edge_search_pitch_limit) BOTTOM_EDGE_SEARCH_PITCH_LIMIT="$value" ;;
    adjust_head_step_rad) ADJUST_HEAD_STEP_RAD="$value" ;;
    adjust_target_range) ADJUST_TARGET_RANGE="$value" ;;
    adjust_target_y_offset) ADJUST_TARGET_Y_OFFSET="$value" ;;
    adjust_theta_offset) ADJUST_THETA_OFFSET="$value" ;;
    adjust_range_tolerance) ADJUST_RANGE_TOLERANCE="$value" ;;
    adjust_y_tolerance) ADJUST_Y_TOLERANCE="$value" ;;
    adjust_stop_angle) ADJUST_STOP_ANGLE="$value" ;;
    adjust_range_gain) ADJUST_RANGE_GAIN="$value" ;;
    adjust_y_gain) ADJUST_Y_GAIN="$value" ;;
    adjust_goal_alignment_gain) ADJUST_GOAL_ALIGNMENT_GAIN="$value" ;;
    adjust_goal_alignment_tolerance_px) ADJUST_GOAL_ALIGNMENT_TOLERANCE_PX="$value" ;;
    adjust_goal_alignment_hysteresis_px) ADJUST_GOAL_ALIGNMENT_HYSTERESIS_PX="$value" ;;
    adjust_goal_min_post_separation_px) ADJUST_GOAL_MIN_POST_SEPARATION_PX="$value" ;;
    adjust_goal_max_age_msec) ADJUST_GOAL_MAX_AGE_MSEC="$value" ;;
    adjust_goal_ball_max_skew_msec) ADJUST_GOAL_BALL_MAX_SKEW_MSEC="$value" ;;
    adjust_ball_max_age_msec) ADJUST_BALL_MAX_AGE_MSEC="$value" ;;
    adjust_ball_yaw_gain) ADJUST_BALL_YAW_GAIN="$value" ;;
    adjust_vx_limit) ADJUST_VX_LIMIT="$value" ;;
    adjust_vy_limit) ADJUST_VY_LIMIT="$value" ;;
    adjust_vtheta_limit) ADJUST_VTHETA_LIMIT="$value" ;;
    adjust_turn_first_threshold) ADJUST_TURN_FIRST_THRESHOLD="$value" ;;
    adjust_fixed_head_yaw) ADJUST_FIXED_HEAD_YAW="$value" ;;
    adjust_max_ball_range) ADJUST_MAX_BALL_RANGE="$value" ;;
    striker_stop_dist) STRIKER_STOP_DIST="$value" ;;
    striker_adjust_target_range) STRIKER_ADJUST_TARGET_RANGE="$value" ;;
    striker_adjust_max_ball_range) STRIKER_ADJUST_MAX_BALL_RANGE="$value" ;;
    score_vx) SCORE_VX="$value" ;;
    score_start_max_range) SCORE_START_MAX_RANGE="$value" ;;
    score_ball_max_age_msec) SCORE_BALL_MAX_AGE_MSEC="$value" ;;
    score_ball_lost_grace_msec) SCORE_BALL_LOST_GRACE_MSEC="$value" ;;
    score_max_duration_msec) SCORE_MAX_DURATION_MSEC="$value" ;;
    score_head_pitch) SCORE_HEAD_PITCH="$value" ;;
    score_head_yaw) SCORE_HEAD_YAW="$value" ;;
    object_avoid_distance) OBJECT_AVOID_DISTANCE="$value" ;;
    object_stop) OBJECT_STOP="$value" ;;
    object_stop_distance)
      OBJECT_STOP="$value"
      OBJECT_STOP_DISTANCE_ALIAS_USED="true"
      ;;
    ready_samples) READY_SAMPLES="$value" ;;
    ready_min_msec) READY_MIN_MSEC="$value" ;;
    ready_data_max_age_msec) READY_DATA_MAX_AGE_MSEC="$value" ;;
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
  if [[ "$LEGACY_YAW_LIMIT_USED" == "true" && "$SEARCH_YAW_LIMIT_EXPLICIT" == "true" ]]; then
    echo "Do not set both yaw_limit and search_yaw_limit; yaw_limit is only a compatibility alias." >&2
    exit 2
  fi

  float_lt "$STOP_DIST" "$ADJUST_MAX_BALL_RANGE" || {
    echo "Invalid automatic hysteresis: stop_dist (${STOP_DIST}) must be less than adjust_max_ball_range (${ADJUST_MAX_BALL_RANGE})." >&2
    exit 2
  }

  float_le "$ADJUST_TARGET_RANGE" "$ADJUST_MAX_BALL_RANGE" || {
    echo "Invalid adjustment target: adjust_target_range (${ADJUST_TARGET_RANGE}) must not exceed adjust_max_ball_range (${ADJUST_MAX_BALL_RANGE})." >&2
    exit 2
  }

  if [[ "$SWITCH_MODE" == "auto" && "$START_MODE" == "score" ]]; then
    echo "Invalid selection: start_mode=score is a manual operation; use switch=manual or switch=striker." >&2
    exit 2
  fi

  float_le "$BALL_TARGET_Y_RATIO" "1.0" || {
    echo "Invalid ball target: ball_target_y_ratio (${BALL_TARGET_Y_RATIO}) must be between 0.0 and 1.0." >&2
    exit 2
  }

  float_le "$HEAD_PITCH_LIMIT_DOWN" "0.90" || {
    echo "Invalid head pitch limit: head_pitch_limit_down (${HEAD_PITCH_LIMIT_DOWN}) must not exceed the physical safety cap (0.90)." >&2
    exit 2
  }

  if float_lt "0" "$BALL_BOTTOM_MARGIN"; then
    float_lt "$BOTTOM_EDGE_SEARCH_MARGIN" "$BALL_BOTTOM_MARGIN" || {
      echo "Invalid bottom-edge recovery: bottom_edge_search_margin (${BOTTOM_EDGE_SEARCH_MARGIN}) must be smaller than ball_bottom_margin (${BALL_BOTTOM_MARGIN})." >&2
      exit 2
    }
  fi

  float_le "$BOTTOM_EDGE_SEARCH_PITCH_LIMIT" "$HEAD_PITCH_LIMIT_DOWN" || {
    echo "Invalid bottom-edge recovery: bottom_edge_search_pitch_limit (${BOTTOM_EDGE_SEARCH_PITCH_LIMIT}) must not exceed head_pitch_limit_down (${HEAD_PITCH_LIMIT_DOWN})." >&2
    exit 2
  }

  float_le "$SCORE_HEAD_PITCH" "$HEAD_PITCH_LIMIT_DOWN" || {
    echo "Invalid score head pitch: score_head_pitch (${SCORE_HEAD_PITCH}) must not exceed head_pitch_limit_down (${HEAD_PITCH_LIMIT_DOWN})." >&2
    exit 2
  }

  float_lt "$STRIKER_STOP_DIST" "$STRIKER_ADJUST_MAX_BALL_RANGE" || {
    echo "Invalid striker hysteresis: striker_stop_dist (${STRIKER_STOP_DIST}) must be less than striker_adjust_max_ball_range (${STRIKER_ADJUST_MAX_BALL_RANGE})." >&2
    exit 2
  }

  float_le "$STRIKER_ADJUST_TARGET_RANGE" "$STRIKER_ADJUST_MAX_BALL_RANGE" || {
    echo "Invalid striker target: striker_adjust_target_range (${STRIKER_ADJUST_TARGET_RANGE}) must not exceed striker_adjust_max_ball_range (${STRIKER_ADJUST_MAX_BALL_RANGE})." >&2
    exit 2
  }

  float_lt "0" "$SCORE_VX" || {
    echo "Invalid score speed: score_vx (${SCORE_VX}) must be greater than 0." >&2
    exit 2
  }

  float_lt "0" "$OBJECT_STOP" || {
    echo "Invalid obstacle stop distance: object_stop (${OBJECT_STOP}) must be greater than 0." >&2
    exit 2
  }

  float_lt "$OBJECT_STOP" "$OBJECT_AVOID_DISTANCE" || {
    echo "Invalid obstacle distances: object_stop (${OBJECT_STOP}) must be less than object_avoid_distance (${OBJECT_AVOID_DISTANCE})." >&2
    exit 2
  }

  float_le "$OBJECT_AVOID_DISTANCE" "3.0" || {
    echo "Invalid obstacle avoid distance: object_avoid_distance (${OBJECT_AVOID_DISTANCE}) must not exceed the 3.0 m depth-map range." >&2
    exit 2
  }

  if [[ "$OBJECT_AVOIDANCE" == "false" ]]; then
    OBSTACLE_STATE="DISABLED"
    SHOOT_PATH_CLEAR="true"
    if [[ "$SWITCH_MODE" == "striker" || "$START_MODE" == "score" ]]; then
      echo "score and striker require object_avoidance=true so the full-speed run remains guarded." >&2
      exit 2
    fi
  fi

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

  object_avoidance=${OBJECT_AVOIDANCE}
  object_avoid_distance=${OBJECT_AVOID_DISTANCE}
  object_stop=${OBJECT_STOP}

  vx_limit=${VX_LIMIT}
  vy_limit=${VY_LIMIT}
  stop_dist=${STOP_DIST}
  y_tolerance=${Y_TOLERANCE}

  stop_angle=${STOP_ANGLE}
  ball_yaw_gain=${BALL_YAW_GAIN}
  pitch_turn_gain=${PITCH_TURN_GAIN}
  head_search_speed=${HEAD_SEARCH_SPEED}
  body_search_speed=${BODY_SEARCH_SPEED}
  track_turn_yaw_limit=${TRACK_TURN_YAW_LIMIT}
  loss_turn_pitch_limit=${LOSS_TURN_PITCH_LIMIT}
  search_yaw_limit=${SEARCH_YAW_LIMIT}
  cmd_interval_msec=${CMD_INTERVAL_MSEC}
  head_step_rad=${HEAD_STEP_RAD}
  head_settle_step_rad=${HEAD_SETTLE_STEP_RAD}
  head_deadband_x_px=${HEAD_DEADBAND_X_PX}
  head_deadband_y_px=${HEAD_DEADBAND_Y_PX}
  ball_target_y_ratio=${BALL_TARGET_Y_RATIO}
  ball_bottom_margin=${BALL_BOTTOM_MARGIN}
  head_pitch_limit_down=${HEAD_PITCH_LIMIT_DOWN}
  bottom_edge_search_margin=${BOTTOM_EDGE_SEARCH_MARGIN}
  bottom_edge_search_pitch_step=${BOTTOM_EDGE_SEARCH_PITCH_STEP}
  bottom_edge_search_pitch_limit=${BOTTOM_EDGE_SEARCH_PITCH_LIMIT}
  adjust_head_step_rad=${ADJUST_HEAD_STEP_RAD}

  adjust_target_range=${ADJUST_TARGET_RANGE}
  adjust_target_y_offset=${ADJUST_TARGET_Y_OFFSET}
  adjust_theta_offset=${ADJUST_THETA_OFFSET}
  adjust_range_tolerance=${ADJUST_RANGE_TOLERANCE}
  adjust_y_tolerance=${ADJUST_Y_TOLERANCE}
  adjust_stop_angle=${ADJUST_STOP_ANGLE}
  adjust_range_gain=${ADJUST_RANGE_GAIN}
  adjust_y_gain=${ADJUST_Y_GAIN}
  adjust_goal_alignment_gain=${ADJUST_GOAL_ALIGNMENT_GAIN}
  adjust_goal_alignment_tolerance_px=${ADJUST_GOAL_ALIGNMENT_TOLERANCE_PX}
  adjust_goal_alignment_hysteresis_px=${ADJUST_GOAL_ALIGNMENT_HYSTERESIS_PX}
  adjust_goal_min_post_separation_px=${ADJUST_GOAL_MIN_POST_SEPARATION_PX}
  adjust_goal_max_age_msec=${ADJUST_GOAL_MAX_AGE_MSEC}
  adjust_goal_ball_max_skew_msec=${ADJUST_GOAL_BALL_MAX_SKEW_MSEC}
  adjust_ball_max_age_msec=${ADJUST_BALL_MAX_AGE_MSEC}
  adjust_ball_yaw_gain=${ADJUST_BALL_YAW_GAIN}
  adjust_vx_limit=${ADJUST_VX_LIMIT}
  adjust_vy_limit=${ADJUST_VY_LIMIT}
  adjust_vtheta_limit=${ADJUST_VTHETA_LIMIT}
  adjust_turn_first_threshold=${ADJUST_TURN_FIRST_THRESHOLD}
  adjust_fixed_head_yaw=${ADJUST_FIXED_HEAD_YAW}
  adjust_max_ball_range=${ADJUST_MAX_BALL_RANGE}

  striker_stop_dist=${STRIKER_STOP_DIST}
  striker_adjust_target_range=${STRIKER_ADJUST_TARGET_RANGE}
  striker_adjust_max_ball_range=${STRIKER_ADJUST_MAX_BALL_RANGE}

  score_vx=${SCORE_VX}
  score_start_max_range=${SCORE_START_MAX_RANGE}
  score_ball_max_age_msec=${SCORE_BALL_MAX_AGE_MSEC}
  score_ball_lost_grace_msec=${SCORE_BALL_LOST_GRACE_MSEC}
  score_max_duration_msec=${SCORE_MAX_DURATION_MSEC}
  score_head_pitch=${SCORE_HEAD_PITCH}
  score_head_yaw=${SCORE_HEAD_YAW}

  ready_samples=${READY_SAMPLES}
  ready_min_msec=${READY_MIN_MSEC}
  ready_data_max_age_msec=${READY_DATA_MAX_AGE_MSEC}
SETTINGS

  if [[ "$LEGACY_YAW_LIMIT_USED" == "true" ]]; then
    echo
    echo "Warning: yaw_limit is deprecated; use search_yaw_limit."
  fi

  if [[ "$LEGACY_OBJECT_AVOIDNCE_USED" == "true" ]]; then
    echo
    echo "Warning: object_avoidnce is deprecated; use object_avoidance."
  fi

  if [[ "$OBJECT_STOP_DISTANCE_ALIAS_USED" == "true" ]]; then
    echo
    echo "Note: object_stop_distance is a compatibility alias for object_stop."
  fi

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
  local timeout_seconds="${2:-8}"
  timeout "$timeout_seconds" ros2 topic pub --once /booster_agent/soccer_game_control \
    std_msgs/msg/String "{data: ${command}}" >/dev/null 2>&1
}

send_game_stop() {
  for _ in 1 2 3; do
    ros2 topic pub --once /booster_agent/soccer_game_control \
      std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
    sleep 0.1
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
  local reason="${1:-readiness reset without a diagnostic reason}"

  LAST_READINESS_REASON="$reason"
  if [[ "$SHOOT_READY" == "true" ]]; then
    echo
    echo "Shoot readiness revoked: ${reason}"
  fi
  READY_COUNT=0
  LAST_READY_EVIDENCE_MSEC=0
  SHOOT_READY="false"
  READY_PROMPTED="false"
}

mark_shoot_path_blocked() {
  SHOOT_PATH_CLEAR_SINCE_MSEC=0
  reset_readiness "shooting corridor is blocked or obstacle data is stale"

  if [[ "$ACTIVE_BEHAVIOR" == "adjust" && "$SHOOT_BLOCKED_PROMPTED" != "true" ]]; then
    SHOOT_BLOCKED_PROMPTED="true"
    echo
    echo "OBJECT IN WAY OF SHOOTING"
  fi
}

evaluate_adjust_readiness() {
  local ball_range="$1"
  local range_error="$2"
  local y_error="$3"
  local theta_error="$4"
  local goal_count="$5"
  local goal_center_x="$6"
  local alignment_error_px="$7"
  local goal_aligned="$8"
  local lateral_source="$9"
  local vx="${10}"
  local vy="${11}"
  local theta="${12}"
  local source="${13}"
  local controller_ready_count="${14}"
  local controller_ready_required="${15}"
  local controller_ready="${16}"
  local now
  local elapsed
  local failure_reason

  now="$(now_msec)"
  if [[ "$OBJECT_AVOIDANCE" == "true" ]]; then
    if [[ "$SHOOT_PATH_CLEAR" == "false" ]]; then
      mark_shoot_path_blocked
      return
    fi
    if [[ "$SHOOT_PATH_CLEAR" != "true" ]]; then
      reset_readiness "waiting for obstacle safety to evaluate the shooting corridor"
      return
    fi

    if ((SHOOT_PATH_CLEAR_SINCE_MSEC == 0)); then
      SHOOT_PATH_CLEAR_SINCE_MSEC="$now"
    fi
    elapsed=$((now - SHOOT_PATH_CLEAR_SINCE_MSEC))
    if ((elapsed < SHOOT_PATH_CLEAR_MIN_MSEC)); then
      reset_readiness "shooting corridor clear for ${elapsed}/${SHOOT_PATH_CLEAR_MIN_MSEC}ms"
      return
    fi
  fi

  failure_reason="$(awk \
    -v ball_range="$ball_range" \
    -v max_ball_range="$ADJUST_MAX_BALL_RANGE" \
    -v range_error="$range_error" \
    -v range_tolerance="$ADJUST_RANGE_TOLERANCE" \
    -v y_error="$y_error" \
    -v y_tolerance="$ADJUST_Y_TOLERANCE" \
    -v theta_error="$theta_error" \
    -v theta_tolerance="$ADJUST_STOP_ANGLE" \
    -v goal_count="$goal_count" \
    -v goal_center_x="$goal_center_x" \
    -v alignment_error_px="$alignment_error_px" \
    -v goal_aligned="$goal_aligned" \
    -v lateral_source="$lateral_source" \
    -v vx="$vx" \
    -v vy="$vy" \
    -v theta="$theta" \
    -v source="$source" '
      function abs(v) { return v < 0 ? -v : v }
      function add(reason) {
        if (reasons != "") {
          reasons = reasons "; "
        }
        reasons = reasons reason
      }
      BEGIN {
        if (source == "NO_BALL") {
          add("source=NO_BALL (no current visual ball observation)")
        } else if (source == "NO_HEAD") {
          add("source=NO_HEAD (head-state feedback is unavailable)")
        } else if (source == "NO_DEPTH") {
          add("source=NO_DEPTH (no usable body-frame depth)")
        } else if (source == "STALE_BALL") {
          add("source=STALE_BALL (ball observation is too old; all adjustment motion is disabled)")
        } else if (source == "OUT_OF_RANGE") {
          add(sprintf("source=OUT_OF_RANGE: ball_range=%.3f > adjust_max_ball_range=%.3f", ball_range, max_ball_range))
        }
        if (lateral_source == "NO_GOAL") {
          add(sprintf("lateralSource=NO_GOAL: goalCount=%d; no fresh synchronized two-post goal", goal_count))
        }
        if (goal_aligned != 1) {
          add(sprintf("goalAligned=%d: goalCenterX=%.1f alignmentErrorPx=%.1f", goal_aligned, goal_center_x, alignment_error_px))
        }
        if (abs(range_error) > range_tolerance) {
          add(sprintf("|range_error|=%.3f > adjust_range_tolerance=%.3f", abs(range_error), range_tolerance))
        }
        if (abs(y_error) > y_tolerance) {
          add(sprintf("|y_error|=%.3f > adjust_y_tolerance=%.3f", abs(y_error), y_tolerance))
        }
        if (abs(theta_error) > theta_tolerance) {
          add(sprintf("|theta_error|=%.3f > adjust_stop_angle=%.3f", abs(theta_error), theta_tolerance))
        }
        if (abs(vx) > 0.0005) {
          add(sprintf("|vx|=%.3f > ready_zero_limit=0.0005", abs(vx)))
        }
        if (abs(vy) > 0.0005) {
          add(sprintf("|vy|=%.3f > ready_zero_limit=0.0005", abs(vy)))
        }
        if (abs(theta) > 0.0005) {
          add(sprintf("|theta|=%.3f > ready_zero_limit=0.0005", abs(theta)))
        }
        print reasons
      }
    ')"

  if [[ -z "$failure_reason" ]]; then
    READY_COUNT="$controller_ready_count"
    LAST_READY_EVIDENCE_MSEC="$now"
    if [[ "$controller_ready" == "1" ]]; then
      SHOOT_READY="true"
      LAST_READINESS_REASON="controller ready: samples=${READY_COUNT}/${controller_ready_required}, minimum dwell=${READY_MIN_MSEC}ms"
      if [[ "$READY_PROMPTED" != "true" ]]; then
        READY_PROMPTED="true"
        echo
        echo "========================================"
        echo "READY TO SHOOT"
        echo "Type shoot to fire."
        echo "Type chase, adjust, track, auto, or stop."
        echo "========================================"
      fi
    else
      if [[ "$SHOOT_READY" == "true" ]]; then
        echo
        echo "Shoot readiness revoked: controller readiness returned false"
      fi
      SHOOT_READY="false"
      READY_PROMPTED="false"
      LAST_READINESS_REASON="controller stabilizing: samples=${READY_COUNT}/${controller_ready_required}, minimum dwell=${READY_MIN_MSEC}ms"
    fi
  else
    reset_readiness "$failure_reason"
  fi
}

process_log_line() {
  local line="$1"
  local number_re='[-+]?[0-9]*[.]?[0-9]+|[-+]?[Ii][Nn][Ff]|[Nn][Aa][Nn]'
  local pattern

  pattern='Autonomy switch => striker [(]phase: track[)]'
  if [[ "$line" =~ $pattern ]]; then
    RUNTIME_SWITCH="striker"
    RUNTIME_STRIKER_PHASE="track"
  else
    pattern='Autonomy switch => auto [(]phase: (track|chase|adjust)[)]'
    if [[ "$line" =~ $pattern ]]; then
      RUNTIME_SWITCH="auto"
      RUNTIME_AUTO_PHASE="${BASH_REMATCH[1]}"
    else
      pattern='Autonomy saved phase => (track|chase|adjust)'
      if [[ "$line" =~ $pattern ]]; then
        RUNTIME_AUTO_PHASE="${BASH_REMATCH[1]}"
      else
        pattern='Autonomy switch => manual [(]mode: (track|chase|adjust|score)[)]'
        if [[ "$line" =~ $pattern ]]; then
          RUNTIME_SWITCH="manual"
          RUNTIME_MANUAL_MODE="${BASH_REMATCH[1]}"
        else
          pattern='Autonomy manual mode => (track|chase|adjust|score)'
          if [[ "$line" =~ $pattern ]]; then
            RUNTIME_SWITCH="manual"
            RUNTIME_MANUAL_MODE="${BASH_REMATCH[1]}"
          fi
        fi
      fi
    fi
  fi

  if [[ "$line" == *"ScoreBall/vector"* ]]; then
    pattern="state: ([A-Z_]+).*vx: (${number_re}).*done: (0|1)"
    if [[ "$line" =~ $pattern ]]; then
      SCORE_STATE="${BASH_REMATCH[1]}"
      LAST_VX="${BASH_REMATCH[2]}"
      LAST_VY="0.0"
      LAST_THETA="0.0"
      if [[ "$SCORE_STATE" == "INACTIVE" ]]; then
        return 0
      else
        ACTIVE_BEHAVIOR="score"
        if [[ "$RUNTIME_SWITCH" == "striker" ]]; then
          RUNTIME_STRIKER_PHASE="score"
        else
          RUNTIME_MANUAL_MODE="score"
        fi
      fi
      if [[ "${BASH_REMATCH[3]}" == "1" ]]; then
        if [[ "$RUNTIME_SWITCH" == "striker" ]]; then
          RUNTIME_STRIKER_PHASE="track"
        else
          RUNTIME_MANUAL_MODE="track"
        fi
      fi
      reset_readiness "active_behavior=score; adjustment phase has ended"
    fi
  elif [[ "$line" == *"CamFindBall/"* ]]; then
    ACTIVE_BEHAVIOR="search"
    BALL_VISIBLE="false"
    BALL_DEPTH_USABLE="false"
    reset_readiness "active_behavior=search; ball_visible=false; usable depth is unavailable"
  elif [[ "$line" == *"CamTrackBall/direct_pixel"* ]]; then
    ACTIVE_BEHAVIOR="track"
    BALL_VISIBLE="true"
    BALL_DEPTH_USABLE="false"
    reset_readiness "active_behavior=track; shooting adjustment is not active"
  elif [[ "$line" == *"SimpleChase/vector"* ]]; then
    ACTIVE_BEHAVIOR="chase"
    BALL_VISIBLE="true"
    BALL_DEPTH_USABLE="true"
    reset_readiness "active_behavior=chase; shooting adjustment is not active"
    if [[ "$RUNTIME_SWITCH" == "auto" ]]; then
      RUNTIME_AUTO_PHASE="chase"
    elif [[ "$RUNTIME_SWITCH" == "striker" ]]; then
      RUNTIME_STRIKER_PHASE="chase"
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
    elif [[ "$RUNTIME_SWITCH" == "striker" ]]; then
      RUNTIME_STRIKER_PHASE="adjust"
    fi

    pattern="ballRange: (${number_re}).*rangeError: (${number_re}).*yError: (${number_re}).*thetaError: (${number_re}).*goalCount: (${number_re}).*goalCenterX: (${number_re}).*alignmentErrorPx: (${number_re}).*goalAligned: (0|1).*lateralSource: (GOAL_CENTER|NO_GOAL).*vx: (${number_re}).*vy: (${number_re}).*theta: (${number_re}).*source: ([A-Z_]+).*readySamples: ([0-9]+)/([0-9]+) ready: (0|1)"
    if [[ "$line" =~ $pattern ]]; then
      LAST_BALL_RANGE="${BASH_REMATCH[1]}"
      LAST_GOAL_COUNT="${BASH_REMATCH[5]}"
      LAST_GOAL_CENTER_X="${BASH_REMATCH[6]}"
      LAST_ALIGNMENT_ERROR_PX="${BASH_REMATCH[7]}"
      LAST_GOAL_ALIGNED="${BASH_REMATCH[8]}"
      LAST_LATERAL_SOURCE="${BASH_REMATCH[9]}"
      LAST_VX="${BASH_REMATCH[10]}"
      LAST_VY="${BASH_REMATCH[11]}"
      LAST_THETA="${BASH_REMATCH[12]}"
      LAST_ADJUST_SOURCE="${BASH_REMATCH[13]}"
      case "$LAST_ADJUST_SOURCE" in
        NO_BALL|STALE_BALL)
          BALL_VISIBLE="false"
          BALL_DEPTH_USABLE="false"
          ;;
        NO_DEPTH)
          BALL_DEPTH_USABLE="false"
          ;;
      esac
      evaluate_adjust_readiness \
        "${BASH_REMATCH[1]}" \
        "${BASH_REMATCH[2]}" \
        "${BASH_REMATCH[3]}" \
        "${BASH_REMATCH[4]}" \
        "${BASH_REMATCH[5]}" \
        "${BASH_REMATCH[6]}" \
        "${BASH_REMATCH[7]}" \
        "${BASH_REMATCH[8]}" \
        "${BASH_REMATCH[9]}" \
        "${BASH_REMATCH[10]}" \
        "${BASH_REMATCH[11]}" \
        "${BASH_REMATCH[12]}" \
        "${BASH_REMATCH[13]}" \
        "${BASH_REMATCH[14]}" \
        "${BASH_REMATCH[15]}" \
        "${BASH_REMATCH[16]}"
    else
      reset_readiness "ShootingAdjust/vector diagnostic line could not be parsed"
    fi
  elif [[ "$line" == *"ObstacleAvoidance/state"* ]]; then
    pattern="state: ([A-Z_]+) mode: ([a-z]+) nearest: (${number_re}) side: ([A-Za-z_]+) depthAgeMs: (${number_re}) ballState: ([A-Z_]+) expectedBallX: (${number_re}) expectedBallY: (${number_re}) shootPathClear: (true|false) vx: (${number_re}) vy: (${number_re}) theta: (${number_re})"
    if [[ "$line" =~ $pattern ]]; then
      OBSTACLE_STATE="${BASH_REMATCH[1]}"
      OBSTACLE_MODE="${BASH_REMATCH[2]}"
      NEAREST_OBSTACLE="${BASH_REMATCH[3]}"
      OBSTACLE_SIDE="${BASH_REMATCH[4]}"
      OBSTACLE_DEPTH_AGE_MSEC="${BASH_REMATCH[5]}"
      OBSTACLE_BALL_STATE="${BASH_REMATCH[6]}"
      EXPECTED_BALL_X="${BASH_REMATCH[7]}"
      EXPECTED_BALL_Y="${BASH_REMATCH[8]}"
      LAST_VX="${BASH_REMATCH[10]}"
      LAST_VY="${BASH_REMATCH[11]}"
      LAST_THETA="${BASH_REMATCH[12]}"

      if [[ "${BASH_REMATCH[9]}" == "true" ]]; then
        if [[ "$ACTIVE_BEHAVIOR" == "adjust" &&
              ( "$SHOOT_PATH_CLEAR" != "true" || "$SHOOT_PATH_CLEAR_SINCE_MSEC" == "0" ) ]]; then
          SHOOT_PATH_CLEAR_SINCE_MSEC="$(now_msec)"
        elif [[ "$ACTIVE_BEHAVIOR" != "adjust" ]]; then
          SHOOT_PATH_CLEAR_SINCE_MSEC=0
        fi
        SHOOT_PATH_CLEAR="true"
        SHOOT_BLOCKED_PROMPTED="false"
      else
        SHOOT_PATH_CLEAR="false"
        if [[ "$ACTIVE_BEHAVIOR" == "adjust" ]]; then
          mark_shoot_path_blocked
        else
          SHOOT_PATH_CLEAR_SINCE_MSEC=0
        fi
      fi
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
  local max_lines="${1:-$MONITOR_BATCH_LINES}"
  local total_lines
  local first_line
  local last_line

  [[ -f brain.log ]] || return 0
  total_lines="$(wc -l < brain.log | tr -d ' ')"
  [[ "$total_lines" =~ ^[0-9]+$ ]] || return 0
  ((total_lines > MONITOR_LINE)) || return 0

  first_line=$((MONITOR_LINE + 1))
  last_line="$total_lines"
  if ((max_lines > 0 && last_line - MONITOR_LINE > max_lines)); then
    last_line=$((MONITOR_LINE + max_lines))
  fi
  while IFS= read -r line; do
    process_log_line "$line"
  done < <(sed -n "${first_line},${last_line}p" brain.log)
  MONITOR_LINE="$last_line"
}

expire_readiness() {
  local now
  local age
  [[ "$SHOOT_READY" == "true" ]] || return 0
  now="$(now_msec)"
  age=$((now - LAST_READY_EVIDENCE_MSEC))
  if ((age > READY_DATA_MAX_AGE_MSEC)); then
    reset_readiness "freshness_age=${age}ms > ready_data_max_age_msec=${READY_DATA_MAX_AGE_MSEC}ms"
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
  elif [[ "$RUNTIME_SWITCH" == "striker" ]]; then
    resume_mode="$RUNTIME_STRIKER_PHASE"
  else
    resume_mode="$RUNTIME_MANUAL_MODE"
  fi
  cat <<STATUS
Autonomous status:
  switch: ${RUNTIME_SWITCH}
  manual mode: ${RUNTIME_MANUAL_MODE}
  automatic phase: ${RUNTIME_AUTO_PHASE}
  striker phase: ${RUNTIME_STRIKER_PHASE}
  score state: ${SCORE_STATE}
  active behavior: ${ACTIVE_BEHAVIOR}
  resume after ball loss: ${resume_mode}
  ball visible: ${BALL_VISIBLE}
  depth usable: ${BALL_DEPTH_USABLE}
  last ball range: ${LAST_BALL_RANGE}
  last adjustment source: ${LAST_ADJUST_SOURCE}
  last goal count: ${LAST_GOAL_COUNT}
  last goal center x: ${LAST_GOAL_CENTER_X}
  last alignment error px: ${LAST_ALIGNMENT_ERROR_PX}
  last goal aligned: ${LAST_GOAL_ALIGNED}
  last lateral source: ${LAST_LATERAL_SOURCE}
  last velocity: vx=${LAST_VX} vy=${LAST_VY} theta=${LAST_THETA}
  obstacle avoidance configured: ${OBJECT_AVOIDANCE}
  obstacle state: ${OBSTACLE_STATE}
  obstacle mode: ${OBSTACLE_MODE}
  nearest obstacle: ${NEAREST_OBSTACLE} m
  chosen side: ${OBSTACLE_SIDE}
  obstacle depth age: ${OBSTACLE_DEPTH_AGE_MSEC} ms
  obstacle ball state: ${OBSTACLE_BALL_STATE}
  expected ball: x=${EXPECTED_BALL_X} y=${EXPECTED_BALL_Y}
  shooting path clear: ${SHOOT_PATH_CLEAR}
  shoot ready: ${SHOOT_READY}
  readiness progress: controller samples=${READY_COUNT}/${READY_SAMPLES}, minimum dwell=${READY_MIN_MSEC}ms
  readiness diagnostic: ${LAST_READINESS_REASON}
  vision: $(process_state "$VISION_PID")
  brain: $(process_state "$BRAIN_PID")
  game controller: $(process_state "$GAME_CONTROLLER_PID")
STATUS
}

publish_runtime_command() {
  local command="$1"
  local attempt
  local ack_text
  local first_ack_line
  local wait_step

  case "$command" in
    autonomy_save_track) ack_text="Autonomy saved phase => track" ;;
    autonomy_save_chase) ack_text="Autonomy saved phase => chase" ;;
    autonomy_save_adjust) ack_text="Autonomy saved phase => adjust" ;;
    autonomy_auto_track) ack_text="Autonomy switch => auto (phase: track)" ;;
    autonomy_auto_chase) ack_text="Autonomy switch => auto (phase: chase)" ;;
    autonomy_auto_adjust) ack_text="Autonomy switch => auto (phase: adjust)" ;;
    autonomy_auto) ack_text="Autonomy switch => auto (phase:" ;;
    autonomy_striker) ack_text="Autonomy switch => striker (phase: track)" ;;
    autonomy_manual) ack_text="Autonomy switch => manual (mode:" ;;
    autonomy_track) ack_text="Autonomy manual mode => track" ;;
    autonomy_chase) ack_text="Autonomy manual mode => chase" ;;
    autonomy_adjust) ack_text="Autonomy manual mode => adjust" ;;
    autonomy_score) ack_text="Autonomy manual mode => score" ;;
    *)
      echo "No acknowledgement rule exists for runtime command ${command}." >&2
      return 1
      ;;
  esac

  first_ack_line=$(( $(wc -l < brain.log) + 1 ))

  for attempt in 1 2 3; do
    send_agent_command "$command" 8 || true

    for wait_step in {1..20}; do
      if awk -v first="$first_ack_line" -v ack="$ack_text" \
        'NR >= first && index($0, ack) { found=1; exit } END { exit !found }' \
        brain.log; then
        monitor_brain_log
        return 0
      fi
      sleep 0.1
    done

    echo "No brain acknowledgement for ${command} after attempt ${attempt}/3; retrying..." >&2
    check_processes
  done

  echo "The brain did not acknowledge ${command} after 3 attempts; keeping the previous selection." >&2
  return 1
}

select_manual_mode() {
  local mode="$1"
  if [[ "$mode" == "score" && "$OBJECT_AVOIDANCE" != "true" ]]; then
    echo "score requires object_avoidance=true; command ignored." >&2
    return 0
  fi
  publish_runtime_command "autonomy_${mode}" || return 0
  RUNTIME_SWITCH="manual"
  RUNTIME_MANUAL_MODE="$mode"
  ACTIVE_BEHAVIOR="${mode} pending"
  CHASE_STOP_PROMPTED="false"
  reset_readiness "operator selected manual mode=${mode}; waiting for adjustment readiness"
  echo "Manual mode selected: ${mode}"
}

select_manual_switch() {
  publish_runtime_command autonomy_manual || return 0
  if [[ "$RUNTIME_SWITCH" == "auto" ]]; then
    RUNTIME_MANUAL_MODE="$RUNTIME_AUTO_PHASE"
  elif [[ "$RUNTIME_SWITCH" == "striker" ]]; then
    RUNTIME_MANUAL_MODE="track"
  fi
  RUNTIME_SWITCH="manual"
  ACTIVE_BEHAVIOR="${RUNTIME_MANUAL_MODE} pending"
  reset_readiness "operator selected switch=manual; waiting for adjustment readiness"
  echo "Automatic transitions disabled; manual mode: ${RUNTIME_MANUAL_MODE}"
}

select_striker_switch() {
  if [[ "$OBJECT_AVOIDANCE" != "true" ]]; then
    echo "striker requires object_avoidance=true; command ignored." >&2
    return 0
  fi
  publish_runtime_command autonomy_striker || return 0
  RUNTIME_SWITCH="striker"
  RUNTIME_STRIKER_PHASE="track"
  ACTIVE_BEHAVIOR="track pending"
  SCORE_STATE="INACTIVE"
  reset_readiness "operator selected striker; starting from track"
  echo "Striker mode selected: track -> chase -> adjust -> score"
}

select_auto_switch() {
  publish_runtime_command autonomy_auto || return 0
  if [[ "$RUNTIME_MANUAL_MODE" == "chase" || "$RUNTIME_MANUAL_MODE" == "adjust" ]]; then
    RUNTIME_AUTO_PHASE="$RUNTIME_MANUAL_MODE"
  fi
  RUNTIME_SWITCH="auto"
  ACTIVE_BEHAVIOR="${RUNTIME_AUTO_PHASE} pending"
  reset_readiness "operator selected switch=auto; waiting for adjustment readiness"
  echo "Automatic transitions enabled; saved phase: ${RUNTIME_AUTO_PHASE}"
}

send_shoot() {
  local shoot_result

  if [[ "$OBJECT_AVOIDANCE" == "true" &&
        ( "$SHOOT_PATH_CLEAR" != "true" ||
          "$OBSTACLE_STATE" == "WAITING_FOR_DEPTH" ||
          "$OBSTACLE_STATE" == "SENSOR_STALE" ||
          "$OBSTACLE_STATE" == "HOLD_BLOCKED" ||
          "$OBSTACLE_STATE" == "STOP_DISTANCE" ) ]]; then
    echo "Warning: obstacle safety reports a blocked, unknown, or stale shooting path; manual shoot remains unconditional." >&2
  fi

  echo "Operator shoot received. Sending the standalone shoot command immediately..."
  set +e
  ./scripts/shoot_once.sh
  shoot_result=$?
  set -e

  if ((shoot_result == 0)); then
    echo "Shoot command sent once. Autonomous_run remains active."
  else
    echo "Shoot command failed with exit code ${shoot_result}; it was not retried." >&2
  fi
  return 0
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
    striker) select_striker_switch ;;
    manual) select_manual_switch ;;
    track|chase|adjust|score) select_manual_mode "$command" ;;
    status)
      monitor_brain_log 0
      expire_readiness
      show_status
      ;;
    help) runtime_help ;;
    shoot) send_shoot ;;
    stop) controlled_stop "operator stop" ;;
    "") ;;
    *) echo "Unknown runtime command: ${command}. Type help for commands." ;;
  esac
}

runtime_iteration() {
  local command=""

  check_processes

  if read -r -t 0.05 command; then
    handle_runtime_command "$command"
  elif [[ ! -t 0 ]]; then
    controlled_stop "input closed" 1
  fi

  monitor_brain_log
  expire_readiness
}

write_tree() {
  local tree_path="$1"
  local stop_condition
  local run_condition
  local rgb_track_condition

  if [[ "$REQUIRE_PLAY" == "true" ]]; then
    stop_condition="!autonomy_enabled || gc_game_state!='PLAY'"
    run_condition="autonomy_enabled &amp;&amp; gc_game_state=='PLAY'"
  else
    stop_condition="!autonomy_enabled || gc_game_state=='END'"
    run_condition="autonomy_enabled &amp;&amp; gc_game_state!='END'"
  fi

  if [[ "$OBJECT_AVOIDANCE" == "true" ]]; then
    # Enabled safety tracks an RGB-only reacquisition with the head while the
    # ObstacleAvoidance node holds translation until metric depth returns.
    rgb_track_condition="ball_visible"
  else
    # Preserve the exact legacy branch selection when avoidance is disabled.
    rgb_track_condition="ball_visible &amp;&amp; ball_depth_acquired"
  fi

  cat > "$tree_path" <<XML
<root BTCPP_format="4">
  <BehaviorTree ID="TrackVisible">
    <Sequence>
      <CamTrackBall stop_angle="${STOP_ANGLE}"
                    ball_yaw_gain="${BALL_YAW_GAIN}"
                    pitch_turn_gain="${PITCH_TURN_GAIN}"
                    track_turn_yaw_limit="${TRACK_TURN_YAW_LIMIT}"
                    loss_turn_pitch_limit="${LOSS_TURN_PITCH_LIMIT}"
                    head_step_rad="${HEAD_STEP_RAD}"
                    head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                    head_deadband_x_px="${HEAD_DEADBAND_X_PX}"
                    head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                    target_y_ratio="${BALL_TARGET_Y_RATIO}"
                    bottom_margin_px="${BALL_BOTTOM_MARGIN}"
                    head_pitch_limit_down="${HEAD_PITCH_LIMIT_DOWN}"
                    theta="{tracking_theta}" />
      <Script code="autonomy_motion_mode='track'; autonomy_command_vx=0.0; autonomy_command_vy=0.0; autonomy_command_theta=tracking_theta; autonomy_command_vx_limit=${VX_LIMIT}; autonomy_command_vy_limit=${VY_LIMIT}; autonomy_command_vtheta_limit=${TRACK_TURN_YAW_LIMIT}; autonomy_apply_min_theta=true" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="SearchBall">
    <Sequence>
      <Fallback>
        <Sequence>
          <ScriptCondition code="autonomy_obstacle_ball_state=='OCCLUDED_STATIONARY'" />
          <Script code="tracking_theta=0.0" />
        </Sequence>
        <CamFindBall yaw_limit="${SEARCH_YAW_LIMIT}"
                     track_turn_yaw_limit="${TRACK_TURN_YAW_LIMIT}"
                     loss_turn_pitch_limit="${LOSS_TURN_PITCH_LIMIT}"
                     head_search_speed="${HEAD_SEARCH_SPEED}"
                     body_search_speed="${BODY_SEARCH_SPEED}"
                     cmd_interval_msec="${CMD_INTERVAL_MSEC}"
                     bottom_edge_margin_px="${BOTTOM_EDGE_SEARCH_MARGIN}"
                     bottom_edge_pitch_step_rad="${BOTTOM_EDGE_SEARCH_PITCH_STEP}"
                     bottom_edge_pitch_limit_rad="${BOTTOM_EDGE_SEARCH_PITCH_LIMIT}"
                     theta="{tracking_theta}" />
      </Fallback>
      <Script code="autonomy_motion_mode='search'; autonomy_command_vx=0.0; autonomy_command_vy=0.0; autonomy_command_theta=tracking_theta; autonomy_command_vx_limit=${VX_LIMIT}; autonomy_command_vy_limit=${VY_LIMIT}; autonomy_command_vtheta_limit=${BODY_SEARCH_SPEED}; autonomy_apply_min_theta=false" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="ChaseBall">
    <Sequence>
      <CamTrackBall stop_angle="${STOP_ANGLE}"
                    ball_yaw_gain="${BALL_YAW_GAIN}"
                    pitch_turn_gain="${PITCH_TURN_GAIN}"
                    track_turn_yaw_limit="${TRACK_TURN_YAW_LIMIT}"
                    loss_turn_pitch_limit="${LOSS_TURN_PITCH_LIMIT}"
                    head_step_rad="${HEAD_STEP_RAD}"
                    head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                    head_deadband_x_px="${HEAD_DEADBAND_X_PX}"
                    head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                    target_y_ratio="${BALL_TARGET_Y_RATIO}"
                    bottom_margin_px="${BALL_BOTTOM_MARGIN}"
                    head_pitch_limit_down="${HEAD_PITCH_LIMIT_DOWN}"
                    theta="{tracking_theta}" />
      <SimpleChase vx_limit="${VX_LIMIT}"
                   vy_limit="${VY_LIMIT}"
                   stop_dist="${STOP_DIST}"
                   y_tolerance="${Y_TOLERANCE}"
                   vx="{chase_vx}"
                   vy="{chase_vy}" />
      <Script code="autonomy_motion_mode='chase'; autonomy_command_vx=chase_vx; autonomy_command_vy=chase_vy; autonomy_command_theta=tracking_theta; autonomy_command_vx_limit=${VX_LIMIT}; autonomy_command_vy_limit=${VY_LIMIT}; autonomy_command_vtheta_limit=${TRACK_TURN_YAW_LIMIT}; autonomy_apply_min_theta=true" />
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
                      goal_alignment_gain="${ADJUST_GOAL_ALIGNMENT_GAIN}"
                      goal_alignment_tolerance_px="${ADJUST_GOAL_ALIGNMENT_TOLERANCE_PX}"
                      goal_alignment_hysteresis_px="${ADJUST_GOAL_ALIGNMENT_HYSTERESIS_PX}"
                      goal_min_post_separation_px="${ADJUST_GOAL_MIN_POST_SEPARATION_PX}"
                      goal_max_age_msec="${ADJUST_GOAL_MAX_AGE_MSEC}"
                      goal_ball_max_skew_msec="${ADJUST_GOAL_BALL_MAX_SKEW_MSEC}"
                      ball_max_age_msec="${ADJUST_BALL_MAX_AGE_MSEC}"
                      ball_yaw_gain="${ADJUST_BALL_YAW_GAIN}"
                      vx_limit="${ADJUST_VX_LIMIT}"
                      vy_limit="${ADJUST_VY_LIMIT}"
                      vtheta_limit="${ADJUST_VTHETA_LIMIT}"
                      turn_first_threshold="${ADJUST_TURN_FIRST_THRESHOLD}"
                      fixed_head_yaw="${ADJUST_FIXED_HEAD_YAW}"
                      max_ball_range="${ADJUST_MAX_BALL_RANGE}"
                      target_y_ratio="${BALL_TARGET_Y_RATIO}"
                      bottom_margin_px="${BALL_BOTTOM_MARGIN}"
                      head_step_rad="${ADJUST_HEAD_STEP_RAD}"
                      head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                      head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                      head_pitch_limit_down="${HEAD_PITCH_LIMIT_DOWN}"
                      ready_samples="${READY_SAMPLES}"
                      ready_min_msec="${READY_MIN_MSEC}"
                      ready_data_max_age_msec="${READY_DATA_MAX_AGE_MSEC}"
                      reset_epoch="{autonomy_adjust_epoch}"
                      vx="{autonomy_adjust_vx}"
                      vy="{autonomy_adjust_vy}"
                      theta="{autonomy_adjust_theta}"
                      ready="{autonomy_adjust_ready}" />
      <Script code="autonomy_motion_mode='adjust'; autonomy_command_vx=autonomy_adjust_vx; autonomy_command_vy=autonomy_adjust_vy; autonomy_command_theta=autonomy_adjust_theta; autonomy_command_vx_limit=${ADJUST_VX_LIMIT}; autonomy_command_vy_limit=${ADJUST_VY_LIMIT}; autonomy_command_vtheta_limit=${ADJUST_VTHETA_LIMIT}; autonomy_apply_min_theta=true" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="StrikerChaseBall">
    <Sequence>
      <CamTrackBall stop_angle="${STOP_ANGLE}"
                    ball_yaw_gain="${BALL_YAW_GAIN}"
                    pitch_turn_gain="${PITCH_TURN_GAIN}"
                    track_turn_yaw_limit="${TRACK_TURN_YAW_LIMIT}"
                    loss_turn_pitch_limit="${LOSS_TURN_PITCH_LIMIT}"
                    head_step_rad="${HEAD_STEP_RAD}"
                    head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                    head_deadband_x_px="${HEAD_DEADBAND_X_PX}"
                    head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                    target_y_ratio="${BALL_TARGET_Y_RATIO}"
                    bottom_margin_px="${BALL_BOTTOM_MARGIN}"
                    head_pitch_limit_down="${HEAD_PITCH_LIMIT_DOWN}"
                    theta="{tracking_theta}" />
      <SimpleChase vx_limit="${VX_LIMIT}"
                   vy_limit="${VY_LIMIT}"
                   stop_dist="${STRIKER_STOP_DIST}"
                   y_tolerance="${Y_TOLERANCE}"
                   vx="{chase_vx}"
                   vy="{chase_vy}" />
      <Script code="autonomy_motion_mode='chase'; autonomy_command_vx=chase_vx; autonomy_command_vy=chase_vy; autonomy_command_theta=tracking_theta; autonomy_command_vx_limit=${VX_LIMIT}; autonomy_command_vy_limit=${VY_LIMIT}; autonomy_command_vtheta_limit=${TRACK_TURN_YAW_LIMIT}; autonomy_apply_min_theta=true" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="StrikerAdjustForShot">
    <Sequence>
      <ShootingAdjust target_range="${STRIKER_ADJUST_TARGET_RANGE}"
                      target_y_offset="${ADJUST_TARGET_Y_OFFSET}"
                      theta_offset="${ADJUST_THETA_OFFSET}"
                      range_tolerance="${ADJUST_RANGE_TOLERANCE}"
                      y_tolerance="${ADJUST_Y_TOLERANCE}"
                      stop_angle="${ADJUST_STOP_ANGLE}"
                      range_gain="${ADJUST_RANGE_GAIN}"
                      y_gain="${ADJUST_Y_GAIN}"
                      goal_alignment_gain="${ADJUST_GOAL_ALIGNMENT_GAIN}"
                      goal_alignment_tolerance_px="${ADJUST_GOAL_ALIGNMENT_TOLERANCE_PX}"
                      goal_alignment_hysteresis_px="${ADJUST_GOAL_ALIGNMENT_HYSTERESIS_PX}"
                      goal_min_post_separation_px="${ADJUST_GOAL_MIN_POST_SEPARATION_PX}"
                      goal_max_age_msec="${ADJUST_GOAL_MAX_AGE_MSEC}"
                      goal_ball_max_skew_msec="${ADJUST_GOAL_BALL_MAX_SKEW_MSEC}"
                      ball_max_age_msec="${ADJUST_BALL_MAX_AGE_MSEC}"
                      ball_yaw_gain="${ADJUST_BALL_YAW_GAIN}"
                      vx_limit="${ADJUST_VX_LIMIT}"
                      vy_limit="${ADJUST_VY_LIMIT}"
                      vtheta_limit="${ADJUST_VTHETA_LIMIT}"
                      turn_first_threshold="${ADJUST_TURN_FIRST_THRESHOLD}"
                      fixed_head_yaw="${ADJUST_FIXED_HEAD_YAW}"
                      max_ball_range="${STRIKER_ADJUST_MAX_BALL_RANGE}"
                      target_y_ratio="${BALL_TARGET_Y_RATIO}"
                      bottom_margin_px="${BALL_BOTTOM_MARGIN}"
                      head_step_rad="${ADJUST_HEAD_STEP_RAD}"
                      head_settle_step_rad="${HEAD_SETTLE_STEP_RAD}"
                      head_deadband_y_px="${HEAD_DEADBAND_Y_PX}"
                      head_pitch_limit_down="${HEAD_PITCH_LIMIT_DOWN}"
                      ready_samples="${READY_SAMPLES}"
                      ready_min_msec="${READY_MIN_MSEC}"
                      ready_data_max_age_msec="${READY_DATA_MAX_AGE_MSEC}"
                      reset_epoch="{autonomy_adjust_epoch}"
                      vx="{autonomy_adjust_vx}"
                      vy="{autonomy_adjust_vy}"
                      theta="{autonomy_adjust_theta}"
                      ready="{autonomy_adjust_ready}" />
      <Script code="autonomy_motion_mode='adjust'; autonomy_command_vx=autonomy_adjust_vx; autonomy_command_vy=autonomy_adjust_vy; autonomy_command_theta=autonomy_adjust_theta; autonomy_command_vx_limit=${ADJUST_VX_LIMIT}; autonomy_command_vy_limit=${ADJUST_VY_LIMIT}; autonomy_command_vtheta_limit=${ADJUST_VTHETA_LIMIT}; autonomy_apply_min_theta=true" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="MainTree">
    <Sequence name="autonomous run root">
      <ReactiveSequence _while="${stop_condition}" name="autonomous run disabled">
        <Script code="autonomy_command_vx=0.0; autonomy_command_vy=0.0; autonomy_command_theta=0.0; autonomy_score_active=false; autonomy_manual_mode=(autonomy_manual_mode=='score') ? 'track' : autonomy_manual_mode; autonomy_striker_phase=(autonomy_striker_phase=='score') ? 'track' : autonomy_striker_phase; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
        <SetVelocity x="{autonomy_command_vx}"
                     y="{autonomy_command_vy}"
                     theta="{autonomy_command_theta}" />
      </ReactiveSequence>

      <ReactiveSequence _while="${run_condition}" name="holistic autonomous run">
        <CheckAndStandUp />
        <Script code="autonomy_score_active=(autonomy_switch=='manual' &amp;&amp; autonomy_manual_mode=='score') || (autonomy_switch=='striker' &amp;&amp; autonomy_striker_phase=='score')" />
        <ScoreBall active="{autonomy_score_active}"
                   vx="${SCORE_VX}"
                   start_max_range="${SCORE_START_MAX_RANGE}"
                   start_max_yaw="${ADJUST_STOP_ANGLE}"
                   ball_max_age_msec="${SCORE_BALL_MAX_AGE_MSEC}"
                   ball_lost_grace_msec="${SCORE_BALL_LOST_GRACE_MSEC}"
                   max_duration_msec="${SCORE_MAX_DURATION_MSEC}"
                   head_pitch="${SCORE_HEAD_PITCH}"
                   head_pitch_limit_down="${HEAD_PITCH_LIMIT_DOWN}"
                   head_yaw="${SCORE_HEAD_YAW}"
                   command_vx="{autonomy_score_vx}"
                   command_vy="{autonomy_score_vy}"
                   command_theta="{autonomy_score_theta}"
                   done="{autonomy_score_done}"
                   state="{autonomy_score_state}" />
        <IfThenElse>
          <ScriptCondition name="Score selected?" code="autonomy_score_active" />
          <IfThenElse>
            <ScriptCondition name="Score finished?" code="autonomy_score_done" />
            <Sequence name="finish score safely">
              <IfThenElse>
                <ScriptCondition code="autonomy_switch=='striker'" />
                <Script code="autonomy_striker_phase='track'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
                <Script code="autonomy_manual_mode='track'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
              </IfThenElse>
              <Script code="autonomy_motion_mode='score'; autonomy_command_vx=0.0; autonomy_command_vy=0.0; autonomy_command_theta=0.0; autonomy_command_vx_limit=${SCORE_VX}; autonomy_command_vy_limit=0.0; autonomy_command_vtheta_limit=0.0; autonomy_apply_min_theta=false" />
            </Sequence>
            <Script code="autonomy_motion_mode='score'; autonomy_command_vx=autonomy_score_vx; autonomy_command_vy=autonomy_score_vy; autonomy_command_theta=autonomy_score_theta; autonomy_command_vx_limit=${SCORE_VX}; autonomy_command_vy_limit=0.0; autonomy_command_vtheta_limit=0.0; autonomy_apply_min_theta=false" />
          </IfThenElse>
          <IfThenElse>
            <ScriptCondition name="Usable ball depth?" code="ball_visible &amp;&amp; ball_location_known" />
            <IfThenElse>
              <ScriptCondition name="Striker switch?" code="autonomy_switch=='striker'" />
              <Sequence name="striker phase">
                <IfThenElse>
                  <ScriptCondition code="autonomy_striker_phase=='track'" />
                  <Script code="autonomy_striker_phase='chase'" />
                  <Script code="autonomy_striker_phase=autonomy_striker_phase" />
                </IfThenElse>
                <IfThenElse>
                  <ScriptCondition code="autonomy_striker_phase=='adjust' &amp;&amp; ball_range&gt;=${STRIKER_ADJUST_MAX_BALL_RANGE}" />
                  <Script code="autonomy_striker_phase='chase'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
                  <Script code="autonomy_striker_phase=autonomy_striker_phase" />
                </IfThenElse>
                <IfThenElse>
                  <ScriptCondition code="autonomy_striker_phase=='chase' &amp;&amp; ball_range&lt;=${STRIKER_STOP_DIST}" />
                  <Script code="autonomy_striker_phase='adjust'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
                  <Script code="autonomy_striker_phase=autonomy_striker_phase" />
                </IfThenElse>
                <IfThenElse>
                  <ScriptCondition code="autonomy_striker_phase=='chase'" />
                  <SubTree ID="StrikerChaseBall" _autoremap="true" />
                  <Sequence name="evaluate current striker adjustment">
                    <SubTree ID="StrikerAdjustForShot" _autoremap="true" />
                    <IfThenElse>
                      <ScriptCondition code="autonomy_adjust_ready" />
                      <Script code="autonomy_striker_phase='score'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
                      <Script code="autonomy_striker_phase=autonomy_striker_phase" />
                    </IfThenElse>
                  </Sequence>
                </IfThenElse>
              </Sequence>
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
                      <Script code="autonomy_auto_phase='adjust'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
                      <Script code="autonomy_auto_phase='chase'" />
                    </IfThenElse>
                    <IfThenElse>
                      <ScriptCondition code="autonomy_auto_phase=='chase' &amp;&amp; ball_range&lt;=${STOP_DIST}" />
                      <Script code="autonomy_auto_phase='adjust'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
                      <IfThenElse>
                        <ScriptCondition code="autonomy_auto_phase=='adjust' &amp;&amp; ball_range&gt;${ADJUST_MAX_BALL_RANGE}" />
                        <Script code="autonomy_auto_phase='chase'; autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
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
            </IfThenElse>
            <Sequence name="ball depth unavailable">
              <Script code="autonomy_adjust_ready=false; autonomy_adjust_epoch+=1" />
              <IfThenElse>
                <ScriptCondition name="RGB ball visible?" code="${rgb_track_condition}" />
                <SubTree ID="TrackVisible" _autoremap="true" />
                <SubTree ID="SearchBall" _autoremap="true" />
              </IfThenElse>
            </Sequence>
          </IfThenElse>
        </IfThenElse>
        <ObstacleAvoidance enabled="${OBJECT_AVOIDANCE}"
                           mode="{autonomy_motion_mode}"
                           avoid_distance="${OBJECT_AVOID_DISTANCE}"
                           stop_distance="${OBJECT_STOP}"
                           desired_vx="{autonomy_command_vx}"
                           desired_vy="{autonomy_command_vy}"
                           desired_theta="{autonomy_command_theta}"
                           vx_limit="{autonomy_command_vx_limit}"
                           vy_limit="{autonomy_command_vy_limit}"
                           vtheta_limit="{autonomy_command_vtheta_limit}"
                           safe_vx="{autonomy_safe_vx}"
                           safe_vy="{autonomy_safe_vy}"
                           safe_theta="{autonomy_safe_theta}"
                           apply_min_x="{autonomy_safe_apply_min_x}"
                           apply_min_y="{autonomy_safe_apply_min_y}"
                           apply_min_theta="{autonomy_safe_apply_min_theta}"
                           state="{autonomy_obstacle_state}"
                           nearest_distance="{autonomy_nearest_obstacle}"
                           side="{autonomy_obstacle_side}"
                           ball_state="{autonomy_obstacle_ball_state}"
                           expected_ball_x="{autonomy_expected_ball_x}"
                           expected_ball_y="{autonomy_expected_ball_y}"
                           depth_age_ms="{autonomy_obstacle_depth_age_ms}"
                           shoot_path_clear="{autonomy_shoot_path_clear}" />
        <SetVelocity x="{autonomy_safe_vx}"
                     y="{autonomy_safe_vy}"
                     theta="{autonomy_safe_theta}"
                     apply_min_x="{autonomy_safe_apply_min_x}"
                     apply_min_y="{autonomy_safe_apply_min_y}"
                     apply_min_theta="{autonomy_safe_apply_min_theta}" />
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
  obstacle_avoidance:="$OBJECT_AVOIDANCE" \
  object_avoid_distance:="$OBJECT_AVOID_DISTANCE" \
  object_stop:="$OBJECT_STOP" \
  agent_mode:=false \
  disable_com:=true \
  > brain.log 2>&1 &
BRAIN_PID=$!
ros2 launch game_controller launch.py > game_controller.log 2>&1 &
GAME_CONTROLLER_PID=$!

sleep 5
check_processes

RUNTIME_SWITCH="$SWITCH_MODE"
RUNTIME_MANUAL_MODE="$START_MODE"
RUNTIME_AUTO_PHASE="$START_MODE"
RUNTIME_STRIKER_PHASE="track"

if [[ "$SWITCH_MODE" == "auto" ]]; then
  publish_runtime_command "autonomy_auto_${START_MODE}" ||
    controlled_stop "failed to initialize automatic mode" 1
elif [[ "$SWITCH_MODE" == "striker" ]]; then
  publish_runtime_command autonomy_striker ||
    controlled_stop "failed to initialize striker mode" 1
elif [[ "$START_MODE" == "score" ]]; then
  publish_runtime_command autonomy_save_track ||
    controlled_stop "failed to initialize the saved automatic phase" 1
  publish_runtime_command autonomy_score ||
    controlled_stop "failed to initialize score mode" 1
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
echo "Diagnostics: tail -f brain.log | grep -E 'CamTrackBall/direct_pixel|CamFindBall/|SimpleChase/vector|ShootingAdjust/vector|ScoreBall/vector|ObstacleAvoidance/state|RobotClient/setVelocity_out'"

while true; do
  runtime_iteration
done
