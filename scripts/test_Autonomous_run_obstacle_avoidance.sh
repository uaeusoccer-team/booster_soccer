#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="${SCRIPT_DIR}/Autonomous_run.sh"
TEST_TMP="$(mktemp -d /tmp/autonomous_run_obstacle_test.XXXXXX)"
PREFIX_PATH="${TEST_TMP}/runner_functions.sh"
TREE_PATH="${TEST_TMP}/autonomous_run.xml"
DISABLED_TREE_PATH="${TEST_TMP}/autonomous_run_disabled.xml"

RUN_OUTPUT=""
RUN_STATUS=0
TEST_COUNT=0

cleanup() {
  rm -f -- "$PREFIX_PATH" "$TREE_PATH" "$DISABLED_TREE_PATH"
  rmdir "$TEST_TMP" 2>/dev/null || true
}
trap cleanup EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

pass() {
  TEST_COUNT=$((TEST_COUNT + 1))
  echo "PASS: $*"
}

assert_contains() {
  local value="$1"
  local expected="$2"
  local context="$3"

  [[ "$value" == *"$expected"* ]] ||
    fail "${context}: expected output to contain: ${expected}"
}

run_runner() {
  set +e
  RUN_OUTPUT="$("$RUNNER" "$@" 2>&1)"
  RUN_STATUS=$?
  set -e
}

expect_success() {
  local context="$1"
  shift

  run_runner "$@"
  ((RUN_STATUS == 0)) ||
    fail "${context}: expected exit 0, got ${RUN_STATUS}: ${RUN_OUTPUT}"
}

expect_failure_2() {
  local context="$1"
  local expected="$2"
  shift 2

  run_runner "$@"
  ((RUN_STATUS == 2)) ||
    fail "${context}: expected exit 2, got ${RUN_STATUS}: ${RUN_OUTPUT}"
  assert_contains "$RUN_OUTPUT" "$expected" "$context"
  pass "$context"
}

bash -n "$RUNNER"
pass "Autonomous_run.sh has valid Bash syntax"

expect_success "default dry run" --dry-run
for expected_default in \
  "switch=manual" "start_mode=track" "require_play=false" \
  "role=striker" "team_id=5" "player_id=1" \
  "vx_limit=0.30" "vy_limit=0.30" "stop_dist=1.00" "y_tolerance=0.05" \
  "stop_angle=0.12" "ball_yaw_gain=2.0" "pitch_turn_gain=1.0" \
  "head_search_speed=0.15" "body_search_speed=0.00" \
  "track_turn_yaw_limit=0.35" "loss_turn_pitch_limit=0.70" \
  "search_yaw_limit=1.15" "cmd_interval_msec=100" \
  "head_step_rad=0.04" "head_settle_step_rad=0.02" \
  "head_deadband_x_px=35" "head_deadband_y_px=35" \
  "adjust_target_range=1.70" "adjust_target_y_offset=0.00" \
  "adjust_theta_offset=0.00" "adjust_range_tolerance=0.10" \
  "adjust_y_tolerance=0.05" "adjust_stop_angle=0.12" \
  "adjust_range_gain=1.0" "adjust_y_gain=1.0" \
  "adjust_goal_alignment_gain=1.0" \
  "adjust_goal_alignment_tolerance_px=50" \
  "adjust_goal_alignment_hysteresis_px=20" \
  "adjust_goal_min_post_separation_px=40" \
  "adjust_goal_max_age_msec=300" "adjust_goal_ball_max_skew_msec=100" \
  "adjust_ball_max_age_msec=300" "adjust_ball_yaw_gain=2.0" \
  "adjust_vx_limit=0.30" "adjust_vy_limit=0.30" \
  "adjust_vtheta_limit=0.35" "adjust_turn_first_threshold=0.50" \
  "adjust_fixed_head_yaw=0.00" "adjust_max_ball_range=2.20" \
  "ready_samples=10" "ready_min_msec=1000" "ready_data_max_age_msec=500" \
  "object_avoidance=true" "object_avoid_distance=1.40" "object_stop=0.50"
do
  assert_contains "$RUN_OUTPUT" "$expected_default" "default dry run"
done
assert_contains "$RUN_OUTPUT" "Dry run passed" "default dry run"
pass "requested defaults and obstacle settings"

expect_success "striker dry run" switch=striker start_mode=track --dry-run
assert_contains "$RUN_OUTPUT" "switch=striker" "striker dry run"
assert_contains "$RUN_OUTPUT" "striker_stop_dist=1.30" "striker dry run"
assert_contains "$RUN_OUTPUT" "striker_adjust_target_range=1.30" "striker dry run"
assert_contains "$RUN_OUTPUT" "striker_adjust_max_ball_range=1.70" "striker dry run"
pass "striker mode settings"

expect_success "score dry run" switch=manual start_mode=score --dry-run
assert_contains "$RUN_OUTPUT" "start_mode=score" "score dry run"
assert_contains "$RUN_OUTPUT" "score_vx=1.00" "score dry run"
pass "score operation settings"

expect_success "disabled dry run" object_avoidance=false --dry-run
assert_contains "$RUN_OUTPUT" "object_avoidance=false" "disabled dry run"
pass "object_avoidance=false"

expect_success "alias dry run" \
  object_avoidnce=false \
  object_stop_distance=0.60 \
  object_avoid_distance=1.40 \
  --dry-run
assert_contains "$RUN_OUTPUT" "object_avoidance=false" "alias dry run"
assert_contains "$RUN_OUTPUT" "object_stop=0.60" "alias dry run"
assert_contains "$RUN_OUTPUT" \
  "Warning: object_avoidnce is deprecated; use object_avoidance." \
  "alias dry run"
assert_contains "$RUN_OUTPUT" \
  "object_stop_distance is a compatibility alias for object_stop." \
  "alias dry run"
pass "deprecated and compatibility aliases"

expect_success "help" --help
assert_contains "$RUN_OUTPUT" "object_avoidance=true" "help"
assert_contains "$RUN_OUTPUT" "object_avoid_distance=1.40" "help"
assert_contains "$RUN_OUTPUT" "object_stop=0.50" "help"
pass "help exposes obstacle settings"

expect_failure_2 \
  "zero stop distance rejected" \
  "object_stop (0) must be greater than 0" \
  object_stop=0 \
  --dry-run

expect_failure_2 \
  "stop must be below avoid distance" \
  "object_stop (1.40) must be less than object_avoid_distance (1.40)" \
  object_stop=1.40 \
  object_avoid_distance=1.40 \
  --dry-run

expect_failure_2 \
  "avoid distance limited to depth map" \
  "object_avoid_distance (3.01) must not exceed the 3.0 m depth-map range" \
  object_avoid_distance=3.01 \
  --dry-run

expect_failure_2 \
  "unguarded score rejected" \
  "score and striker require object_avoidance=true" \
  object_avoidance=false \
  start_mode=score \
  --dry-run

expect_failure_2 \
  "unguarded striker rejected" \
  "score and striker require object_avoidance=true" \
  object_avoidance=false \
  switch=striker \
  --dry-run

expect_failure_2 \
  "bottom recovery cap rejected" \
  "bottom_edge_search_pitch_limit (0.91) must not exceed head_pitch_limit_down (0.90)" \
  bottom_edge_search_pitch_limit=0.91 \
  --dry-run

expect_failure_2 \
  "physical head pitch cap rejected" \
  "head_pitch_limit_down (0.91) must not exceed the physical safety cap (0.90)" \
  head_pitch_limit_down=0.91 \
  --dry-run

expect_failure_2 \
  "score head pitch cap rejected" \
  "score_head_pitch (0.91) must not exceed head_pitch_limit_down (0.90)" \
  score_head_pitch=0.91 \
  --dry-run

# Source only the declarations and functions above Autonomous_run.sh's main
# entry point, then generate the same tree used at runtime without launching ROS.
awk '/^parse_args "\$@"/ { exit } { print }' "$RUNNER" > "$PREFIX_PATH"
# shellcheck source=/dev/null
source "$PREFIX_PATH"

ACTIVE_BEHAVIOR="adjust"
SHOOT_READY="true"
READY_COUNT=10
process_log_line "ScoreBall/vector state: INACTIVE active: 0 freshVisualBall: 1 freshDepthBall: 1 ballRange: 1.300 ballAgeMsec: 20.0 elapsedMsec: 0.0 lostMsec: 0.0 vx: 0.000 vy: 0.000 theta: 0.000 done: 0"
[[ "$ACTIVE_BEHAVIOR" == "adjust" && "$SHOOT_READY" == "true" && "$READY_COUNT" == "10" ]] ||
  fail "inactive ScoreBall heartbeat must not override active adjustment diagnostics"
pass "inactive score diagnostics are ignored"

OBJECT_AVOIDANCE="false"
SHOOT_READY="false"
READY_PROMPTED="false"
READY_COUNT=0
adjust_not_ready="ShootingAdjust/vector ballX: 1.300 ballY: 0.000 ballRange: 1.300 maxBallRange: 1.700 ballYaw: 0.000 rangeError: 0.000 yError: 0.000 thetaError: 0.000 goalCount: 2 goalCenterX: 640.000 alignmentErrorPx: 0.000 goalAligned: 1 lateralSource: GOAL_CENTER vx: 0.000 vy: 0.000 theta: 0.000 source: DEADBAND measuredHeadPitch: 0.450 commandedHeadPitch: 0.450 targetY: 504.0 bboxYMax: 600.0 bottomMarginPx: 30.0 measuredHeadYaw: 0.000 fixedHeadYaw: 0.000 headCommandSent: 0 visualSearchDirection: 0 goalPair: LABELED goalSeparationPx: 400.0 ballAgeMsec: 20.0 readyCandidate: 1 readySamples: 1/10 ready: 0"
for _ in $(seq 1 20); do
  process_log_line "$adjust_not_ready"
done
[[ "$READY_COUNT" == "1" && "$SHOOT_READY" == "false" ]] ||
  fail "shell diagnostics must mirror the C++ distinct-frame ready sample count"

adjust_ready="${adjust_not_ready%readySamples:*}readySamples: 10/10 ready: 1"
process_log_line "$adjust_ready" >/dev/null
[[ "$READY_COUNT" == "10" && "$SHOOT_READY" == "true" ]] ||
  fail "shell diagnostics must accept the C++ ready latch"

adjust_unavailable="ShootingAdjust/vector ballX: 0.000 ballY: 0.000 ballRange: 0.000 maxBallRange: 1.700 ballYaw: 0.000 rangeError: 0.000 yError: 0.000 thetaError: 0.000 goalCount: 0 goalCenterX: 0.000 alignmentErrorPx: 0.000 goalAligned: 0 lateralSource: NO_GOAL vx: 0.000 vy: 0.000 theta: 0.000 source: NO_BALL ballAgeMsec: 999.0 readyCandidate: 0 readySamples: 0/10 ready: 0"
process_log_line "$adjust_unavailable" >/dev/null
[[ "$BALL_VISIBLE" == "false" && "$BALL_DEPTH_USABLE" == "false" && "$SHOOT_READY" == "false" ]] ||
  fail "unavailable adjustment diagnostics must clear perception and readiness"
OBJECT_AVOIDANCE="true"
pass "readiness diagnostics mirror controller samples"

write_tree "$TREE_PATH"

python3 - "$TREE_PATH" <<'PY'
import sys
import xml.etree.ElementTree as ET


tree_path = sys.argv[1]
root = ET.parse(tree_path).getroot()

active = root.find('.//ReactiveSequence[@name="holistic autonomous run"]')
assert active is not None, 'active ReactiveSequence is missing'

active_children = list(active)
disabled = root.find('.//ReactiveSequence[@name="autonomous run disabled"]')
assert disabled is not None
disabled_script = disabled.find('./Script')
assert disabled_script is not None
assert 'autonomy_score_active=false' in disabled_script.attrib['code']
assert "autonomy_manual_mode=='score'" in disabled_script.attrib['code']
assert "autonomy_striker_phase=='score'" in disabled_script.attrib['code']
guard_indexes = [
    index for index, child in enumerate(active_children)
    if child.tag == 'ObstacleAvoidance'
]
velocity_indexes = [
    index for index, child in enumerate(active_children)
    if child.tag == 'SetVelocity'
]

assert len(root.findall('.//ObstacleAvoidance')) == 1, (
    'generated tree must contain exactly one ObstacleAvoidance node')
assert len(guard_indexes) == 1, (
    'active sequence must contain exactly one direct ObstacleAvoidance node')
assert len(velocity_indexes) == 1, (
    'active sequence must contain exactly one direct SetVelocity node')
assert velocity_indexes[0] == guard_indexes[0] + 1, (
    'ObstacleAvoidance must be immediately before active SetVelocity')

score_indexes = [
    index for index, child in enumerate(active_children)
    if child.tag == 'ScoreBall'
]
assert len(score_indexes) == 1, 'active sequence must tick exactly one ScoreBall'
assert score_indexes[0] < guard_indexes[0], (
    'ScoreBall must run before the final obstacle guard')

for node in root.findall('.//IfThenElse'):
    assert len(list(node)) == 3, (
        'every IfThenElse must have condition/then/else children')

guard = active_children[guard_indexes[0]]
velocity = active_children[velocity_indexes[0]]
assert guard.attrib['enabled'] == 'true'
assert guard.attrib['avoid_distance'] == '1.40'
assert guard.attrib['stop_distance'] == '0.50'
assert guard.attrib['desired_vx'] == '{autonomy_command_vx}'
assert guard.attrib['desired_vy'] == '{autonomy_command_vy}'
assert guard.attrib['desired_theta'] == '{autonomy_command_theta}'
assert guard.attrib['vx_limit'] == '{autonomy_command_vx_limit}'
assert guard.attrib['vy_limit'] == '{autonomy_command_vy_limit}'
assert guard.attrib['vtheta_limit'] == '{autonomy_command_vtheta_limit}'
assert velocity.attrib['x'] == '{autonomy_safe_vx}'
assert velocity.attrib['y'] == '{autonomy_safe_vy}'
assert velocity.attrib['theta'] == '{autonomy_safe_theta}'
assert velocity.attrib['apply_min_x'] == '{autonomy_safe_apply_min_x}'
assert velocity.attrib['apply_min_y'] == '{autonomy_safe_apply_min_y}'
assert velocity.attrib['apply_min_theta'] == '{autonomy_safe_apply_min_theta}'

for tree_id, mode in (
    ('TrackVisible', 'track'),
    ('SearchBall', 'search'),
    ('ChaseBall', 'chase'),
    ('AdjustForShot', 'adjust'),
    ('StrikerChaseBall', 'chase'),
    ('StrikerAdjustForShot', 'adjust'),
):
    behavior_tree = root.find(f'.//BehaviorTree[@ID="{tree_id}"]')
    assert behavior_tree is not None, f'{tree_id} subtree is missing'
    scripts = [
        node.attrib.get('code', '')
        for node in behavior_tree.findall('.//Script')
    ]
    assert any(
        f"autonomy_motion_mode='{mode}'" in code
        and 'autonomy_command_vx=' in code
        and 'autonomy_command_vy=' in code
        and 'autonomy_command_theta=' in code
        for code in scripts
    ), f'{tree_id} does not set mode={mode} and all desired commands'

for tracker in root.findall('.//CamTrackBall'):
    assert tracker.attrib['target_y_ratio'] == '0.70'
    assert tracker.attrib['bottom_margin_px'] == '30'

score = root.find('.//ScoreBall')
assert score is not None
assert score.attrib['vx'] == '1.00'
assert score.attrib['start_max_range'] == '1.70'
assert score.attrib['start_max_yaw'] == '0.12'
assert score.attrib['ball_lost_grace_msec'] == '800'
assert score.attrib['max_duration_msec'] == '2500'
assert score.attrib['head_pitch'] == '0.45'
assert score.attrib['head_pitch_limit_down'] == '0.90'

striker_chase = root.find('.//BehaviorTree[@ID="StrikerChaseBall"]')
striker_chase_node = striker_chase.find('.//SimpleChase')
assert striker_chase_node.attrib['stop_dist'] == '1.30'

striker_adjust = root.find('.//BehaviorTree[@ID="StrikerAdjustForShot"]')
striker_adjust_node = striker_adjust.find('.//ShootingAdjust')
assert striker_adjust_node.attrib['target_range'] == '1.30'
assert striker_adjust_node.attrib['max_ball_range'] == '1.70'
assert striker_adjust_node.attrib['ready_data_max_age_msec'] == '500'
assert striker_adjust_node.attrib['reset_epoch'] == '{autonomy_adjust_epoch}'

for adjust in root.findall('.//ShootingAdjust'):
    assert adjust.attrib['reset_epoch'] == '{autonomy_adjust_epoch}'

scripts = [node.attrib.get('code', '') for node in root.findall('.//Script')]
assert any('autonomy_adjust_epoch+=1' in code for code in scripts), (
    'generated tree must invalidate private adjustment readiness on phase changes')

current_adjust = root.find('.//Sequence[@name="evaluate current striker adjustment"]')
assert current_adjust is not None
current_adjust_children = list(current_adjust)
assert current_adjust_children[0].tag == 'SubTree'
assert current_adjust_children[0].attrib['ID'] == 'StrikerAdjustForShot'
assert current_adjust_children[1].tag == 'IfThenElse'
ready_condition = current_adjust_children[1].find('./ScriptCondition')
assert ready_condition is not None
assert ready_condition.attrib['code'] == 'autonomy_adjust_ready'

max_range_condition = next(
    node for node in root.findall('.//ScriptCondition')
    if "autonomy_striker_phase=='adjust'" in node.attrib.get('code', '')
    and 'ball_range>=' in node.attrib.get('code', ''))
assert max_range_condition.attrib['code'].endswith('ball_range>=1.70')

rgb_condition = root.find('.//ScriptCondition[@name="RGB ball visible?"]')
assert rgb_condition is not None
assert rgb_condition.attrib['code'] == 'ball_visible', (
    'enabled avoidance must head-track an RGB-only reacquisition')

print('Generated XML structure, framing, score, and striker modes are valid.')
PY

OBJECT_AVOIDANCE=false
write_tree "$DISABLED_TREE_PATH"
python3 - "$DISABLED_TREE_PATH" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
guard = root.find('.//ObstacleAvoidance')
assert guard is not None and guard.attrib['enabled'] == 'false'
rgb_condition = root.find('.//ScriptCondition[@name="RGB ball visible?"]')
assert rgb_condition is not None
assert rgb_condition.attrib['code'] == 'ball_visible && ball_depth_acquired', (
    'disabled avoidance must preserve the legacy RGB-only branch')
print('Disabled XML preserves legacy RGB-only branch selection.')
PY
pass "generated obstacle-avoidance tree structure"

echo "All ${TEST_COUNT} Autonomous_run obstacle-avoidance regression checks passed."
