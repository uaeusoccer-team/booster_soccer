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
assert_contains "$RUN_OUTPUT" "object_avoidance=true" "default dry run"
assert_contains "$RUN_OUTPUT" "object_avoid_distance=1.40" "default dry run"
assert_contains "$RUN_OUTPUT" "object_stop=0.50" "default dry run"
assert_contains "$RUN_OUTPUT" "Dry run passed" "default dry run"
pass "default obstacle settings"

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

# Source only the declarations and functions above Autonomous_run.sh's main
# entry point, then generate the same tree used at runtime without launching ROS.
awk '/^parse_args "\$@"/ { exit } { print }' "$RUNNER" > "$PREFIX_PATH"
# shellcheck source=/dev/null
source "$PREFIX_PATH"
write_tree "$TREE_PATH"

python3 - "$TREE_PATH" <<'PY'
import sys
import xml.etree.ElementTree as ET


tree_path = sys.argv[1]
root = ET.parse(tree_path).getroot()

active = root.find('.//ReactiveSequence[@name="holistic autonomous run"]')
assert active is not None, 'active ReactiveSequence is missing'

active_children = list(active)
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

rgb_condition = root.find('.//ScriptCondition[@name="RGB ball visible?"]')
assert rgb_condition is not None
assert rgb_condition.attrib['code'] == 'ball_visible', (
    'enabled avoidance must head-track an RGB-only reacquisition')

print('Generated XML structure and all four motion modes are valid.')
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
