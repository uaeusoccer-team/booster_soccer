#include "shooting_adjust_utils.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

using shooting_adjust::AlignmentDeadband;
using shooting_adjust::GoalpostPixelObservation;
using shooting_adjust::GoalpostSide;

void expect(bool condition, const std::string &message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void expectNear(double actual, double expected, double tolerance, const std::string &message)
{
    expect(std::fabs(actual - expected) <= tolerance, message);
}

GoalpostPixelObservation post(GoalpostSide side, double pixelX, bool eligible = true)
{
    return {side, pixelX, 90.0, eligible};
}

void testUniqueOpponentPair()
{
    const auto selection = shooting_adjust::selectGoalCenter(
        {post(GoalpostSide::Right, 900.0), post(GoalpostSide::Left, 300.0)},
        1280.0,
        40.0);

    expect(selection.valid, "a unique OL/OR pair should be accepted");
    expectNear(selection.leftX, 300.0, 1e-9, "left image edge");
    expectNear(selection.rightX, 900.0, 1e-9, "right image edge");
    expectNear(selection.centerX, 600.0, 1e-9, "goal midpoint");
    expectNear(selection.separationPx, 600.0, 1e-9, "post separation");
    expect(selection.eligibleCount == 2, "eligible post count");
}

void testPairingFailsClosed()
{
    expect(
        !shooting_adjust::selectGoalCenter(
             {post(GoalpostSide::Unknown, 300.0), post(GoalpostSide::Unknown, 900.0)},
             1280.0,
             40.0)
             .valid,
        "unlabeled visual posts must not be used");

    expect(
        !shooting_adjust::selectGoalCenter(
             {post(GoalpostSide::Left, 300.0),
              post(GoalpostSide::Right, 900.0),
              post(GoalpostSide::Unknown, 1100.0)},
             1280.0,
             40.0)
             .valid,
        "an additional eligible unlabeled post must make the target ambiguous");

    expect(
        !shooting_adjust::selectGoalCenter(
             {post(GoalpostSide::Left, 250.0),
              post(GoalpostSide::Left, 300.0),
              post(GoalpostSide::Right, 900.0)},
             1280.0,
             40.0)
             .valid,
        "duplicate same-side detections must be rejected as ambiguous");

    expect(
        !shooting_adjust::selectGoalCenter(
             {post(GoalpostSide::Left, 500.0), post(GoalpostSide::Right, 530.0)},
             1280.0,
             40.0)
             .valid,
        "posts below minimum separation must be rejected");

    expect(
        !shooting_adjust::selectGoalCenter(
             {post(GoalpostSide::Left, -1.0), post(GoalpostSide::Right, 900.0)},
             1280.0,
             40.0)
             .valid,
        "out-of-image posts must be rejected");

    expect(
        !shooting_adjust::selectGoalCenter(
             {post(GoalpostSide::Left, 300.0, false), post(GoalpostSide::Right, 900.0)},
             1280.0,
             40.0)
             .valid,
        "ineligible posts must be rejected");
}

void testLateralDirectionAndLimit()
{
    AlignmentDeadband deadband;
    deadband.configure(35.0, 15.0);

    const double rightCommand = shooting_adjust::lateralVelocityFromPixelError(
        -200.0, 1280.0, 1.0, 0.4, deadband);
    expect(rightCommand < 0.0, "goal left of ball must command robot-right vy");

    deadband.reset();
    const double leftCommand = shooting_adjust::lateralVelocityFromPixelError(
        200.0, 1280.0, 1.0, 0.4, deadband);
    expect(leftCommand > 0.0, "goal right of ball must command robot-left vy");

    deadband.reset();
    expectNear(
        shooting_adjust::lateralVelocityFromPixelError(
            1000.0, 1280.0, 10.0, 0.4, deadband),
        0.4,
        1e-9,
        "positive vy must be capped");

    deadband.reset();
    expectNear(
        shooting_adjust::lateralVelocityFromPixelError(
            -1000.0, 1280.0, 10.0, 0.4, deadband),
        -0.4,
        1e-9,
        "negative vy must be capped");
}

void testDeadbandAndHysteresis()
{
    AlignmentDeadband deadband;
    deadband.configure(35.0, 15.0);

    expect(!deadband.update(35.0), "initial target inside tolerance is aligned");
    expect(deadband.aligned(), "aligned state is reported");
    expect(!deadband.update(50.0), "hysteresis boundary does not restart motion");
    expect(deadband.update(50.1), "error beyond tolerance plus hysteresis restarts motion");
    expect(deadband.active(), "active state is reported");
    expect(deadband.update(35.1), "active motion continues until base tolerance");
    expect(!deadband.update(35.0), "base tolerance stops active motion");

    expect(!deadband.update(std::optional<double>{}), "missing target stops motion");
    expect(!deadband.hasTarget(), "missing target resets alignment state");

    expect(deadband.update(35.1), "a new target just outside tolerance starts motion");
}

} // namespace

int main()
{
    testUniqueOpponentPair();
    testPairingFailsClosed();
    testLateralDirectionAndLimit();
    testDeadbandAndHysteresis();
    std::cout << "shooting_adjust_utils_test: PASS\n";
    return EXIT_SUCCESS;
}
