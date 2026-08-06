#include "obstacle_perception_utils.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using namespace booster_soccer::perception;

void require(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error(message);
}

void testHeightAndThinStructures()
{
    require(!obstacleHeightAccepted(0.02, 0.12, 2.0), "flat ground excluded");
    require(obstacleHeightAccepted(0.13, 0.12, 2.0), "low box retained");
    require(obstacleHeightAccepted(0.50, 0.12, 2.0), "chair leg retained");
    require(obstacleHeightAccepted(1.80, 0.12, 2.0), "goalpost retained");
    require(componentEvidenceAccepted(2, 1, 3), "multi-cell thin structure retained");
    require(componentEvidenceAccepted(1, 3, 3), "strong single-cell leg retained");
    require(!componentEvidenceAccepted(1, 2, 3), "weak isolated noise rejected");

    const double gridMinimumX = obstacleGridMinimumX(3.0);
    require(gridMinimumX == -3.0,
        "grid retains the observable rear-side prediction horizon");
    require(obstacleGridContains(-0.45, 1.40, gridMinimumX, 3.0, -5.0, 5.0),
        "moving behind-side obstacle remains available to lateral prediction");

    const double pi = std::acos(-1.0);
    require(headTargetAttained(-pi + 0.02, pi - 0.02, 0.05),
        "head attainment uses wrapped yaw distance");
    require(!headTargetAttained(0.0, 0.20, 0.05),
        "a timer cannot stand in for measured head attainment");
    require(!depthViewReadyToAdvance(1300, 1000, 999, 200),
        "a depth frame captured before head settlement cannot advance a scan");
    require(!depthViewReadyToAdvance(1199, 1000, 1100, 200),
        "settled view observes its complete dwell");
    require(depthViewReadyToAdvance(1200, 1000, 1100, 200),
        "settled view advances only after a fresh depth frame and dwell");
    require(std::fabs(ballVisibleCoverageScanOffset(
                pi / 2.0, 0.0, pi / 4.0) - pi / 4.0) < 1e-12,
        "small distant ball permits a 45-degree side-coverage look");
    require(ballVisibleCoverageScanOffset(
                pi / 2.0, pi / 6.0, pi / 4.0) < pi / 4.0,
        "large nearby ball keeps its complete box in view");
    require(std::fabs(executableDetourScanOffset(
                0.40, 0.20, 0.10, 0.60, 0.50, pi / 4.0) -
            std::atan2(0.30, 0.40)) < 1e-12,
        "gap view follows the minimum-promoted executable detour heading");
    require(executableDetourScanOffset(
                0.40, 0.20, 0.10, 0.60, 0.20, pi / 4.0) == 0.0,
        "an unexecutable lateral command cannot drive a misleading gap scan");
}

void testBallMaskDoesNotErasePersonBehindBall()
{
    const bool ballPoint = physicalBallSample(
        true, 100, 100, 80, 120, 80, 120,
        1.0, 0.0, 0.15, 1.0, 0.0, 0.20, 0.30);
    require(ballPoint, "fresh low ball point masked");

    const bool personTorso = physicalBallSample(
        true, 100, 100, 80, 120, 80, 120,
        1.05, 0.0, 0.80, 1.0, 0.0, 0.20, 0.30);
    require(!personTorso, "person behind ball remains by height");

    const bool nearbyLegOutsideBox = physicalBallSample(
        true, 150, 100, 80, 120, 80, 120,
        1.05, 0.0, 0.20, 1.0, 0.0, 0.20, 0.30);
    require(!nearbyLegOutsideBox, "nearby leg remains when bbox does not confirm ball");
    require(!physicalBallSample(
        false, 100, 100, 80, 120, 80, 120,
        1.0, 0.0, 0.15, 1.0, 0.0, 0.20, 0.30),
        "stale/unconfirmed ball cannot mask depth");

    require(physicalBallComponent(
        true, 3, 2, 3, 0.11, 0.03, 0.20, 0.07),
        "depth-confirmed ball-sized connected component is maskable");
    require(!physicalBallComponent(
        true, 8, 4, 3, 0.42, 0.05, 0.20, 0.07),
        "oversized low person/box component is never masked as the ball");
    require(!physicalBallComponent(
        true, 3, 3, 3, 0.10, 0.15, 0.20, 0.07),
        "separate low component behind the ball remains an obstacle");
    require(!physicalBallComponent(
        false, 3, 3, 3, 0.10, 0.02, 0.20, 0.07),
        "stale ball evidence cannot authorize component masking");
}

void testInvalidFramesPoseSkewAndFrozenPublisher()
{
    double bearing = 0.0;
    double range = 0.0;
    require(planarRayFromCameraOrigin(
            1.10, 0.20, 0.10, 0.20, bearing, range),
        "nonzero camera-origin ray is valid");
    require(std::fabs(bearing) < 1e-12 && std::fabs(range - 1.0) < 1e-12,
        "coverage bearing/range are measured from the physical camera origin");
    require(!planarRayFromCameraOrigin(
            0.10, 0.20, 0.10, 0.20, bearing, range),
        "zero-length camera ray cannot authorize coverage");
    require(!depthFrameHasEnoughSamples(99, 1000),
        "99-percent-invalid spatial dropout frame is preserved as unknown");
    require(depthFrameHasEnoughSamples(100, 1000),
        "ten-percent globally valid frame reaches the calibration floor");
    require(!angularBinObserved(1, 3),
        "one isolated sampled column cannot authorize a five-degree wedge");
    require(angularBinObserved(3, 3),
        "three distinct sampled columns can establish angular evidence");
    require(headPoseAgeAccepted(40.0, 40.0), "40 ms pose accepted");
    require(!headPoseAgeAccepted(40.1, 40.0), "transform skew rejected");
    require(combinedFreshnessAge(5.0, 350.0, true) == 350.0,
        "frozen sensor stamp dominates fresh receive time");
    require(!std::isfinite(combinedFreshnessAge(5.0, 0.0, false)),
        "missing/zero sensor stamp fails closed despite fresh receives");
    require(!std::isfinite(combinedFreshnessAge(
        5.0, std::numeric_limits<double>::infinity(), true)),
        "future/corrupt sensor time cannot authorize motion");
    require(observationWithinMemory(300.0, 300.0),
        "coverage is retained through the configured memory boundary");
    require(!observationWithinMemory(300.1, 300.0),
        "coverage expires against planner time, not snapshot construction time");
    require(!observationWithinMemory(
        std::numeric_limits<double>::infinity(), 300.0),
        "invalid/future coverage time fails closed");
}
} // namespace

int main()
{
    try {
        testHeightAndThinStructures();
        testBallMaskDoesNotErasePersonBehindBall();
        testInvalidFramesPoseSkewAndFrozenPublisher();
    } catch (const std::exception &error) {
        std::cerr << "obstacle_perception_utils_test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "obstacle_perception_utils_test passed\n";
    return 0;
}
