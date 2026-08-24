#include "ball_occlusion_utils.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using namespace booster_soccer::occlusion;

void require(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error(message);
}

std::vector<TimedBallPoint> stationaryHistory()
{
    return {
        {0, 2.00, 0.10}, {75, 2.01, 0.10}, {150, 2.00, 0.11},
        {225, 1.99, 0.10}, {300, 2.00, 0.10}};
}

void testStationaryAndMovingLoss()
{
    const auto stationary = estimateStationaryBall(stationaryHistory(), 310);
    require(stationary.valid, "five stable samples establish stationary ball");

    auto moving = stationaryHistory();
    for (std::size_t i = 0; i < moving.size(); ++i) moving[i].odomX += 0.06 * i;
    require(!estimateStationaryBall(moving, 310).valid,
        "moving ball cannot create stale pursuit");

    auto stale = stationaryHistory();
    require(!estimateStationaryBall(stale, 900).valid,
        "old ball history cannot create occlusion");

    const std::vector<TimedBallPoint> recentFastMove{
        {0, 2.00, 0.0}, {500, 2.00, 0.0}, {625, 2.00, 0.0},
        {750, 2.00, 0.0}, {875, 2.00, 0.0}, {1000, 2.09, 0.0}};
    require(!estimateStationaryBall(recentFastMove, 1010).valid,
        "old history cannot dilute a fast movement immediately before loss");

    auto outAndBack = stationaryHistory();
    outAndBack[3].odomX += 0.09;
    require(!estimateStationaryBall(outAndBack, 310).valid,
        "recent out-and-back movement cannot masquerade as zero endpoint speed");
}

void testOccluderEvidence()
{
    require(freshObstacleOccludesExpectedBall(
        true, 2.0, 0.0, 1.0, 0.02, 0.25),
        "fresh nearer person overlapping bearing is an occluder");
    require(!freshObstacleOccludesExpectedBall(
        false, 2.0, 0.0, 1.0, 0.02, 0.25),
        "stale obstacle cannot create occlusion");
    require(!freshObstacleOccludesExpectedBall(
        true, 2.0, 0.0, 1.0, 1.0, 0.10),
        "side obstacle does not explain ball loss");
    require(!freshObstacleOccludesExpectedBall(
        true, 2.0, 0.0, 2.5, 0.0, 0.20),
        "obstacle behind expected ball does not explain loss");
}

void testReacquisitionStatesAndLimits()
{
    require(classifyBallEvidence(false, true, false, true) ==
        BallEvidenceState::VisibleRgbOnly,
        "RGB-only return remains non-metric");
    require(classifyBallEvidence(false, true, true, true) ==
        BallEvidenceState::VisibleDepth,
        "fresh depth reacquisition anywhere replaces hypothesis");
    require(classifyBallEvidence(false, false, false, true) ==
        BallEvidenceState::OccludedStationary,
        "stationary hidden ball keeps hypothesis");
    require(classifyBallEvidence(true, false, false, true) ==
        BallEvidenceState::DetectionStale,
        "detector freeze cancels stale pursuit");
    require(recoveryLimitExceeded(5001, 0.2), "five-second fallback");
    require(recoveryLimitExceeded(1000, 1.01), "one-metre fallback");
    require(!recoveryLimitExceeded(5000, 1.0), "limits are inclusive");

    int misses = updateClearViewMissCount(0, true, false, true, true);
    misses = updateClearViewMissCount(misses, true, false, true, true);
    misses = updateClearViewMissCount(misses, true, false, true, true);
    require(misses == 3, "three fresh clear-view frames invalidate empty hypothesis");
    require(updateClearViewMissCount(misses, false, false, true, true) == 3,
        "BT ticks do not count as new detector frames");
    require(updateClearViewMissCount(misses, true, true, true, true) == 0,
        "ball reacquisition resets empty-view misses");
    require(updateClearViewMissCount(misses, true, false, false, true) == 0,
        "stale depth cannot invalidate expected ball");
}
} // namespace

int main()
{
    try {
        testStationaryAndMovingLoss();
        testOccluderEvidence();
        testReacquisitionStatesAndLimits();
    } catch (const std::exception &error) {
        std::cerr << "ball_occlusion_utils_test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "ball_occlusion_utils_test passed\n";
    return 0;
}
