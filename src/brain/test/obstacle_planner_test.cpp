#include "obstacle_planner.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

using booster_soccer::AngularFreeRangeCoverage;
using booster_soccer::ObstaclePlannerInput;
using booster_soccer::ObstaclePlannerState;
using booster_soccer::ObstacleVelocityPlanner;

constexpr double kTolerance = 1e-9;

void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(double left, double right)
{
    return std::abs(left - right) <= kTolerance;
}

double returnedPathSurfaceDistance(
    const ObstaclePlannerInput &input,
    const booster_soccer::ObstaclePlannerOutput &output,
    const booster_soccer::ObstacleComponent &obstacle)
{
    const double speed = std::hypot(output.safeVx, output.safeVy);
    if (speed <= kTolerance) {
        return std::hypot(obstacle.x, obstacle.y) - obstacle.radius;
    }
    const double pathLength = std::max(input.avoidDistance, speed);
    const double endX = pathLength * output.safeVx / speed;
    const double endY = pathLength * output.safeVy / speed;
    const double lengthSquared = endX * endX + endY * endY;
    const double projection = std::clamp(
        (obstacle.x * endX + obstacle.y * endY) / lengthSquared,
        0.0,
        1.0);
    const double closestX = projection * endX;
    const double closestY = projection * endY;
    return std::hypot(
        obstacle.x - closestX,
        obstacle.y - closestY) - obstacle.radius;
}

std::vector<AngularFreeRangeCoverage> fullCoverage(double freeRange = 5.0)
{
    return {{-4.0, 4.0, freeRange}};
}

std::vector<AngularFreeRangeCoverage> directAndLeftCoverage()
{
    return {
        {-0.15, 1.55, 5.0},
    };
}

std::vector<AngularFreeRangeCoverage> directAndRightCoverage()
{
    return {
        {-1.55, 0.15, 5.0},
    };
}

ObstaclePlannerInput normalInput()
{
    ObstaclePlannerInput input;
    input.enabled = true;
    input.mapReady = true;
    input.depthAgeMs = 20;
    input.maxDepthAgeMs = 300;
    input.desiredVx = 0.40;
    input.desiredVy = 0.0;
    input.desiredTheta = 0.20;
    input.mode = "chase";
    input.avoidDistance = 1.0;
    input.stopDistance = 0.15;
    input.corridorHalfWidth = 0.10;
    input.minExecutableX = 0.20;
    input.minExecutableY = 0.10;
    input.minExecutableTheta = 0.10;
    input.vxLimit = 0.60;
    input.vyLimit = 0.50;
    input.vthetaLimit = 0.80;
    input.ballRange = std::sqrt(2.0);
    input.ballX = 1.0;
    input.ballY = 1.0;
    input.freeRanges = fullCoverage();
    return input;
}

void testDisabledIsExactPassthrough()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.enabled = false;
    input.desiredVx = 1.20;
    input.desiredVy = -0.03;
    input.desiredTheta = 2.0;
    input.mode = "search";

    auto output = planner.plan(input);
    require(output.state == ObstaclePlannerState::Disabled, "disabled state");
    require(near(output.safeVx, 1.20), "disabled vx must be exact");
    require(near(output.safeVy, -0.03), "disabled vy must be exact");
    require(near(output.safeTheta, 2.0), "disabled theta must be exact");
    require(output.applyMinX && output.applyMinY, "disabled legacy XY min flags");
    require(!output.applyMinTheta, "search keeps its legacy theta min flag");

    input.mode = "score";
    output = planner.plan(input);
    require(output.state == ObstaclePlannerState::NoRoute,
        "score fails closed when obstacle avoidance is disabled");
    require(near(output.safeVx, 0.0) && near(output.safeVy, 0.0) &&
            near(output.safeTheta, 0.0),
        "disabled score never becomes an unchecked passthrough");
}

void testUnreadyAndStaleStop()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.mapReady = false;
    auto output = planner.plan(input);
    require(output.state == ObstaclePlannerState::MapUnready, "map-unready state");
    require(near(output.safeVx, 0.0) && near(output.safeTheta, 0.0), "map-unready stop");
    require(!output.shootPathClear, "unready map cannot clear a shot");

    input.mapReady = true;
    input.depthAgeMs = 301;
    output = planner.plan(input);
    require(output.state == ObstaclePlannerState::DepthStale, "depth-stale state");
    require(near(output.safeVx, 0.0) && near(output.safeTheta, 0.0), "stale-depth stop");
    require(!output.shootPathClear, "stale depth cannot clear a shot");
}

void testClearPathAndExecutableQuantization()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.desiredVx = 0.05;
    input.desiredVy = 0.05;
    input.desiredTheta = 0.02;

    auto output = planner.plan(input);
    require(output.state == ObstaclePlannerState::Direct, "clear direct path");
    require(near(output.safeVx, 0.20), "vx raised to executable minimum");
    require(near(output.safeVy, 0.30), "vy raised to executable lateral minimum");
    require(near(output.safeTheta, 0.10), "theta raised to executable minimum");
    require(!output.applyMinX && !output.applyMinY && !output.applyMinTheta,
        "enabled commands must not receive downstream min promotion");

    input.desiredVx = 1.0;
    input.desiredVy = 0.80;
    input.desiredTheta = 2.0;
    output = planner.plan(input);
    require(near(output.safeVx, 0.60), "vx capped");
    require(near(output.safeVy, 0.50), "vy capped");
    require(near(output.safeTheta, 0.80), "theta capped");
}

void testSearchRotationAndPureLateralMotion()
{
    ObstacleVelocityPlanner searchPlanner;
    auto input = normalInput();
    input.mode = "search";
    input.desiredVx = 0.0;
    input.desiredVy = 0.0;
    input.desiredTheta = 0.05;
    auto output = searchPlanner.plan(input);
    require(output.state == ObstaclePlannerState::Direct, "safe search rotation allowed");
    require(near(output.safeTheta, 0.05),
        "search preserves legacy sub-minimum rotation");

    ObstacleVelocityPlanner lateralPlanner;
    input = normalInput();
    input.desiredVx = 0.0;
    input.desiredVy = 0.05;
    input.desiredTheta = 0.0;
    output = lateralPlanner.plan(input);
    require(output.state == ObstaclePlannerState::Direct, "pure lateral route is valid");
    require(near(output.safeVx, 0.0), "lateral peek is not promoted forward");
    require(output.safeVy >= 0.30 - kTolerance,
        "lateral peek is quantized to an executable command");
}

void testBallIgnoredAndEmergencyStop()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.obstacles.push_back({0.10, 0.0, 0.0, 0.0, 0.20, "Ball"});
    auto output = planner.plan(input);
    require(output.state == ObstaclePlannerState::Direct, "Ball is not an obstacle");
    require(std::isinf(output.nearestDistance), "Ball excluded from nearest distance");

    input.obstacles.push_back({0.20, 0.0, 0.0, 0.0, 0.10, "Person"});
    output = planner.plan(input);
    require(output.state == ObstaclePlannerState::EmergencyStop, "stop-distance state");
    require(near(output.safeVx, 0.0) && near(output.safeVy, 0.0)
            && near(output.safeTheta, 0.0),
        "stop-distance zeros every axis");
}

void testLeftAndRightDetours()
{
    auto input = normalInput();
    input.obstacles.push_back({0.60, 0.0, 0.0, 0.0, 0.10, "Person"});

    ObstacleVelocityPlanner leftPlanner;
    input.freeRanges = directAndLeftCoverage();
    auto output = leftPlanner.plan(input);
    require(output.state == ObstaclePlannerState::AvoidingLeft, "left detour selected");
    require(output.side == 1 && output.safeVx >= 0.0, "left detour direction");
    require(output.safeVy >= 0.30 - kTolerance, "left lateral speed is executable");

    ObstacleVelocityPlanner rightPlanner;
    input.freeRanges = directAndRightCoverage();
    output = rightPlanner.plan(input);
    require(output.state == ObstaclePlannerState::AvoidingRight, "right detour selected");
    require(output.side == -1 && output.safeVx >= 0.0, "right detour direction");
    require(output.safeVy <= -0.30 + kTolerance, "right lateral speed is executable");
}

void testScoreIsDirectOrStop()
{
    auto input = normalInput();
    input.obstacles.push_back({0.60, 0.0, 0.0, 0.0, 0.10, "Goalpost"});
    input.freeRanges = directAndLeftCoverage();

    ObstacleVelocityPlanner chasePlanner;
    require(chasePlanner.plan(input).state == ObstaclePlannerState::AvoidingLeft,
        "ordinary chase may use a proven left detour");

    ObstacleVelocityPlanner scorePlanner;
    input.mode = "score";
    const auto blocked = scorePlanner.plan(input);
    require(blocked.state == ObstaclePlannerState::NoRoute,
        "score does not detour around a goalpost");
    require(near(blocked.safeVx, 0.0) && near(blocked.safeVy, 0.0),
        "blocked score stops translation");

    input.obstacles.clear();
    input.freeRanges = fullCoverage();
    const auto clear = scorePlanner.plan(input);
    require(clear.state == ObstaclePlannerState::Direct,
        "clear score path remains direct");
    require(clear.safeVx > 0.0 && near(clear.safeVy, 0.0),
        "clear score preserves straight motion");
}

void testPredictedPathKeepsStopEnvelope()
{
    ObstacleVelocityPlanner goalpostPlanner;
    auto input = normalInput();
    input.desiredVx = 0.30;
    input.desiredVy = 0.0;
    input.desiredTheta = 0.0;
    input.avoidDistance = 1.40;
    input.stopDistance = 0.50;
    input.corridorHalfWidth = 0.40;
    input.minExecutableX = 0.30;
    input.minExecutableY = 0.30;
    input.vxLimit = 0.30;
    input.vyLimit = 0.40;
    input.freeRanges = fullCoverage();
    input.obstacles = {{0.575, -0.025, 0.0, 0.0, 0.05, "Goalpost"}};

    const auto goalpost = goalpostPlanner.plan(input);
    require(goalpost.nearestDistance > input.stopDistance,
        "goalpost starts just outside the instantaneous stop envelope");
    require(
        goalpost.state == ObstaclePlannerState::NoRoute ||
        returnedPathSurfaceDistance(input, goalpost, input.obstacles.front()) +
                kTolerance >= input.stopDistance,
        "every returned goalpost route preserves the stop envelope");

    ObstacleVelocityPlanner wallPlanner;
    input.obstacles = {{0.91, 0.0, 0.0, 0.0, 0.40, "Wall"}};
    const auto wall = wallPlanner.plan(input);
    require(wall.nearestDistance > input.stopDistance,
        "wall starts just outside the instantaneous stop envelope");
    require(
        wall.state == ObstaclePlannerState::NoRoute ||
        returnedPathSurfaceDistance(input, wall, input.obstacles.front()) +
                kTolerance >= input.stopDistance,
        "every returned wall route preserves the stop envelope");
}

void testUnknownAndNarrowCoverageHaveNoRoute()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.freeRanges.clear();
    auto output = planner.plan(input);
    require(output.state == ObstaclePlannerState::NoRoute, "unknown space is unsafe");
    require(near(output.safeVx, 0.0) && near(output.safeVy, 0.0), "unknown stops translation");
    require(near(output.safeTheta, 0.20),
        "in-place rotation remains inside the already occupied footprint");

    input.freeRanges = fullCoverage(0.50);
    output = planner.plan(input);
    require(output.state == ObstaclePlannerState::NoRoute, "insufficient radial range");
    require(near(output.safeVx, 0.0) && near(output.safeVy, 0.0), "narrow range stops");

    input.freeRanges = {{-0.01, 0.01, 5.0}};
    output = planner.plan(input);
    require(output.state == ObstaclePlannerState::NoRoute,
        "observed centre ray cannot authorize unknown corridor edges");

    input.freeRanges = {
        {-3.0, 0.26, 5.0},
        {0.30, 3.0, 5.0},
    };
    output = planner.plan(input);
    require(output.state != ObstaclePlannerState::Direct,
        "an internal unknown angular strip blocks the direct swept corridor");
}

void testRememberedCoverageKeepsItsOwnOrigin()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.desiredTheta = 0.0;
    input.freeRanges = {{-1.60, 1.60, 1.5, 0.0, 0.0}};
    require(planner.plan(input).state == ObstaclePlannerState::Direct,
        "forward corridor is observed from the current ray origin");

    ObstacleVelocityPlanner shiftedPlanner;
    input.freeRanges = {{-1.60, 1.60, 1.5, 0.0, 2.0}};
    require(shiftedPlanner.plan(input).state != ObstaclePlannerState::Direct,
        "remembered ray is not re-anchored to the newest robot origin");
}

void testNoReverse()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.desiredVx = -0.40;
    const auto output = planner.plan(input);
    require(output.state == ObstaclePlannerState::NoRoute, "reverse is rejected");
    require(near(output.safeVx, 0.0) && near(output.safeVy, 0.0), "no reverse output");
}

void testSideLatchAndDirectRelease()
{
    auto input = normalInput();
    input.obstacles.push_back({0.60, 0.0, 0.0, 0.0, 0.10, "Person"});
    input.freeRanges = directAndLeftCoverage();
    input.nowMs = 0;

    ObstacleVelocityPlanner latchPlanner;
    auto output = latchPlanner.plan(input);
    require(output.side == 1, "initial left latch");
    input.freeRanges = directAndRightCoverage();
    input.nowMs = 1;
    output = latchPlanner.plan(input);
    require(output.state == ObstaclePlannerState::AvoidingRight && output.side == -1,
        "opposite safe side overrides latch instead of stopping");

    ObstacleVelocityPlanner zeroTickPlanner;
    input = normalInput();
    input.obstacles.push_back({0.60, 0.0, 0.0, 0.0, 0.10, "Person"});
    input.freeRanges = directAndLeftCoverage();
    input.nowMs = 0;
    require(zeroTickPlanner.plan(input).side == 1,
        "zero-tick test starts with a left detour");
    input.desiredVx = 0.0;
    input.desiredVy = 0.0;
    input.nowMs = 100;
    output = zeroTickPlanner.plan(input);
    require(output.side == 1,
        "one zero-translation tick preserves the selected side");
    input.desiredVx = 0.40;
    input.freeRanges = fullCoverage();
    input.nowMs = 101;
    output = zeroTickPlanner.plan(input);
    require(output.state == ObstaclePlannerState::AvoidingLeft && output.side == 1,
        "symmetric obstruction cannot flip sides through a zero tick");

    ObstacleVelocityPlanner releasePlanner;
    input = normalInput();
    input.obstacles.push_back({0.60, 0.0, 0.0, 0.0, 0.10, "Person"});
    input.freeRanges = directAndLeftCoverage();
    input.nowMs = 0;
    require(releasePlanner.plan(input).side == 1, "release test starts avoiding");
    input.obstacles.clear();
    input.freeRanges = fullCoverage();
    input.nowMs = 100;
    require(releasePlanner.plan(input).state != ObstaclePlannerState::Direct,
        "direct path is held initially");
    input.nowMs = 599;
    require(releasePlanner.plan(input).state != ObstaclePlannerState::Direct,
        "direct path remains held for 499 ms");
    input.nowMs = 600;
    require(releasePlanner.plan(input).state != ObstaclePlannerState::Direct,
        "the 800 ms side latch outlives the clear-path timer");
    input.nowMs = 799;
    require(releasePlanner.plan(input).state != ObstaclePlannerState::Direct,
        "side latch remains active through 799 ms");
    input.nowMs = 800;
    output = releasePlanner.plan(input);
    require(output.state == ObstaclePlannerState::Direct,
        "direct path releases after both latch and clear hysteresis");
    require(near(output.safeVy, 0.0), "release restores desired direct command");
}

void testDynamicCrossingBlocksDirectPath()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.obstacles.push_back({0.50, 1.0, 0.0, -2.0, 0.05, "Person"});
    const auto output = planner.plan(input);
    require(output.state != ObstaclePlannerState::Direct,
        "one-second moving-obstacle sweep blocks direct path");
}

void testRotationFootprintGuard()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.desiredVx = 0.0;
    input.desiredVy = 0.0;
    input.desiredTheta = 0.20;
    input.stopDistance = 0.10;
    input.corridorHalfWidth = 0.30;
    input.obstacles.push_back({0.32, 0.0, 0.0, 0.0, 0.03, "Person"});
    const auto output = planner.plan(input);
    require(output.state == ObstaclePlannerState::NoRoute, "rotation footprint blocked");
    require(near(output.safeTheta, 0.0), "blocked rotational footprint zeros theta");
}

void testNewCorridorRequiresObservedEdgesWithoutImpossibleSideCrescents()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    input.corridorHalfWidth = 0.40;
    input.stopDistance = 0.50;
    input.freeRanges = {{-0.65, 0.65, 5.0}};
    auto output = planner.plan(input);
    require(output.state != ObstaclePlannerState::Direct,
        "unknown leading-corridor corners remain blocked");

    planner.reset();
    input.freeRanges = {{-0.80, 0.80, 5.0}};
    output = planner.plan(input);
    require(output.state == ObstaclePlannerState::Direct,
        "a realistic centered depth view clears the new forward corridor");
    require(near(output.safeTheta, 0.20),
        "guarded chase yaw is preserved after corridor proof");
}

void testExactShootingSegmentAndBallPlane()
{
    ObstacleVelocityPlanner planner;
    auto input = normalInput();
    require(planner.plan(input).shootPathClear, "empty diagonal shot path");

    input.freeRanges = {{0.775, 0.795, 5.0}};
    require(!planner.plan(input).shootPathClear,
        "a visible shot centre ray cannot clear unknown corridor edges");
    input.freeRanges.clear();
    require(!planner.plan(input).shootPathClear,
        "an unobserved shooting corridor is blocked");
    input.freeRanges = fullCoverage();

    input.obstacles = {{0.50, 0.50, 0.0, 0.0, 0.05, "Person"}};
    require(!planner.plan(input).shootPathClear, "component on exact diagonal blocks shot");

    input.obstacles = {{0.50, 0.0, 0.0, 0.0, 0.05, "Person"}};
    require(planner.plan(input).shootPathClear,
        "component clear of diagonal proves path is not assumed to be +X");

    input.obstacles = {{1.60, 1.60, 0.0, 0.0, 0.20, "Person"}};
    require(planner.plan(input).shootPathClear,
        "component wholly beyond ball plane does not block");

    input.obstacles = {{1.20, 1.20, 0.0, 0.0, 0.50, "Person"}};
    require(!planner.plan(input).shootPathClear,
        "component radius extending before the ball plane blocks");

    input.obstacles = {{-0.80, -0.80, 0.0, 0.0, 0.20, "Person"}};
    require(planner.plan(input).shootPathClear,
        "component wholly behind robot does not block");

    input.obstacles = {{1.20, 1.20, -1.0, -1.0, 0.05, "Person"}};
    require(!planner.plan(input).shootPathClear,
        "moving sweep crossing back through shot segment blocks");
}

} // namespace

int main()
{
    try {
        testDisabledIsExactPassthrough();
        testUnreadyAndStaleStop();
        testClearPathAndExecutableQuantization();
        testSearchRotationAndPureLateralMotion();
        testBallIgnoredAndEmergencyStop();
        testLeftAndRightDetours();
        testScoreIsDirectOrStop();
        testPredictedPathKeepsStopEnvelope();
        testUnknownAndNarrowCoverageHaveNoRoute();
        testRememberedCoverageKeepsItsOwnOrigin();
        testNoReverse();
        testSideLatchAndDirectRelease();
        testDynamicCrossingBlocksDirectPath();
        testRotationFootprintGuard();
        testNewCorridorRequiresObservedEdgesWithoutImpossibleSideCrescents();
        testExactShootingSegmentAndBallPlane();
    } catch (const std::exception &error) {
        std::cerr << "obstacle_planner_test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "obstacle_planner_test passed\n";
    return 0;
}
