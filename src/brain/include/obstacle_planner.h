#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace booster_soccer
{

/** A tracked obstacle expressed in the robot frame. */
struct ObstacleComponent
{
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    double radius{0.0};
    std::string label;
};

/**
 * An observed angular interval and the known-free radial distance in it.
 * Angles are radians in the robot frame: zero is forward and positive is left.
 * Intervals whose start is greater than their end wrap across +/- pi.
 */
struct AngularFreeRangeCoverage
{
    double startAngleRad{0.0};
    double endAngleRad{0.0};
    double freeRange{0.0};
    // Ray origin in the same captured robot frame as the interval angles.
    double originX{0.0};
    double originY{0.0};
};

enum class ObstaclePlannerState
{
    Disabled,
    MapUnready,
    DepthStale,
    EmergencyStop,
    Direct,
    AvoidingLeft,
    AvoidingRight,
    NoRoute,
};

const char *toString(ObstaclePlannerState state) noexcept;

struct ObstaclePlannerInput
{
    bool enabled{false};
    bool mapReady{false};
    std::int64_t depthAgeMs{0};
    std::int64_t maxDepthAgeMs{300};

    double desiredVx{0.0};
    double desiredVy{0.0};
    double desiredTheta{0.0};
    std::string mode{"track"};

    double avoidDistance{1.0};
    double stopDistance{0.30};
    double corridorHalfWidth{0.30};
    double minExecutableX{0.0};
    double minExecutableY{0.0};
    double minExecutableTheta{0.0};
    double vxLimit{0.60};
    double vyLimit{0.40};
    double vthetaLimit{1.0};

    double ballRange{0.0};
    double ballX{0.0};
    double ballY{0.0};
    std::int64_t nowMs{0};

    // Pose of the current robot frame expressed in the robot frame in which
    // freeRanges was captured. This keeps observed-space claims anchored to
    // their original depth-camera viewpoint while the robot moves.
    double coverageOriginX{0.0};
    double coverageOriginY{0.0};
    double coverageYaw{0.0};

    std::vector<ObstacleComponent> obstacles;
    std::vector<AngularFreeRangeCoverage> freeRanges;
};

struct ObstaclePlannerOutput
{
    double safeVx{0.0};
    double safeVy{0.0};
    double safeTheta{0.0};

    bool applyMinX{false};
    bool applyMinY{false};
    bool applyMinTheta{false};

    ObstaclePlannerState state{ObstaclePlannerState::Disabled};
    std::string stateText{"DISABLED"};
    double nearestDistance{std::numeric_limits<double>::infinity()};
    int side{0}; // +1 is left, -1 is right, 0 is no selected side.
    bool shootPathClear{false};
};

/**
 * Stateful, ROS-independent velocity safety planner.
 *
 * A planner instance owns the side-choice and direct-path-release hysteresis.
 * Call reset() when its coordinate frame or obstacle source is replaced.
 */
class ObstacleVelocityPlanner
{
public:
    ObstaclePlannerOutput plan(const ObstaclePlannerInput &input);
    void reset() noexcept;

private:
    int latchedSide_{0};
    std::int64_t sideLatchUntilMs_{0};
    std::int64_t directClearSinceMs_{-1};
    std::int64_t lastNowMs_{-1};
    bool avoidanceActive_{false};
};

} // namespace booster_soccer
