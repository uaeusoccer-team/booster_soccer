#include "obstacle_planner.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>

namespace booster_soccer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
// +/-90 degrees is the closed boundary of the forward half-plane.  The
// interior candidates remain spaced at ten degrees; the boundary enables a
// genuine lateral peek without ever authorizing reverse motion.
constexpr double kCandidateLimitRad = 90.0 * kPi / 180.0;
constexpr double kCandidateStepRad = 10.0 * kPi / 180.0;
constexpr double kPredictionSeconds = 1.0;
constexpr double kLateralCommandFloor = 0.30;
constexpr double kLongitudinalCoverageStep = 0.05;
constexpr double kLateralCoverageStep = 0.01;
constexpr std::int64_t kSideLatchMs = 800;
constexpr std::int64_t kDirectReleaseMs = 500;
constexpr double kEpsilon = 1e-9;

struct Vec2
{
    double x{0.0};
    double y{0.0};
};

struct Candidate
{
    double vx{0.0};
    double vy{0.0};
    double clearance{-std::numeric_limits<double>::infinity()};
    double score{-std::numeric_limits<double>::infinity()};
    int side{0};
    bool valid{false};
};

struct PreparedCoverageInterval
{
    double startAngle{0.0};
    double endAngle{0.0};
    double freeRange{0.0};
    bool wraps{false};
    bool coversAll{false};
    bool valid{false};
};

struct PreparedCoverageGroup
{
    Vec2 origin;
    std::vector<PreparedCoverageInterval> intervals;
};

using PreparedCoverage = std::vector<PreparedCoverageGroup>;

double clampNonnegative(double value) noexcept
{
    return std::isfinite(value) ? std::max(0.0, value) : 0.0;
}

double quantizeAxis(double value, double minimum, double limit) noexcept
{
    if (!std::isfinite(value)) {
        return 0.0;
    }
    minimum = clampNonnegative(minimum);
    limit = clampNonnegative(limit);
    if (std::abs(value) <= kEpsilon || limit <= kEpsilon || minimum > limit + kEpsilon) {
        return 0.0;
    }
    const double magnitude = std::min(limit, std::max(std::abs(value), minimum));
    return std::copysign(magnitude, value);
}

double limitAxis(double value, double limit) noexcept
{
    if (!std::isfinite(value)) return 0.0;
    limit = clampNonnegative(limit);
    return std::clamp(value, -limit, limit);
}

double norm(Vec2 value) noexcept
{
    return std::hypot(value.x, value.y);
}

Vec2 subtract(Vec2 left, Vec2 right) noexcept
{
    return {left.x - right.x, left.y - right.y};
}

Vec2 add(Vec2 left, Vec2 right) noexcept
{
    return {left.x + right.x, left.y + right.y};
}

Vec2 multiply(Vec2 value, double scalar) noexcept
{
    return {value.x * scalar, value.y * scalar};
}

double dot(Vec2 left, Vec2 right) noexcept
{
    return left.x * right.x + left.y * right.y;
}

double normalizeAngle(double angle) noexcept
{
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

double angleDistance(double left, double right) noexcept
{
    return std::abs(normalizeAngle(left - right));
}

bool equalsIgnoreCase(const std::string &left, const char *right)
{
    const std::string rhs(right);
    if (left.size() != rhs.size()) {
        return false;
    }
    for (std::size_t i = 0; i < left.size(); ++i) {
        const auto lhsChar = static_cast<unsigned char>(left[i]);
        const auto rhsChar = static_cast<unsigned char>(rhs[i]);
        if (std::tolower(lhsChar) != std::tolower(rhsChar)) {
            return false;
        }
    }
    return true;
}

bool isBall(const ObstacleComponent &component)
{
    return equalsIgnoreCase(component.label, "Ball");
}

bool validComponent(const ObstacleComponent &component) noexcept
{
    return std::isfinite(component.x) && std::isfinite(component.y)
        && std::isfinite(component.vx) && std::isfinite(component.vy)
        && std::isfinite(component.radius) && component.radius >= 0.0;
}

double pointToSegmentDistance(Vec2 point, Vec2 start, Vec2 end) noexcept
{
    const Vec2 segment = subtract(end, start);
    const double lengthSquared = dot(segment, segment);
    if (lengthSquared <= kEpsilon) {
        return norm(subtract(point, start));
    }

    const double projection = std::clamp(
        dot(subtract(point, start), segment) / lengthSquared, 0.0, 1.0);
    const Vec2 closest = add(start, multiply(segment, projection));
    return norm(subtract(point, closest));
}

double orientation(Vec2 first, Vec2 second, Vec2 third) noexcept
{
    return (second.x - first.x) * (third.y - first.y)
        - (second.y - first.y) * (third.x - first.x);
}

bool onSegment(Vec2 point, Vec2 start, Vec2 end) noexcept
{
    return point.x >= std::min(start.x, end.x) - kEpsilon
        && point.x <= std::max(start.x, end.x) + kEpsilon
        && point.y >= std::min(start.y, end.y) - kEpsilon
        && point.y <= std::max(start.y, end.y) + kEpsilon;
}

bool segmentsIntersect(Vec2 firstStart, Vec2 firstEnd, Vec2 secondStart, Vec2 secondEnd) noexcept
{
    const double o1 = orientation(firstStart, firstEnd, secondStart);
    const double o2 = orientation(firstStart, firstEnd, secondEnd);
    const double o3 = orientation(secondStart, secondEnd, firstStart);
    const double o4 = orientation(secondStart, secondEnd, firstEnd);

    if (((o1 > kEpsilon && o2 < -kEpsilon) || (o1 < -kEpsilon && o2 > kEpsilon))
        && ((o3 > kEpsilon && o4 < -kEpsilon) || (o3 < -kEpsilon && o4 > kEpsilon))) {
        return true;
    }
    if (std::abs(o1) <= kEpsilon && onSegment(secondStart, firstStart, firstEnd)) {
        return true;
    }
    if (std::abs(o2) <= kEpsilon && onSegment(secondEnd, firstStart, firstEnd)) {
        return true;
    }
    if (std::abs(o3) <= kEpsilon && onSegment(firstStart, secondStart, secondEnd)) {
        return true;
    }
    return std::abs(o4) <= kEpsilon && onSegment(firstEnd, secondStart, secondEnd);
}

double segmentToSegmentDistance(Vec2 firstStart, Vec2 firstEnd, Vec2 secondStart, Vec2 secondEnd) noexcept
{
    if (segmentsIntersect(firstStart, firstEnd, secondStart, secondEnd)) {
        return 0.0;
    }
    return std::min({
        pointToSegmentDistance(firstStart, secondStart, secondEnd),
        pointToSegmentDistance(firstEnd, secondStart, secondEnd),
        pointToSegmentDistance(secondStart, firstStart, firstEnd),
        pointToSegmentDistance(secondEnd, firstStart, firstEnd),
    });
}

PreparedCoverage prepareCoverage(
    const std::vector<AngularFreeRangeCoverage> &coverage)
{
    PreparedCoverage prepared;
    prepared.reserve(coverage.size());
    for (const auto &entry : coverage) {
        if (!std::isfinite(entry.originX) || !std::isfinite(entry.originY)) {
            continue;
        }
        if (prepared.empty() || prepared.back().origin.x != entry.originX ||
            prepared.back().origin.y != entry.originY) {
            prepared.push_back({{entry.originX, entry.originY}, {}});
        }

        PreparedCoverageInterval interval;
        interval.freeRange = entry.freeRange;
        interval.valid = std::isfinite(entry.startAngleRad) &&
            std::isfinite(entry.endAngleRad) &&
            std::isfinite(entry.freeRange) && entry.freeRange >= 0.0;
        if (interval.valid) {
            const double rawSpan = entry.endAngleRad - entry.startAngleRad;
            interval.coversAll = std::abs(rawSpan) >= kTwoPi - kEpsilon;
            interval.startAngle = normalizeAngle(entry.startAngleRad);
            interval.endAngle = normalizeAngle(entry.endAngleRad);
            interval.wraps = interval.startAngle > interval.endAngle;
        }
        prepared.back().intervals.push_back(interval);
    }
    return prepared;
}

bool angleInCoverage(
    double normalizedAngle,
    const PreparedCoverageInterval &coverage) noexcept
{
    if (!coverage.valid) return false;
    if (coverage.coversAll) return true;
    if (!coverage.wraps) {
        return normalizedAngle >= coverage.startAngle - kEpsilon &&
            normalizedAngle <= coverage.endAngle + kEpsilon;
    }
    return normalizedAngle >= coverage.startAngle - kEpsilon ||
        normalizedAngle <= coverage.endAngle + kEpsilon;
}

double observedPointClearance(
    Vec2 point,
    const PreparedCoverage &coverage) noexcept
{
    double result = -std::numeric_limits<double>::infinity();
    for (const auto &group : coverage) {
        const Vec2 relative{
            point.x - group.origin.x,
            point.y - group.origin.y};
        const double range = norm(relative);
        if (range <= kEpsilon) {
            const bool hasValidInterval = std::any_of(
                group.intervals.begin(),
                group.intervals.end(),
                [](const PreparedCoverageInterval &interval) {
                    return interval.valid;
                });
            if (hasValidInterval) {
                return std::numeric_limits<double>::infinity();
            }
            continue;
        }
        const double bearing = normalizeAngle(std::atan2(relative.y, relative.x));
        for (const auto &interval : group.intervals) {
            if (!angleInCoverage(bearing, interval)) continue;
            result = std::max(result, interval.freeRange - range);
        }
    }
    return result;
}

double observedCorridorClearance(
    double angle,
    double pathLength,
    double corridorHalfWidth,
    const ObstaclePlannerInput &input,
    const PreparedCoverage &coverage) noexcept
{
    // Depth cannot observe through the robot itself. Treat the current
    // collision footprint as the already occupied near field, then require every
    // cross-section of the *new* motion-aligned corridor to be observed. Known
    // components are still checked against the complete capsule from the base
    // origin, including this near field, by sweptPathClearance().
    //
    // Starting at its front edge keeps a forward corridor inside a path-centred
    // camera view. It also avoids the impossible +/-90-degree side-crescent
    // proof that made a clear field permanently immobile with one forward-facing
    // depth camera. stopDistance is deliberately not used for this exemption:
    // increasing the public emergency distance must never enlarge a blind zone.
    const double halfWidth = clampNonnegative(corridorHalfWidth);
    const double guardedNearField = halfWidth;
    const double step = kLongitudinalCoverageStep;
    const Vec2 forward{std::cos(angle), std::sin(angle)};
    const Vec2 lateral{-forward.y, forward.x};
    const double coverageCos = std::cos(input.coverageYaw);
    const double coverageSin = std::sin(input.coverageYaw);
    double clearance = std::numeric_limits<double>::infinity();

    auto checkCrossSection = [&](double longitudinal) {
        const int lateralSegments = std::max(
            1,
            static_cast<int>(std::ceil(
                (2.0 * halfWidth) / kLateralCoverageStep)));
        for (int index = 0; index <= lateralSegments; ++index) {
            const double offset = lateralSegments > 0
                ? -halfWidth + (2.0 * halfWidth * index) / lateralSegments
                : 0.0;
            const Vec2 point = add(
                multiply(forward, longitudinal),
                multiply(lateral, offset));
            const Vec2 pointInCoverageFrame{
                input.coverageOriginX + coverageCos * point.x - coverageSin * point.y,
                input.coverageOriginY + coverageSin * point.x + coverageCos * point.y};
            const double pointClearance = observedPointClearance(
                pointInCoverageFrame, coverage);
            if (!(pointClearance > kEpsilon)) return false;
            clearance = std::min(clearance, pointClearance);
        }
        return true;
    };

    const double firstSample = std::min(pathLength, guardedNearField);
    for (double distance = firstSample;
         distance < pathLength - kEpsilon;
         distance += step) {
        if (!checkCrossSection(distance)) {
            return -std::numeric_limits<double>::infinity();
        }
    }
    if (!checkCrossSection(pathLength)) {
        return -std::numeric_limits<double>::infinity();
    }
    return clearance;
}

double sweptPathClearance(
    double angle,
    double pathLength,
    double corridorHalfWidth,
    const std::vector<ObstacleComponent> &obstacles) noexcept
{
    const Vec2 pathEnd{pathLength * std::cos(angle), pathLength * std::sin(angle)};
    double clearance = std::numeric_limits<double>::infinity();

    for (const auto &obstacle : obstacles) {
        if (isBall(obstacle)) {
            continue;
        }
        if (!validComponent(obstacle)) {
            return -std::numeric_limits<double>::infinity();
        }

        const Vec2 obstacleStart{obstacle.x, obstacle.y};
        const Vec2 obstacleEnd{
            obstacle.x + obstacle.vx * kPredictionSeconds,
            obstacle.y + obstacle.vy * kPredictionSeconds,
        };
        const double centerDistance = segmentToSegmentDistance(
            {0.0, 0.0}, pathEnd, obstacleStart, obstacleEnd);
        clearance = std::min(
            clearance, centerDistance - obstacle.radius - corridorHalfWidth);
    }
    return clearance;
}

bool rotationalFootprintClear(
    double footprintRadius,
    const std::vector<ObstacleComponent> &obstacles) noexcept
{
    for (const auto &obstacle : obstacles) {
        if (isBall(obstacle)) {
            continue;
        }
        if (!validComponent(obstacle)) {
            return false;
        }
        const Vec2 current{obstacle.x, obstacle.y};
        const Vec2 predicted{
            obstacle.x + obstacle.vx * kPredictionSeconds,
            obstacle.y + obstacle.vy * kPredictionSeconds,
        };
        if (pointToSegmentDistance({0.0, 0.0}, current, predicted)
            <= footprintRadius + obstacle.radius) {
            return false;
        }
    }
    return true;
}

double nearestSurfaceDistance(const std::vector<ObstacleComponent> &obstacles) noexcept
{
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto &obstacle : obstacles) {
        if (isBall(obstacle)) {
            continue;
        }
        if (!validComponent(obstacle)) {
            return 0.0;
        }
        nearest = std::min(nearest, std::hypot(obstacle.x, obstacle.y) - obstacle.radius);
    }
    return nearest;
}

bool shootingPathClear(
    const ObstaclePlannerInput &input,
    const PreparedCoverage &coverage) noexcept
{
    if (!std::isfinite(input.ballX) || !std::isfinite(input.ballY)) {
        return false;
    }

    const double halfWidth = clampNonnegative(input.corridorHalfWidth);
    const Vec2 ball{input.ballX, input.ballY};
    const double ballDistance = norm(ball);
    if (ballDistance <= kEpsilon) {
        return false;
    }
    const Vec2 ballDirection{ball.x / ballDistance, ball.y / ballDistance};
    for (const auto &obstacle : input.obstacles) {
        if (isBall(obstacle)) {
            continue;
        }
        if (!validComponent(obstacle)) {
            return false;
        }
        const Vec2 obstacleStart{obstacle.x, obstacle.y};
        const Vec2 obstacleEnd{
            obstacle.x + obstacle.vx * kPredictionSeconds,
            obstacle.y + obstacle.vy * kPredictionSeconds,
        };
        const double startProjection = dot(obstacleStart, ballDirection);
        const double endProjection = dot(obstacleEnd, ballDirection);
        const double minimumProjection = std::min(startProjection, endProjection);
        const double maximumProjection = std::max(startProjection, endProjection);
        // Ignore a component only when its complete predicted footprint lies
        // outside the robot-to-ball segment.  A large obstacle whose centre is
        // beyond the ball can still extend into the shooting corridor.
        if (maximumProjection + obstacle.radius < 0.0
            || minimumProjection - obstacle.radius > ballDistance) {
            continue;
        }
        if (segmentToSegmentDistance(
                {0.0, 0.0}, ball, obstacleStart, obstacleEnd)
            <= halfWidth + obstacle.radius) {
            return false;
        }
    }
    return observedCorridorClearance(
        std::atan2(ball.y, ball.x),
        ballDistance,
        halfWidth,
        input,
        coverage) > kEpsilon;
}

void setState(ObstaclePlannerOutput &output, ObstaclePlannerState state)
{
    output.state = state;
    output.stateText = toString(state);
}

void setPassthroughMinFlags(ObstaclePlannerOutput &output, const std::string &mode)
{
    output.applyMinX = true;
    output.applyMinY = true;
    output.applyMinTheta = !equalsIgnoreCase(mode, "search");
}

bool pathIsClear(
    double angle,
    double pathLength,
    double corridorHalfWidth,
    const ObstaclePlannerInput &input,
    const PreparedCoverage &coverage,
    double *clearanceOut = nullptr) noexcept
{
    if (!std::isfinite(angle) || angle < -kCandidateLimitRad - kEpsilon
        || angle > kCandidateLimitRad + kEpsilon) {
        return false;
    }

    const double geometricClearance = sweptPathClearance(
        angle, pathLength, corridorHalfWidth, input.obstacles);
    if (!(geometricClearance > kEpsilon)) {
        if (clearanceOut != nullptr) {
            *clearanceOut = geometricClearance;
        }
        return false;
    }
    const double coverageClearance = observedCorridorClearance(
        angle, pathLength, corridorHalfWidth, input, coverage);
    const double clearance = std::min(geometricClearance, coverageClearance);
    if (clearanceOut != nullptr) {
        *clearanceOut = clearance;
    }
    return clearance > kEpsilon;
}

Candidate bestCandidate(
    const ObstaclePlannerInput &input,
    double desiredAngle,
    double desiredSpeed,
    double avoidDistance,
    double corridorHalfWidth,
    int requiredSide,
    const PreparedCoverage &coverage)
{
    Candidate best;
    const double vxLimit = clampNonnegative(input.vxLimit);
    const double vyLimit = clampNonnegative(input.vyLimit);
    const double minX = clampNonnegative(input.minExecutableX);
    const double lateralFloor = std::max(kLateralCommandFloor, clampNonnegative(input.minExecutableY));
    const double baseSpeed = std::min(vxLimit, std::max(desiredSpeed, minX));

    if (baseSpeed <= kEpsilon) {
        return best;
    }

    for (int step = -9; step <= 9; ++step) {
        if (step == 0) {
            continue;
        }
        const double sampledAngle = static_cast<double>(step) * kCandidateStepRad;
        double vx = baseSpeed * std::cos(sampledAngle);
        double vy = baseSpeed * std::sin(sampledAngle);
        if (std::abs(vx) <= kEpsilon) vx = 0.0;
        if (std::abs(vy) <= kEpsilon) vy = 0.0;
        const int side = vy > kEpsilon ? 1 : (vy < -kEpsilon ? -1 : 0);

        if (requiredSide != 0 && side != requiredSide) {
            continue;
        }
        if (side != 0) {
            if (vyLimit + kEpsilon < lateralFloor) {
                continue;
            }
            vy = static_cast<double>(side) * std::min(vyLimit, std::max(std::abs(vy), lateralFloor));
        }
        if (vx > kEpsilon && vx < minX) {
            if (minX > vxLimit + kEpsilon) {
                continue;
            }
            vx = minX;
        }
        vx = std::clamp(vx, 0.0, vxLimit);
        if (vx <= kEpsilon && std::abs(vy) <= kEpsilon) {
            continue;
        }

        const double actualAngle = std::atan2(vy, vx);
        const double commandSpeed = std::hypot(vx, vy);
        const double pathLength = std::max(avoidDistance, commandSpeed * kPredictionSeconds);
        double clearance = 0.0;
        if (!pathIsClear(
                actualAngle,
                pathLength,
                corridorHalfWidth,
                input,
                coverage,
                &clearance)) {
            continue;
        }

        const double progress = std::cos(angleDistance(actualAngle, desiredAngle));
        if (progress < -kEpsilon) {
            continue;
        }
        const double clearanceScale = std::max(avoidDistance, 0.1);
        const double normalizedClearance = std::clamp(clearance / clearanceScale, 0.0, 1.0);
        const double deviation = angleDistance(actualAngle, desiredAngle) / kCandidateLimitRad;
        const double sideLatchScore = requiredSide != 0 && side == requiredSide ? 0.75 : 0.0;
        const double score = 3.0 * progress + 1.5 * normalizedClearance - deviation
            + sideLatchScore;

        if (!best.valid || score > best.score + kEpsilon
            || (std::abs(score - best.score) <= kEpsilon && clearance > best.clearance)) {
            best = {vx, vy, clearance, score, side, true};
        }
    }
    return best;
}

} // namespace

const char *toString(ObstaclePlannerState state) noexcept
{
    switch (state) {
    case ObstaclePlannerState::Disabled:
        return "DISABLED";
    case ObstaclePlannerState::MapUnready:
        return "MAP_UNREADY";
    case ObstaclePlannerState::DepthStale:
        return "DEPTH_STALE";
    case ObstaclePlannerState::EmergencyStop:
        return "EMERGENCY_STOP";
    case ObstaclePlannerState::Direct:
        return "DIRECT";
    case ObstaclePlannerState::AvoidingLeft:
        return "AVOIDING_LEFT";
    case ObstaclePlannerState::AvoidingRight:
        return "AVOIDING_RIGHT";
    case ObstaclePlannerState::NoRoute:
        return "NO_ROUTE";
    }
    return "NO_ROUTE";
}

void ObstacleVelocityPlanner::reset() noexcept
{
    latchedSide_ = 0;
    sideLatchUntilMs_ = 0;
    directClearSinceMs_ = -1;
    lastNowMs_ = -1;
    avoidanceActive_ = false;
}

ObstaclePlannerOutput ObstacleVelocityPlanner::plan(const ObstaclePlannerInput &input)
{
    ObstaclePlannerOutput output;
    const PreparedCoverage coverage = prepareCoverage(input.freeRanges);
    output.nearestDistance = nearestSurfaceDistance(input.obstacles);
    output.shootPathClear = shootingPathClear(input, coverage);

    if (lastNowMs_ >= 0 && input.nowMs < lastNowMs_) {
        reset();
    }
    lastNowMs_ = input.nowMs;

    if (!input.enabled) {
        output.safeVx = input.desiredVx;
        output.safeVy = input.desiredVy;
        output.safeTheta = input.desiredTheta;
        setPassthroughMinFlags(output, input.mode);
        setState(output, ObstaclePlannerState::Disabled);
        reset();
        lastNowMs_ = input.nowMs;
        return output;
    }

    if (!input.mapReady) {
        output.shootPathClear = false;
        reset();
        lastNowMs_ = input.nowMs;
        setState(output, ObstaclePlannerState::MapUnready);
        return output;
    }

    const std::int64_t maxDepthAgeMs = std::max<std::int64_t>(0, input.maxDepthAgeMs);
    if (input.depthAgeMs < 0 || input.depthAgeMs > maxDepthAgeMs) {
        output.shootPathClear = false;
        reset();
        lastNowMs_ = input.nowMs;
        setState(output, ObstaclePlannerState::DepthStale);
        return output;
    }

    const double stopDistance = clampNonnegative(input.stopDistance);
    if (output.nearestDistance <= stopDistance) {
        avoidanceActive_ = true;
        directClearSinceMs_ = -1;
        output.side = latchedSide_;
        setState(output, ObstaclePlannerState::EmergencyStop);
        return output;
    }

    const double avoidDistance = clampNonnegative(input.avoidDistance);
    const double corridorHalfWidth = clampNonnegative(input.corridorHalfWidth);
    // A circular in-place footprint does not enter new space: the robot already
    // occupies that disk. Guard rotation against current/predicted components
    // in the emergency envelope without requiring impossible rear-camera
    // coverage before the body can begin a visual search.
    const bool rotationClear =
        rotationalFootprintClear(corridorHalfWidth, input.obstacles);
    const double desiredVx = quantizeAxis(
        input.desiredVx, input.minExecutableX, input.vxLimit);
    const double desiredVy = quantizeAxis(
        input.desiredVy,
        std::max(kLateralCommandFloor, clampNonnegative(input.minExecutableY)),
        input.vyLimit);
    // Search intentionally used apply_min_theta=false before this filter was
    // introduced. Preserve that fine rotation while still limiting it and
    // guarding the swept footprint. Other modes are quantized before checking
    // so downstream SetVelocity cannot silently promote an unchecked command.
    const double desiredTheta = equalsIgnoreCase(input.mode, "search")
        ? limitAxis(input.desiredTheta, input.vthetaLimit)
        : quantizeAxis(
            input.desiredTheta, input.minExecutableTheta, input.vthetaLimit);
    const double desiredSpeed = std::hypot(desiredVx, desiredVy);

    if (desiredSpeed <= kEpsilon) {
        output.safeTheta = rotationClear ? desiredTheta : 0.0;
        setState(output, rotationClear ? ObstaclePlannerState::Direct : ObstaclePlannerState::NoRoute);
        // A tracking/search/behavior-transition tick with zero translation is
        // not evidence that the previously selected route became clear. Keep
        // side and release hysteresis so a single zero command cannot make a
        // symmetric obstruction flip the robot to the opposite side.
        output.side = latchedSide_;
        return output;
    }

    const double desiredAngle = std::atan2(desiredVy, desiredVx);
    const double directPathLength = std::max(avoidDistance, desiredSpeed * kPredictionSeconds);
    const bool directClear = desiredVx >= -kEpsilon
        && pathIsClear(
            desiredAngle, directPathLength, corridorHalfWidth, input, coverage)
        && (std::abs(desiredTheta) <= kEpsilon || rotationClear);

    if (directClear) {
        if (!avoidanceActive_) {
            output.safeVx = desiredVx;
            output.safeVy = desiredVy;
            output.safeTheta = desiredTheta;
            setState(output, ObstaclePlannerState::Direct);
            return output;
        }
        if (directClearSinceMs_ < 0) {
            directClearSinceMs_ = input.nowMs;
        }
        if (input.nowMs - directClearSinceMs_ >= kDirectReleaseMs &&
            input.nowMs >= sideLatchUntilMs_) {
            output.safeVx = desiredVx;
            output.safeVy = desiredVy;
            output.safeTheta = desiredTheta;
            setState(output, ObstaclePlannerState::Direct);
            reset();
            lastNowMs_ = input.nowMs;
            return output;
        }
    } else {
        avoidanceActive_ = true;
        directClearSinceMs_ = -1;
    }

    if (desiredVx < -kEpsilon) {
        output.safeTheta = rotationClear ? desiredTheta : 0.0;
        output.side = latchedSide_;
        setState(output, ObstaclePlannerState::NoRoute);
        return output;
    }

    int requiredSide = 0;
    if (directClear && latchedSide_ != 0) {
        requiredSide = latchedSide_;
    } else if (latchedSide_ != 0 && input.nowMs < sideLatchUntilMs_) {
        requiredSide = latchedSide_;
    }

    Candidate candidate = bestCandidate(
        input,
        desiredAngle,
        desiredSpeed,
        avoidDistance,
        corridorHalfWidth,
        requiredSide,
        coverage);
    if (!candidate.valid && requiredSide != 0) {
        // The latch is a preference, never a reason to stop when the opposite
        // observed route is the only safe translational path.
        candidate = bestCandidate(
            input,
            desiredAngle,
            desiredSpeed,
            avoidDistance,
            corridorHalfWidth,
            0,
            coverage);
    }
    if (!candidate.valid) {
        output.safeTheta = rotationClear ? desiredTheta : 0.0;
        output.side = latchedSide_;
        setState(output, ObstaclePlannerState::NoRoute);
        return output;
    }

    if (candidate.side != 0 && candidate.side != latchedSide_) {
        latchedSide_ = candidate.side;
        sideLatchUntilMs_ = input.nowMs + kSideLatchMs;
    }

    output.safeVx = candidate.vx;
    output.safeVy = candidate.vy;
    output.safeTheta = rotationClear ? desiredTheta : 0.0;
    output.applyMinX = false;
    output.applyMinY = false;
    output.side = candidate.side;
    setState(
        output,
        candidate.side > 0 ? ObstaclePlannerState::AvoidingLeft
                           : ObstaclePlannerState::AvoidingRight);
    return output;
}

} // namespace booster_soccer
