#include "obstacle_avoidance_node.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "brain.h"
#include "ball_occlusion_utils.h"
#include "obstacle_perception_utils.h"
#include "utils/math.h"
#include "utils/misc.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::int64_t timeMsec(const rclcpp::Time &time)
{
    return time.nanoseconds() / 1000000;
}

double wrappedAngle(double angle)
{
    if (!std::isfinite(angle)) return 0.0;
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

bool angleInInterval(double angle, double start, double end)
{
    if (!std::isfinite(angle) || !std::isfinite(start) || !std::isfinite(end)) {
        return false;
    }
    angle = wrappedAngle(angle);
    start = wrappedAngle(start);
    end = wrappedAngle(end);
    if (start <= end) return angle >= start && angle <= end;
    return angle >= start || angle <= end;
}

Point2D robotToOdom(const Pose2D &robotPose, double x, double y)
{
    return {
        robotPose.x + std::cos(robotPose.theta) * x - std::sin(robotPose.theta) * y,
        robotPose.y + std::sin(robotPose.theta) * x + std::cos(robotPose.theta) * y};
}

Point2D odomToRobot(const Pose2D &robotPose, double x, double y)
{
    const double dx = x - robotPose.x;
    const double dy = y - robotPose.y;
    return {
        std::cos(robotPose.theta) * dx + std::sin(robotPose.theta) * dy,
        -std::sin(robotPose.theta) * dx + std::cos(robotPose.theta) * dy};
}

double finiteOr(double value, double fallback)
{
    return std::isfinite(value) ? value : fallback;
}

double timestampAgeMsecs(const rclcpp::Time &now, const rclcpp::Time &then)
{
    if (then.nanoseconds() <= 0 || now.get_clock_type() != then.get_clock_type()) {
        return std::numeric_limits<double>::infinity();
    }
    const auto deltaNsec = (now - then).nanoseconds();
    return deltaNsec >= 0
        ? static_cast<double>(deltaNsec) / 1e6
        : std::numeric_limits<double>::infinity();
}

double snapshotAgeMsecs(const rclcpp::Time &now, const ObstacleSnapshot &snapshot)
{
    const bool sensorStampValid = snapshot.stamp.nanoseconds() > 0 &&
        snapshot.stamp.get_clock_type() == now.get_clock_type();
    return booster_soccer::perception::combinedFreshnessAge(
        timestampAgeMsecs(now, snapshot.received_at),
        sensorStampValid ? timestampAgeMsecs(now, snapshot.stamp) : 0.0,
        sensorStampValid);
}

std::string externalState(booster_soccer::ObstaclePlannerState state)
{
    using State = booster_soccer::ObstaclePlannerState;
    switch (state) {
        case State::Disabled: return "DISABLED";
        case State::MapUnready: return "WAITING_FOR_DEPTH";
        case State::DepthStale: return "SENSOR_STALE";
        case State::EmergencyStop: return "STOP_DISTANCE";
        case State::Direct: return "CLEAR";
        case State::AvoidingLeft: return "DETOUR_LEFT";
        case State::AvoidingRight: return "DETOUR_RIGHT";
        case State::NoRoute: return "HOLD_BLOCKED";
    }
    return "HOLD_BLOCKED";
}
} // namespace

ObstacleAvoidance::ObstacleAvoidance(
    const std::string &name,
    const BT::NodeConfig &config,
    Brain *brain)
    : BT::SyncActionNode(name, config), brain_(brain)
{
}

BT::PortsList ObstacleAvoidance::providedPorts()
{
    return {
        BT::InputPort<bool>("enabled", true),
        BT::InputPort<std::string>("mode", "track"),
        BT::InputPort<double>("avoid_distance", 1.40),
        BT::InputPort<double>("stop_distance", 0.50),
        BT::InputPort<double>("desired_vx", 0.0),
        BT::InputPort<double>("desired_vy", 0.0),
        BT::InputPort<double>("desired_theta", 0.0),
        BT::InputPort<double>("vx_limit", -1.0),
        BT::InputPort<double>("vy_limit", -1.0),
        BT::InputPort<double>("vtheta_limit", -1.0),
        BT::OutputPort<double>("safe_vx"),
        BT::OutputPort<double>("safe_vy"),
        BT::OutputPort<double>("safe_theta"),
        BT::OutputPort<bool>("apply_min_x"),
        BT::OutputPort<bool>("apply_min_y"),
        BT::OutputPort<bool>("apply_min_theta"),
        BT::OutputPort<std::string>("state"),
        BT::OutputPort<double>("nearest_distance"),
        BT::OutputPort<std::string>("side"),
        BT::OutputPort<std::string>("ball_state"),
        BT::OutputPort<double>("expected_ball_x"),
        BT::OutputPort<double>("expected_ball_y"),
        BT::OutputPort<double>("depth_age_ms"),
        BT::OutputPort<bool>("shoot_path_clear")};
}

void ObstacleAvoidance::rememberCurrentBall(std::int64_t now_msec)
{
    const auto observation = brain_->data->getBallDepthObservation();
    if (!observation.visible || !observation.depth_confirmed ||
        observation.received_at.nanoseconds() <= 0) {
        return;
    }

    const std::int64_t receiveNsec = observation.received_at.nanoseconds();
    if (receiveNsec == last_ball_observation_receive_nsec_) return;
    last_ball_observation_receive_nsec_ = receiveNsec;

    const Point2D ballOdom = robotToOdom(
        observation.robot_pose_to_odom,
        observation.position_to_robot.x,
        observation.position_to_robot.y);
    ball_history_.push_back(BallObservation{
        receiveNsec / 1000000, ballOdom.x, ballOdom.y});

    while (!ball_history_.empty() &&
           now_msec - ball_history_.front().time_msec > 1000) {
        ball_history_.pop_front();
    }
}

bool ObstacleAvoidance::stationaryBallHistory(
    std::int64_t now_msec,
    double &odom_x,
    double &odom_y) const
{
    std::vector<booster_soccer::occlusion::TimedBallPoint> history;
    history.reserve(ball_history_.size());
    for (const auto &observation : ball_history_) {
        history.push_back({
            observation.time_msec,
            observation.odom_x,
            observation.odom_y});
    }
    const auto estimate = booster_soccer::occlusion::estimateStationaryBall(
        history, now_msec);
    if (!estimate.valid) return false;
    odom_x = estimate.odomX;
    odom_y = estimate.odomY;
    return true;
}

void ObstacleAvoidance::clearExpectedBall()
{
    expected_ball_valid_ = false;
    expected_ball_odom_x_ = 0.0;
    expected_ball_odom_y_ = 0.0;
    occlusion_start_msec_ = 0;
    expected_position_miss_frames_ = 0;
    last_occlusion_detection_receive_nsec_ = 0;
    occlusion_look_at_expected_ = true;
    occlusion_gap_side_ = 1;
    occlusion_target_yaw_ = 0.0;
    occlusion_view_attained_since_msec_ = -1;
    last_occlusion_head_command_msec_ = 0;
}

BT::NodeStatus ObstacleAvoidance::tick()
{
    // This node is the final filter before SetVelocity. It deliberately never
    // returns FAILURE: every path below writes a complete velocity command so
    // a stale command cannot remain active in the reactive tree.
    bool enabled = brain_->config->get_enable_obstacle_avoidance();
    std::string mode = "track";
    double avoidDistance = brain_->config->get_obstacle_avoid_distance();
    double stopDistance = brain_->config->get_obstacle_stop_distance();
    double desiredVx = 0.0;
    double desiredVy = 0.0;
    double desiredTheta = 0.0;
    double commandVxLimit = -1.0;
    double commandVyLimit = -1.0;
    double commandVthetaLimit = -1.0;
    getInput("enabled", enabled);
    getInput("mode", mode);
    getInput("avoid_distance", avoidDistance);
    getInput("stop_distance", stopDistance);
    getInput("desired_vx", desiredVx);
    getInput("desired_vy", desiredVy);
    getInput("desired_theta", desiredTheta);
    getInput("vx_limit", commandVxLimit);
    getInput("vy_limit", commandVyLimit);
    getInput("vtheta_limit", commandVthetaLimit);

    const rclcpp::Time now = brain_->get_clock()->now();
    const std::int64_t nowMsec = timeMsec(now);
    const auto snapshot = brain_->data->getObstacleSnapshot();
    const Pose2D currentRobotPose = brain_->data->getRobotPoseToOdom();
    const double snapshotTheta = snapshot.robot_pose_to_odom.theta;
    const double deltaOdomX = currentRobotPose.x - snapshot.robot_pose_to_odom.x;
    const double deltaOdomY = currentRobotPose.y - snapshot.robot_pose_to_odom.y;
    const double coverageOriginX =
        std::cos(snapshotTheta) * deltaOdomX + std::sin(snapshotTheta) * deltaOdomY;
    const double coverageOriginY =
        -std::sin(snapshotTheta) * deltaOdomX + std::cos(snapshotTheta) * deltaOdomY;
    const double coverageYaw = wrappedAngle(currentRobotPose.theta - snapshotTheta);
    // Age exactly the copied snapshot. Reading the age from BrainData in a
    // second lock could pair an old map with a newly arrived frame's age.
    const double depthAge = snapshotAgeMsecs(now, snapshot);
    const double detectionAge = brain_->data->detectionAgeMsecs(now);
    const bool freshDepth = snapshot.ready && std::isfinite(depthAge) &&
        depthAge <= brain_->config->get_depth_stale_msecs();
    const bool detectionStale = !std::isfinite(detectionAge) ||
        detectionAge > brain_->config->get_detection_stale_msecs();

    std::vector<booster_soccer::ObstacleComponent> plannerComponents;
    std::vector<bool> plannerComponentFresh;
    plannerComponents.reserve(snapshot.components.size());
    plannerComponentFresh.reserve(snapshot.components.size());
    const double componentAgeSeconds = freshDepth
        ? std::clamp(depthAge / 1000.0, 0.0, 0.300)
        : 0.0;
    for (const auto &component : snapshot.components) {
        if (component.label == "Ball") continue;
        const Point2D componentOdom = robotToOdom(
            snapshot.robot_pose_to_odom, component.x, component.y);
        const Point2D componentRobot = odomToRobot(
            currentRobotPose, componentOdom.x, componentOdom.y);
        const double velocityRotation =
            snapshot.robot_pose_to_odom.theta - currentRobotPose.theta;
        const double componentVx =
            std::cos(velocityRotation) * component.vx -
            std::sin(velocityRotation) * component.vy;
        const double componentVy =
            std::sin(velocityRotation) * component.vx +
            std::cos(velocityRotation) * component.vy;
        plannerComponents.push_back(booster_soccer::ObstacleComponent{
            componentRobot.x + componentVx * componentAgeSeconds,
            componentRobot.y + componentVy * componentAgeSeconds,
            componentVx,
            componentVy,
            component.radius,
            component.label});
        plannerComponentFresh.push_back(!component.carried);
    }

    std::vector<booster_soccer::AngularFreeRangeCoverage> plannerCoverage;
    std::vector<booster_soccer::AngularFreeRangeCoverage> currentFrameCoverage;
    plannerCoverage.reserve(snapshot.coverage.size());
    currentFrameCoverage.reserve(snapshot.coverage.size());
    const double coverageHalfWidth = std::max(
        0.5 * deg2rad(brain_->config->get_obstacle_angular_resolution_degrees()),
        deg2rad(0.5));
    const double coverageMemoryMsecs = std::max(
        0.0, brain_->config->get_obstacle_memory_msecs());
    for (const auto &coverage : snapshot.coverage) {
        if (!coverage.observed || !std::isfinite(coverage.free_range) ||
            coverage.free_range <= 0.0) {
            continue;
        }
        const rclcpp::Time observedAt = coverage.observed_at.nanoseconds() > 0
            ? coverage.observed_at
            : snapshot.received_at;
        if (!booster_soccer::perception::observationWithinMemory(
                timestampAgeMsecs(now, observedAt), coverageMemoryMsecs)) {
            continue;
        }
        const double center = wrappedAngle(coverage.angle);
        const booster_soccer::AngularFreeRangeCoverage plannerRay{
            wrappedAngle(center - coverageHalfWidth),
            wrappedAngle(center + coverageHalfWidth),
            coverage.free_range,
            coverage.origin_x,
            coverage.origin_y};
        const bool fromCurrentDepthFrame =
            coverage.observed_at.nanoseconds() ==
            snapshot.received_at.nanoseconds();
        if (fromCurrentDepthFrame) {
            // A selected route is kept in the live, measured-attained camera
            // view below. Authorize velocity only from this current origin;
            // retained rays still support map diagnostics, but combining many
            // old viewpoints here is both expensive and vulnerable to parallax.
            plannerCoverage.push_back(plannerRay);
            currentFrameCoverage.push_back(plannerRay);
        }
    }

    const auto ballObservation = brain_->data->getBallDepthObservation();
    const rclcpp::Time lastDetectionReceiveTime =
        brain_->data->getLastDetectionReceiveTime();
    const bool ballEvidenceMatchesLatestDetection =
        ballObservation.received_at.nanoseconds() > 0 &&
        ballObservation.received_at.get_clock_type() ==
            lastDetectionReceiveTime.get_clock_type() &&
        ballObservation.received_at.nanoseconds() ==
            lastDetectionReceiveTime.nanoseconds();
    const bool rgbBallVisible =
        !detectionStale && ballEvidenceMatchesLatestDetection &&
        ballObservation.visible &&
        brain_->tree->getEntry<bool>("ball_visible") &&
        brain_->data->ballDetected.load(std::memory_order_acquire);
    const bool depthBallVisible =
        rgbBallVisible && ballObservation.depth_confirmed &&
        brain_->tree->getEntry<bool>("ball_location_known");

    if (depthBallVisible) {
        rememberCurrentBall(nowMsec);
        clearExpectedBall();
        had_current_depth_ball_ = true;
    } else if (detectionStale) {
        clearExpectedBall();
        had_current_depth_ball_ = false;
    } else if (!rgbBallVisible && had_current_depth_ball_) {
        double stationaryOdomX = 0.0;
        double stationaryOdomY = 0.0;
        if (freshDepth &&
            stationaryBallHistory(nowMsec, stationaryOdomX, stationaryOdomY)) {
            const Point2D expectedRobot = odomToRobot(
                currentRobotPose, stationaryOdomX, stationaryOdomY);
            bool hasNearerOccluder = false;
            for (std::size_t index = 0; index < plannerComponents.size(); ++index) {
                if (!plannerComponentFresh[index]) continue;
                const auto &component = plannerComponents[index];
                if (booster_soccer::occlusion::freshObstacleOccludesExpectedBall(
                        true,
                        expectedRobot.x,
                        expectedRobot.y,
                        component.x,
                        component.y,
                        component.radius)) {
                    hasNearerOccluder = true;
                    break;
                }
            }
            if (hasNearerOccluder) {
                expected_ball_valid_ = true;
                expected_ball_odom_x_ = stationaryOdomX;
                expected_ball_odom_y_ = stationaryOdomY;
                occlusion_start_robot_odom_x_ = currentRobotPose.x;
                occlusion_start_robot_odom_y_ = currentRobotPose.y;
                occlusion_start_msec_ = nowMsec;
                expected_position_miss_frames_ = 0;
            }
        }
        had_current_depth_ball_ = false;
    }

    Point2D expectedRobot{0.0, 0.0};
    if (expected_ball_valid_) {
        expectedRobot = odomToRobot(
            currentRobotPose, expected_ball_odom_x_, expected_ball_odom_y_);
        const bool recoveryExpired = booster_soccer::occlusion::recoveryLimitExceeded(
            nowMsec - occlusion_start_msec_,
            std::hypot(
                currentRobotPose.x - occlusion_start_robot_odom_x_,
                currentRobotPose.y - occlusion_start_robot_odom_y_));
        if (recoveryExpired || detectionStale) {
            clearExpectedBall();
            expectedRobot = {0.0, 0.0};
        }
    }

    auto expectedRayObservedAndClear = [&]() {
        if (!expected_ball_valid_ || !freshDepth) return false;
        const double range = std::hypot(expectedRobot.x, expectedRobot.y);
        const double bearing = std::atan2(expectedRobot.y, expectedRobot.x);
        const Point2D expectedInCoverageFrame{
            coverageOriginX + std::cos(coverageYaw) * expectedRobot.x -
                std::sin(coverageYaw) * expectedRobot.y,
            coverageOriginY + std::sin(coverageYaw) * expectedRobot.x +
                std::cos(coverageYaw) * expectedRobot.y};
        bool covered = false;
        // Hypothesis invalidation requires the expected position to be in the
        // camera *now*. Retained gap rays are valid for motion planning but
        // cannot stand in for the current detector view.
        for (const auto &coverage : currentFrameCoverage) {
            const double rayX = expectedInCoverageFrame.x - coverage.originX;
            const double rayY = expectedInCoverageFrame.y - coverage.originY;
            const double rayRange = std::hypot(rayX, rayY);
            if (angleInInterval(
                    std::atan2(rayY, rayX),
                    coverage.startAngleRad,
                    coverage.endAngleRad) &&
                coverage.freeRange >= rayRange) {
                covered = true;
                break;
            }
        }
        if (!covered) return false;

        for (const auto &component : plannerComponents) {
            const double projection =
                component.x * std::cos(bearing) + component.y * std::sin(bearing);
            if (projection <= 0.0 || projection >= range) continue;
            const double perpendicular = std::fabs(
                -component.x * std::sin(bearing) + component.y * std::cos(bearing));
            if (perpendicular <= component.radius + 0.12) return false;
        }
        return true;
    };

    if (expected_ball_valid_ && !rgbBallVisible && !detectionStale) {
        const std::int64_t detectionReceiveNsec =
            lastDetectionReceiveTime.nanoseconds();
        const bool newDetectionFrame = detectionReceiveNsec > 0 &&
            detectionReceiveNsec != last_occlusion_detection_receive_nsec_;
        if (newDetectionFrame) {
            last_occlusion_detection_receive_nsec_ = detectionReceiveNsec;
        }
        expected_position_miss_frames_ =
            booster_soccer::occlusion::updateClearViewMissCount(
                expected_position_miss_frames_,
                newDetectionFrame,
                false,
                freshDepth,
                expectedRayObservedAndClear());
        if (expected_position_miss_frames_ >= 3) {
            clearExpectedBall();
            expectedRobot = {0.0, 0.0};
        }
    }

    bool occlusionPursuit = enabled && expected_ball_valid_ && !rgbBallVisible;
    if (enabled && detectionStale) {
        // The generated tree may still be on its previously visible branch
        // when the detector freezes. Stop stale pursuit immediately and make
        // the next tick enter the ordinary safe search branch.
        desiredVx = 0.0;
        desiredVy = 0.0;
        mode = "search";
        brain_->tree->setEntry<bool>("ball_visible", false);
        brain_->tree->setEntry<bool>("ball_location_known", false);
        brain_->data->ballDetected.store(false, std::memory_order_release);
    } else if (enabled && rgbBallVisible && !depthBallVisible) {
        // RGB-only detections can steer the head/body yaw, but have no metric
        // position with which to authorize translation.
        desiredVx = 0.0;
        desiredVy = 0.0;
    } else if (occlusionPursuit) {
        const double range = std::hypot(expectedRobot.x, expectedRobot.y);
        if (range > 0.05) {
            desiredVx = std::clamp(expectedRobot.x, 0.0, 0.35);
            desiredVy = std::clamp(expectedRobot.y, -0.30, 0.30);
            const double targetBearing = std::atan2(expectedRobot.y, expectedRobot.x);
            desiredTheta = std::clamp(targetBearing, -0.45, 0.45);
            mode = "occlusion";
        } else {
            desiredVx = 0.0;
            desiredVy = 0.0;
        }
    }

    booster_soccer::ObstaclePlannerInput plannerInput;
    plannerInput.enabled = enabled;
    plannerInput.mapReady = snapshot.ready;
    plannerInput.depthAgeMs = std::isfinite(depthAge)
        ? static_cast<std::int64_t>(std::max(0.0, depthAge))
        : std::numeric_limits<std::int64_t>::max() / 4;
    plannerInput.maxDepthAgeMs = static_cast<std::int64_t>(
        std::max(1.0, brain_->config->get_depth_stale_msecs()));
    plannerInput.desiredVx = enabled ? finiteOr(desiredVx, 0.0) : desiredVx;
    plannerInput.desiredVy = enabled ? finiteOr(desiredVy, 0.0) : desiredVy;
    plannerInput.desiredTheta = enabled ? finiteOr(desiredTheta, 0.0) : desiredTheta;
    plannerInput.mode = mode;
    plannerInput.avoidDistance = avoidDistance;
    plannerInput.stopDistance = stopDistance;
    plannerInput.corridorHalfWidth = brain_->config->get_collision_threshold();
    plannerInput.minExecutableX = brain_->config->get_min_vx();
    plannerInput.minExecutableY = brain_->config->get_min_vy();
    plannerInput.minExecutableTheta = brain_->config->get_min_vtheta();
    const double robotVxLimit = brain_->config->get_vx_limit();
    const double robotVyLimit = brain_->config->get_vy_limit();
    const double robotVthetaLimit = brain_->config->get_vtheta_limit();
    plannerInput.vxLimit = commandVxLimit >= 0.0
        ? std::min(robotVxLimit, commandVxLimit)
        : robotVxLimit;
    // Legacy SetVelocity already raises nonzero lateral commands to min_vy,
    // even when the behavior's nominal cap is lower. Preserve that effective
    // cap while ensuring the checked detour is executable.
    plannerInput.vyLimit = commandVyLimit >= 0.0
        ? std::min(
            robotVyLimit,
            std::max(commandVyLimit, brain_->config->get_min_vy()))
        : robotVyLimit;
    plannerInput.vthetaLimit = commandVthetaLimit >= 0.0
        ? std::min(robotVthetaLimit, commandVthetaLimit)
        : robotVthetaLimit;
    plannerInput.ballRange = depthBallVisible
        ? std::hypot(
            ballObservation.position_to_robot.x,
            ballObservation.position_to_robot.y)
        : 0.0;
    plannerInput.ballX = depthBallVisible
        ? ballObservation.position_to_robot.x
        : 0.0;
    plannerInput.ballY = depthBallVisible
        ? ballObservation.position_to_robot.y
        : 0.0;
    plannerInput.nowMs = nowMsec;
    plannerInput.coverageOriginX = coverageOriginX;
    plannerInput.coverageOriginY = coverageOriginY;
    plannerInput.coverageYaw = coverageYaw;
    plannerInput.obstacles = std::move(plannerComponents);
    plannerInput.freeRanges = std::move(plannerCoverage);

    auto result = planner_.plan(plannerInput);
    std::string state = externalState(result.state);
    if (occlusionPursuit &&
        (result.state == booster_soccer::ObstaclePlannerState::Direct ||
         result.state == booster_soccer::ObstaclePlannerState::AvoidingLeft ||
         result.state == booster_soccer::ObstaclePlannerState::AvoidingRight)) {
        const double translation = std::hypot(result.safeVx, result.safeVy);
        if (translation <= 1e-4) {
            state = "HOLD_BLOCKED";
        } else if (std::fabs(result.safeVy) > std::fabs(result.safeVx)) {
            state = "OCCLUSION_PEEK";
        } else {
            state = "OCCLUSION_PURSUIT";
        }
    }

    std::string ballState = "LOST";
    switch (booster_soccer::occlusion::classifyBallEvidence(
        detectionStale,
        rgbBallVisible,
        depthBallVisible,
        enabled && expected_ball_valid_)) {
        case booster_soccer::occlusion::BallEvidenceState::VisibleDepth:
            ballState = "VISIBLE_DEPTH";
            break;
        case booster_soccer::occlusion::BallEvidenceState::VisibleRgbOnly:
            ballState = "VISIBLE_RGB_ONLY";
            break;
        case booster_soccer::occlusion::BallEvidenceState::OccludedStationary:
            ballState = "OCCLUDED_STATIONARY";
            break;
        case booster_soccer::occlusion::BallEvidenceState::DetectionStale:
            ballState = "DETECTION_STALE";
            break;
        case booster_soccer::occlusion::BallEvidenceState::Lost:
            break;
    }

    const std::string side = result.side > 0
        ? "left" : (result.side < 0 ? "right" : "none");
    const double nearestDistance = finiteOr(result.nearestDistance, 999.0);
    const double expectedX = enabled && expected_ball_valid_ ? expectedRobot.x : 0.0;
    const double expectedY = enabled && expected_ball_valid_ ? expectedRobot.y : 0.0;
    const bool shootPathClear = enabled
        ? (depthBallVisible && result.shootPathClear)
        : true;
    const bool translationalIntent =
        std::hypot(desiredVx, desiredVy) > 1e-4;
    const bool inspectVisibleGap = enabled && freshDepth && depthBallVisible &&
        translationalIntent &&
        nearestDistance > stopDistance &&
        (result.state == booster_soccer::ObstaclePlannerState::NoRoute ||
         result.state == booster_soccer::ObstaclePlannerState::AvoidingLeft ||
         result.state == booster_soccer::ObstaclePlannerState::AvoidingRight);
    const bool headStateReady =
        brain_->data->headStateReceived.load(std::memory_order_acquire);
    const double measuredHeadYaw = brain_->data->headYaw.load(
        std::memory_order_relaxed);
    const std::int64_t latestDepthReceiveMsec =
        snapshot.received_at.nanoseconds() > 0
        ? timeMsec(snapshot.received_at)
        : -1;
    constexpr std::int64_t viewDwellMsec = 200;
    const double headTargetTolerance = deg2rad(4.0);

    if (occlusionPursuit && headStateReady) {
        const double expectedBearing = std::atan2(expectedRobot.y, expectedRobot.x);
        double targetYaw = expectedBearing;
        const char *headTarget = "expected";
        if (!occlusion_look_at_expected_) {
            if (result.side != 0) occlusion_gap_side_ = result.side;
            if (std::hypot(result.safeVx, result.safeVy) > 1e-4) {
                targetYaw = std::atan2(result.safeVy, result.safeVx);
            } else {
                targetYaw = wrappedAngle(
                    expectedBearing +
                    static_cast<double>(occlusion_gap_side_) * deg2rad(35.0));
            }
            headTarget = occlusion_gap_side_ > 0 ? "gap_left" : "gap_right";
        }
        targetYaw = std::clamp(
            wrappedAngle(targetYaw),
            brain_->config->get_head_yaw_limit_right(),
            brain_->config->get_head_yaw_limit_left());

        if (booster_soccer::perception::wrappedAngularDistance(
                targetYaw, occlusion_target_yaw_) > deg2rad(3.0)) {
            occlusion_view_attained_since_msec_ = -1;
        }
        occlusion_target_yaw_ = targetYaw;
        if (booster_soccer::perception::headTargetAttained(
                measuredHeadYaw, targetYaw, headTargetTolerance)) {
            if (occlusion_view_attained_since_msec_ < 0) {
                occlusion_view_attained_since_msec_ = nowMsec;
            }
        } else {
            occlusion_view_attained_since_msec_ = -1;
        }

        const double expectedRange = std::max(
            0.10, std::hypot(expectedRobot.x, expectedRobot.y));
        const double targetPitch = std::clamp(
            std::atan2(brain_->config->get_robot_height(), expectedRange),
            brain_->config->get_head_pitch_limit_up(),
            0.90);
        brain_->client->moveHead(targetPitch, targetYaw);
        if (last_occlusion_head_command_msec_ <= 0 ||
            nowMsec - last_occlusion_head_command_msec_ >= 100) {
            brain_->log->debug(
                "ObstacleAvoidance/head",
                format(
                    "target: %s pitch: %.3f yaw: %.3f measuredYaw: %.3f expectedBearing: %.3f attained: %s",
                    headTarget,
                    targetPitch,
                    targetYaw,
                    measuredHeadYaw,
                    expectedBearing,
                    occlusion_view_attained_since_msec_ >= 0 ? "true" : "false"));
            last_occlusion_head_command_msec_ = nowMsec;
        }

        if (booster_soccer::perception::depthViewReadyToAdvance(
                nowMsec,
                occlusion_view_attained_since_msec_,
                latestDepthReceiveMsec,
                viewDwellMsec)) {
            if (!occlusion_look_at_expected_ && result.side == 0) {
                occlusion_gap_side_ = -occlusion_gap_side_;
            }
            occlusion_look_at_expected_ = !occlusion_look_at_expected_;
            occlusion_view_attained_since_msec_ = -1;
        }
    } else if (inspectVisibleGap && headStateReady) {
        // A side phase lasts until the measured neck yaw reaches its target
        // and at least one fresh depth frame has been received there. This is
        // intentionally not a timer-only sweep: while the head is travelling,
        // the planner remains stopped in unknown space.
        if (result.side != 0 && result.side != visible_gap_scan_side_) {
            visible_gap_scan_side_ = result.side;
            visible_gap_attained_since_msec_ = -1;
        }
        const double ballBearing = std::atan2(
            ballObservation.position_to_robot.y,
            ballObservation.position_to_robot.x);
        const double configuredHorizontalFov =
            brain_->config->depthCameraFovX.load(std::memory_order_acquire);
        const double horizontalFov = std::isfinite(configuredHorizontalFov)
            ? configuredHorizontalFov
            : deg2rad(90.0);
        const double boxWidth = std::max(
            0.0,
            ballObservation.bounding_box.xmax -
                ballObservation.bounding_box.xmin);
        const double imageWidth = std::max(1, ballObservation.image_width);
        const double ballHalfAngle =
            0.5 * horizontalFov * boxWidth / imageWidth;
        const double scanOffset =
            booster_soccer::perception::ballVisibleCoverageScanOffset(
                horizontalFov,
                ballHalfAngle,
                deg2rad(45.0));
        const double detourScanOffset =
            booster_soccer::perception::executableDetourScanOffset(
                desiredVx,
                plannerInput.minExecutableX,
                plannerInput.minExecutableY,
                plannerInput.vxLimit,
                plannerInput.vyLimit,
                scanOffset);
        if (detourScanOffset >= deg2rad(3.0)) {
            double pathRelativeTarget = ballBearing +
                static_cast<double>(visible_gap_scan_side_) * detourScanOffset;
            if (result.side != 0 &&
                std::hypot(result.safeVx, result.safeVy) > 1e-4) {
                const double routeBearing = std::atan2(result.safeVy, result.safeVx);
                const double routeOffset = wrappedAngle(routeBearing - ballBearing);
                pathRelativeTarget = ballBearing +
                    std::clamp(routeOffset, -scanOffset, scanOffset);
            }
            const double targetYaw = std::clamp(
                wrappedAngle(pathRelativeTarget),
                brain_->config->get_head_yaw_limit_right(),
                brain_->config->get_head_yaw_limit_left());
            if (booster_soccer::perception::wrappedAngularDistance(
                    targetYaw, visible_gap_target_yaw_) > deg2rad(3.0)) {
                visible_gap_attained_since_msec_ = -1;
            }
            visible_gap_target_yaw_ = targetYaw;
            if (booster_soccer::perception::headTargetAttained(
                    measuredHeadYaw, targetYaw, headTargetTolerance)) {
                if (visible_gap_attained_since_msec_ < 0) {
                    visible_gap_attained_since_msec_ = nowMsec;
                }
            } else {
                visible_gap_attained_since_msec_ = -1;
            }

            const double targetPitch = brain_->data->headPitch.load(
                std::memory_order_relaxed);
            // CamTrackBall runs earlier in this sequence, so repeat the target
            // every tick until the depth view has actually settled.
            brain_->client->moveHead(targetPitch, targetYaw);
            if (last_visible_gap_head_command_msec_ <= 0 ||
                nowMsec - last_visible_gap_head_command_msec_ >= 100) {
                brain_->log->debug(
                    "ObstacleAvoidance/head",
                    format(
                        "target: visible_gap_%s pitch: %.3f yaw: %.3f measuredYaw: %.3f ballBearing: %.3f offset: %.3f attained: %s",
                        visible_gap_scan_side_ > 0 ? "left" : "right",
                        targetPitch,
                        targetYaw,
                        measuredHeadYaw,
                        ballBearing,
                        detourScanOffset,
                        visible_gap_attained_since_msec_ >= 0 ? "true" : "false"));
                last_visible_gap_head_command_msec_ = nowMsec;
            }

            if (result.state == booster_soccer::ObstaclePlannerState::NoRoute &&
                booster_soccer::perception::depthViewReadyToAdvance(
                    nowMsec,
                    visible_gap_attained_since_msec_,
                    latestDepthReceiveMsec,
                    viewDwellMsec)) {
                visible_gap_scan_side_ = -visible_gap_scan_side_;
                visible_gap_attained_since_msec_ = -1;
            }
        }
    } else {
        // Let CamTrackBall/search own the head whenever no gap proof is needed.
        last_occlusion_head_command_msec_ = 0;
        visible_gap_attained_since_msec_ = -1;
        last_visible_gap_head_command_msec_ = 0;
    }

    setOutput("safe_vx", result.safeVx);
    setOutput("safe_vy", result.safeVy);
    setOutput("safe_theta", result.safeTheta);
    // Enabled commands have already been quantized and safety checked by the
    // planner. Prevent RobotClient from silently promoting them afterward.
    setOutput("apply_min_x", enabled ? false : result.applyMinX);
    setOutput("apply_min_y", enabled ? false : result.applyMinY);
    setOutput("apply_min_theta", enabled ? false : result.applyMinTheta);
    setOutput("state", state);
    setOutput("nearest_distance", nearestDistance);
    setOutput("side", side);
    setOutput("ball_state", ballState);
    setOutput("expected_ball_x", expectedX);
    setOutput("expected_ball_y", expectedY);
    setOutput("depth_age_ms", finiteOr(depthAge, 999999.0));
    setOutput("shoot_path_clear", shootPathClear);

    if (!has_logged_state_ || state != last_logged_state_ ||
        ballState != last_logged_ball_state_ ||
        shootPathClear != last_logged_shoot_path_clear_ ||
        nowMsec - last_log_msec_ >= 500) {
        brain_->log->log(
            "ObstacleAvoidance/state",
            format(
                "state: %s mode: %s nearest: %.3f side: %s depthAgeMs: %.1f ballState: %s expectedBallX: %.3f expectedBallY: %.3f shootPathClear: %s vx: %.3f vy: %.3f theta: %.3f",
                state.c_str(),
                mode.c_str(),
                nearestDistance,
                side.c_str(),
                finiteOr(depthAge, 999999.0),
                ballState.c_str(),
                expectedX,
                expectedY,
                shootPathClear ? "true" : "false",
                result.safeVx,
                result.safeVy,
                result.safeTheta));
        last_logged_state_ = state;
        last_logged_ball_state_ = ballState;
        last_logged_shoot_path_clear_ = shootPathClear;
        has_logged_state_ = true;
        last_log_msec_ = nowMsec;
    }

    had_current_depth_ball_ = had_current_depth_ball_ || depthBallVisible;
    return BT::NodeStatus::SUCCESS;
}
