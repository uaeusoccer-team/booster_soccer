#include "brain_data.h"
#include "obstacle_perception_utils.h"
#include "utils/math.h"
#include <algorithm>

BrainData::BrainData()
{
    std::fill(std::begin(penalty), std::end(penalty), SUBSTITUTE);
}

namespace
{
double ageMsecs(const rclcpp::Time &now, const rclcpp::Time &then)
{
    if (then.nanoseconds() <= 0 || now.get_clock_type() != then.get_clock_type()) {
        return std::numeric_limits<double>::infinity();
    }
    const auto deltaNsec = (now - then).nanoseconds();
    if (deltaNsec < 0) {
        // A future sensor stamp indicates clock corruption/skew; it must not
        // authorize motion as a zero-age frame.
        return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(deltaNsec) / 1e6;
}

double freshnessAgeMsecs(
    const rclcpp::Time &now,
    const rclcpp::Time &receivedAt,
    const rclcpp::Time &sensorStamp)
{
    const bool sensorAgeValid = sensorStamp.nanoseconds() > 0 &&
        sensorStamp.get_clock_type() == now.get_clock_type();
    return booster_soccer::perception::combinedFreshnessAge(
        ageMsecs(now, receivedAt),
        sensorAgeValid ? ageMsecs(now, sensorStamp) : 0.0,
        sensorAgeValid);
}
} // namespace

ObstacleSnapshot BrainData::getObstacleSnapshot() const
{
    std::lock_guard<std::mutex> lock(_obstacleSnapshotMutex);
    return _obstacleSnapshot;
}

void BrainData::setObstacleSnapshot(const ObstacleSnapshot &snapshot)
{
    std::lock_guard<std::mutex> lock(_obstacleSnapshotMutex);
    _obstacleSnapshot = snapshot;
}

double BrainData::obstacleSnapshotAgeMsecs(const rclcpp::Time &now) const
{
    std::lock_guard<std::mutex> lock(_obstacleSnapshotMutex);
    return freshnessAgeMsecs(
        now, _obstacleSnapshot.received_at, _obstacleSnapshot.stamp);
}

Pose2D BrainData::getRobotPoseToOdom() const
{
    std::lock_guard<std::mutex> lock(_robotPoseToOdomMutex);
    return robotPoseToOdom;
}

void BrainData::setRobotPoseToOdom(const Pose2D &pose)
{
    std::lock_guard<std::mutex> lock(_robotPoseToOdomMutex);
    robotPoseToOdom = pose;
}

void BrainData::markDepthFrameReceived(
    const rclcpp::Time &stamp,
    const rclcpp::Time &receivedAt)
{
    std::lock_guard<std::mutex> lock(_depthFreshnessMutex);
    _lastDepthSensorStamp = stamp;
    _lastDepthReceiveTime = receivedAt;
}

rclcpp::Time BrainData::getLastDepthReceiveTime() const
{
    std::lock_guard<std::mutex> lock(_depthFreshnessMutex);
    return _lastDepthReceiveTime;
}

rclcpp::Time BrainData::getLastDepthSensorStamp() const
{
    std::lock_guard<std::mutex> lock(_depthFreshnessMutex);
    return _lastDepthSensorStamp;
}

double BrainData::depthAgeMsecs(const rclcpp::Time &now) const
{
    std::lock_guard<std::mutex> lock(_depthFreshnessMutex);
    return freshnessAgeMsecs(
        now, _lastDepthReceiveTime, _lastDepthSensorStamp);
}

void BrainData::markDetectionFrameReceived(
    const rclcpp::Time &stamp,
    const rclcpp::Time &receivedAt)
{
    std::lock_guard<std::mutex> lock(_detectionFreshnessMutex);
    _lastDetectionSensorStamp = stamp;
    _lastDetectionReceiveTime = receivedAt;
}

rclcpp::Time BrainData::getLastDetectionReceiveTime() const
{
    std::lock_guard<std::mutex> lock(_detectionFreshnessMutex);
    return _lastDetectionReceiveTime;
}

rclcpp::Time BrainData::getLastDetectionSensorStamp() const
{
    std::lock_guard<std::mutex> lock(_detectionFreshnessMutex);
    return _lastDetectionSensorStamp;
}

double BrainData::detectionAgeMsecs(const rclcpp::Time &now) const
{
    std::lock_guard<std::mutex> lock(_detectionFreshnessMutex);
    return freshnessAgeMsecs(
        now, _lastDetectionReceiveTime, _lastDetectionSensorStamp);
}

BallDepthObservation BrainData::getBallDepthObservation() const
{
    std::lock_guard<std::mutex> lock(_ballDepthObservationMutex);
    return _ballDepthObservation;
}

void BrainData::setBallDepthObservation(const BallDepthObservation &observation)
{
    std::lock_guard<std::mutex> lock(_ballDepthObservationMutex);
    _ballDepthObservation = observation;
}

vector<GameObject> BrainData::getMarkingsByType(set<string> types) {
    if (types.size() == 0) return getMarkings();

    // else
    vector<GameObject> res = {};
    auto markings = getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        if (types.count(markings[i].label) > 0) res.push_back(markings[i]);
    }
    
    return res;
}

vector<FieldMarker> BrainData::getMarkersForLocator()
{
    vector<FieldMarker> res;
    auto markings = getMarkings();
    for (size_t i = 0; i < markings.size(); i++)
    {
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

        char markerType;
        if (label == "LCross")
            markerType = 'L';
        else if (label == "TCross")
            markerType = 'T';
        else if (label == "XCross")
            markerType = 'X';
        else if (label == "PenaltyPoint")
            markerType = 'P';

        res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    return res;
}

Pose2D BrainData::robot2field(const Pose2D &poseToRobot)
{
    Pose2D poseToField;
    transCoord(
        poseToRobot.x, poseToRobot.y, poseToRobot.theta,
        robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
        poseToField.x, poseToField.y, poseToField.theta);
    poseToField.theta = toPInPI(poseToField.theta);
    return poseToField;
}

Pose2D BrainData::field2robot(const Pose2D &poseToField)
{
    Pose2D poseToRobot;
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(robotPoseToField.theta) * robotPoseToField.x - cos(robotPoseToField.theta) * robotPoseToField.y;
    xfr = -cos(robotPoseToField.theta) * robotPoseToField.x - sin(robotPoseToField.theta) * robotPoseToField.y;
    thetafr = -robotPoseToField.theta;
    transCoord(
        poseToField.x, poseToField.y, poseToField.theta,
        xfr, yfr, thetafr,
        poseToRobot.x, poseToRobot.y, poseToRobot.theta);
    return poseToRobot;
}
