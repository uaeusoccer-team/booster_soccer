#pragma once

#include <atomic>
#include <string>
#include <mutex>
#include <tuple>
#include <cstdint>
#include <limits>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include "booster_interface/msg/odometer.hpp"
#include <Eigen/Dense> 

#include "types.h"
#include "RoboCupGameControlData.h"

using namespace std;

/**
 * A connected obstacle extracted from one valid depth frame. Coordinates and
 * velocities are expressed in the robot/base frame.
 */
struct ObstacleComponent
{
    int id = 0;
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double radius = 0.0;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    double bearing = 0.0;
    double confidence = 0.0;
    int occupied_cell_count = 0;
    int sample_count = 0;
    string label = "Obstacle";
    rclcpp::Time last_seen_at;
    bool carried = false;
};

/**
 * Directional depth coverage. `observed == false` means the direction is
 * unknown and must not be assumed clear. `free_range` is the measured clear
 * range along the bin centre, capped by the configured depth-map range.
 */
struct ObstacleAngularCoverage
{
    double angle = 0.0;
    bool observed = false;
    double free_range = 0.0;
    // Origin of this ray in the snapshot's robot frame. Retaining individual
    // origins lets short-lived head-scan coverage survive robot motion without
    // pretending every ray was observed from the newest camera viewpoint.
    double origin_x = 0.0;
    double origin_y = 0.0;
    rclcpp::Time observed_at;
};

/**
 * Immutable-by-convention copy of the latest valid obstacle map. `stamp` is
 * the sensor stamp; `received_at` is the local ROS receive time used for
 * freshness checks. A valid sensor stamp is checked as well, so a publisher
 * repeating a frozen frame cannot accidentally authorize motion.
 */
struct ObstacleSnapshot
{
    bool ready = false;
    rclcpp::Time stamp;
    rclcpp::Time received_at;
    Pose2D robot_pose_to_odom;
    vector<GameObject> obstacles;
    vector<ObstacleComponent> components;
    vector<ObstacleAngularCoverage> coverage;
};

/** Fresh, depth-confirmed visual ball used only to mask the physical ball. */
struct BallDepthObservation
{
    bool visible = false;
    bool depth_confirmed = false;
    Point position_to_robot{0.0, 0.0, 0.0};
    BoundingBox bounding_box{0.0, 0.0, 0.0, 0.0};
    int image_width = 0;
    int image_height = 0;
    // Odom pose paired with this detection receive time. Occlusion recovery
    // must not anchor a delayed robot-frame ball sample using a later pose.
    Pose2D robot_pose_to_odom;
    rclcpp::Time stamp;
    rclcpp::Time received_at;
};

/**
 * `BrainData` stores runtime (dynamic) data used by `Brain` during decision-making.
 * This is separate from `BrainConfig` which holds static configuration.
 * Utility functions for data processing can also be placed here.
 */
class BrainData
{
public:
    BrainData();
    /* ------------------------------------ Match-related state variables ------------------------------------ */

    int score = 0;
    int oppoScore = 0;
    int secsRemaining = 0;  // Remaining match time (seconds)
    int penalty[HL_MAX_NUM_PLAYERS]; 
    int oppoPenalty[HL_MAX_NUM_PLAYERS]; 
    bool isKickingOff = false; 
    rclcpp::Time kickoffStartTime; 
    bool isFreekickKickingOff = false; 
    rclcpp::Time freekickKickoffStartTime; 
    int liveCount = 0; 
    int oppoLiveCount = 0; 
    string realGameSubState; 

    /* ------------------------------------ Data recording ------------------------------------ */

   
    Pose2D robotPoseToOdom;  
    Pose2D odomToField;      
    Pose2D robotPoseToField; 

    std::atomic<double> headPitch{0.0};
    std::atomic<double> headYaw{0.0};
    std::atomic<bool> headStateReceived{false};
    Eigen::Matrix4d camToRobot = Eigen::Matrix4d::Identity(); 


    // Detection callbacks and behavior-tree ticks may run on different
    // executor threads in downstream deployments. Keep this visibility flag
    // atomic; the full ball observation is published through the mutexed
    // BallDepthObservation snapshot below.
    std::atomic<bool> ballDetected{false};
    // A new RGB acquisition must contain usable depth once before tracking is
    // authorized. The latch remains true across temporary depth loss and is
    // reset only when the accepted RGB ball disappears completely.
    std::atomic<bool> ballDepthAcquired{false};
    std::atomic<std::uint64_t> ballTrackingGeneration{0};
    // Last RGB/head observation used to choose the direction and urgency of a
    // search after the ball leaves the image. This memory is deliberately
    // independent of the depth position and the robot's velocity-command
    // history. Positive direction is left and negative direction is right.
    std::atomic<int> lastBallSearchDirection{0};
    std::atomic<double> lastBallPixelX{0.0};
    std::atomic<double> lastBallPixelY{0.0};
    std::atomic<double> lastBallPixelDx{0.0};
    std::atomic<double> lastBallPixelDy{0.0};
    std::atomic<double> lastBallObservedHeadYaw{0.0};
    std::atomic<double> lastBallObservedHeadPitch{0.0};
    std::atomic<std::uint64_t> lastBallSearchGeneration{0};
    GameObject ball{};
    GameObject tmBall{};
    double robotBallAngleToField; 
    bool lose_ball = false;

    inline vector<GameObject> getRobots() const {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        return _robots;
    }
    inline void setRobots(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        _robots = newVec;
    }

    inline vector<GameObject> getSemanticObstacles() const {
        std::lock_guard<std::mutex> lock(_semanticObstaclesMutex);
        return _semanticObstacles;
    }
    inline void setSemanticObstacles(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_semanticObstaclesMutex);
        _semanticObstacles = newVec;
    }


    inline vector<GameObject> getGoalposts() const {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        return _goalposts;
    }
    inline void setGoalposts(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        _goalposts = newVec;
    }


    inline vector<GameObject> getMarkings() const {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        return _markings;
    }
    inline void setMarkings(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        _markings = newVec;
    }

    inline vector<FieldLine> getFieldLines() const {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        return _fieldLines;
    }
    inline void setFieldLines(const vector<FieldLine>& newVec) {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        _fieldLines = newVec;
    }


    inline vector<GameObject> getObstacles() const {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        return _obstacles;
    }
    inline void setObstacles(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        _obstacles = newVec;
    }

    ObstacleSnapshot getObstacleSnapshot() const;
    void setObstacleSnapshot(const ObstacleSnapshot &snapshot);
    double obstacleSnapshotAgeMsecs(const rclcpp::Time &now) const;

    Pose2D getRobotPoseToOdom() const;
    void setRobotPoseToOdom(const Pose2D &pose);

    // Called only after a complete, valid depth snapshot is published.
    void markDepthFrameReceived(const rclcpp::Time &stamp, const rclcpp::Time &receivedAt);
    rclcpp::Time getLastDepthReceiveTime() const;
    rclcpp::Time getLastDepthSensorStamp() const;
    double depthAgeMsecs(const rclcpp::Time &now) const;

    void markDetectionFrameReceived(const rclcpp::Time &stamp, const rclcpp::Time &receivedAt);
    rclcpp::Time getLastDetectionReceiveTime() const;
    rclcpp::Time getLastDetectionSensorStamp() const;
    double detectionAgeMsecs(const rclcpp::Time &now) const;

    BallDepthObservation getBallDepthObservation() const;
    void setBallDepthObservation(const BallDepthObservation &observation);


    double kickDir = 0.; 
    string kickType = "shoot"; 
    bool isDirectShoot = false; 


    TMStatus tmStatus[HL_MAX_NUM_PLAYERS]; 
    int tmCmdId = 0; 
    rclcpp::Time tmLastCmdChangeTime; 
    int tmMyCmd = 0; 
    int tmMyCmdId = 0; 
    int tmReceivedCmd = 0; 
    bool tmImLead = true; 
    bool tmImAlive = true; 
    double tmMyCost = 0.;
    int tmMyCostRank = 0; // Rank of my cost to reach the ball, used for multi-robot coordination. Cost roughly equals seconds to reach/kick the ball.
    int myStrikerIDRank = 0; // My ID rank among strikers, used for multi-robot coordination.
    bool tmImInVisualKick = false; // Whether I am currently in VisualKick mode, used to coordinate with teammates and avoid conflicts.

    bool shouldExitRLVisionKick = false; // Whether to exit RL-based vision kick mode, used to coordinate with brain tree and ensure smooth transition back to normal behavior after visual kick.

    int discoveryMsgId = 0;
    rclcpp::Time discoveryMsgTime;
    int sendId = 0;
    rclcpp::Time sendTime;
    int receiveId[HL_MAX_NUM_PLAYERS];
    rclcpp::Time receiveTime[HL_MAX_NUM_PLAYERS]; 
    string tmIP;
    

    RobotRecoveryState recoveryState = RobotRecoveryState::IS_READY;
    bool isRecoveryAvailable = false; 
    int currentRobotModeIndex = -1;
    int recoveryPerformedRetryCount = 0; 
    bool recoveryPerformed = false;


    rclcpp::Time timeLastDet; 
    bool camConnected = false; 
    rclcpp::Time timeLastLineDet; 
    rclcpp::Time lastSuccessfulLocalizeTime;
    rclcpp::Time timeLastGamecontrolMsg; 
    rclcpp::Time timeLastLogSave; 
    VisionBox visionBox;  
    rclcpp::Time lastTick; 


    /**
     * @brief Get markings by type
     *
     * @param types set<string>, empty set means all types; otherwise specify types such as "LCross", "TCross", "XCross", "PenaltyPoint"
     *
     * @return vector<GameObject> markings that match the specified types
     */
    vector<GameObject> getMarkingsByType(set<string> types={});


    vector<FieldMarker> getMarkersForLocator();


    Pose2D robot2field(const Pose2D &poseToRobot);


    Pose2D field2robot(const Pose2D &poseToField);

private:
    vector<GameObject> _robots = {};
    mutable std::mutex _robotsMutex;

    vector<GameObject> _semanticObstacles = {};
    mutable std::mutex _semanticObstaclesMutex;

    vector<GameObject> _goalposts = {}; 
    mutable std::mutex _goalpostsMutex;

    vector<GameObject> _markings = {};                             
    mutable std::mutex _markingsMutex;

    vector<FieldLine> _fieldLines = {};
    mutable std::mutex _fieldLinesMutex;

    vector<GameObject> _obstacles = {};
    mutable std::mutex _obstaclesMutex;

    ObstacleSnapshot _obstacleSnapshot;
    mutable std::mutex _obstacleSnapshotMutex;

    mutable std::mutex _robotPoseToOdomMutex;

    rclcpp::Time _lastDepthReceiveTime;
    rclcpp::Time _lastDepthSensorStamp;
    mutable std::mutex _depthFreshnessMutex;

    rclcpp::Time _lastDetectionReceiveTime;
    rclcpp::Time _lastDetectionSensorStamp;
    mutable std::mutex _detectionFreshnessMutex;

    BallDepthObservation _ballDepthObservation;
    mutable std::mutex _ballDepthObservationMutex;

};
