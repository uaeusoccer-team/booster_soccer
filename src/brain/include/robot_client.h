#pragma once

#include <atomic>
#include <iostream>
#include <string>

#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "booster_msgs/msg/rpc_req_msg.hpp"

using namespace std;

class Brain; // Mutually dependent classes; forward declaration


/**
 * `RobotClient` contains calls to RobotSDK to operate the robot.
 * The implementation currently depends on some `Brain` internals, so they are mutually dependent.
 */
class RobotClient
{
public:
    RobotClient(Brain* argBrain) : brain(argBrain) {}

    void init(string robot_name);

    /**
     * @brief 
     *
     * @param pitch
     * @param yaw
     *
    * @return int , 0 indicates success
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief 
     * 
     * @param x double, 
     * @param y double, 
     * @param theta double, 
     * @param applyMinX, applyMinY, applyMinTheta Whether to raise a small
     * nonzero command to the configured minimum speed for that axis.
     * 
    * @return int , 0 indicates success
     * 
    */
    int setVelocity(
        double x,
        double y,
        double theta,
        bool applyMinX = true,
        bool applyMinY = true,
        bool applyMinTheta = true);

    /**
     * @brief Sign of the last non-zero angular velocity actually sent.
     *
     * Positive is left, negative is right, and zero means no non-zero turn
     * has been recorded yet. Zero velocity commands preserve the previous
     * recorded direction.
     */
    int getLastTurnDirection() const
    {
        return _lastNonZeroThetaSign.load(std::memory_order_relaxed);
    }

    /**
     * @brief Sign of the angular velocity in the command currently active.
     *
     * Unlike getLastTurnDirection(), this returns zero as soon as the active
     * command stops turning. CamFindBall snapshots it at ball loss to decide
     * whether to continue an ongoing body turn or run a head-only scan.
     */
    int getCurrentTurnDirection(double epsilon = 1e-3) const
    {
        if (_vtheta > epsilon) return 1;
        if (_vtheta < -epsilon) return -1;
        return 0;
    }

    int crabWalk(double angle, double speed);

    /**
     * @brief 
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double,
     * @param avoidObstacle bool, 
     * 
    * @return int Return value of motion-control command; 0 indicates success
     */
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief   
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double,
     * @param avoidObstacle bool, 
     * 
    * @return int Return value of motion-control command; 0 indicates success
     */

    int moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);
    /**
     * @brief 
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double, 
     * @param avoidObstacle bool, 
     * 
    * @return int Return value of motion-control command; 0 indicates success
     */
    int moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief Wave hand
     */
    int waveHand(bool doWaveHand);

    /**
     * @brief Stand up
     */
    int standUp();

    /**
     * @brief switch to RL-based vision kick mode
     * @param start true indicates entering VisualKick mode, false indicates exiting VisualKick mode
     */
     int RLVisionKick(bool start = true);

     /**
      * @brief switch to robocup gait
      */
     int robocupWalk();

    /**
     * @brief Enter damping mode
     */
    int enterDamping();

    double msecsToCollide(double vx, double vy, double vtheta, double maxTime=10000);

    bool isStandingStill(double timeBuffer = 1000);

private:
    int call(booster_interface::msg::BoosterApiReqMsg msg);
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
    double _vx = 0.0;
    double _vy = 0.0;
    double _vtheta = 0.0;
    rclcpp::Time _lastCmdTime;
    rclcpp::Time _lastNonZeroCmdTime;
    std::atomic<int> _lastNonZeroThetaSign{0};
};
