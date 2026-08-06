#pragma once

#include <behaviortree_cpp/behavior_tree.h>

#include <cstdint>
#include <deque>
#include <string>

#include "obstacle_planner.h"

class Brain;

/**
 * Final velocity safety filter used by Autonomous_run.sh.
 *
 * The node never publishes motion.  It consumes the vector produced by the
 * selected behavior, applies the latest depth-map safety constraints, and
 * returns the only vector that the tree may pass to SetVelocity.
 */
class ObstacleAvoidance : public BT::SyncActionNode
{
public:
    ObstacleAvoidance(
        const std::string &name,
        const BT::NodeConfig &config,
        Brain *brain);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    struct BallObservation
    {
        std::int64_t time_msec = 0;
        double odom_x = 0.0;
        double odom_y = 0.0;
    };

    Brain *brain_;
    booster_soccer::ObstacleVelocityPlanner planner_;
    std::deque<BallObservation> ball_history_;

    bool had_current_depth_ball_ = false;
    bool expected_ball_valid_ = false;
    double expected_ball_odom_x_ = 0.0;
    double expected_ball_odom_y_ = 0.0;
    double occlusion_start_robot_odom_x_ = 0.0;
    double occlusion_start_robot_odom_y_ = 0.0;
    std::int64_t occlusion_start_msec_ = 0;
    std::int64_t last_ball_observation_receive_nsec_ = 0;
    std::int64_t last_occlusion_detection_receive_nsec_ = 0;
    int expected_position_miss_frames_ = 0;

    std::string last_logged_state_;
    std::string last_logged_ball_state_;
    bool last_logged_shoot_path_clear_ = false;
    bool has_logged_state_ = false;
    std::int64_t last_log_msec_ = 0;
    std::int64_t last_occlusion_head_command_msec_ = 0;
    bool occlusion_look_at_expected_ = true;
    int occlusion_gap_side_ = 1;
    double occlusion_target_yaw_ = 0.0;
    std::int64_t occlusion_view_attained_since_msec_ = -1;

    int visible_gap_scan_side_ = 1;
    double visible_gap_target_yaw_ = 0.0;
    std::int64_t visible_gap_attained_since_msec_ = -1;
    std::int64_t last_visible_gap_head_command_msec_ = 0;

    void rememberCurrentBall(std::int64_t now_msec);
    bool stationaryBallHistory(
        std::int64_t now_msec,
        double &odom_x,
        double &odom_y) const;
    void clearExpectedBall();
};
