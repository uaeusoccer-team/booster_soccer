#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <fstream> 
#include <yaml-cpp/yaml.h>

#include "brain.h"
#include "obstacle_perception_utils.h"
#include "utils/print.h"
#include "utils/math.h"
#include "utils/misc.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using std::placeholders::_1;

#define SUB_STATE_QUEUE_SIZE 1

Brain::Brain() : rclcpp::Node("brain_node")
{
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
        *this,
        rclcpp::QoS(10).transient_local());

    // need to declare parameters before accessing them, and use dot notation for nested parameters
    
    declare_parameter<int>("game.team_id", 0);
    declare_parameter<bool>(GAME_AGENT_MODE_PARAM, false);
    // player_id parameter validation
    rcl_interfaces::msg::ParameterDescriptor player_id_desc;
    player_id_desc.description = "player_id must be an integer between 1 and 11";
    player_id_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    player_id_desc.integer_range.resize(1);
    player_id_desc.integer_range[0].from_value = 1;
    player_id_desc.integer_range[0].to_value = HL_MAX_NUM_PLAYERS;
    player_id_desc.integer_range[0].step = 1;
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    // player_role parameter validation
    rcl_interfaces::msg::ParameterDescriptor player_role_desc;
    player_role_desc.description = "Player role must be either 'striker' or 'goal_keeper'";
    player_role_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    declare_parameter<string>("game.player_role", "", player_role_desc);
    declare_parameter<bool>("game.treat_person_as_robot", false);
    declare_parameter<int>("game.number_of_players", 2);

    declare_parameter<string>("robot.robot_name", "");
    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.5);
    declare_parameter<double>("robot.yaw_offset", 0.0);
    declare_parameter<double>("robot.vx_limit", 1.0);
    declare_parameter<double>("robot.vy_limit", 0.4);
    declare_parameter<double>("robot.vtheta_limit", 1.0);
    declare_parameter<double>("robot.min_vx", 0.4);
    declare_parameter<double>("robot.min_vy", 0.3);
    declare_parameter<double>("robot.min_vtheta", 0.2);

    declare_parameter<double>("strategy.ball_confidence_threshold", 50.0);
    declare_parameter<double>("strategy.ball_memory_timeout", 3.0);
    declare_parameter<double>("strategy.tm_ball_dist_threshold", 3.0);
    declare_parameter<bool>("strategy.limit_near_ball_speed", true);
    declare_parameter<double>("strategy.near_ball_speed_limit", 0.3);
    declare_parameter<double>("strategy.near_ball_range", 4.0);
    declare_parameter<bool>("strategy.abort_kick_when_ball_moved", false);
    declare_parameter<bool>("strategy.enable_bypass", false);
    declare_parameter<bool>("strategy.enable_shoot", false);
    declare_parameter<bool>("strategy.enable_directional_kick", false);

    declare_parameter<bool>("strategy.use_squat_block", false);
    declare_parameter<double>("strategy.squat_block_msecs", 2000.0);
    declare_parameter<bool>("strategy.use_move_block", true);
    declare_parameter<double>("strategy.move_block_msecs", 2000.0);
    declare_parameter<bool>("strategy.enable_auto_visual_kick", false);
    declare_parameter<bool>("strategy.enable_auto_visual_defend", false);

    declare_parameter<bool>("strategy.power_shoot.enable", false);
    declare_parameter<bool>("strategy.power_shoot.use_for_kickoff", false);
    declare_parameter<double>("strategy.power_shoot.xmin", 0.5);
    declare_parameter<double>("strategy.power_shoot.xmax", 1.0);
    declare_parameter<double>("strategy.power_shoot.ymin", -0.5);
    declare_parameter<double>("strategy.power_shoot.ymax", 0.5);
    declare_parameter<double>("strategy.shoot.threat_threshold", 0.0);
    declare_parameter<double>("strategy.shoot.xmin", 0.5);
    declare_parameter<double>("strategy.shoot.xmax", 1.0);
    declare_parameter<double>("strategy.shoot.ymin", -0.5);
    declare_parameter<double>("strategy.shoot.ymax", 0.5);
    declare_parameter<bool>("strategy.cooperation.enable_role_switch", true);
    declare_parameter<double>("strategy.cooperation.ball_control_cost_threshold", 10.0);

    declare_parameter<bool>("obstacle_avoidance.enable", true);
    declare_parameter<int>("obstacle_avoidance.depth_sample_step", 8);
    declare_parameter<double>("obstacle_avoidance.obstacle_min_height", 0.12);
    declare_parameter<double>("obstacle_avoidance.obstacle_max_height", 2.0);
    declare_parameter<double>("obstacle_avoidance.grid_size", 0.1);
    declare_parameter<double>("obstacle_avoidance.max_x", 3.0);
    declare_parameter<double>("obstacle_avoidance.max_y", 5.0);
    declare_parameter<double>("obstacle_avoidance.exclusion_x", 0.12);
    declare_parameter<double>("obstacle_avoidance.exclusion_y", 0.18);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_radius", 0.2);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_height", 0.3);
    declare_parameter<double>("obstacle_avoidance.occupancy_threshold", 3.0);
    declare_parameter<double>("obstacle_avoidance.collision_threshold", 0.4);
    declare_parameter<double>("obstacle_avoidance.safe_distance", 2.0);
    declare_parameter<double>("obstacle_avoidance.avoid_secs", 3.0);
    declare_parameter<bool>("obstacle_avoidance.enable_freekick_avoid", false);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_safe_distance", 0.5);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_avoid_secs", 1.5);
    declare_parameter<double>("obstacle_avoidance.obstacle_memory_msecs", 300.0);
    declare_parameter<double>("obstacle_avoidance.avoid_distance", 1.4);
    declare_parameter<double>("obstacle_avoidance.stop_distance", 0.5);
    declare_parameter<double>("obstacle_avoidance.depth_stale_msecs", 300.0);
    declare_parameter<double>("obstacle_avoidance.detection_stale_msecs", 300.0);
    declare_parameter<double>("obstacle_avoidance.head_pose_tolerance_msecs", 40.0);
    declare_parameter<double>("obstacle_avoidance.angular_resolution_degrees", 5.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_chase", false);
    declare_parameter<double>("obstacle_avoidance.chase_ao_safe_dist", 2.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_kick", false);
    declare_parameter<double>("obstacle_avoidance.kick_ao_safe_dist", 1.0);
    
    declare_parameter<bool>("RLVisionKick.enableAutoVisualKick", true);
    declare_parameter<double>("RLVisionKick.autoVisualKickEnableDistMin", 0.5);
    declare_parameter<double>("RLVisionKick.autoVisualKickEnableDistMax", 1.5);
    declare_parameter<double>("RLVisionKick.autoVisualKickEnableAngle", 0.5);
    
    declare_parameter<int>("locator.min_marker_count", 5);
    declare_parameter<double>("locator.max_residual", 0.3);

    declare_parameter<bool>("enable_com", true);

    declare_parameter<string>("vision.image_camera_info_topic", "/camera/color/camera_info");
    declare_parameter<string>("vision.depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<string>("vision.depth_camera_info_topic", "/camera/depth/camera_info");


    declare_parameter<string>("game_control_ip", "0.0.0.0");

    declare_parameter<string>("tree_file_path", "");
    declare_parameter<string>("vision_config_path", "");
    declare_parameter<string>("vision_config_local_path", "");

    declare_parameter<int>("recovery.retry_max_count", 2);
}

Brain::~Brain()
{

}

void Brain::init()
{
    
    config = std::make_shared<BrainConfig>(this);
    loadConfig();

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();
    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);
    communication = std::make_shared<BrainCommunication>(this);
    visualizer = std::make_shared<VisualizationPublisher>(this);

    locator->init(config->fieldDimensions, config->get_min_marker_count(), config->get_max_residual());

   
    tree->init();

   
    client->init(config->get_robot_name());

    
    communication->initCommunication();

    data->lastSuccessfulLocalizeTime = get_clock()->now();
    data->timeLastDet = get_clock()->now();
    data->timeLastLineDet = get_clock()->now();
    data->timeLastGamecontrolMsg = get_clock()->now();
    data->ball.timePoint = get_clock()->now();

    
    auto now = get_clock()->now();
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->tmStatus[i].isAlive = false;
        data->tmStatus[i].timeLastCom = now;
    }
    data->tmLastCmdChangeTime = now;


    string topic_suffix = "";
    if(config->get_robot_name() != "") {
        RCLCPP_INFO(this->get_logger(), "Robot name set to: %s", config->get_robot_name().c_str());
        topic_suffix = "/" + config->get_robot_name();
    }
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_soccer/detection" + topic_suffix, SUB_STATE_QUEUE_SIZE, bind(&Brain::detectionsCallback, this, _1));
    subFieldLine = create_subscription<vision_interface::msg::LineSegments>("/booster_soccer/line_segments" + topic_suffix, SUB_STATE_QUEUE_SIZE, bind(&Brain::fieldLineCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state" + topic_suffix,  SUB_STATE_QUEUE_SIZE, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state" + topic_suffix, SUB_STATE_QUEUE_SIZE, bind(&Brain::lowStateCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose" + topic_suffix, SUB_STATE_QUEUE_SIZE, bind(&Brain::headPoseCallback, this, _1));
    recoveryStateSubscription = create_subscription<booster_interface::msg::RawBytesMsg>("fall_down_recovery_state" + topic_suffix, SUB_STATE_QUEUE_SIZE, bind(&Brain::recoveryStateCallback, this, _1));

    imageCameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
        config->get_image_camera_info_topic(), SUB_STATE_QUEUE_SIZE, bind(&Brain::imageCameraInfoCallback, this, _1));

    depthCameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
        config->get_depth_camera_info_topic(), SUB_STATE_QUEUE_SIZE, bind(&Brain::depthCameraInfoCallback, this, _1));

    // create publisher for field dimensions
    pubFieldDimensions = create_publisher<std_msgs::msg::Float64MultiArray>("/booster_soccer/field_dimensions" + topic_suffix, rclcpp::QoS(1).transient_local());
    
    // create publisher for robot pose
    pubRobotPose = create_publisher<geometry_msgs::msg::Pose2D>("/booster_soccer/robot_pose" + topic_suffix, 10);
    pubBallPosition = create_publisher<geometry_msgs::msg::Point>("/booster_soccer/ball_position" + topic_suffix, 10);
    pubTeammatesPoses = create_publisher<std_msgs::msg::Float64MultiArray>("/booster_soccer/teammates_poses" + topic_suffix, 10);
    pubKickBall = create_publisher<brain::msg::Kick>("/kick_ball", 10);

    // subscribe to depth image topic
    string depthTopic = config->get_depth_image_topic();
    if (depthTopic.find("compressed") != std::string::npos) {
        compressedDepthImageSubscription = create_subscription<sensor_msgs::msg::CompressedImage>(
            depthTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::compressedDepthImageCallback, this, _1));
    } else {
        depthImageSubscription = create_subscription<sensor_msgs::msg::Image>(
            depthTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::depthImageCallback, this, _1));
    }


    // agent soccer demo without referee, run specialized settings
    if (get_parameter(GAME_AGENT_MODE_PARAM).as_bool()) {
        RCLCPP_INFO(get_logger(), "Running in agent mode, subscribing game state");

        RCLCPP_INFO(get_logger(), "Running in agent mode, subscribing team_id changes");
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto team_id_cb = [this](const rclcpp::Parameter &p) {
            RCLCPP_INFO(this->get_logger(), "[team_id_subscripter] team_id changed to %ld", p.as_int());
            if (!this->communication->isTeamChanged()) {
                RCLCPP_INFO(this->get_logger(), "[team_id_subscripter] team_id not changed, no need to re-init communication");
                return;
            }
            this->communication.reset();
            RCLCPP_INFO(this->get_logger(), "[team_id_subscripter] communication reset success");
            // Recreate communication object
            this->communication = std::make_shared<BrainCommunication>(this);
            this->communication->initCommunication();
            // Reset teammates communication status
            auto now = get_clock()->now();
            for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
                this->data->tmStatus[i].isAlive = false;
                this->data->tmStatus[i].timeLastCom = now;
            }
            RCLCPP_INFO(this->get_logger(), "[team_id_subscripter] communication re-init success");
        };

        auto player_role_cb = [this](const rclcpp::Parameter &p) {
            RCLCPP_INFO(this->get_logger(), "[player_role_subscripter] role from %s changed to %s",
                        this->tree->getEntry<string>("player_role").c_str(),
                        p.as_string().c_str());
            tree->setEntry<string>("player_role", config->get_player_role());
        };

        team_id_handle_ = param_subscriber_->add_parameter_callback("game.team_id", team_id_cb);
        player_role_handle_ = param_subscriber_->add_parameter_callback("game.player_role", player_role_cb);

        // agent mode does not have referee, force clear penalty status to allow normal communication between teammates
        std::fill(std::begin(this->data->penalty), std::end(this->data->penalty), PENALTY_NONE);
    };

    // Publish field dimensions information (called after publisher creation)
    publishFieldDimensions();
}

void Brain::loadConfig()
{
    // Load relevant parameters from vision config
    string visionConfigPath, visionConfigLocalPath;
    get_parameter("vision_config_path", visionConfigPath);
    get_parameter("vision_config_local_path", visionConfigLocalPath);
    if (!filesystem::exists(visionConfigPath)) {
        // Error and exit
        RCLCPP_ERROR(get_logger(), "vision_config_path %s not exists", visionConfigPath.c_str());
        exit(1);
    }
    // else
    YAML::Node vConfig = YAML::LoadFile(visionConfigPath);
    if (filesystem::exists(visionConfigLocalPath)) {
        YAML::Node vConfigLocal = YAML::LoadFile(visionConfigLocalPath);
        MergeYAML(vConfig, vConfigLocal);
    }

    auto extrin = vConfig["camera"]["extrin"];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            config->camToHead(i, j) = extrin[i][j].as<double>();
        }
    }

    // Match vision's transform order exactly:
    // head_to_base * headprime_to_head(compensation) * camera_to_head.
    const auto cameraConfig = vConfig["camera"];
    const double pitchCompensation = cameraConfig["pitch_compensation"]
        ? cameraConfig["pitch_compensation"].as<double>() * M_PI / 180.0
        : 0.0;
    const double yawCompensation = cameraConfig["yaw_compensation"]
        ? cameraConfig["yaw_compensation"].as<double>() * M_PI / 180.0
        : 0.0;
    const double zCompensation = cameraConfig["z_compensation"]
        ? cameraConfig["z_compensation"].as<double>()
        : 0.0;
    Eigen::Matrix3d pitchRotation;
    pitchRotation <<
        cos(pitchCompensation), 0.0, sin(pitchCompensation),
        0.0, 1.0, 0.0,
        -sin(pitchCompensation), 0.0, cos(pitchCompensation);
    Eigen::Matrix3d yawRotation;
    yawRotation <<
        cos(yawCompensation), -sin(yawCompensation), 0.0,
        sin(yawCompensation), cos(yawCompensation), 0.0,
        0.0, 0.0, 1.0;
    config->headCompensation = Eigen::Matrix4d::Identity();
    config->headCompensation.block<3, 3>(0, 0) = yawRotation * pitchRotation;
    config->headCompensation(2, 3) = zCompensation;

    string str_cam2head = "camToHead: \n";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            str_cam2head += format("%.3f ", config->camToHead(i, j));
        }
        str_cam2head += "\n";
    }
    prtDebug(str_cam2head);
    prtDebug(format(
        "camera compensation: pitch %.3f deg, yaw %.3f deg, z %.3f m",
        pitchCompensation * 180.0 / M_PI,
        yawCompensation * 180.0 / M_PI,
        zCompensation));


    config->handle();

    const double obstacleStopDistance = config->get_obstacle_stop_distance();
    const double obstacleAvoidDistance = config->get_obstacle_avoid_distance();
    const double obstacleDepthRange = config->get_max_x();
    if (!(0.0 < obstacleStopDistance &&
          obstacleStopDistance < obstacleAvoidDistance &&
          obstacleAvoidDistance <= obstacleDepthRange)) {
        throw invalid_argument(format(
            "Obstacle distances must satisfy 0 < stop_distance < avoid_distance <= max_x; got %.3f < %.3f with max_x %.3f",
            obstacleStopDistance,
            obstacleAvoidDistance,
            obstacleDepthRange));
    }
    if (config->get_depth_sample_step() <= 0 ||
        !(config->get_grid_size() > 0.0) ||
        !(config->get_max_y() > 0.0) ||
        !(config->get_obstacle_min_height() >= 0.0) ||
        !(config->get_obstacle_max_height() > config->get_obstacle_min_height()) ||
        !(config->get_occupancy_threshold() >= 1.0) ||
        !(config->get_collision_threshold() > 0.0) ||
        !(config->get_collision_threshold() <= config->get_max_x()) ||
        !(config->get_collision_threshold() <= config->get_max_y()) ||
        !(config->get_obstacle_memory_msecs() >= 0.0) ||
        !(config->get_depth_stale_msecs() > 0.0) ||
        !(config->get_detection_stale_msecs() > 0.0) ||
        !(config->get_head_pose_tolerance_msecs() >= 0.0) ||
        !(config->get_obstacle_angular_resolution_degrees() >= 1.0) ||
        !(config->get_obstacle_angular_resolution_degrees() <= 30.0) ||
        !(config->get_ball_exclusion_radius() > 0.0) ||
        !(config->get_ball_exclusion_height() >=
            config->get_obstacle_min_height())) {
        throw invalid_argument(
            "Invalid obstacle_avoidance perception/safety configuration");
    }
    const double exclusionX = config->get_exclusion_x();
    const double exclusionY = config->get_exclusion_y();
    const double maximumSelfExclusionX = std::min(0.15, obstacleStopDistance * 0.75);
    const double maximumSelfExclusionY = std::min(0.25, obstacleStopDistance * 0.75);
    if (!(exclusionX >= 0.0) || exclusionX > maximumSelfExclusionX) {
        throw invalid_argument(format(
            "obstacle_avoidance.exclusion_x %.3f creates a blind zone inside stop_distance %.3f; maximum allowed is %.3f",
            exclusionX,
            obstacleStopDistance,
            maximumSelfExclusionX));
    }
    if (!(exclusionY >= 0.0) || exclusionY > maximumSelfExclusionY) {
        throw invalid_argument(format(
            "obstacle_avoidance.exclusion_y %.3f creates a blind zone inside stop_distance %.3f; maximum allowed is %.3f",
            exclusionY,
            obstacleStopDistance,
            maximumSelfExclusionY));
    }

    // playerRole [striker, goal_keeper]
    string _playerRole = config->get_player_role();
    if (_playerRole != "striker" && _playerRole != "goal_keeper") {
        throw invalid_argument("player_role must be one of [striker, goal_keeper]. Got: " + _playerRole);
    }


    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}


void Brain::tick()
{
    std::lock_guard<std::mutex> detectionTickLock(detectionTickMutex_);
    // Output debug & log related information
    logDebugInfo();
    logLags();
    logStatusToConsole();
    
    // Publish visualization markers
    publishVisualizationMarkers();
    
    // Publish odom to map TF transform
    publishOdomToMapTF();
    
    // Publish position information
    publishRobotPose();
    publishBallPosition();
    publishTeammatesPoses();
    
    // Publish kick message
    pubKickMsg();
    
    updateMemory();
    handleSpecialStates();
    handleCooperation();

    tree->tick();
}

void Brain::handleSpecialStates() {

    const double KICKOFF_DURATION = 10.0; 
    string gameState = tree->getEntry<string>("gc_game_state");
    bool isKickoffSide = tree->getEntry<bool>("gc_is_kickoff_side");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    bool isFreekickKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    auto now = get_clock()->now();

    if (gameState == "SET" && isKickoffSide) {
        data->isKickingOff = true;
        data->kickoffStartTime = now;
    } else if (msecsSince(data->kickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isKickingOff = false;
    }

    if (gameState == "PLAY" && gameSubStateType == "FREE_KICK" && isFreekickKickoffSide) {
        data->isFreekickKickingOff = true;
        data->freekickKickoffStartTime = now;
    } else if (msecsSince(data->freekickKickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isFreekickKickingOff = false;
        data->isDirectShoot = false;
    }

    static int lastScore = 0;
    if (data->score > lastScore) {
        tree->setEntry<bool>("we_just_scored", true);
        lastScore = data->score;
    }
    if (gameState == "SET") {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

void Brain::handleCooperation() {
    auto log_ = [=](string msg) {
        log->debug("handleCooperation", msg);
    };
    log_("handle cooperation");

    const int CMD_COOLDOWN = 2000; // Minimum interval between commands in milliseconds to prevent oscillation
    const int COM_TIMEOUT = 5000; // Teammate information timeout in milliseconds, if no information is received within this time, the teammate is considered offline

    int selfId = config->get_player_id();
    int selfIdx = selfId - 1;
    int numOfPlayers = config->get_num_of_players();
    string selfRole = config->get_player_role();

    vector<int> aliveTmIdxs = {}; // Indices of all alive teammates, excluding self

    // Update own status
    data->tmImAlive = 
        (data->penalty[selfIdx] == PENALTY_NONE) // Am I online, i.e., not penalized
        && tree->getEntry<bool>("odom_calibrated"); // Before localization is successful, consider own information unreliable, not alive
    updateCostToKick(); // Current cost to kick
    log_(format("ImAlive: %d, myCost: %.1f", data->tmImAlive, data->tmMyCost));

    
    // Determine the number of alive teammates on the field based on referee data and log it
    int gcAliveCount = 0; 
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++)
    {
        if (data->penalty[i] == PENALTY_NONE) {
            gcAliveCount++;
        }
    }
    log_(format("gcAliveCnt: %d", gcAliveCount));

    // Process teammate information. If no information is received from a teammate for a long time, or the teammate is in a penalty state according to the referee, the teammate is considered offline. Then get the indices of all online teammates.
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; // Skip self

        if (
            data->penalty[i] != PENALTY_NONE // Penalized by referee
            || msecsSince(data->tmStatus[i].timeLastCom) > COM_TIMEOUT // Or communication timeout
        ) {
            data->tmStatus[i].isAlive = false;
            data->tmStatus[i].isLead = false;
        }
        
        if (data->tmStatus[i].isAlive) {
            aliveTmIdxs.push_back(i);
            log->log_scalar("tm_status",format("tm_alive_scalar_%d", i + 1), data->tmStatus[i].cost);
            log->log_scalar("tm_status",format("tm_lead_scalar_%d", i + 1), data->tmStatus[i].isLead ? 1 : 0);
        }
    }
    log_(format("alive TM Count: %d", aliveTmIdxs.size()));

    // Log information of currently alive teammates
    log_(format("Self: cost: %.1f, isLead: %d", data->tmMyCost, data->tmImLead));


    // Process ball position information reported by teammates
    static rclcpp::Time lastTmBallPosTime = get_clock()->now();
    const double TM_BALL_TIMEOUT = 1000.; 
    const double RANGE_THRESHOLD = config->get_tm_ball_dist_threshold(); 
    int trustedTMIdx = -1;
    double minRange = 1e6;
    log_(format("Find ball info among %d alive TMs", aliveTmIdxs.size()));
    for (int i = 0; i < aliveTmIdxs.size(); i++) {
        auto status = data->tmStatus[aliveTmIdxs[i]];
        log_(format("TM %d, ballDetected: %d, ballRange: %.1f", i + 1, status.ballDetected, status.ballRange));
        if (status.ballDetected && status.ballRange < minRange) {
            log_(format("tm ball range(%.1f) < minRange(%.1f)", status.ballRange, minRange));
            double dist = norm(status.ballPosToField.x - data->robotPoseToField.x, status.ballPosToField.y - data->robotPoseToField.y);
            if (dist > RANGE_THRESHOLD) {
                log_(format("tm ball dist to me(%.1f) > threshold(%.1f), TM %d can be trusted", dist, RANGE_THRESHOLD, i+ 1));
                minRange = status.ballRange;
                trustedTMIdx = aliveTmIdxs[i];
            }  else {
                log_(format("tm ball dist to me(%.1f) < threshold(%.1f), TM %d can NOT be trusted", dist, RANGE_THRESHOLD, i+ 1));
            }
        }
    }
    if (trustedTMIdx >= 0) { // Teammate saw the ball
        log_(format("Reliable tm ball found. PlayerID = %d", trustedTMIdx + 1));
        data->tmBall.posToField = data->tmStatus[trustedTMIdx].ballPosToField;
        updateRelativePos(data->tmBall);

        tree->setEntry<bool>("tm_ball_pos_reliable", true);
        lastTmBallPosTime = get_clock()->now();
        if (!tree->getEntry<bool>("ball_location_known")) { // If I have forgotten, but the teammate's information is reliable, update my memory of the ball's position with the teammate's information.
            log_("update ball.posToField");
            data->ball.posToField = data->tmBall.posToField;
            updateRelativePos(data->ball);
        }
    } else {
        log_("TM reported NO BALL or can not be trusted");
        if (msecsSince(lastTmBallPosTime) > TM_BALL_TIMEOUT) {
            log_("TM ball timeout reached");
            tree->setEntry<bool>("tm_ball_pos_reliable", false);
        }
    }

    bool isAgentMode = get_parameter(GAME_AGENT_MODE_PARAM).as_bool();
    // Based on the number of players on the field (referee data is more accurate), decide whether the goal_keeper should switch to striker
    bool switchRole = config->get_enable_role_switch();

    // In agent mode, there is no referee, so do not switch roles
    if (!isAgentMode && switchRole) {
        if (data->penalty[selfIdx] == PENALTY_NONE) { // I am not penalized
            if (gcAliveCount < numOfPlayers) { // And the field is not full
                log_("Not full team. I must be Striker");
                tree->setEntry<string>("player_role", "striker"); // Then I must be the striker
            }
        } else { // I am penalized
            if (gcAliveCount == numOfPlayers - 1) { // And there is exactly one player missing on the field
                log_("I am only on under penalty, I must be goal keeper");
                tree->setEntry<string>("player_role", "goal_keeper"); // Then I must be the goal keeper. Because I am the last one on the field, the last one is the goal keeper.
            }
    
        }
    }
    // If there is no strategy during halftime, switch the role back to the initial setting at the start of the second half
    if (tree->getEntry<string>("gc_game_state") == "INITIAL") {
        tree->setEntry<string>("player_role", selfRole);
    }

    // New lead logic, no longer sending commands, but judging by itself 
    double tmMinCost = 1e5;
    int myCostRank = 0;
    int myStrikerIDRank = 0;
    for (int i = 0; i < aliveTmIdxs.size(); i++) {
        int tmIdx = aliveTmIdxs[i];
        auto tmStatus = data->tmStatus[tmIdx];
        if (tmStatus.cost < tmMinCost) tmMinCost = tmStatus.cost;
        if (tmStatus.cost < data->tmMyCost) myCostRank++;
        if (tmIdx < selfIdx && tmStatus.role == "striker") myStrikerIDRank++;
    }
    data->tmMyCostRank = myCostRank;
    data->myStrikerIDRank = myStrikerIDRank;
    if (
        (tmMinCost <  config->get_ball_control_cost_threshold() && data->tmMyCost > tmMinCost)
        || myCostRank >= 2
    ) {
        // A teammate is controlling the ball
        data->tmImLead = false;
        tree->setEntry<bool>("is_lead", false);
        log_("I am not lead");

    } else {
        data->tmImLead = true;
        tree->setEntry<bool>("is_lead", true);
        log_("I am Lead");
    }
    log_(format("tmMinCost: %.1f, myCost: %.1f, myCostRank: %d, myStrikerIDRank: %d", tmMinCost, data->tmMyCost, myCostRank, myStrikerIDRank));

    // Publish command: Goalkeeper should attack, instruct to switch goalkeeper
    if (
        !isAgentMode
        && data->tmImAlive // I am online
        && tree->getEntry<string>("player_role") == "goal_keeper" // I am the goalkeeper
        && data->tmImLead // I am the lead
        && msecsSince(data->tmLastCmdChangeTime) > CMD_COOLDOWN // Enough time has passed since the last command.
    ) {
        auto distToGoal = [=](Pose2D pose) {
            return norm(pose.x + config->fieldDimensions.length / 2.0, pose.y);
        };
        double maxDist = 0.0;
        double minDist = 1e6;
        int minIndex = -1; // Index of the teammate closest to the goal
        double myDist = distToGoal(data->robotPoseToField);
        for (int i = 0; i < aliveTmIdxs.size(); i++) {
            int tmIdx = aliveTmIdxs[i];
            auto tmPose = data->tmStatus[tmIdx].robotPoseToField;
            double dist = distToGoal(tmPose);
            if (dist > maxDist) maxDist = dist;
            if (dist < minDist) {
                minIndex = tmIdx;
                minDist = dist;
            }
        }
        if (minIndex >= 0 && myDist > maxDist) {
            data->tmLastCmdChangeTime = get_clock()->now();
            data->tmMyCmd = 10 + minIndex + 1; // Command is 10 + teammate id, e.g., 10 + 1 = 11, means player 1 becomes the goalkeeper
            data->tmCmdId += 1; // Increase command ID to make the command unique
            data->tmMyCmdId = data->tmCmdId;
            tree->setEntry<string>("player_role", "striker");
            log_(format("goalie: i am too far from goal, i ask player %d to attack", minIndex + 1));
        } else {
            log_(format("goalie: i am close enough to goal, no need to attack, my dist: %.2f", myDist));
        }
    }

    // Handle received commands
    auto cmd = data->tmReceivedCmd;
    if (cmd != 0) { // Teammate wants to take control
        log_(format("received cmd %d from teammate", cmd));
        if (cmd == 100) { // Teammate wants to take control
            data->tmImLead = false; // I am no longer the lead
            tree->setEntry<bool>("is_lead", false);
            log_("teammate wants to take lead, i'll assist");
        } else if (cmd > 10 && cmd < 20) { // Goalkeeper wants to attack
            log_("goalie wants to attack");
            int newGoalieId = cmd - 10;
            if (newGoalieId == selfId) { // I am the new goalkeeper
                log_("i become goalie");
                tree->setEntry<string>("player_role", "goal_keeper");
            } else { // I am not the new goalkeeper
                log_(format("teammate %d becomes goalie", newGoalieId));
            }
        } else {
            log_(format("unknown cmd %d from teammate", cmd));
        }

        data->tmReceivedCmd = 0; // Clear the received command, so it is executed only once.
    }

    tree->setEntry<bool>("is_lead", data->tmImLead);

    // In the READY state, based on the initial settings and the number of players on the field, reassign roles.
    if (
        (tree->getEntry<string>("gc_game_state") == "READY" || tree->getEntry<string>("gc_game_sub_state") == "GET_READY") 
        && gcAliveCount == numOfPlayers
    ) {
        // READY state, and all players are on the field, return to initial roles
        tree->setEntry<string>("player_role", selfRole);
        log_(format("all teammates on field. Back to initial role: %s", selfRole.c_str()));
    }

    return;
}

void Brain::updateMemory()
{
    updateBallMemory();
    updateRobotMemory();
    updateObstacleMemory();
    updateKickoffMemory();
}

void Brain::updateObstacleMemory() {
   
    auto obstacles = data->getObstacles();
    vector<GameObject> obs_new = {};

    const double OBS_EXPIRE_TIME = config->get_obstacle_memory_msecs();
    for (int i = 0; i < obstacles.size(); i++) {
        auto obs = obstacles[i];
        if (obs.label == "Ball") continue; 
        if (msecsSince(obs.timePoint) > OBS_EXPIRE_TIME)  continue; 


        updateRelativePos(obs);
        obs_new.push_back(obs);
    }


    if (
        (config->get_enable_obstacle_avoidance() && isFreekickStartPlacing())
        || tree->getEntry<string>("gc_game_state") == "READY"
    ) {
        obs_new.push_back(data->ball);
    }

    data->setObstacles(obs_new);
}

void Brain::updateBallMemory()
{

    double secs = msecsSince(data->ball.timePoint) / 1000;
    
    double ballMemTimeout = config->get_ball_memory_timeout();

    if (secs > ballMemTimeout) 
    { 
        tree->setEntry<bool>("ball_location_known", false);
        tree->setEntry<bool>("ball_out", false); 
    }

    
    updateRelativePos(data->ball);
    updateRelativePos(data->tmBall);
    tree->setEntry<double>("ball_range", data->ball.range);
}

void Brain::updateRobotMemory() {
    auto robots = data->getRobots();
    vector<GameObject> newRobots = {};

    for (int i = 0; i < robots.size(); i++) {
        auto r = robots[i];


        if (msecsSince(r.timePoint) > 1000)  continue;


        updateRelativePos(r);
        newRobots.push_back(r);
    }

    data->setRobots(newRobots);
}

void Brain::updateKickoffMemory() {
    
    static Point ballPos;
    const double BALL_MOVE_THRESHOLD_FACTOR = 0.15; 
    const double BALL_MOVE_THRESHOLD_MIN = 0.3; 
    auto ballMoved = [=]() {
        if (!data->ballDetected.load(std::memory_order_acquire)) return false;
        double range = data->ball.range;
        double threshold = max(range * BALL_MOVE_THRESHOLD_FACTOR, BALL_MOVE_THRESHOLD_MIN);
        double posChange = norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y);
        return posChange > threshold;
    };
    static rclcpp::Time kickOffTime;
    const double TIMEOUT = 1000 * 10; 
    auto timeReached = [=]() {
        return msecsSince(kickOffTime) > TIMEOUT;
    };
    bool isWaitingForKickoff = (
        (tree->getEntry<string>("gc_game_state") == "SET"  || tree->getEntry<string>("gc_game_state") == "READY")
        && !tree->getEntry<bool>("gc_is_kickoff_side")
    );
    bool isWaitingForFreekickKickoff = (
        (tree->getEntry<string>("gc_game_sub_state") == "SET" || tree->getEntry<string>("gc_game_sub_state") == "GET_READY")
        && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
    );
    if ( isWaitingForFreekickKickoff || isWaitingForKickoff) {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    } else if (tree->getEntry<bool>("wait_for_opponent_kickoff")) {
        if (ballMoved() || timeReached()) {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY; 

    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;


    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++)
    {
        auto post = goalposts[i];
        if (post.name == "OL")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.name == "OR")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

double Brain::calcKickDir(double goalPostMargin) {
    double dir_rb_f = data->robotBallAngleToField; 
    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1];
    
    if (isAngleGood(goalPostMargin)) return dir_rb_f;

    double delta_l = fabs(toPInPI(theta_l - dir_rb_f));
    double delta_r = fabs(toPInPI(theta_r - dir_rb_f));
    if (delta_l < delta_r) return theta_l;
    // else 
    return theta_r;
}

void Brain::updateCostToKick() {
    auto log_ = [=](string msg) {
        log->debug("updateCostToKick", msg);
    };
    double cost = 0.;

    // ball not detected
    double secsSinceBallDet = msecsSince(data->ball.timePoint) / 1000;
    cost += secsSinceBallDet;
    log_(format("ball not dectect cost: %.1f", secsSinceBallDet));

    // Ball lost
    if (!tree->getEntry<bool>("ball_location_known")) {
        cost += 5.0;
        log_(format("ball lost cost: %.1f", 5.0));
    }

    // cost of chasing the ball
    cost += data->ball.range;
    log_(format("ball range cost: %.1f", data->ball.range));
    
    
    // cost of obstacles on the way to the ball
    if (distToObstacle(data->ball.yawToRobot) < 1.5) {
        log_(format("obstacle cost: %.1f", 2.0));
        cost += 0.5;
    }

    // cost of turning towards the ball
    cost += fabs(data->ball.yawToRobot) / 1.0; 
    log_(format("ball yaw cost: %.1f", fabs(data->ball.yawToRobot) / 1.0));


    // cost of bumping into teammates
    int selfIdx = config->get_player_id() - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; // Skip self

        auto status = data->tmStatus[i]; // Teammate status
        if (!status.isAlive) continue; // Skip offline teammates

        double theta_tm2ball = atan2(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double range_tm2ball = norm(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double theta_me2ball = data->robotBallAngleToField;
        double range_me2ball = data->ball.range;
        double deltaTheta = fabs(toPInPI(theta_tm2ball - theta_me2ball));

        const double BUMP_DIST = 1.0;
        if (range_tm2ball < range_me2ball && sin(deltaTheta) * range_tm2ball < BUMP_DIST) {
            cost += 2.0;
            log_(format("bump cost: %.1f", 2.0));  
        }
    }

    // cost of adjusting
    cost += fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3; // 0.4 is the approximate distance to turn around the ball, 0.3 is the approximate tangential speed to turn around the ball
    log_(format("adjust cost: %.1f", fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3));
    

    // cost increase after falling
    if (data->recoveryState == RobotRecoveryState::HAS_FALLEN) {
        cost += 15.0;
        log_(format("fall cost: %.1f", 15.0));  
    }

    
    // cost increase if localization fails
    if (!tree->getEntry<bool>("odom_calibrated")) {
        cost += 100;
        log_(format("localization cost: %.1f", 100.0));  

    }
    

    // smoothing
    double lastCost = data->tmMyCost;
    data->tmMyCost = lastCost * 0.8 + cost * 0.2;
    log_(format("lastCost: %.1f, newCost: %.1f, smoothCost: %.1f", lastCost, cost, data->tmMyCost));

    return;
}

bool Brain::isAngleGood(double goalPostMargin, string type) {
    double angle = 0;
    if (type == "kick") angle = data->robotBallAngleToField; // type=="kick" robot to ball, direction in field coordinate system
    if (type == "shoot") angle = data->robotPoseToField.theta; // type=="shoot" robot orientation
    

    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1]; 
    
    if (theta_l - theta_r < M_PI / 3 * 2) { 
        goalPostAngles = getGoalPostAngles(0.5);
        theta_l = goalPostAngles[0]; 
        theta_r = goalPostAngles[1]; 
    }

    return (theta_l > angle && theta_r < angle);
}

bool Brain::isBallOut(double locCompareDist, double lineCompareDist)
{
    auto ball = data->ball;
    auto fd = config->fieldDimensions;

    if (fabs(ball.posToField.x) > fd.length / 2 + locCompareDist)
        return true;
    if (fabs(ball.posToField.y) > fd.width / 2 + locCompareDist)
        return true;
    
    auto fieldLines = data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (
            (line.type == LineType::TouchLine || line.type == LineType::GoalLine)
            && line.confidence > 1.0
         ) {
            Point2D p = {ball.posToField.x, ball.posToField.y};
            // prtWarn(format("Ball: %.2f, %.2f PerpDist: %.2f", ball.posToField.x, ball.posToField.y, pointPerpDistToLine(p, line.posToField)));
            if (pointPerpDistToLine(p, line.posToField) > lineCompareDist) return true;
        }
    }
    return false;
}

void Brain::updateBallOut() {
    bool lastBallOut = tree->getEntry<bool>("ball_out");
    double range = lastBallOut ? 4.0 : 2.5;
    double threshold = config->get_ball_out_threshold();
    threshold += (data->isFreekickKickingOff ? 1.0 : 0.0); // If a free kick is being taken, relax the out-of-bounds judgment
    threshold *= (lastBallOut ? 1.0 : 1.5); // Prevent oscillation. If the last judgment was out-of-bounds, relax the out-of-bounds judgment
    tree->setEntry<bool>("ball_out", isBallOut(threshold, 10.0) && data->ball.range < range); // Strictly determine out-of-bounds through localization
}

double Brain::distToBorder() {
    vector<Line> borders;
    auto fd = config->fieldDimensions;
    borders.push_back({fd.length / 2, fd.width / 2, -fd.length / 2, fd.width / 2});
    borders.push_back({fd.length / 2, -fd.width / 2, -fd.length / 2, -fd.width / 2});
    borders.push_back({fd.length / 2, fd.width / 2, fd.length / 2, -fd.width / 2});
    borders.push_back({-fd.length / 2, fd.width / 2, -fd.length / 2, -fd.width / 2});
    double maxDist = -100;
    Point2D robot = {data->robotPoseToField.x, data->robotPoseToField.y};
    for (int i = 0; i < borders.size(); i++) {
        auto line = borders[i];
        double dist = pointPerpDistToLine(robot, line);
        if (dist > maxDist) maxDist = dist;
    }
    return maxDist;
}

bool Brain::isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio, double yRatio) {
    double x = (boundingBox.xmin + boundingBox.xmax) / 2.0;
    double y = (boundingBox.ymin + boundingBox.ymax) / 2.0;

    return (x  > config->cameraImageWidth * (1 - xRatio) / 2)
        && (x < config->cameraImageWidth * (1 + xRatio) / 2)
        && (y > config->cameraImageHeight * (1 - yRatio) / 2)
        && (y < config->cameraImageHeight * (1 + yRatio) / 2);
}

bool Brain::isDefensing() {
    bool isFreeKick = tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK";
    bool isKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    
    return isFreeKick && (!isKickoffSide);
}

void Brain::calibrateOdom(double x, double y, double theta)
{

    const Pose2D robotPoseToOdom = data->getRobotPoseToOdom();
    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(robotPoseToOdom.theta) * robotPoseToOdom.x - sin(robotPoseToOdom.theta) * robotPoseToOdom.y;
    y_or = sin(robotPoseToOdom.theta) * robotPoseToOdom.x - cos(robotPoseToOdom.theta) * robotPoseToOdom.y;
    theta_or = -robotPoseToOdom.theta;

    
    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);


    transCoord(
        robotPoseToOdom.x, robotPoseToOdom.y, robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);


    double placeHolder;
    // ball
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder 
    );

    // robots
    auto robots = data->getRobots();
    for (int i = 0; i < robots.size(); i++) {
        updateFieldPos(robots[i]);
    }
    data->setRobots(robots);

    // goalposts
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        updateFieldPos(goalposts[i]);
    }
    
    // markers
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        updateFieldPos(markings[i]);
    }

    // relog
    vector<GameObject> gameObjects = {};
    if (data->ballDetected.load(std::memory_order_acquire)) {
        gameObjects.push_back(data->ball);
    }
    for (int i = 0; i < markings.size(); i++) gameObjects.push_back(markings[i]);
    for (int i = 0; i < robots.size(); i++) gameObjects.push_back(robots[i]);
    for (int i = 0; i < goalposts.size(); i++) gameObjects.push_back(goalposts[i]);
}


void Brain::pubKickMsg() {
    if (!pubKickBall) return;
    if (!data->ballDetected.load(std::memory_order_acquire)) return;
    brain::msg::Kick kickMsg;
    kickMsg.header.stamp = get_clock()->now();
    kickMsg.x = data->ball.posToRobot.x;
    kickMsg.y = data->ball.posToRobot.y;
    kickMsg.dir = toPInPI(data->kickDir - data->robotPoseToField.theta);

    double goal_x = config->fieldDimensions.length / 2;
    double goal_y = 0.0;
    Pose2D goalPose;
    goalPose.x = goal_x;
    goalPose.y = goal_y;
    double ball_x = data->ball.posToField.x;
    double ball_y = data->ball.posToField.y;
    double dist = std::sqrt((goal_x - ball_x) * (goal_x - ball_x) + (goal_y - ball_y) * (goal_y - ball_y));
    dist = std::abs(dist);
    double power = 0.0;

    if (dist > 6.0) {
        power = 1.5;
    } else {
        power = 6.0;
    }
    kickMsg.power = power;

    auto goalPose_r = data->field2robot(goalPose);
    kickMsg.goal_x = goalPose_r.x;  
    kickMsg.goal_y = goalPose_r.y;

    kickMsg.robot_theta_to_field = data->robotPoseToField.theta;

    pubKickBall->publish(kickMsg);
}

double Brain::msecsSince(rclcpp::Time time)
{
    auto now = this->get_clock()->now();
    if (time.get_clock_type() != now.get_clock_type()) return 1e18;
    return (now - time).nanoseconds() / 1e6;
}

rclcpp::Time Brain::timePointFromHeader(std_msgs::msg::Header header) {
    auto stamp = header.stamp;
    // NOTE It seems that regardless of whether use_sim_time is true or false, ROS_TIME is used
    int32_t sec = stamp.sec;
    uint32_t nanosec = stamp.nanosec;
    if (sec < 0) {
        prtErr(format("Negative time: sec: %d nanosec: %u", sec, nanosec));
        sec = 0; // Prevent crash, but likely needs better handling
        nanosec = 1;
    }
    return rclcpp::Time(sec, nanosec, RCL_ROS_TIME); // should not crash
}


void Brain::joystickCallback(const booster_interface::msg::RemoteControllerState &joy)
{
    auto log_ = [=](string msg) {
        log->debug("joystick", msg);
    };


    // Control the robot via joystick, non-blocking buttons
    if (
        fabs(joy.lx) > 0.1
        || fabs(joy.ly) > 0.1
        || fabs(joy.rx) > 0.1
        || fabs(joy.ry) > 0.1
    ) {
        tree->setEntry<bool>("go_manual", true);
        // prtWarn("GO Manual");
    } else {
        tree->setEntry<bool>("go_manual", false);
        // prtWarn("Axe manual take over end");
    }

    // Button response order: LT combination, RT combination, single button
    if (joy.lt && !joy.rt) { // LT combination
        // Used to control switching between different states
        if (joy.x)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
        }
        if (joy.a)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
        }
        if (joy.b)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
        }
        else if (joy.y)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
            log_("SWITCH ROLE");
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    if (get_parameter(GAME_AGENT_MODE_PARAM).as_bool()) {
        prtWarn("Agent mode, ignore game control");
        return;
    };
    data->timeLastGamecontrolMsg = get_clock()->now();

    // Handle primary game state
    auto lastGameState = tree->getEntry<string>("gc_game_state"); // Primary game state
    vector<string> gameStateMap = {
        "INITIAL", // Initialization state, players prepare off the field
        "READY",   // Ready state, players enter the field and move to their starting positions
        "SET",     // Stop action, wait for the referee to start the game
        "PLAY",    // Normal gameplay
        "END"      // Game over
    };

    int teamId = config->get_team_id();
    int playerId = config->get_player_id();
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    tree->setEntry<string>("gc_game_state", gameState);
    bool isKickOffSide = (msg.kick_off_team == teamId); // Whether our team is the kickoff side
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    // Handle secondary game state
    string gameSubStateType;
    switch (static_cast<int>(msg.secondary_state)) {
        case 0:
            gameSubStateType = "NONE";
            data->realGameSubState = "NONE";
            break;
        case 3:
            gameSubStateType = "TIMEOUT"; // Includes both team timeouts and referee timeout
            data->realGameSubState = "TIMEOUT";
            break;
        // Temporarily do not handle other states, except TIMEOUT, all are treated as FREE_KICK
        case 4:
            // gameSubStateType = "DIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "DIRECT_FREEKICK";
            data->isDirectShoot = true;
            break;
        case 5:
            // gameSubStateType = "INDIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "INDIRECT_FREEKICK";
            break;
        case 6:
            // gameSubStateType = "PENALTY_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "PENALTY_KICK";
            data->isDirectShoot = true;
            break;
        case 7:
            // gameSubStateType = "CORNER_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "CORNER_KICK";
            break;
        case 8:
            // gameSubStateType = "GOAL_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "GOAL_KICK";
            data->isDirectShoot = true;
            break;
        case 9:
            // gameSubStateType = "THROW_IN";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "THROW_IN";
            break;
        default:
            gameSubStateType = "FREE_KICK";
            break;
    }
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};                               // STOP: Stop; -> GET_READY: Move to offensive or defensive position; -> SET: Stand still
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == teamId); // In the secondary state, whether our team is the kickoff side. For example, if the current secondary state is a free kick, whether our team is taking the free kick
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    // cout << "game state: " << gameState << " game sub state type: " << gameSubStateType << endl;
    // Find team information
    game_controller_interface::msg::TeamInfo myTeamInfo;
    game_controller_interface::msg::TeamInfo oppoTeamInfo;
    if (msg.teams[0].team_number == teamId) {
        myTeamInfo = msg.teams[0];
        oppoTeamInfo = msg.teams[1];
    } else if (msg.teams[1].team_number == teamId) {
        myTeamInfo = msg.teams[1];
        oppoTeamInfo = msg.teams[0];
    } else {
        // Our team is not included in the data packet, should not process further
        prtErr(format("received invalid game controller message team0 %d, team1 %d, teamId %d",
            msg.teams[0].team_number, msg.teams[1].team_number,teamId));
        return;
    }

    int liveCount = 0;
    int oppoLiveCount = 0;
    // Handle penalty state. penalty[playerId - 1] represents whether our player is in a penalty state. Handling penalty state means the player cannot move.
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->penalty[i] = static_cast<int>(myTeamInfo.players[i].penalty);
        
        if (static_cast<int>(myTeamInfo.players[i].red_card_count) > 0) {
            data->penalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->penalty[i] == PENALTY_NONE) liveCount++;
        data->oppoPenalty[i] = static_cast<int>(oppoTeamInfo.players[i].penalty);

        if (static_cast<int>(oppoTeamInfo.players[i].red_card_count) > 0) {
            data->oppoPenalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->oppoPenalty[i] == PENALTY_NONE) oppoLiveCount++;
    }
    data->liveCount = liveCount;
    data->oppoLiveCount = oppoLiveCount;

    bool lastIsUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");
    bool isUnderPenalty = (data->penalty[playerId - 1] != PENALTY_NONE); // Whether the current robot is under penalty
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);
    if (isUnderPenalty && !lastIsUnderPenalty) tree->setEntry<bool>("odom_calibrated", false); // If penalized, need to re-enter the field, thus need to re-localize

    // FOR FUN Handle goal celebration waving logic
    data->score = static_cast<int>(myTeamInfo.score);
    data->oppoScore = static_cast<int>(oppoTeamInfo.score);
    data->secsRemaining = static_cast<int>(msg.secs_remaining);
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    std::lock_guard<std::mutex> detectionTickLock(detectionTickMutex_);
    // std::lock_guard<std::mutex> guard(data->brainMutex);
    
    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    data->camConnected = true;
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->markDetectionFrameReceived(timePoint, now);
    data->timeLastDet = timePoint; // Used to output delay information during debugging

    auto gameObjects = getGameObjects(msg);

    // Group the detected objects
    vector<GameObject> balls, goalposts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalposts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            // For debugging purposes, you can set treat_person_as_robot in the config to treat Person as Robot
            if (config->get_treat_person_as_robot())
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    // Process the grouped objects separately
    detectProcessBalls(balls, now);
    detectProcessGoalposts(goalposts);
    detectProcessMarkings(markings);
    detectProcessRobots(robots);

    // Depth remains the authority for collision geometry. These fresh labels
    // only enrich a spatially matching physical component.
    vector<GameObject> semanticObstacles = persons;
    for (const auto &robot : robots) {
        if (robot.label == "Opponent") semanticObstacles.push_back(robot);
    }
    data->setSemanticObstacles(semanticObstacles);

    // Handle and record vision information
    detectProcessVisionBox(msg);

}

void Brain::updateLinePosToField(FieldLine& line) {
    double __; // __ is a placeholder for transformations
    transCoord(
        line.posToRobot.x0, line.posToRobot.y0, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x0, line.posToField.y0, __
    );
    transCoord(
        line.posToRobot.x1, line.posToRobot.y1, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x1, line.posToField.y1, __
    );
}

void Brain::fieldLineCallback(const vision_interface::msg::LineSegments &msg)
{
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastLineDet = timePoint; // Used to output delay information during debugging

    vector<FieldLine> lines = {};
    FieldLine line;

    double x0, y0, x1, y1, __; // __ is a placeholder for transformations 
    for (int i = 0; i < msg.coordinates.size() / 4; i++) {
        int index = i * 4;
        line.posToRobot.x0 = msg.coordinates[index]; line.posOnCam.x0 = msg.coordinates_uv[index];
        line.posToRobot.y0 = msg.coordinates[index + 1]; line.posOnCam.y0 = msg.coordinates_uv[index + 1];
        line.posToRobot.x1 = msg.coordinates[index + 2]; line.posOnCam.x1 = msg.coordinates_uv[index + 2];
        line.posToRobot.y1 = msg.coordinates[index + 3]; line.posOnCam.y1 = msg.coordinates_uv[index + 3];
        updateLinePosToField(line);
        line.timePoint = timePoint;

        lines.push_back(line);
    }
    lines = processFieldLines(lines);
    data->setFieldLines(lines);

    

    
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{
    const Pose2D robotPoseToOdom{
        msg.x * config->get_robot_odom_factor(),
        msg.y * config->get_robot_odom_factor(),
        msg.theta};
    if (!std::isfinite(robotPoseToOdom.x) ||
        !std::isfinite(robotPoseToOdom.y) ||
        !std::isfinite(robotPoseToOdom.theta)) {
        RCLCPP_WARN(get_logger(), "Ignoring invalid odometry pose");
        return;
    }
    data->setRobotPoseToOdom(robotPoseToOdom);

    // Based on Odom information, update the robot's position in the Field coordinate system
    transCoord(
        robotPoseToOdom.x, robotPoseToOdom.y, robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    log->debug("odom_callback", format("x: %.1f, y: %.1f, z: %.1f", robotPoseToOdom.x, robotPoseToOdom.y, robotPoseToOdom.theta));
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    const double headYaw = msg.motor_state_serial[0].q;
    const double headPitch = msg.motor_state_serial[1].q;
    data->headYaw.store(headYaw, std::memory_order_relaxed);
    data->headPitch.store(headPitch, std::memory_order_relaxed);
    data->headStateReceived.store(true, std::memory_order_release);
    log->debug("head_angles", format("pitch: %.1f, yaw: %.1f", headPitch, headYaw));
}

void Brain::imageCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> detectionTickLock(detectionTickMutex_);
    config->cameraImageWidth = msg->width;
    config->cameraImageHeight = msg->height;
}

void Brain::depthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> detectionTickLock(detectionTickMutex_);
    std::lock_guard<std::mutex> cameraInfoLock(depthCameraInfoMutex_);

    // Using CameraInfo, calculate the camera's fovx and fovy
    config->depthCameraFx = static_cast<double>(msg->k[0]);
    config->depthCameraCx = static_cast<double>(msg->k[2]);
    config->depthCameraFy = static_cast<double>(msg->k[4]);
    config->depthCameraCy = static_cast<double>(msg->k[5]);
    double w = static_cast<double>(msg->width);
    double h = static_cast<double>(msg->height);
    config->depthCameraFovX.store(
        2.0 * atan(w / (2.0 * config->depthCameraFx)),
        std::memory_order_release);
    config->depthCameraFovY.store(
        2.0 * atan(h / (2.0 * config->depthCameraFy)),
        std::memory_order_release);
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose& msg)
{
    // Calculate head_to_base matrix
    Eigen::Matrix4d headToBase = Eigen::Matrix4d::Identity();
    
    // Get rotation matrix from quaternion
    Eigen::Quaterniond q(
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    );
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
        !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
        q.norm() < 1e-6 ||
        !std::isfinite(msg.position.x) ||
        !std::isfinite(msg.position.y) ||
        !std::isfinite(msg.position.z)) {
        RCLCPP_WARN(get_logger(), "Ignoring invalid head pose");
        return;
    }
    q.normalize();
    headToBase.block<3,3>(0,0) = q.toRotationMatrix();
    
    // Set translation vector
    headToBase.block<3,1>(0,3) = Eigen::Vector3d(
        msg.position.x,
        msg.position.y,
        msg.position.z
    );

    // Calculate the compensated camera-to-base transform with the same order
    // used by vision, then retain receive-timestamped samples for depth sync.
    const Eigen::Matrix4d camToRobot =
        headToBase * config->headCompensation * config->camToHead;
    data->camToRobot = camToRobot; // Legacy consumers use the latest pose.

    const auto receivedAt = get_clock()->now();
    std::lock_guard<std::mutex> lock(headPoseBufferMutex_);
    headPoseBuffer_.push_back(HeadPoseSample{receivedAt, camToRobot});
    while (headPoseBuffer_.size() > 32) {
        headPoseBuffer_.pop_front();
    }
}

bool Brain::selectDepthHeadPose(
    const rclcpp::Time &depthReceivedAt,
    Eigen::Matrix4d &camToRobot) const
{
    std::lock_guard<std::mutex> lock(headPoseBufferMutex_);
    const double toleranceMsecs = config->get_head_pose_tolerance_msecs();
    for (auto it = headPoseBuffer_.rbegin(); it != headPoseBuffer_.rend(); ++it) {
        if (it->receivedAt.get_clock_type() != depthReceivedAt.get_clock_type()) {
            continue;
        }
        const double ageMsecs =
            static_cast<double>((depthReceivedAt - it->receivedAt).nanoseconds()) / 1e6;
        if (ageMsecs < 0.0) {
            continue;
        }
        if (booster_soccer::perception::headPoseAgeAccepted(
                ageMsecs, toleranceMsecs)) {
            camToRobot = it->camToRobot;
            return true;
        }
        // Samples are ordered newest-first, so earlier ones will only be older.
        break;
    }
    return false;
}

void Brain::recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg)
{
    // uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,  
    // uint8_t is_recovery_available; // 1 for available, 0 for not available
    // Using RobotRecoveryState structure, convert msg inside msg to RobotRecoveryState
    try
    {
        const std::vector<unsigned char>& buffer = msg.msg;
        RobotRecoveryStateData recoveryState;
        memcpy(&recoveryState, buffer.data(), buffer.size());

        vector<RobotRecoveryState> recoveryStateMap = {
            RobotRecoveryState::IS_READY,
            RobotRecoveryState::IS_FALLING,
            RobotRecoveryState::HAS_FALLEN,
            RobotRecoveryState::IS_GETTING_UP
        };
        this->data->recoveryState = recoveryStateMap[static_cast<int>(recoveryState.state)];
        this->data->isRecoveryAvailable = static_cast<bool>(recoveryState.is_recovery_available);
        this->data->currentRobotModeIndex = static_cast<int>(recoveryState.current_planner_index);
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


int Brain::markCntOnFieldLine(const string markType, const FieldLine line, const double margin) {
    int cnt = 0;
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto marking = markings[i];
        if (marking.label == markType) {
            Point2D point = {marking.posToField.x, marking.posToField.y};
            if (fabs(pointPerpDistToLine(point, line.posToField)) < margin) {
                cnt += 1;
            }
        }
    }
    return cnt;
}

int Brain::goalpostCntOnFieldLine(const FieldLine line, const double margin) {
    int cnt = 0;
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        auto post = goalposts[i];
        Point2D point = {post.posToField.x, post.posToField.y};
        if (pointMinDistToLine(point, line.posToField) < margin) {
            cnt += 1;
        }
    }
    return cnt;
}

bool Brain::isBallOnFieldLine(const FieldLine line, const double margin) {
    auto ballPos = data->ball.posToField;
    Point2D point = {ballPos.x, ballPos.y}; 
    return fabs(pointPerpDistToLine(point, line.posToField)) < margin;
}

void Brain::identifyFieldLine(FieldLine& line) {
    auto mapLines = config->mapLines;
    FieldLine mapLine;
    double confidence;
    line.type = LineType::NA;

    double bestConfidence = 0;
    double secondBestConfidence = 0;
    int bestIndex = -1;
    for (int i = 0; i < mapLines.size(); i++) {
        mapLine = mapLines[i];
        confidence = line.dir == mapLine.dir ? 
            probPartOfLine(line.posToField, mapLine.posToField)
            : 0.0;

        // Boost confidence with other features
        if (mapLine.type == LineType::GoalLine) { 
            confidence += 0.3 * markCntOnFieldLine("TCross", line, 0.2);
            confidence += 0.5 * goalpostCntOnFieldLine(line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "CORNER_KICK")
            ) confidence += 0.3; // During corner kick, the ball is on the goal line
        }
        if (mapLine.type == LineType::MiddleLine) {
            confidence += 0.3 * markCntOnFieldLine("XCross", line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK")
            ) confidence += 0.3; // During goal kick, the ball is on the middle line
        }
        if (mapLine.type == LineType::TouchLine) {
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK" || data->realGameSubState == "CORNER_KICK" || data->realGameSubState == "THROW_IN")
            ) confidence += 0.3; // During goal kick, corner kick, and throw-in, the ball is on the touch line
        }
        
        // Prevent misidentifying goal area line as goal line
        auto fd = config->fieldDimensions;
        if (
            mapLine.type == LineType::GoalLine
            && fabs(line.posToField.y0) < fd.goalAreaWidth / 2 + 0.5
            && fabs(line.posToField.y1) < fd.goalAreaWidth / 2 + 0.5
        ) confidence -= 0.3;

        // Prevent misidentifying penalty area as touchline
        if (
            mapLine.type == LineType::TouchLine
            && min(fabs(line.posToField.x0), fabs(line.posToField.x1)) > fd.length / 2.0 -  fd.penaltyAreaLength - 0.5
            && line.posToField.x0 * line.posToField.x1 > 0
        ) confidence -= 0.3;

        double length = norm(line.posToField.x0 - line.posToField.x1, line.posToField.y0 - line.posToField.y1);
        if (length < 0.5) confidence -= 0.5;
        else if (length < 1.0) confidence -= 0.1;
        
        if (confidence > bestConfidence) {
            secondBestConfidence = bestConfidence;
            bestConfidence = confidence;
            bestIndex = i;
        }
    }

    if (bestConfidence - secondBestConfidence < 0.5) bestConfidence -= 0.5;



    if (bestIndex >= 0 && bestIndex < mapLines.size()) {
        line.type = mapLines[bestIndex].type;
        line.half = mapLines[bestIndex].half;
        line.side = mapLines[bestIndex].side;
        line.confidence = bestConfidence;
        return;
    }

    // else 
    line.type = LineType::NA;
    line.half = LineHalf::NA;
    line.side = LineSide::NA;
    line.confidence = 0.0;
    return;
}

void Brain::identifyMarking(GameObject& marking) {
    double minDist = 100;
    double secMinDist = 100;
    int mmIndex = -1;
    for (int i = 0; i < config->mapMarkings.size(); i++) {
       auto mm = config->mapMarkings[i];
       
       if (mm.type != marking.label) continue;

       double dist = norm(marking.posToField.x - mm.x, marking.posToField.y - mm.y);

       if (dist < minDist) {
           secMinDist = minDist;
           minDist = dist;
           mmIndex = i;
       } else if (dist < secMinDist) {
           secMinDist = dist; 
       }
    }

    auto fd = config->fieldDimensions;
    if (
        mmIndex >=0 && mmIndex < config->mapMarkings.size()
        && minDist < 1.5 * 14 / fd.length // 1.0 for adultsize
        && secMinDist - minDist > 1.5 * 14 / fd.length // 2.0 for adultsize
    ) {
        marking.id = mmIndex;
        marking.name = config->mapMarkings[mmIndex].name;
        marking.idConfidence = 1.0;
    } else {
        marking.id = -1;
        marking.name = "NA";
        marking.idConfidence = 0.0;
    }
}


void Brain::identifyGoalpost(GameObject& goalpost) {
    string side = "NA";
    string half = "NA";
    if (goalpost.posToField.x > 0) half = "O";
    else half = "S";

    if (goalpost.posToField.y > 0) side = "L";
    else side = "R";
    
    goalpost.id = 0;
    goalpost.name = half + side;
    goalpost.idConfidence = 1.0;
}

vector<FieldLine> Brain::processFieldLines(vector<FieldLine>& fieldLines) {
    vector<FieldLine> original = fieldLines;
    vector<FieldLine> res;
    

    int sizeBefore = original.size();
    // merge lines that are actually the same line
    for (int i = 0; i < original.size(); i++) {
        for (int j = i + 1; j < original.size(); j++) {
            auto line1 = original[i].posToField;
            auto line2 = original[j].posToField;
            if (isSameLine(line1, line2, 0.1, 1.0)) {
                FieldLine mergedLine;
                mergedLine.posToField = mergeLines(line1, line2);
                mergedLine.posToRobot = mergeLines(original[i].posToRobot, original[j].posToRobot);
                mergedLine.posOnCam = mergeLines(original[i].posOnCam, original[j].posOnCam);
                mergedLine.timePoint = original[i].timePoint;

                // replace first line in original with merged line and remove second line
                original[i] = mergedLine;
                original.erase(original.begin() + j);
                j--;
            }
        }
    }
    int sizeAfter = original.size();
    // filter out lines that are too short and infer direction while ditch lines whose dir cannot be inferred
    double valve = 0.2;
    for (int i = 0; i < original.size(); i++) {
        auto line = original[i];
        auto lineDir = atan2(line.posToField.y1 - line.posToField.y0, line.posToField.x1 - line.posToField.x0);

        if (fabs(toPInPI(lineDir - M_PI)) < 0.1 || fabs(lineDir) < 0.1) line.dir = LineDir::Vertical;
        else if (fabs(toPInPI(lineDir - M_PI/2)) < 0.1 || fabs(toPInPI(lineDir + M_PI/2)) < 0.1) line.dir = LineDir::Horizontal;
        else continue;

        // if line is direction can be verified, check if it is long enough
        if (lineLength(line.posToField) > valve) {
            res.push_back(line);
        }
    }

    // identify each line 
    for (int i = 0; i < res.size(); i++) {
        identifyFieldLine(res[i]);
    }
    return res;
}


vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    // auto timestamp = detections.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(detections.header);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;
        gObj.color = obj.color;

        if (obj.target_uv.size() == 2)
        { // Precise pixel position information of ground markings
            gObj.precisePixelPoint.x = static_cast<double>(obj.target_uv[0]);
            gObj.precisePixelPoint.y = static_cast<double>(obj.target_uv[1]);
        }

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;

        gObj.projectionToRobot = {0.0, 0.0, 0.0};
        if (obj.position_projection.size() >= 2 &&
            std::isfinite(obj.position_projection[0]) &&
            std::isfinite(obj.position_projection[1])) {
            gObj.projectionToRobot.x = obj.position_projection[0];
            gObj.projectionToRobot.y = obj.position_projection[1];
            if (obj.position_projection.size() >= 3 && std::isfinite(obj.position_projection[2])) {
                gObj.projectionToRobot.z = obj.position_projection[2];
            }
        }

        const bool isBall = obj.label == "Ball";
        const bool hasValidDepthPosition =
            isBall &&
            obj.position_confidence > 0 &&
            obj.position.size() >= 3 &&
            std::isfinite(obj.position[0]) &&
            std::isfinite(obj.position[1]) &&
            std::isfinite(obj.position[2]) &&
            norm(obj.position[0], obj.position[1]) > 0.0001;

        gObj.positionConfidence = isBall
            ? (hasValidDepthPosition ? obj.position_confidence : 0)
            : obj.position_confidence;
        if (hasValidDepthPosition) {
            gObj.posToRobot.x = obj.position[0];
            gObj.posToRobot.y = obj.position[1];
            gObj.posToRobot.z = obj.position[2];
        } else if (isBall) {
            // A visual-only ball may move the head, but it must not supply body
            // coordinates through the RGB ground projection.
            gObj.posToRobot = {0.0, 0.0, 0.0};
        } else {
            gObj.posToRobot = gObj.projectionToRobot;
        }

        // Calculate angles
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->get_robot_height(), gObj.range); // Note: this is an approximate value

        // Calculate the position of the object in the field coordinate system
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z // Note: z is not used elsewhere, here it is just a placeholder
        );

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(
    const vector<GameObject> &ballObjs,
    const rclcpp::Time &receivedAt)
{
    static rclcpp::Time lastSeenRealBallTime;
    const bool wasDepthAcquired =
        data->ballDepthAcquired.load(std::memory_order_acquire);
    double bestConfidence = -1.0;
    bool bestHasDepth = false;
    int indexRealBall = -1;

    for (int i = 0; i < ballObjs.size(); i++)
    {
        const auto &ballObj = ballObjs[i];

        const bool hasDepth = ballObj.positionConfidence > 0;
        const double candidateX = hasDepth
            ? ballObj.posToRobot.x
            : ballObj.projectionToRobot.x;

        // Prevent misidentifying lights in the sky as balls. Visual-only
        // candidates use projection for filtering, never for body motion.
        if (candidateX < -0.5 || candidateX > 15.0)
            continue;

        if (ballObj.confidence < config->get_ball_confidence_threshold())
            continue;

        if ((hasDepth && !bestHasDepth) ||
            (hasDepth == bestHasDepth && ballObj.confidence > bestConfidence))
        {
            bestConfidence = ballObj.confidence;
            bestHasDepth = hasDepth;
            indexRealBall = i;
        }
    }

    const auto now = receivedAt;

    if (indexRealBall >= 0)
    { // Ball detected
        data->ball = ballObjs[indexRealBall];
        data->ball.confidence = bestConfidence;
        data->ballDetected.store(true, std::memory_order_release);
        tree->setEntry<bool>("ball_visible", true);
        if (!wasDepthAcquired && bestHasDepth)
        {
            data->ballDepthAcquired.store(true, std::memory_order_release);
            data->ballTrackingGeneration.fetch_add(1, std::memory_order_relaxed);
            log->log(
                "BallAcquisition",
                format("Depth-confirmed RGB acquisition accepted; generation: %llu",
                       static_cast<unsigned long long>(
                           data->ballTrackingGeneration.load(std::memory_order_relaxed))));
        }

        const bool depthAcquired =
            data->ballDepthAcquired.load(std::memory_order_acquire);
        tree->setEntry<bool>("ball_depth_acquired", depthAcquired);
        tree->setEntry<bool>("ball_location_known", bestHasDepth);
        if (bestHasDepth) {
            updateBallOut();
        }

        lastSeenRealBallTime = now;
        data->lose_ball = false;
    }
    else
    { // No ball detected
        data->ballDetected.store(false, std::memory_order_release);
        data->ballDepthAcquired.store(false, std::memory_order_release);
        tree->setEntry<bool>("ball_visible", false);
        tree->setEntry<bool>("ball_location_known", false);
        tree->setEntry<bool>("ball_depth_acquired", false);
        if (wasDepthAcquired)
        {
            log->log(
                "BallAcquisition",
                "Accepted RGB ball lost; depth-confirmed acquisition latch reset");
        }
        if (lastSeenRealBallTime.seconds() > 0.0)
        {
            double msecs = (now - lastSeenRealBallTime).nanoseconds() / 1e6;
            data->lose_ball = (msecs > 2000.0);
        }
        else
        {
            data->lose_ball = false;
        }
    }

    // Calculate the vector from the robot to the ball in the field coordinate system
    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);

    BallDepthObservation ballObservation;
    ballObservation.visible = indexRealBall >= 0;
    ballObservation.depth_confirmed = indexRealBall >= 0 && bestHasDepth;
    ballObservation.stamp = data->getLastDetectionSensorStamp();
    // Tie the evidence payload to the exact receive marker published at the
    // start of this detection callback so readers can reject half-updated
    // callback state without relying on approximate timestamps.
    ballObservation.received_at = receivedAt;
    ballObservation.image_width = config->cameraImageWidth;
    ballObservation.image_height = config->cameraImageHeight;
    ballObservation.robot_pose_to_odom = data->getRobotPoseToOdom();
    if (ballObservation.depth_confirmed) {
        const auto &observedBall = ballObjs[indexRealBall];
        ballObservation.position_to_robot = observedBall.posToRobot;
        ballObservation.bounding_box = observedBall.boundingBox;
    }
    data->setBallDepthObservation(ballObservation);
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    const double confidenceValve = 50; // Exclude markings with confidence below this threshold
    vector<GameObject> markings = {};
    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        // If the confidence is too low, consider it a false detection
        if (marking.confidence < confidenceValve)
            continue;

        // Exclude markings misidentified in the sky
        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 15.0)
            continue;

        // If the marking passes all checks, record it in the brain
        identifyMarking(marking);
        markings.push_back(marking);
    }
    data->setMarkings(markings);
}

void Brain::detectProcessGoalposts(const vector<GameObject> &goalpostObjs)
{
    const double confidenceValve = 50; // Exclude goalposts with confidence below this threshold
    vector<GameObject> goalposts = {};

    for (int i = 0; i < goalpostObjs.size(); i++) {
        auto goalpost = goalpostObjs[i];

        // If the confidence is too low, consider it a false detection
        if (goalpost.confidence < confidenceValve)
            continue;

        identifyGoalpost(goalpost);
        goalposts.push_back(goalpost);
    }
    data->setGoalposts(goalposts);

}


void Brain::detectProcessRobots(const vector<GameObject> &robotObjs) {

    vector<GameObject> robots = {};
    for (int i = 0; i < robotObjs.size(); i++) {
        auto rbt = robotObjs[i];
        if (rbt.confidence < 50) continue;
        
        // else
        robots.push_back(rbt);
    }

    data->setRobots(robots);
}


void Brain::detectProcessVisionBox(const vision_interface::msg::Detections &msg) {    
    auto timePoint = timePointFromHeader(msg.header);

    // Process and record vision information
    VisionBox vbox;
    vbox.timePoint = timePoint;
    for (int i = 0; i < msg.corner_pos.size(); i++) vbox.posToRobot.push_back(msg.corner_pos[i]);

    // Handle the case where the top-left and top-right points have x < 0, indicating an infinitely distant scene
    const double VISION_LIMIT = 20.0;
    vector<vector<double>> v = {};
    for (int i = 0; i < 4; i++) {
        int start = i; int end = (i + 1) % 4;
        v.push_back({vbox.posToRobot[end * 2] - vbox.posToRobot[start * 2], vbox.posToRobot[end * 2 + 1] - vbox.posToRobot[start * 2 + 1]});
        v.push_back({-vbox.posToRobot[end * 2] + vbox.posToRobot[start * 2], -vbox.posToRobot[end * 2 + 1] + vbox.posToRobot[start * 2 + 1]});
    }

    for (int i = 0; i < 2; i++) {
        double ox = vbox.posToRobot[2* i]; double oy = vbox.posToRobot[2 * i + 1];
        if (
            (i == 0 && crossProduct(v[5], v[6]) < 0)
            || (i == 1 && crossProduct(v[3], v[4]) < 0)
        ){
            vbox.posToRobot[2 * i] = -ox / fabs(ox) * VISION_LIMIT;
            vbox.posToRobot[2 * i + 1] = -oy / fabs(oy) * VISION_LIMIT;
        }
    }

    // transform to field coordinate system
    for (int i = 0; i < 5; i++) {
        double xr, yr, xf, yf, __;
        xr = vbox.posToRobot[2 * i];
        yr = vbox.posToRobot[2 * i + 1];
        transCoord(
            xr, yr, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            xf, yf, __
        );
        vbox.posToField.push_back(xf);
        vbox.posToField.push_back(yf);
    }
    
    data->visionBox = vbox;
}

void Brain::logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid_occupied, vector<std::array<float, 3>> &points_robot) {
    // time is set on the outside
    const double grid_size = config->get_grid_size();  // Grid size
    const double x_min = booster_soccer::perception::obstacleGridMinimumX(
        config->get_max_x());
    const double x_max = config->get_max_x();
    const double y_min = -config->get_max_y();
    const double y_max = -y_min;

    // Publish point cloud to ROS2 topic
    std::vector<std::tuple<float, float, float>> cloud_points;
    cloud_points.reserve(points_robot.size());
    for (const auto &point : points_robot) {
        cloud_points.emplace_back(point[0], point[1], point[2]);
    }
    visualizer->publishPointCloud(cloud_points, "base_link");
    
    // Publish obstacle grid to ROS2 topic
    // ROS OccupancyGrid uses row-major format: index = y * width + x
    // width corresponds to the number of cells in the X direction, height corresponds to the number of cells in the Y direction
    std::vector<int8_t> grid_data(grid_x_count * grid_y_count, -1);  // -1 represents unknown
    for (int j = 0; j < grid_y_count; j++) {        // Y direction (rows)
        for (int i = 0; i < grid_x_count; i++) {    // X direction (columns)
            int index = j * grid_x_count + i;        // row-major: y * width + x
            if (grid_occupied[i][j] > 0) {
                // Map occupancy count to the range 0-100
                int occupancy = std::min(100, static_cast<int>(grid_occupied[i][j] * 10));
                grid_data[index] = static_cast<int8_t>(occupancy);
            } else {
                grid_data[index] = 0;  // Free
            }
        }
    }
    visualizer->publishObstacleGrid(grid_data, grid_x_count, grid_y_count,
                                   grid_size, x_min, y_min, "base_link");

    // Log ball exclusion box
    const double r = config->get_ball_exclusion_radius();
    const auto ballObservation = data->getBallDepthObservation();
    log->debug(
        "depth/ball_exclusion_box",
        format(
            "Ball exclusion box at (%.2f, %.2f) with radius %.2f",
            ballObservation.position_to_robot.x,
            ballObservation.position_to_robot.y,
            r)
    );
}

void Brain::logDebugInfo() {
    auto log_ = [=](string msg) {
        log->debug("brain_tick", msg);
    };
    string gameState = tree->getEntry<string>("gc_game_state");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string isLead = data->tmImLead ? "ON" : "OFF";
    string ballOut = tree->getEntry<bool>("ball_out") ? "YES" : "NO";
    string ballDetected =
        data->ballDetected.load(std::memory_order_acquire) ? "YES" : "NO";
    string decision = tree->getEntry<string>("decision");
    string freeKickKickingOff = data->isFreekickKickingOff ? "YES" : "NO";
    string directShoot = data->isDirectShoot ? "YES" : "NO";
    int myStrikerIDRank = data->myStrikerIDRank;
    log_(format("Game State: %s, SubState: %s, SubStateType: %s, Lead: %s, Decision: %s, FreeKickKickingOff: %s, DirectShoot: %s, PrimaryStriker: %d",
        gameState.c_str(), gameSubState.c_str(), gameSubStateType.c_str(), isLead.c_str(), decision.c_str(), freeKickKickingOff.c_str(), directShoot.c_str(), myStrikerIDRank));

    log->debug("my_cost_scalar", format("cost: %.2f", data->tmMyCost));
    log->debug("my_lead_scalar", format("lead: %d", data->tmImLead));
}

void Brain::updateRelativePos(GameObject &obj) {
    Pose2D pf;
    pf.x = obj.posToField.x;
    pf.y = obj.posToField.y;
    pf.theta = 0;
    Pose2D pr = data->field2robot(pf);
    obj.posToRobot.x = pr.x;
    obj.posToRobot.y = pr.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = asin(config->get_robot_height() / obj.range);
}

void Brain::updateFieldPos(GameObject &obj) {
    Pose2D pr;
    pr.x = obj.posToRobot.x;
    pr.y = obj.posToRobot.y;
    pr.theta = 0;
    Pose2D pf = data->robot2field(pr);
    obj.posToField.x = pf.x;
    obj.posToField.y = pf.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = asin(config->get_robot_height() / obj.range);
}

void Brain::compressedDepthImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    const auto receivedAt = get_clock()->now();
    try {
        // Decode compressed image
        cv::Mat compressed_data = cv::Mat(msg->data);
        cv::Mat depth_decoded = cv::imdecode(compressed_data, cv::IMREAD_ANYDEPTH);
        
        if (depth_decoded.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to decode compressed depth image");
            return;
        }

        // Convert to floating-point format
        cv::Mat depthFloat;
        if (depth_decoded.type() == CV_16UC1) {
            depth_decoded.convertTo(depthFloat, CV_32FC1, 1.0 / 1000.0);
        } else if (depth_decoded.type() == CV_32FC1) {
            depthFloat = depth_decoded;
        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported decoded depth image type: %d", depth_decoded.type());
            return;
        }
        
        // Call the unified depth image processing function
        processDepthImage(
            depthFloat, depth_decoded.cols, depth_decoded.rows, msg->header, receivedAt);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Error in compressedDepthImageCallback: %s", e.what());
    }
}

void Brain::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    const auto receivedAt = get_clock()->now();
    try {
        // Check if the image data is valid
        if (msg->data.empty() || msg->height == 0 || msg->width == 0) {
            RCLCPP_WARN(get_logger(), "Received empty depth image");
            return;
        }

        // Create depth image and conversion
        cv::Mat depthFloat;
        // Process based on image encoding
        if (msg->encoding == "16UC1" || msg->encoding == "mono16") {
            size_t expected = (size_t)msg->width * msg->height * sizeof(uint16_t);
            if (msg->data.size() < expected) {
                RCLCPP_ERROR(get_logger(), "Depth mono16 size mismatch");
                return;
            }
            cv::Mat depthRaw(msg->height, msg->width, CV_16UC1, const_cast<uint8_t*>(msg->data.data()));
            depthRaw.convertTo(depthFloat, CV_32FC1, 1.0 / 1000.0); // If actual depth unit is mm
        } else if (msg->encoding == "32FC1") {
            // Check if data size is correct
            size_t expected_size = msg->height * msg->width * sizeof(float);
            if (msg->data.size() != expected_size) {
                RCLCPP_ERROR(get_logger(), "Depth image size mismatch: expected %zu, got %zu", 
                    expected_size, msg->data.size());
                return;
            }

            // Directly create 32-bit floating-point depth image
            depthFloat = cv::Mat(msg->height, msg->width, CV_32FC1, 
                const_cast<float*>(reinterpret_cast<const float*>(msg->data.data()))).clone();
            
        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported depth image encoding: %s", msg->encoding.c_str());
            return;
        }

        // Call the unified depth image processing function
        processDepthImage(depthFloat, msg->width, msg->height, msg->header, receivedAt);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in depth image callback: %s", e.what());
    }
}

void Brain::processDepthImage(
    const cv::Mat &depthFloat,
    int width,
    int height,
    const std_msgs::msg::Header &header,
    const rclcpp::Time &receivedAt)
{
    // Raw and compressed depth subscriptions may coexist. Build/publish one
    // snapshot at a time so component IDs, memory, and coverage history form a
    // single ordered stream.
    std::lock_guard<std::mutex> processingLock(depthProcessingMutex_);
    try {
        if (depthFloat.empty() || depthFloat.type() != CV_32FC1 ||
            width <= 0 || height <= 0 ||
            depthFloat.cols < width || depthFloat.rows < height) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid decoded depth frame");
            return;
        }

        double fx = 0.0;
        double fy = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        {
            std::lock_guard<std::mutex> cameraInfoLock(depthCameraInfoMutex_);
            fx = config->depthCameraFx;
            fy = config->depthCameraFy;
            cx = config->depthCameraCx;
            cy = config->depthCameraCy;
        }
        if (!std::isfinite(fx) || !std::isfinite(fy) ||
            !std::isfinite(cx) || !std::isfinite(cy) ||
            fx <= 0.0 || fy <= 0.0) {
            RCLCPP_WARN(get_logger(), "Ignoring depth frame until camera intrinsics are valid");
            return;
        }

        Eigen::Matrix4d camToRobot;
        if (!selectDepthHeadPose(receivedAt, camToRobot)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Ignoring depth frame: no head pose within %.0f ms",
                config->get_head_pose_tolerance_msecs());
            return;
        }
        if (!camToRobot.allFinite()) {
            RCLCPP_WARN(get_logger(), "Ignoring depth frame with invalid camera transform");
            return;
        }
        const Pose2D depthRobotPose = data->getRobotPoseToOdom();
        if (!std::isfinite(depthRobotPose.x) || !std::isfinite(depthRobotPose.y) ||
            !std::isfinite(depthRobotPose.theta)) {
            RCLCPP_WARN(get_logger(), "Ignoring depth frame with invalid odometry pose");
            return;
        }

        vector<std::array<float, 3>> points_robot;
        const double grid_size = config->get_grid_size();
        // Keep the full observable rear-side horizon. A moving person outside
        // the current footprint can enter a lateral route within the planner's
        // one-second prediction sweep.
        const double x_min = booster_soccer::perception::obstacleGridMinimumX(
            config->get_max_x());
        const double x_max = config->get_max_x();
        const double y_min = -config->get_max_y();
        const double y_max = -y_min;
        if (!std::isfinite(grid_size) || grid_size <= 0.0 ||
            x_max <= x_min || y_max <= y_min) {
            RCLCPP_ERROR(get_logger(), "Invalid obstacle grid configuration");
            return;
        }
        const int grid_x_count = static_cast<int>(std::ceil((x_max - x_min) / grid_size));
        const int grid_y_count = static_cast<int>(std::ceil((y_max - y_min) / grid_size));
        vector<vector<int>> grid_occupied(grid_x_count, vector<int>(grid_y_count, 0));
        vector<vector<int>> grid_ball_candidates(
            grid_x_count, vector<int>(grid_y_count, 0));

        const int sampleStep = std::max(1, config->get_depth_sample_step());
        const int sampledColumnCount =
            (width + sampleStep - 1) / sampleStep;
        const double angularResolution = std::max(
            deg2rad(1.0), deg2rad(config->get_obstacle_angular_resolution_degrees()));
        const int angularBinCount = std::max(
            1, static_cast<int>(std::ceil(2.0 * M_PI / angularResolution)));
        const int minimumAngularObservations = std::max(
            3,
            static_cast<int>(std::ceil(config->get_occupancy_threshold())));
        vector<int> angularObservations(angularBinCount, 0);
        vector<vector<bool>> angularObservedColumns(
            angularBinCount,
            vector<bool>(sampledColumnCount, false));
        vector<double> angularFreeRange(angularBinCount, 0.0);
        auto angleToBin = [&](double angle) {
            const double normalized = toPInPI(angle);
            int bin = static_cast<int>(std::floor((normalized + M_PI) / (2.0 * M_PI) * angularBinCount));
            return std::clamp(bin, 0, angularBinCount - 1);
        };
        auto binToAngle = [&](int bin) {
            return -M_PI + (static_cast<double>(bin) + 0.5) *
                (2.0 * M_PI / static_cast<double>(angularBinCount));
        };

        const BallDepthObservation ballObservation = data->getBallDepthObservation();
        // Use the same fail-closed receive/sensor timestamp policy as the
        // planner. A frozen, future, or corrupt detector timestamp must never
        // authorize removal of physical depth samples.
        const double ballAgeMsecs = data->detectionAgeMsecs(receivedAt);
        const bool bboxValid =
            ballObservation.bounding_box.xmax > ballObservation.bounding_box.xmin &&
            ballObservation.bounding_box.ymax > ballObservation.bounding_box.ymin;
        const bool useBallMask =
            ballObservation.visible && ballObservation.depth_confirmed && bboxValid &&
            ballAgeMsecs >= 0.0 &&
            ballAgeMsecs <= config->get_detection_stale_msecs() &&
            std::hypot(
                ballObservation.position_to_robot.x,
                ballObservation.position_to_robot.y) > 0.01;
        const double bboxScaleX = ballObservation.image_width > 0
            ? static_cast<double>(width) / ballObservation.image_width
            : 1.0;
        const double bboxScaleY = ballObservation.image_height > 0
            ? static_cast<double>(height) / ballObservation.image_height
            : 1.0;
        const double ballBoxXMin = ballObservation.bounding_box.xmin * bboxScaleX - 2.0;
        const double ballBoxXMax = ballObservation.bounding_box.xmax * bboxScaleX + 2.0;
        const double ballBoxYMin = ballObservation.bounding_box.ymin * bboxScaleY - 2.0;
        const double ballBoxYMax = ballObservation.bounding_box.ymax * bboxScaleY + 2.0;

        const int totalSampleSlots =
            ((width + sampleStep - 1) / sampleStep) *
            ((height + sampleStep - 1) / sampleStep);
        int validDepthSamples = 0;
        const double minObstacleHeight = config->get_obstacle_min_height();
        const double maxObstacleHeight = config->get_obstacle_max_height();
        const double excludeX = config->get_exclusion_x();
        const double excludeY = config->get_exclusion_y();
        const double ballRadius = config->get_ball_exclusion_radius();
        const double ballHeight = config->get_ball_exclusion_height();
        const double cameraOriginX = camToRobot(0, 3);
        const double cameraOriginY = camToRobot(1, 3);
        if (!std::isfinite(cameraOriginX) || !std::isfinite(cameraOriginY)) {
            RCLCPP_WARN(get_logger(), "Ignoring depth frame with invalid camera origin");
            return;
        }

        for (int y = 0; y < height; y += sampleStep)
        {
            for (int x = 0; x < width; x += sampleStep)
            {
                const float depth = depthFloat.at<float>(y, x);
                if (!std::isfinite(depth) || depth <= 0.05f || depth > 20.0f) {
                    continue;
                }

                const Eigen::Vector4d pointCam(
                    (x - cx) * depth / fx,
                    (y - cy) * depth / fy,
                    depth,
                    1.0);
                const Eigen::Vector4d pointRobot = camToRobot * pointCam;
                if (!pointRobot.allFinite()) {
                    continue;
                }
                validDepthSamples++;
                points_robot.push_back({
                    static_cast<float>(pointRobot(0)),
                    static_cast<float>(pointRobot(1)),
                    static_cast<float>(pointRobot(2))});

                double rayBearing = 0.0;
                double rayRange = 0.0;
                if (booster_soccer::perception::planarRayFromCameraOrigin(
                        pointRobot(0), pointRobot(1),
                        cameraOriginX, cameraOriginY,
                        rayBearing, rayRange)) {
                    const int angularBin = angleToBin(rayBearing);
                    // Several valid pixels down one image column are still a
                    // single narrow ray, not evidence for a complete angular
                    // wedge. Require distinct sampled columns before a bin can
                    // authorize motion through otherwise unknown space.
                    const int sampledColumn = std::clamp(
                        x / sampleStep, 0, sampledColumnCount - 1);
                    if (!angularObservedColumns[angularBin][sampledColumn]) {
                        angularObservedColumns[angularBin][sampledColumn] = true;
                        angularObservations[angularBin]++;
                    }
                    angularFreeRange[angularBin] = std::max(
                        angularFreeRange[angularBin], std::min(rayRange, x_max));
                }

                const bool inGrid = booster_soccer::perception::obstacleGridContains(
                    pointRobot(0), pointRobot(1),
                    x_min, x_max, y_min, y_max);
                const bool isSelfBody =
                    pointRobot(0) >= -excludeX && pointRobot(0) <= excludeX &&
                    pointRobot(1) >= -excludeY && pointRobot(1) <= excludeY;
                const bool isPhysicalBallCandidate =
                    booster_soccer::perception::physicalBallSample(
                        useBallMask,
                        x, y,
                        ballBoxXMin, ballBoxXMax,
                        ballBoxYMin, ballBoxYMax,
                        pointRobot(0), pointRobot(1), pointRobot(2),
                        ballObservation.position_to_robot.x,
                        ballObservation.position_to_robot.y,
                        ballRadius, ballHeight);

                if (booster_soccer::perception::obstacleHeightAccepted(
                        pointRobot(2), minObstacleHeight, maxObstacleHeight) &&
                    inGrid && !isSelfBody) {
                    const int gridX = static_cast<int>((pointRobot(0) - x_min) / grid_size);
                    const int gridY = static_cast<int>((pointRobot(1) - y_min) / grid_size);
                    if (gridX >= 0 && gridX < grid_x_count &&
                        gridY >= 0 && gridY < grid_y_count) {
                        grid_occupied[gridX][gridY]++;
                        if (isPhysicalBallCandidate) {
                            grid_ball_candidates[gridX][gridY]++;
                        }
                    }
                }
            }
        }

        // A frame with almost no valid measurements is not evidence that the
        // world is clear. Preserve the preceding snapshot and let it age out.
        if (!booster_soccer::perception::depthFrameHasEnoughSamples(
                validDepthSamples, totalSampleSlots)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Ignoring sparse depth frame (%d/%d valid samples)",
                validDepthSamples, totalSampleSlots);
            return;
        }

        struct GridCell { int x; int y; };
        vector<vector<bool>> visited(grid_x_count, vector<bool>(grid_y_count, false));
        vector<vector<int>> filteredGrid(grid_x_count, vector<int>(grid_y_count, 0));
        vector<GameObject> obstacles;
        vector<ObstacleComponent> components;
        vector<int> matchedOldComponentIds;
        const auto previousSnapshot = data->getObstacleSnapshot();
        const vector<GameObject> semanticObstacles =
            data->detectionAgeMsecs(receivedAt) <= config->get_detection_stale_msecs()
                ? data->getSemanticObstacles()
                : vector<GameObject>{};
        const int strongCellThreshold = std::max(
            1, static_cast<int>(std::ceil(config->get_occupancy_threshold())));

        // A bounding-box match is only a candidate.  Confirm one complete,
        // connected, ball-sized low component before subtracting its samples
        // from occupancy. Other low clusters in the same box/radius (for
        // example a leg or box behind the ball) remain physical obstacles.
        if (useBallMask) {
            vector<vector<bool>> candidateVisited(
                grid_x_count, vector<bool>(grid_y_count, false));
            vector<GridCell> bestBallCells;
            double bestBallScore = std::numeric_limits<double>::infinity();

            for (int startX = 0; startX < grid_x_count; ++startX) {
                for (int startY = 0; startY < grid_y_count; ++startY) {
                    if (candidateVisited[startX][startY] ||
                        grid_ball_candidates[startX][startY] <= 0) {
                        continue;
                    }

                    vector<GridCell> pending{{startX, startY}};
                    vector<GridCell> candidateCells;
                    candidateVisited[startX][startY] = true;
                    int candidateSamples = 0;
                    int strongestCellSamples = 0;
                    double weightedX = 0.0;
                    double weightedY = 0.0;
                    double candidateMinX = std::numeric_limits<double>::infinity();
                    double candidateMaxX = -std::numeric_limits<double>::infinity();
                    double candidateMinY = std::numeric_limits<double>::infinity();
                    double candidateMaxY = -std::numeric_limits<double>::infinity();

                    while (!pending.empty()) {
                        const GridCell cell = pending.back();
                        pending.pop_back();
                        candidateCells.push_back(cell);
                        const int count = grid_ball_candidates[cell.x][cell.y];
                        candidateSamples += count;
                        strongestCellSamples = std::max(strongestCellSamples, count);
                        const double cellX = x_min + (cell.x + 0.5) * grid_size;
                        const double cellY = y_min + (cell.y + 0.5) * grid_size;
                        weightedX += cellX * count;
                        weightedY += cellY * count;
                        candidateMinX = std::min(candidateMinX, cellX - 0.5 * grid_size);
                        candidateMaxX = std::max(candidateMaxX, cellX + 0.5 * grid_size);
                        candidateMinY = std::min(candidateMinY, cellY - 0.5 * grid_size);
                        candidateMaxY = std::max(candidateMaxY, cellY + 0.5 * grid_size);

                        for (int dx = -1; dx <= 1; ++dx) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                const int nx = cell.x + dx;
                                const int ny = cell.y + dy;
                                if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 ||
                                    nx >= grid_x_count || ny >= grid_y_count ||
                                    candidateVisited[nx][ny] ||
                                    grid_ball_candidates[nx][ny] <= 0) {
                                    continue;
                                }
                                candidateVisited[nx][ny] = true;
                                pending.push_back(GridCell{nx, ny});
                            }
                        }
                    }

                    if (candidateSamples <= 0) continue;
                    const double candidateX = weightedX / candidateSamples;
                    const double candidateY = weightedY / candidateSamples;
                    double candidateRadius = 0.0;
                    for (const double cornerX : {candidateMinX, candidateMaxX}) {
                        for (const double cornerY : {candidateMinY, candidateMaxY}) {
                            candidateRadius = std::max(
                                candidateRadius,
                                std::hypot(cornerX - candidateX, cornerY - candidateY));
                        }
                    }
                    const double centerError = std::hypot(
                        candidateX - ballObservation.position_to_robot.x,
                        candidateY - ballObservation.position_to_robot.y);
                    if (!booster_soccer::perception::physicalBallComponent(
                            true,
                            candidateCells.size(),
                            strongestCellSamples,
                            strongCellThreshold,
                            candidateRadius,
                            centerError,
                            ballRadius,
                            grid_size / std::sqrt(2.0))) {
                        continue;
                    }

                    const double score = centerError + 0.1 * candidateRadius;
                    if (score < bestBallScore) {
                        bestBallScore = score;
                        bestBallCells = std::move(candidateCells);
                    }
                }
            }

            for (const GridCell &cell : bestBallCells) {
                grid_occupied[cell.x][cell.y] = std::max(
                    0,
                    grid_occupied[cell.x][cell.y] -
                        grid_ball_candidates[cell.x][cell.y]);
            }
        }

        for (int startX = 0; startX < grid_x_count; ++startX) {
            for (int startY = 0; startY < grid_y_count; ++startY) {
                if (visited[startX][startY] || grid_occupied[startX][startY] <= 0) {
                    continue;
                }

                vector<GridCell> pending{{startX, startY}};
                vector<GridCell> cells;
                visited[startX][startY] = true;
                int sampleCount = 0;
                int maxCellCount = 0;
                while (!pending.empty()) {
                    const auto cell = pending.back();
                    pending.pop_back();
                    cells.push_back(cell);
                    const int count = grid_occupied[cell.x][cell.y];
                    sampleCount += count;
                    maxCellCount = std::max(maxCellCount, count);
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            const int nx = cell.x + dx;
                            const int ny = cell.y + dy;
                            if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 ||
                                nx >= grid_x_count || ny >= grid_y_count ||
                                visited[nx][ny] || grid_occupied[nx][ny] <= 0) {
                                continue;
                            }
                            visited[nx][ny] = true;
                            pending.push_back(GridCell{nx, ny});
                        }
                    }
                }

                // Preserve thin structures represented by adjacent cells, but
                // demand stronger evidence for an isolated cell.
                if (!booster_soccer::perception::componentEvidenceAccepted(
                        cells.size(), maxCellCount, strongCellThreshold)) {
                    continue;
                }

                ObstacleComponent component;
                component.id = nextObstacleComponentId_++;
                component.min_x = std::numeric_limits<double>::infinity();
                component.max_x = -std::numeric_limits<double>::infinity();
                component.min_y = std::numeric_limits<double>::infinity();
                component.max_y = -std::numeric_limits<double>::infinity();
                component.nearest_distance = std::numeric_limits<double>::infinity();
                component.occupied_cell_count = static_cast<int>(cells.size());
                component.sample_count = sampleCount;
                component.confidence = sampleCount;
                component.last_seen_at = receivedAt;
                component.carried = false;
                double weightedX = 0.0;
                double weightedY = 0.0;

                for (const auto &cell : cells) {
                    const int count = grid_occupied[cell.x][cell.y];
                    filteredGrid[cell.x][cell.y] = count;
                    const double cellX = x_min + (cell.x + 0.5) * grid_size;
                    const double cellY = y_min + (cell.y + 0.5) * grid_size;
                    weightedX += cellX * count;
                    weightedY += cellY * count;
                    component.min_x = std::min(component.min_x, cellX - 0.5 * grid_size);
                    component.max_x = std::max(component.max_x, cellX + 0.5 * grid_size);
                    component.min_y = std::min(component.min_y, cellY - 0.5 * grid_size);
                    component.max_y = std::max(component.max_y, cellY + 0.5 * grid_size);
                    component.nearest_distance = std::min(
                        component.nearest_distance,
                        std::max(0.0, std::hypot(cellX, cellY) - grid_size / std::sqrt(2.0)));

                    GameObject obstacle{};
                    obstacle.label = "Obstacle";
                    obstacle.timePoint = receivedAt;
                    obstacle.posToRobot.x = cellX;
                    obstacle.posToRobot.y = cellY;
                    obstacle.posToRobot.z = 0.0;
                    obstacle.confidence = count;
                    updateFieldPos(obstacle);
                    obstacles.push_back(obstacle);
                }

                component.x = weightedX / std::max(1, sampleCount);
                component.y = weightedY / std::max(1, sampleCount);
                component.bearing = atan2(component.y, component.x);
                component.radius = 0.0;
                for (const double cornerX : {component.min_x, component.max_x}) {
                    for (const double cornerY : {component.min_y, component.max_y}) {
                        component.radius = std::max(
                            component.radius,
                            std::hypot(cornerX - component.x, cornerY - component.y));
                    }
                }

                const GameObject *nearestSemantic = nullptr;
                double nearestSemanticDistance = std::numeric_limits<double>::infinity();
                for (const auto &semantic : semanticObstacles) {
                    if (semantic.label != "Person" && semantic.label != "Opponent") continue;
                    if (!std::isfinite(semantic.posToRobot.x) ||
                        !std::isfinite(semantic.posToRobot.y) ||
                        std::hypot(semantic.posToRobot.x, semantic.posToRobot.y) <= 0.05) continue;
                    const double associationDistance = std::hypot(
                        component.x - semantic.posToRobot.x,
                        component.y - semantic.posToRobot.y);
                    if (associationDistance < nearestSemanticDistance &&
                        associationDistance <= std::max(0.45, component.radius + 0.25)) {
                        nearestSemantic = &semantic;
                        nearestSemanticDistance = associationDistance;
                    }
                }
                if (nearestSemantic != nullptr) component.label = nearestSemantic->label;

                if (previousSnapshot.ready &&
                    previousSnapshot.received_at.get_clock_type() == receivedAt.get_clock_type()) {
                    const double dt = static_cast<double>(
                        (receivedAt - previousSnapshot.received_at).nanoseconds()) / 1e9;
                    if (dt > 0.005 && dt <= 0.5) {
                        const ObstacleComponent *nearest = nullptr;
                        double nearestDistance = std::numeric_limits<double>::infinity();
                        double nearestProjectedX = 0.0;
                        double nearestProjectedY = 0.0;
                        for (const auto &oldComponent : previousSnapshot.components) {
                            if (std::find(
                                    matchedOldComponentIds.begin(),
                                    matchedOldComponentIds.end(),
                                    oldComponent.id) != matchedOldComponentIds.end()) {
                                continue;
                            }
                            const double oldTheta = previousSnapshot.robot_pose_to_odom.theta;
                            const double currentTheta = depthRobotPose.theta;
                            const double worldX = previousSnapshot.robot_pose_to_odom.x +
                                cos(oldTheta) * oldComponent.x - sin(oldTheta) * oldComponent.y;
                            const double worldY = previousSnapshot.robot_pose_to_odom.y +
                                sin(oldTheta) * oldComponent.x + cos(oldTheta) * oldComponent.y;
                            const double deltaWorldX = worldX - depthRobotPose.x;
                            const double deltaWorldY = worldY - depthRobotPose.y;
                            const double projectedX =
                                cos(currentTheta) * deltaWorldX + sin(currentTheta) * deltaWorldY;
                            const double projectedY =
                                -sin(currentTheta) * deltaWorldX + cos(currentTheta) * deltaWorldY;
                            const double frameRotation = oldTheta - currentTheta;
                            const double oldVxInCurrent =
                                cos(frameRotation) * oldComponent.vx -
                                sin(frameRotation) * oldComponent.vy;
                            const double oldVyInCurrent =
                                sin(frameRotation) * oldComponent.vx +
                                cos(frameRotation) * oldComponent.vy;
                            const double distance = std::hypot(
                                component.x - (projectedX + oldVxInCurrent * dt),
                                component.y - (projectedY + oldVyInCurrent * dt));
                            if (distance < nearestDistance) {
                                nearestDistance = distance;
                                nearest = &oldComponent;
                                nearestProjectedX = projectedX;
                                nearestProjectedY = projectedY;
                            }
                        }
                        if (nearest != nullptr &&
                            nearestDistance <= std::max(0.75, component.radius + nearest->radius + 0.30)) {
                            component.id = nearest->id;
                            matchedOldComponentIds.push_back(nearest->id);
                            const double frameRotation =
                                previousSnapshot.robot_pose_to_odom.theta - depthRobotPose.theta;
                            const double oldVxInCurrent =
                                cos(frameRotation) * nearest->vx - sin(frameRotation) * nearest->vy;
                            const double oldVyInCurrent =
                                sin(frameRotation) * nearest->vx + cos(frameRotation) * nearest->vy;
                            component.vx = 0.5 * oldVxInCurrent +
                                0.5 * (component.x - nearestProjectedX) / dt;
                            component.vy = 0.5 * oldVyInCurrent +
                                0.5 * (component.y - nearestProjectedY) / dt;
                            const double speed = std::hypot(component.vx, component.vy);
                            if (speed > 3.0) {
                                component.vx *= 3.0 / speed;
                                component.vy *= 3.0 / speed;
                            }
                        }
                    }
                }
                components.push_back(component);
            }
        }

        // Keep briefly occluded or intermittently missed obstacles alive across
        // otherwise valid frames. Positions/bounds are transformed through odom
        // and advanced using the component's measured velocity; last_seen_at is
        // deliberately not refreshed until depth observes the component again.
        if (previousSnapshot.ready &&
            previousSnapshot.received_at.get_clock_type() == receivedAt.get_clock_type()) {
            const double frameDt = static_cast<double>(
                (receivedAt - previousSnapshot.received_at).nanoseconds()) / 1e9;
            const double memoryMsecs = std::max(0.0, config->get_obstacle_memory_msecs());
            const double oldTheta = previousSnapshot.robot_pose_to_odom.theta;
            const double currentTheta = depthRobotPose.theta;
            const double frameRotation = oldTheta - currentTheta;

            auto previousPointToCurrent = [&](double oldX, double oldY) {
                const double worldX = previousSnapshot.robot_pose_to_odom.x +
                    cos(oldTheta) * oldX - sin(oldTheta) * oldY;
                const double worldY = previousSnapshot.robot_pose_to_odom.y +
                    sin(oldTheta) * oldX + cos(oldTheta) * oldY;
                const double deltaWorldX = worldX - depthRobotPose.x;
                const double deltaWorldY = worldY - depthRobotPose.y;
                return Point2D{
                    cos(currentTheta) * deltaWorldX + sin(currentTheta) * deltaWorldY,
                    -sin(currentTheta) * deltaWorldX + cos(currentTheta) * deltaWorldY};
            };

            for (const auto &oldComponent : previousSnapshot.components) {
                if (oldComponent.label == "Ball" ||
                    std::find(
                        matchedOldComponentIds.begin(),
                        matchedOldComponentIds.end(),
                        oldComponent.id) != matchedOldComponentIds.end()) {
                    continue;
                }

                const rclcpp::Time lastSeenAt = oldComponent.last_seen_at.nanoseconds() > 0
                    ? oldComponent.last_seen_at
                    : previousSnapshot.received_at;
                if (lastSeenAt.get_clock_type() != receivedAt.get_clock_type()) {
                    continue;
                }
                const double unseenMsecs = static_cast<double>(
                    (receivedAt - lastSeenAt).nanoseconds()) / 1e6;
                if (frameDt < 0.0 || unseenMsecs < 0.0 || unseenMsecs > memoryMsecs) {
                    continue;
                }

                const double carriedVx =
                    cos(frameRotation) * oldComponent.vx -
                    sin(frameRotation) * oldComponent.vy;
                const double carriedVy =
                    sin(frameRotation) * oldComponent.vx +
                    cos(frameRotation) * oldComponent.vy;
                const Point2D staticCenter =
                    previousPointToCurrent(oldComponent.x, oldComponent.y);

                ObstacleComponent carried = oldComponent;
                carried.x = staticCenter.x + carriedVx * frameDt;
                carried.y = staticCenter.y + carriedVy * frameDt;
                carried.vx = carriedVx;
                carried.vy = carriedVy;
                carried.last_seen_at = lastSeenAt;
                carried.carried = true;

                carried.min_x = std::numeric_limits<double>::infinity();
                carried.max_x = -std::numeric_limits<double>::infinity();
                carried.min_y = std::numeric_limits<double>::infinity();
                carried.max_y = -std::numeric_limits<double>::infinity();
                for (const double cornerX : {oldComponent.min_x, oldComponent.max_x}) {
                    for (const double cornerY : {oldComponent.min_y, oldComponent.max_y}) {
                        const Point2D transformed = previousPointToCurrent(cornerX, cornerY);
                        const double predictedX = transformed.x + carriedVx * frameDt;
                        const double predictedY = transformed.y + carriedVy * frameDt;
                        carried.min_x = std::min(carried.min_x, predictedX);
                        carried.max_x = std::max(carried.max_x, predictedX);
                        carried.min_y = std::min(carried.min_y, predictedY);
                        carried.max_y = std::max(carried.max_y, predictedY);
                    }
                }
                carried.radius = 0.0;
                for (const double cornerX : {carried.min_x, carried.max_x}) {
                    for (const double cornerY : {carried.min_y, carried.max_y}) {
                        carried.radius = std::max(
                            carried.radius,
                            std::hypot(cornerX - carried.x, cornerY - carried.y));
                    }
                }
                carried.bearing = atan2(carried.y, carried.x);
                carried.nearest_distance = std::max(
                    0.0, std::hypot(carried.x, carried.y) - carried.radius);

                if (!std::isfinite(carried.x) || !std::isfinite(carried.y) ||
                    !std::isfinite(carried.radius) || carried.radius < 0.0) {
                    continue;
                }

                components.push_back(carried);
                matchedOldComponentIds.push_back(carried.id);

                // Keep legacy distance queries conservative without drawing
                // carried memory into the current-frame occupancy grid.
                GameObject carriedObstacle{};
                carriedObstacle.label = "Obstacle";
                carriedObstacle.timePoint = lastSeenAt;
                carriedObstacle.posToRobot.x = carried.x;
                carriedObstacle.posToRobot.y = carried.y;
                carriedObstacle.posToRobot.z = 0.0;
                carriedObstacle.confidence = std::max(
                    carried.confidence, config->get_occupancy_threshold());
                updateFieldPos(carriedObstacle);
                obstacles.push_back(carriedObstacle);
            }
        }

        // Convert valid directional measurements to an explicit coverage map.
        // Components may shorten only bins observed by current-frame depth
        // rays. Neither a current component's geometric extent nor carried
        // memory is evidence that an otherwise-unobserved direction is clear.
        for (const auto &component : components) {
            const double centerRange = std::hypot(component.x, component.y);
            const double halfAngle = centerRange > component.radius
                ? asin(std::clamp(component.radius / centerRange, 0.0, 1.0))
                : M_PI / 2.0;
            for (int bin = 0; bin < angularBinCount; ++bin) {
                if (booster_soccer::perception::angularBinObserved(
                        angularObservations[bin], minimumAngularObservations) &&
                    fabs(toPInPI(binToAngle(bin) - component.bearing)) <= halfAngle) {
                    const double obstacleFreeRange = std::max(0.0, component.nearest_distance);
                    angularFreeRange[bin] = angularFreeRange[bin] > 0.0
                        ? std::min(angularFreeRange[bin], obstacleFreeRange)
                        : obstacleFreeRange;
                }
            }
        }

        ObstacleSnapshot snapshot;
        snapshot.ready = true;
        snapshot.stamp = timePointFromHeader(header);
        snapshot.received_at = receivedAt;
        snapshot.robot_pose_to_odom = depthRobotPose;
        snapshot.obstacles = obstacles;
        snapshot.components = components;
        snapshot.coverage.reserve(
            angularBinCount +
            (previousSnapshot.ready ? previousSnapshot.coverage.size() : 0));
        for (int bin = 0; bin < angularBinCount; ++bin) {
            snapshot.coverage.push_back(ObstacleAngularCoverage{
                binToAngle(bin),
                booster_soccer::perception::angularBinObserved(
                    angularObservations[bin], minimumAngularObservations),
                booster_soccer::perception::angularBinObserved(
                    angularObservations[bin], minimumAngularObservations)
                    ? angularFreeRange[bin]
                    : 0.0,
                cameraOriginX,
                cameraOriginY,
                receivedAt});
        }

        // Retain a short, odometry-anchored union of rays gathered while the
        // head looks from the ball toward a candidate gap. Each ray keeps its
        // own origin, so later planning does not pretend it came from the most
        // recent camera pose. Moving components remain separately predicted.
        if (previousSnapshot.ready &&
            previousSnapshot.received_at.get_clock_type() == receivedAt.get_clock_type() &&
            std::isfinite(previousSnapshot.robot_pose_to_odom.x) &&
            std::isfinite(previousSnapshot.robot_pose_to_odom.y) &&
            std::isfinite(previousSnapshot.robot_pose_to_odom.theta) &&
            std::isfinite(snapshot.robot_pose_to_odom.x) &&
            std::isfinite(snapshot.robot_pose_to_odom.y) &&
            std::isfinite(snapshot.robot_pose_to_odom.theta)) {
            const double oldTheta = previousSnapshot.robot_pose_to_odom.theta;
            const double currentTheta = snapshot.robot_pose_to_odom.theta;
            const double memoryMsecs = std::max(
                0.0, config->get_obstacle_memory_msecs());
            for (const auto &oldCoverage : previousSnapshot.coverage) {
                if (!oldCoverage.observed ||
                    !std::isfinite(oldCoverage.free_range) ||
                    oldCoverage.free_range <= 0.0) {
                    continue;
                }
                const rclcpp::Time observedAt =
                    oldCoverage.observed_at.nanoseconds() > 0
                        ? oldCoverage.observed_at
                        : previousSnapshot.received_at;
                if (observedAt.get_clock_type() != receivedAt.get_clock_type()) {
                    continue;
                }
                const double ageMsecs = static_cast<double>(
                    (receivedAt - observedAt).nanoseconds()) / 1e6;
                if (ageMsecs < 0.0 || ageMsecs > memoryMsecs) continue;

                const double originOdomX = previousSnapshot.robot_pose_to_odom.x +
                    cos(oldTheta) * oldCoverage.origin_x -
                    sin(oldTheta) * oldCoverage.origin_y;
                const double originOdomY = previousSnapshot.robot_pose_to_odom.y +
                    sin(oldTheta) * oldCoverage.origin_x +
                    cos(oldTheta) * oldCoverage.origin_y;
                const double deltaOriginX =
                    originOdomX - snapshot.robot_pose_to_odom.x;
                const double deltaOriginY =
                    originOdomY - snapshot.robot_pose_to_odom.y;
                const double originCurrentX =
                    cos(currentTheta) * deltaOriginX +
                    sin(currentTheta) * deltaOriginY;
                const double originCurrentY =
                    -sin(currentTheta) * deltaOriginX +
                    cos(currentTheta) * deltaOriginY;
                snapshot.coverage.push_back(ObstacleAngularCoverage{
                    toPInPI(oldCoverage.angle + oldTheta - currentTheta),
                    true,
                    oldCoverage.free_range,
                    originCurrentX,
                    originCurrentY,
                    observedAt});
            }
        }

        // Publish the complete snapshot atomically only after every validation
        // and transformation step succeeds. Invalid frames leave the old map.
        data->setObstacleSnapshot(snapshot);
        data->setObstacles(obstacles);
        data->markDepthFrameReceived(snapshot.stamp, receivedAt);
        logDepth(grid_x_count, grid_y_count, filteredGrid, points_robot);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in depth image processing: %s", e.what());
    }
}

double Brain::distToObstacle(double angle) {
    const auto snapshot = data->getObstacleSnapshot();
    const auto obs = snapshot.ready ? snapshot.obstacles : data->getObstacles();
    double minDist = 1e9;
    double obstacleThreshold = config->get_occupancy_threshold();
    double collisionThreshold = config->get_collision_threshold();

    for (int i = 0; i < obs.size(); i++) {
        // Semantic balls may be inserted into the legacy obstacle list during
        // READY/free-kick handling. A ball is never a collision obstacle.
        if (obs[i].label == "Ball") continue;
        if (!snapshot.ready && obs[i].confidence < obstacleThreshold) continue;

        auto o = obs[i];
        Line line = {
            0, 0,
            cos(angle) * 100, sin(angle) * 100
        };
        double perpDist = fabs(pointPerpDistToLine(Point2D{o.posToRobot.x, o.posToRobot.y}, line));
        if (perpDist < collisionThreshold) {
            double dist = innerProduct(vector<double>{o.posToRobot.x, o.posToRobot.y}, vector<double>{cos(angle), sin(angle)});
            if (dist > 0 && dist < minDist) {
                minDist = dist;
            }
        }
    }
    return minDist;
}

vector<double> Brain::findSafeDirections(double startAngle, double safeDist, double step) {
    double safeAngleLeft = startAngle;
    double safeAngleRight = startAngle;
    double leftFound = 0;
    double rightFound = 0;
    for (double angle = startAngle; angle < startAngle + M_PI; angle += step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleLeft = angle;
            leftFound = 1;
            break;
        }
    }
    for (double angle = startAngle; angle > startAngle - M_PI; angle -= step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleRight = angle;
            rightFound = 1;
            break;
        }
    }

    return vector<double>{leftFound, toPInPI(safeAngleLeft), rightFound, toPInPI(safeAngleRight)};
}

double Brain::calcAvoidDir(double startAngle, double safeDist) {
    auto res = findSafeDirections(startAngle, safeDist);
    bool leftFound = res[0] > 0.5;
    bool rightFound = res[2] > 0.5;
    double angleLeft = res[1];
    double angleRight = res[3]; 
    double determinedAngle = 0;
    if (leftFound && rightFound) {
        determinedAngle = fabs(angleLeft) < fabs(angleRight) ? angleLeft : angleRight;
    } else if (leftFound) {
        determinedAngle = angleLeft;
    } else if (rightFound) {
        determinedAngle = angleRight;
    } else {
        return 0;
    }
    return toPInPI(determinedAngle);
}


// ------------------------------------------------------ Debug log related ------------------------------------------------------
void Brain::logLags() {
    
    double detLag = msecsSince(data->timeLastDet);

    double MAX_LAG_LENGTH = config->cameraImageWidth;
    log->log_scalar(
        "performance",
        "detection_lag",
        detLag
    );
    

    // log fieldline detection delay
    double lineLag = msecsSince(data->timeLastLineDet);


    log->log_scalar(
        "performance",
        "fieldline_detection_lag",
        lineLag
    );

    // log game control delay
    double gcLag = msecsSince(data->timeLastGamecontrolMsg);

    log->log_scalar(
        "performance",
        "gamecontrol_lag",
        gcLag
    );
}

void Brain::logStatusToConsole() {
    static int cnt = 0;
    const int LOG_INTERVAL = 30;
    cnt++;
    if (cnt % LOG_INTERVAL == 0) {
        string msg = "";
        string gameState = tree->getEntry<string>("gc_game_state");
        gameState = gameState == "" ? "-----" : gameState;
        string gameSubType = tree->getEntry<string>("gc_game_sub_state_type");
        gameSubType = gameSubType == "" ? "-----" : gameSubType;
        string gameSubState = tree->getEntry<string>("gc_game_sub_state");
        gameSubState = gameSubState == "" ? "-----" : gameSubState;

        msg += format(
            "ROBOT:\n\tTeamID: %d\tPlayerID: %d\tNumberOfPlayers: %d\tRole: %s\tStartRole: %s\n\n",
            config->get_team_id(),
            config->get_player_id(),
            config->get_num_of_players(),
            tree->getEntry<string>("player_role").c_str(),
            config->get_player_role().c_str()
        );
        msg += format(
            "GAME:\n\tState: %s\tKickOffSide: %s\tisKickingOff: %s(%s)\n\tSubType: %s\tSubState: %s\tSubKickOffSide: %s\tisKickingOff: %s(%s)\n\tScore: %s\tJustScored: %s\n\tLiveCount: %d\tOppoLiveCount: %d\n\n", 
            gameState.c_str(), 
            tree->getEntry<bool>("gc_is_kickoff_side") ? "YES" : "NO",
            data->isKickingOff ? "YES" : "NO",
            msecsSince(data->kickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->kickoffStartTime)/1000).c_str(),
            gameSubType.c_str(),
            gameSubState.c_str(),
            tree->getEntry<bool>("gc_is_sub_state_kickoff_side") ? "YES" : "NO",
            data->isFreekickKickingOff ? "YES" : "NO",
            msecsSince(data->freekickKickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->freekickKickoffStartTime)/1000).c_str(),
            format("%d:%d", data->score, data->oppoScore).c_str(),
            tree->getEntry<bool>("we_just_scored") ? "YES" : "NO",
            data->liveCount,
            data->oppoLiveCount
        );
        
        msg += getComLogString();

        msg += format(
            "DEBUG:\n\tcom: %s\tvxFactor: %.2f\tyawOffset: %.2f\n\tControlState: %d\n\tTickTime: %.0fms",
            config->get_enable_com() ? "YES" : "NO",
            config->get_vx_factor(),
            config->get_yaw_offset(),
            tree->getEntry<int>("control_state"),
            msecsSince(data->lastTick)
        );
        prtDebug(msg);
    }
    data->lastTick = get_clock()->now();
}

string Brain::getComLogString() {
    stringstream ss;
    int onFieldCnt = 0;
    int aliveCnt = 0;
    int selfIdx = config->get_player_id() - 1;
    vector<int> onFieldIdxs = {};
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        if (data->penalty[i] == PENALTY_NONE) {
            onFieldCnt += 1;
            onFieldIdxs.push_back(i);
        }

        if (data->tmStatus[i].isAlive) aliveCnt += 1;
    }
    ss << CYAN_CODE << "COM: " << "\n";
    ss << "Teammates: OnField: " << onFieldCnt << "[";
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        ss << " P" << idx + 1 << " ";
    }
    ss << "]";
    ss << "  Alive: " << aliveCnt << "  TMCMDID: " << data->tmCmdId << "  ReceivedDMD: " << data->tmReceivedCmd << "\n";

    // Self info
    ss << "Self\tCost: " << format("%.1f", data->tmMyCost) << "\tLead: ";
    if (data->tmImLead)
        ss << GREEN_CODE << "YES" << CYAN_CODE;
    else
        ss << RED_CODE << "NO" << CYAN_CODE;
    ss << "    TMCMD: " << data->tmCmdId << format("\tCMD: [%d]%d", data->tmMyCmdId, data->tmMyCmd);
    ss << "\n";

    // Teammates info
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        auto status = data->tmStatus[idx];
        ss << "P" << idx + 1 << "[";
        if (status.isAlive)
            ss << GREEN_CODE << "★" << CYAN_CODE;
        else 
            ss << RED_CODE << "☆" << CYAN_CODE;
        ss << "]\tCost: " << format("%.1f", status.cost);
        ss << "\tLead: ";
        if (status.isLead)
            ss << GREEN_CODE << "YES" << CYAN_CODE;
        else
            ss << RED_CODE << "NO" << CYAN_CODE;
        ss << "\tCMD: " << format("[%d]%d", status.cmdId, status.cmd);
        ss << "\tLag: " << format("%.0f", msecsSince(status.timeLastCom)) << "ms" << "\n";
    }
    ss << "\n";
    
    return ss.str();
}

bool Brain::isFreekickStartPlacing() {
    return (tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK" && tree->getEntry<string>("gc_game_state") == "PLAY" && tree->getEntry<string>("gc_game_sub_state") == "GET_READY");
}


void Brain::agentCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received agent command: %s", msg->data.c_str());

    if (msg->data == "autonomy_stop") {
        tree->setEntry<bool>("autonomy_enabled", false);
        tree->setEntry<double>("autonomy_command_vx", 0.0);
        tree->setEntry<double>("autonomy_command_vy", 0.0);
        tree->setEntry<double>("autonomy_command_theta", 0.0);
        client->setVelocity(0.0, 0.0, 0.0);
        RCLCPP_INFO(get_logger(), "Autonomy disabled with zero velocity");
        return;
    }

    if (msg->data == "autonomy_save_track" ||
        msg->data == "autonomy_save_chase" ||
        msg->data == "autonomy_save_adjust") {
        const string phase = msg->data.substr(string("autonomy_save_").size());
        tree->setEntry<string>("autonomy_auto_phase", phase);
        RCLCPP_INFO(get_logger(), "Autonomy saved phase => %s", phase.c_str());
        return;
    }

    if (msg->data == "autonomy_auto_track" ||
        msg->data == "autonomy_auto_chase" ||
        msg->data == "autonomy_auto_adjust") {
        const string phase = msg->data.substr(string("autonomy_auto_").size());
        tree->setEntry<string>("autonomy_auto_phase", phase);
        tree->setEntry<string>("autonomy_switch", "auto");
        tree->setEntry<bool>("autonomy_enabled", true);
        RCLCPP_INFO(get_logger(), "Autonomy switch => auto (phase: %s)", phase.c_str());
        return;
    }

    if (msg->data == "autonomy_auto") {
        const string manual_mode = tree->getEntry<string>("autonomy_manual_mode");
        if (manual_mode == "chase" || manual_mode == "adjust") {
            tree->setEntry<string>("autonomy_auto_phase", manual_mode);
        }
        tree->setEntry<string>("autonomy_switch", "auto");
        tree->setEntry<bool>("autonomy_enabled", true);
        RCLCPP_INFO(
            get_logger(),
            "Autonomy switch => auto (phase: %s)",
            tree->getEntry<string>("autonomy_auto_phase").c_str());
        return;
    }

    if (msg->data == "autonomy_manual") {
        if (tree->getEntry<string>("autonomy_switch") == "auto") {
            tree->setEntry<string>(
                "autonomy_manual_mode",
                tree->getEntry<string>("autonomy_auto_phase"));
        }
        tree->setEntry<string>("autonomy_switch", "manual");
        tree->setEntry<bool>("autonomy_enabled", true);
        RCLCPP_INFO(
            get_logger(),
            "Autonomy switch => manual (mode: %s)",
            tree->getEntry<string>("autonomy_manual_mode").c_str());
        return;
    }

    if (msg->data == "autonomy_track" ||
        msg->data == "autonomy_chase" ||
        msg->data == "autonomy_adjust") {
        const string mode = msg->data.substr(string("autonomy_").size());
        tree->setEntry<string>("autonomy_manual_mode", mode);
        tree->setEntry<string>("autonomy_switch", "manual");
        tree->setEntry<bool>("autonomy_enabled", true);
        RCLCPP_INFO(get_logger(), "Autonomy manual mode => %s", mode.c_str());
        return;
    }

    data->timeLastGamecontrolMsg = get_clock()->now();



    // agent_command    control_state   gc_game_state
    // locate           2               INITIAL
    // ready            3               READY
    // play             3               PLAY
    // stop             3               SET
    string game_state;
    int control_state;

    // Set control_state and gc_game_state based on msg->data
    if (msg->data == "locate") {
        control_state = 2;
        game_state = "INITIAL";
    } else if (msg->data == "ready") {
        control_state = 3;
        game_state = "READY";
    } else if (msg->data == "play") {
        control_state = 3;
        game_state = "PLAY";

        // In agent mode, start the kickoff directly
        tree->setEntry<bool>("gc_is_kickoff_side", true);
        tree->setEntry<bool>("gc_is_sub_state_kickoff_side", true);
    } else if (msg->data == "stop") {
        control_state = 3;
        game_state = "END";
    } else {
        RCLCPP_WARN(get_logger(), "Unknown agent command: %s", msg->data.c_str());
        return;
    }

    tree->setEntry<int>("control_state", control_state);
    tree->setEntry<string>("gc_game_state", game_state);
    // Currently, playerRole is automatically modified during runtime in the following situations:
    // handleCooperation
    //   1. If there is no strategy during halftime, the role will be switched back to the initial setting at the start of the second half, requiring gc_game_state to be INITIAL corresponding to the soccer_agent's positioning state.
    //   2. If role_switch is enabled, the role will be automatically switched when the number of players on the field is insufficient after a penalty.
    //   3. In the ready state, the role will be set according to the player_role parameter only when the field is full.
    //   4. When the goalkeeper goes out, the role will be automatically switched.
    // Remote control switches playerRole
    //
    // After switching the control state, set the player_role to ensure it takes effect
    tree->setEntry<string>("player_role", config->get_player_role());
}

void Brain::publishVisualizationMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;

    // 1. Publish field map (fixed)
    auto &fd = config->fieldDimensions;
    
    // Center line
    marker_array.markers.push_back(
        visualizer->createFieldCenterLineMarker(fd.width, "map"));
    
    // Center circle
    marker_array.markers.push_back(
        visualizer->createFieldCenterCircleMarker(fd.circleRadius, "map"));
    
    // Field boundary
    marker_array.markers.push_back(
        visualizer->createFieldBoundaryMarker(fd.length, fd.width, "map"));
    
    // Our goal area
    marker_array.markers.push_back(
        visualizer->createGoalAreaMarker(true, fd.length, fd.goalAreaLength, fd.goalAreaWidth, "map"));
    
    // Opponent's goal area
    marker_array.markers.push_back(
        visualizer->createGoalAreaMarker(false, fd.length, fd.goalAreaLength, fd.goalAreaWidth, "map"));
    
    // Our penalty area (large penalty area)
    marker_array.markers.push_back(
        visualizer->createPenaltyAreaMarker(true, fd.length, fd.penaltyAreaLength, fd.penaltyAreaWidth, "map"));
    
    // Opponent's penalty area (large penalty area)
    marker_array.markers.push_back(
        visualizer->createPenaltyAreaMarker(false, fd.length, fd.penaltyAreaLength, fd.penaltyAreaWidth, "map"));
    
    // Our penalty point
    marker_array.markers.push_back(
        visualizer->createPenaltyPointMarker(true, fd.length, fd.penaltyDist, "map"));
    
    // Opponent's penalty point
    marker_array.markers.push_back(
        visualizer->createPenaltyPointMarker(false, fd.length, fd.penaltyDist, "map"));

    // 2. Publish robot position - with orientation arrow
    auto robot_marker = visualizer->createRobotMarker(
        data->robotPoseToField.x,
        data->robotPoseToField.y,
        data->robotPoseToField.theta,
        "map");
    marker_array.markers.push_back(robot_marker);

    // 3. Publish ball position
    auto ball_marker = visualizer->createBallMarker(
        data->ball.posToField.x,
        data->ball.posToField.y,
        0.11, // ball radius is 0.11
        "map");
    marker_array.markers.push_back(ball_marker);

    // 4. Publish observed Mark points (dynamic)
    std::vector<std::tuple<double, double, char>> mark_points;
    auto markings = data->getMarkings();
    for (const auto &marking : markings)
    {
        char type = marking.label == "LCross" ? 'L' : 
                   (marking.label == "TCross" ? 'T' : 
                   (marking.label == "XCross" ? 'X' : 'P'));
        mark_points.push_back({marking.posToField.x, marking.posToField.y, type});
    }
    
    if (!mark_points.empty())
    {
        auto mark_markers = visualizer->createObservedMarkPointMarkers(mark_points, "map");
        marker_array.markers.insert(marker_array.markers.end(), 
            mark_markers.begin(), mark_markers.end());
    }

    // 5. Publish observed field lines (dynamic)
    std::vector<std::vector<geometry_msgs::msg::Point>> field_lines;
    auto lines = data->getFieldLines();
    for (const auto &line : lines)
    {
        std::vector<geometry_msgs::msg::Point> line_points;
        geometry_msgs::msg::Point p1, p2;
        p1.x = line.posToField.x0;
        p1.y = line.posToField.y0;
        p1.z = 0.0;
        p2.x = line.posToField.x1;
        p2.y = line.posToField.y1;
        p2.z = 0.0;
        line_points.push_back(p1);
        line_points.push_back(p2);
        field_lines.push_back(line_points);
    }
    
    if (!field_lines.empty())
    {
        auto line_markers = visualizer->createObservedFieldLineMarkers(field_lines, "map");
        marker_array.markers.insert(marker_array.markers.end(), 
            line_markers.begin(), line_markers.end());
    }

    // 6. Publish GameController information (game state, score, etc.)
    string gameState = tree->getEntry<string>("gc_game_state");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    int myScore = data->score;
    int oppoScore = data->oppoScore;
    int remainingTime = data->secsRemaining;
    
    auto gc_marker = visualizer->createGameControllerInfoMarker(
        myScore,
        oppoScore,
        remainingTime,
        "map");
    marker_array.markers.push_back(gc_marker);
    
    auto gc_state_marker = visualizer->createGameControllerStateMarker(
        gameState,
        gameSubState,
        "map");
    marker_array.markers.push_back(gc_state_marker);

    visualizer->publishMarkers(marker_array);
}

void Brain::publishOdomToMapTF()
{
    // Create transform message
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Set timestamp
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    
    // Set translation (robot position in the field coordinate system)
    transformStamped.transform.translation.x = data->robotPoseToField.x;
    transformStamped.transform.translation.y = data->robotPoseToField.y;
    transformStamped.transform.translation.z = 0.0;
    
    // Set rotation (robot orientation in the field coordinate system)
    tf2::Quaternion q;
    q.setRPY(0, 0, data->robotPoseToField.theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    // Publish tf transform
    tf_broadcaster_->sendTransform(transformStamped);
}

void Brain::publishFieldDimensions()
{
    auto msg = std_msgs::msg::Float64MultiArray();
    auto &fd = config->fieldDimensions;
    
    // Pack field dimensions into an array
    // Order: length, width, penaltyDist, goalWidth, circleRadius, 
    //       penaltyAreaLength, penaltyAreaWidth, goalAreaLength, goalAreaWidth
    msg.data = {
        fd.length,
        fd.width,
        fd.penaltyDist,
        fd.goalWidth,
        fd.circleRadius,
        fd.penaltyAreaLength,
        fd.penaltyAreaWidth,
        fd.goalAreaLength,
        fd.goalAreaWidth
    };
    
    pubFieldDimensions->publish(msg);
    RCLCPP_INFO(get_logger(), "Published field dimensions: length=%.2f, width=%.2f", fd.length, fd.width);
}

void Brain::publishRobotPose()
{
    auto msg = geometry_msgs::msg::Pose2D();
    msg.x = data->robotPoseToField.x;
    msg.y = data->robotPoseToField.y;
    msg.theta = data->robotPoseToField.theta;
    
    pubRobotPose->publish(msg);
}

void Brain::publishBallPosition()
{
    auto msg = geometry_msgs::msg::Point();
    msg.x = data->ball.posToField.x;
    msg.y = data->ball.posToField.y;
    msg.z = 0.0;
    
    pubBallPosition->publish(msg);
}

void Brain::publishTeammatesPoses()
{
    auto msg = std_msgs::msg::Float64MultiArray();
    
    // Data format: every 3 numbers represent a group (x, y, theta)
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (data->tmStatus[i].isAlive && i != (config->get_player_id() - 1)) {
            msg.data.push_back(data->tmStatus[i].robotPoseToField.x);
            msg.data.push_back(data->tmStatus[i].robotPoseToField.y);
            msg.data.push_back(data->tmStatus[i].robotPoseToField.theta);
        }
    }
    
    pubTeammatesPoses->publish(msg);
}
