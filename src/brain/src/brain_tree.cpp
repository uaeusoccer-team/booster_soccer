#include <cmath>
#include <cstdlib>
#include "brain_tree.h"
#include "locator.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "locator.h"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <ios>

/**
 * Here we use a macro definition to reduce the code for RegisterBuilder. The effect of REGISTER_BUILDER(Test) after expansion is
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *     [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(ShootingAdjust)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(GoToFreekickPosition)
    REGISTER_BUILDER(GoToReadyPosition)
    REGISTER_BUILDER(GoToGoalBlockingPosition)
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(MoveHead)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(RLVisionKick)
    REGISTER_BUILDER(Assist)

    // Register Locator related nodes
    brain->registerLocatorNodes(factory);

    // Action Nodes for debug
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)

    factory.registerBehaviorTreeFromFile(brain->config->get_tree_file_path());
    tree = factory.createTree("MainTree");

    // After construction, initialize blackboard entries
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->get_player_role());
    setEntry<bool>("ball_visible", false);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("ball_depth_acquired", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);
    setEntry<double>("tracking_theta", 0.0);
    setEntry<double>("chase_vx", 0.0);
    setEntry<double>("chase_vy", 0.0);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("need_check_behind", false);

    setEntry<bool>("is_lead", true); 
    setEntry<string>("goalie_mode", "attack"); 

    setEntry<int>("test_choice", 0);
    setEntry<int>("control_state", 0);
    setEntry<bool>("assist_chase", false);
    setEntry<bool>("assist_kick", false);
    setEntry<bool>("go_manual", false);
    setEntry<string>("autonomy_switch", "manual");
    setEntry<string>("autonomy_manual_mode", "track");
    setEntry<string>("autonomy_auto_phase", "chase");
    setEntry<bool>("autonomy_enabled", true);
    setEntry<double>("autonomy_command_vx", 0.0);
    setEntry<double>("autonomy_command_vy", 0.0);
    setEntry<double>("autonomy_command_theta", 0.0);
    setEntry<double>("autonomy_adjust_vx", 0.0);
    setEntry<double>("autonomy_adjust_vy", 0.0);
    setEntry<double>("autonomy_adjust_theta", 0.0);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    // Automatic vision calibration related
    setEntry<string>("calibrate_state", "pitch");
    setEntry<double>("calibrate_pitch_center", 0.0);
    setEntry<double>("calibrate_pitch_step", 1.0);
    setEntry<double>("calibrate_yaw_center", 0.0);
    setEntry<double>("calibrate_yaw_step", 1.0);
    setEntry<double>("calibrate_z_center", 0.0);
    setEntry<double>("calibrate_z_step", 0.01);
}

void BrainTree::tick()
{
    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    bool applyMinX, applyMinY, applyMinTheta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);
    getInput("apply_min_x", applyMinX);
    getInput("apply_min_y", applyMinY);
    getInput("apply_min_theta", applyMinTheta);

    auto res = brain->client->setVelocity(
        x,
        y,
        theta,
        applyMinX,
        applyMinY,
        applyMinTheta);
    return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick()
{
    std::srand(std::time(0));
    double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

    auto res = brain->client->setVelocity(vx, 0, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double stopAngle = 0.1;
    double ballYawGain = 4.0;
    double pitchTurnGain = 1.0;
    double trackTurnYawLimit = 0.75;
    double lossTurnPitchLimit = 0.70;
    double headStep = 0.04;
    double headSettleStep = 0.02;
    double headDeadbandX = 35.0;
    double headDeadbandY = 35.0;
    getInput("stop_angle", stopAngle);
    getInput("ball_yaw_gain", ballYawGain);
    getInput("pitch_turn_gain", pitchTurnGain);
    getInput("track_turn_yaw_limit", trackTurnYawLimit);
    getInput("loss_turn_pitch_limit", lossTurnPitchLimit);
    getInput("head_step_rad", headStep);
    getInput("head_settle_step_rad", headSettleStep);
    getInput("head_deadband_x_px", headDeadbandX);
    getInput("head_deadband_y_px", headDeadbandY);

    stopAngle = std::fabs(stopAngle);
    ballYawGain = std::max(0.0, ballYawGain);
    pitchTurnGain = std::max(0.0, pitchTurnGain);
    trackTurnYawLimit = std::fabs(trackTurnYawLimit);
    lossTurnPitchLimit = std::fabs(lossTurnPitchLimit);
    headStep = std::fabs(headStep);
    headSettleStep = std::fabs(headSettleStep);
    headDeadbandX = std::fabs(headDeadbandX);
    headDeadbandY = std::fabs(headDeadbandY);

    if (!brain->data->headStateReceived.load(std::memory_order_acquire))
    {
        _hasLastProcessedBallFrame = false;
        setOutput("theta", 0.0);
        return NodeStatus::SUCCESS;
    }

    const double measuredHeadPitch = brain->data->headPitch.load(std::memory_order_relaxed);
    const double measuredHeadYaw = brain->data->headYaw.load(std::memory_order_relaxed);

    const double xCenter = brain->config->cameraImageWidth / 2.0;
    const double yCenter = brain->config->cameraImageHeight / 2.0;
    const bool iSeeBall = brain->data->ballDetected;
    const bool bboxValid =
        brain->data->ball.boundingBox.xmax > brain->data->ball.boundingBox.xmin &&
        brain->data->ball.boundingBox.ymax > brain->data->ball.boundingBox.ymin;

    double pitch = measuredHeadPitch;
    double yaw = measuredHeadYaw;

    if (!iSeeBall || !bboxValid)
    {
        _hasLastProcessedBallFrame = false;
        setOutput("theta", 0.0);
        brain->client->moveHead(pitch, yaw);
        return NodeStatus::SUCCESS;
    }

    const bool ballLocationKnown = brain->tree->getEntry<bool>("ball_location_known");
    const double ballYaw = ballLocationKnown ? brain->data->ball.yawToRobot : 0.0;
    const double normalPitch = brain->config->get_head_pitch_limit_up();
    const double pitchDown = std::max(0.0, measuredHeadPitch - normalPitch);
    const double pitchMultiplier = 1.0 + pitchTurnGain * pitchDown;
    const double vthetaLimit = std::fabs(brain->config->get_vtheta_limit());
    double theta = 0.0;
    const char *thetaSource = ballLocationKnown ? "DEADBAND" : "NO_DEPTH";

    if (ballLocationKnown && std::fabs(ballYaw) > stopAngle)
    {
        const double requestedTheta = ballYaw * ballYawGain * pitchMultiplier;
        theta = cap(requestedTheta, vthetaLimit, -vthetaLimit);
        thetaSource = "BALL_YAW";
    }

    setOutput("theta", theta);

    const double ballX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin);
    const double ballY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin);
    const auto ballTime = brain->data->ball.timePoint;
    const bool sameVisionFrame =
        _hasLastProcessedBallFrame &&
        ballTime.nanoseconds() == _lastProcessedBallTime.nanoseconds();

    if (sameVisionFrame)
    {
        return NodeStatus::SUCCESS;
    }

    const double dx = ballX - xCenter;  // + means ball is right of image center
    const double dy = ballY - yCenter;  // + means ball is below image center

    const double settleZonePx = 20.0;

    // Search direction is a visual/head memory, not a reuse of the projected
    // ground point. A pixel outside centre tells us which way the head was
    // about to look; a centred pixel uses the head's measured aim. Keeping
    // this separate prevents a moving-head projection error from deciding a
    // later stationary head scan.
    int visualSearchDirection = 0;
    if (std::fabs(dx) > headDeadbandX)
    {
        visualSearchDirection = dx > 0.0 ? -1 : 1;
    }
    else if (std::fabs(measuredHeadYaw) > 0.02)
    {
        visualSearchDirection = measuredHeadYaw > 0.0 ? 1 : -1;
    }
    const auto ballTrackingGeneration =
        brain->data->ballTrackingGeneration.load(std::memory_order_relaxed);
    const auto previousVisualGeneration =
        brain->data->lastBallSearchGeneration.load(std::memory_order_relaxed);
    if (previousVisualGeneration != ballTrackingGeneration)
    {
        // Never carry a direction from an older acquisition into a newly
        // observed, centered ball.
        brain->data->lastBallSearchDirection.store(0, std::memory_order_relaxed);
    }
    if (visualSearchDirection != 0)
    {
        // Preserve the most recent meaningful RGB/head direction if the last
        // frame happens to be centered immediately before the ball is lost.
        brain->data->lastBallSearchDirection.store(
            visualSearchDirection,
            std::memory_order_relaxed);
    }
    brain->data->lastBallPixelX.store(ballX, std::memory_order_relaxed);
    brain->data->lastBallPixelY.store(ballY, std::memory_order_relaxed);
    brain->data->lastBallPixelDx.store(dx, std::memory_order_relaxed);
    brain->data->lastBallPixelDy.store(dy, std::memory_order_relaxed);
    brain->data->lastBallObservedHeadYaw.store(measuredHeadYaw, std::memory_order_relaxed);
    brain->data->lastBallObservedHeadPitch.store(measuredHeadPitch, std::memory_order_relaxed);
    // Publish the generation last so CamFindBall never treats a partially
    // updated observation as belonging to the current acquisition.
    brain->data->lastBallSearchGeneration.store(
        ballTrackingGeneration,
        std::memory_order_release);

    if (std::fabs(dx) > headDeadbandX)
    {
        const double step = std::fabs(dx) <= headDeadbandX + settleZonePx ? headSettleStep : headStep;
        yaw += (dx > 0.0) ? -step : step;
    }

    if (std::fabs(dy) > headDeadbandY)
    {
        const double step = std::fabs(dy) <= headDeadbandY + settleZonePx ? headSettleStep : headStep;
        pitch += (dy > 0.0) ? step : -step;
    }

    brain->client->moveHead(pitch, yaw);
    _lastProcessedBallTime = ballTime;
    _hasLastProcessedBallFrame = true;

    brain->log->log(
        "CamTrackBall/direct_pixel",
        format("ballX: %.1f ballY: %.1f dx: %.1f dy: %.1f headPitch: %.3f headYaw: %.3f depthX: %.3f depthY: %.3f ballYaw: %.3f pitchMultiplier: %.3f theta: %.3f source: %s visualSearchDirection: %d yawLossTrigger: %s pitchLossTrigger: %s trackTurnYawLimit: %.3f lossTurnPitchLimit: %.3f",
               ballX,
               ballY,
               dx,
               dy,
               measuredHeadPitch,
               measuredHeadYaw,
               brain->data->ball.posToRobot.x,
               brain->data->ball.posToRobot.y,
               ballYaw,
               pitchMultiplier,
               theta,
               thetaSource,
               visualSearchDirection,
               std::fabs(measuredHeadYaw) >= trackTurnYawLimit ? "true" : "false",
               measuredHeadPitch >= lossTurnPitchLimit ? "true" : "false",
               trackTurnYawLimit,
               lossTurnPitchLimit));

    return NodeStatus::SUCCESS;
}


CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    _timeLastCmd = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

NodeStatus CamFindBall::tick()
{
    auto curTime = brain->get_clock()->now();
    setOutput("theta", 0.0);

    if (brain->data->ballDetected &&
        brain->data->ballDepthAcquired.load(std::memory_order_acquire))
    {
        _searchInitialized = false;
        _waitingForObservationLogged = false;
        _timeLastCmd = rclcpp::Time(0, 0, RCL_ROS_TIME);
        return NodeStatus::SUCCESS;
    }

    if (!brain->data->headStateReceived.load(std::memory_order_acquire))
    {
        _searchInitialized = false;
        _timeLastCmd = rclcpp::Time(0, 0, RCL_ROS_TIME);
        return NodeStatus::SUCCESS;
    }

    const double measuredHeadYaw = brain->data->headYaw.load(std::memory_order_relaxed);
    const double measuredHeadPitch = brain->data->headPitch.load(std::memory_order_relaxed);

    double yawLimit, trackTurnYawLimit, lossTurnPitchLimit;
    double headSearchSpeed, bodySearchSpeed, cmdIntervalMsec;
    getInput("yaw_limit", yawLimit);
    getInput("track_turn_yaw_limit", trackTurnYawLimit);
    getInput("loss_turn_pitch_limit", lossTurnPitchLimit);
    getInput("head_search_speed", headSearchSpeed);
    getInput("body_search_speed", bodySearchSpeed);
    getInput("cmd_interval_msec", cmdIntervalMsec);

    yawLimit = std::fabs(yawLimit);
    trackTurnYawLimit = std::fabs(trackTurnYawLimit);
    lossTurnPitchLimit = std::fabs(lossTurnPitchLimit);
    headSearchSpeed = std::fabs(headSearchSpeed);
    bodySearchSpeed = std::fabs(bodySearchSpeed);
    cmdIntervalMsec = std::max(20.0, cmdIntervalMsec);

    // Only a depth-confirmed RGB acquisition advances this generation.
    // RGB-only candidates do not stop CamFindBall or create a new episode.
    const auto ballTrackingGeneration =
        brain->data->ballTrackingGeneration.load(std::memory_order_relaxed);
    const bool ballTrackingGenerationChanged =
        ballTrackingGeneration != _searchBallGeneration;

    // Never let search create startup motion before this stack has accepted a
    // depth-confirmed ball.
    if (ballTrackingGeneration == 0)
    {
        if (!_waitingForObservationLogged)
        {
            brain->log->log(
                "CamFindBall/wait_observation",
                "No depth-confirmed ball has been acquired; holding head and body still");
            _waitingForObservationLogged = true;
        }

        _searchInitialized = false;
        _timeLastCmd = rclcpp::Time(0, 0, RCL_ROS_TIME);
        _searchBallGeneration = ballTrackingGeneration;
        return NodeStatus::SUCCESS;
    }

    if (!_searchInitialized || ballTrackingGenerationChanged)
    {
        const auto visualSearchGeneration =
            brain->data->lastBallSearchGeneration.load(std::memory_order_acquire);
        const bool hasVisualObservation =
            ballTrackingGeneration != 0 &&
            visualSearchGeneration == ballTrackingGeneration;
        const int visualSearchDirection = hasVisualObservation
            ? brain->data->lastBallSearchDirection.load(std::memory_order_relaxed)
            : 0;
        const double lastBallPixelX = hasVisualObservation
            ? brain->data->lastBallPixelX.load(std::memory_order_relaxed)
            : 0.0;
        const double lastBallPixelY = hasVisualObservation
            ? brain->data->lastBallPixelY.load(std::memory_order_relaxed)
            : 0.0;
        const double lastBallPixelDx = hasVisualObservation
            ? brain->data->lastBallPixelDx.load(std::memory_order_relaxed)
            : 0.0;
        const double lastBallPixelDy = hasVisualObservation
            ? brain->data->lastBallPixelDy.load(std::memory_order_relaxed)
            : 0.0;
        const double lastObservedHeadYaw = hasVisualObservation
            ? brain->data->lastBallObservedHeadYaw.load(std::memory_order_relaxed)
            : measuredHeadYaw;
        const double lastObservedHeadPitch = hasVisualObservation
            ? brain->data->lastBallObservedHeadPitch.load(std::memory_order_relaxed)
            : measuredHeadPitch;
        const bool yawTurnTriggered =
            hasVisualObservation &&
            std::fabs(lastObservedHeadYaw) >= trackTurnYawLimit;
        // In this stack, larger positive pitch means the head is looking
        // farther downward.
        const bool pitchTurnTriggered =
            hasVisualObservation &&
            lastObservedHeadPitch >= lossTurnPitchLimit;
        const bool fastTurnRequested =
            visualSearchDirection != 0 &&
            (yawTurnTriggered || pitchTurnTriggered);
        const char *directionSource = "";

        if (fastTurnRequested)
        {
            _searchMode = SearchMode::CONTINUE_TURN;
            _searchDirection = visualSearchDirection > 0 ? 1.0 : -1.0;
            if (yawTurnTriggered && pitchTurnTriggered)
            {
                directionSource = "RGB_YAW_AND_PITCH";
            }
            else if (yawTurnTriggered)
            {
                directionSource = "RGB_YAW_LIMIT";
            }
            else
            {
                directionSource = "RGB_PITCH_LIMIT";
            }
        }
        else
        {
            _searchMode = SearchMode::HEAD_SCAN;

            // Do not start a surprise scan before a ball has ever been seen.
            // The stationary mode needs a visual observation from this ball
            // generation, not a stale projected ball position or old turn.
            if (!hasVisualObservation)
            {
                if (!_waitingForObservationLogged)
                {
                    brain->log->log(
                        "CamFindBall/wait_observation",
                        "No current ball observation; holding head and body still");
                    _waitingForObservationLogged = true;
                }

                _searchInitialized = false;
                _timeLastCmd = rclcpp::Time(0, 0, RCL_ROS_TIME);
                _searchBallGeneration = ballTrackingGeneration;
                return NodeStatus::SUCCESS;
            }

            if (visualSearchDirection != 0)
            {
                _searchDirection = visualSearchDirection > 0 ? 1.0 : -1.0;
                directionSource = "LAST_VISUAL_DIRECTION";
            }
            else if (std::fabs(measuredHeadYaw) > 0.02)
            {
                // A centred ball still has a useful direction when the head
                // was already aimed left or right.
                _searchDirection = measuredHeadYaw > 0.0 ? 1.0 : -1.0;
                directionSource = "HEAD_AIM";
            }
            else
            {
                // The last ball was centred with a centred head, so neither
                // side is better. Alternate only after a real observation.
                _searchDirection = _alternateScanDirection;
                _alternateScanDirection *= -1.0;
                directionSource = "ALTERNATE_CENTERED";
            }
        }

        _waitingForObservationLogged = false;
        _searchYaw = lastObservedHeadYaw;
        _searchPitch = lastObservedHeadPitch;
        _searchInitialized = true;
        _timeLastCmd = rclcpp::Time(0, 0, RCL_ROS_TIME);
        _searchBallGeneration = ballTrackingGeneration;

        brain->log->log(
            "CamFindBall/start",
            format("mode: %s source: %s direction: %.0f visualDirection: %d lastBallX: %.1f lastBallY: %.1f lastDx: %.1f lastDy: %.1f lastHeadYaw: %.3f lastHeadPitch: %.3f yawTriggered: %s pitchTriggered: %s trackTurnYawLimit: %.3f lossTurnPitchLimit: %.3f bodySearchSpeed: %.3f requestedTheta: %.3f searchYawLimit: %.3f ballTrackingGeneration: %llu",
                   _searchMode == SearchMode::CONTINUE_TURN ? "CONTINUE_TURN" : "HEAD_SCAN",
                   directionSource,
                   _searchDirection,
                   visualSearchDirection,
                   lastBallPixelX,
                   lastBallPixelY,
                   lastBallPixelDx,
                   lastBallPixelDy,
                   lastObservedHeadYaw,
                   lastObservedHeadPitch,
                   yawTurnTriggered ? "true" : "false",
                   pitchTurnTriggered ? "true" : "false",
                   trackTurnYawLimit,
                   lossTurnPitchLimit,
                   bodySearchSpeed,
                   _searchMode == SearchMode::CONTINUE_TURN
                       ? _searchDirection * bodySearchSpeed
                       : 0.0,
                   yawLimit,
                   static_cast<unsigned long long>(_searchBallGeneration)));
    }

    double timeSinceLastCmd = 0.0;
    if (_timeLastCmd.nanoseconds() > 0)
    {
        timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
        if (timeSinceLastCmd < 0.0)
        {
            _timeLastCmd = rclcpp::Time(0, 0, RCL_ROS_TIME);
            timeSinceLastCmd = 0.0;
        }
    }

    // A continued body turn begins immediately. A stationary loss never
    // produces theta; it scans with the head only.
    double theta = 0.0;
    if (_searchMode == SearchMode::CONTINUE_TURN)
    {
        theta = _searchDirection * bodySearchSpeed;
    }
    setOutput("theta", theta);

    if (_timeLastCmd.nanoseconds() > 0 && timeSinceLastCmd < cmdIntervalMsec)
    {
        return NodeStatus::SUCCESS;
    }

    double dtSec = 0.0;
    if (_timeLastCmd.nanoseconds() > 0 && timeSinceLastCmd > 0.0)
    {
        dtSec = std::min(timeSinceLastCmd / 1000.0, 0.25);
    }
    else
    {
        // Start the head scan on the first loss command instead of sending a
        // no-op command and waiting one whole command interval.
        dtSec = cmdIntervalMsec / 1000.0;
    }

    auto getEffectiveYawLimit = [this, yawLimit]()
    {
        const double hardwareYawLimit = _searchDirection > 0.0
            ? std::fabs(brain->config->get_head_yaw_limit_left())
            : std::fabs(brain->config->get_head_yaw_limit_right());
        return std::min(yawLimit, hardwareYawLimit);
    };

    if (headSearchSpeed > 0.0)
    {
        constexpr double SCAN_ENDPOINT_TOLERANCE = 0.05;
        double effectiveYawLimit = getEffectiveYawLimit();
        double targetYaw = _searchDirection * effectiveYawLimit;

        if (_searchMode == SearchMode::HEAD_SCAN && effectiveYawLimit > 0.0)
        {
            const double directedMeasuredYaw = _searchDirection * measuredHeadYaw;
            const double directedCommandedYaw = _searchDirection * _searchYaw;
            const bool measuredAtEndpoint =
                directedMeasuredYaw >= effectiveYawLimit - SCAN_ENDPOINT_TOLERANCE;
            const bool commandAtEndpoint =
                directedCommandedYaw >= effectiveYawLimit - 1e-6;
            const bool endpointCommandSettled =
                commandAtEndpoint &&
                _timeLastCmd.nanoseconds() > 0 &&
                timeSinceLastCmd >= cmdIntervalMsec;

            if (measuredAtEndpoint || endpointCommandSettled)
            {
                _searchDirection *= -1.0;
                _searchYaw = measuredHeadYaw;
                effectiveYawLimit = getEffectiveYawLimit();
                targetYaw = _searchDirection * effectiveYawLimit;
            }
        }

        if (effectiveYawLimit > 0.0)
        {
            const bool keepCurrentOutwardHeadAim =
                _searchMode == SearchMode::CONTINUE_TURN &&
                _searchDirection * measuredHeadYaw >= effectiveYawLimit;

            if (keepCurrentOutwardHeadAim)
            {
                // A turn-loss may begin while the head is already outside a
                // smaller configured search cap. Keep looking in the active
                // turn direction instead of first pulling the head inward.
                // RobotClient still applies the physical hardware limit.
                _searchYaw = measuredHeadYaw;
            }
            else
            {
                _searchYaw += _searchDirection * headSearchSpeed * dtSec;
                if (_searchDirection > 0.0)
                {
                    _searchYaw = std::min(_searchYaw, targetYaw);
                }
                else
                {
                    _searchYaw = std::max(_searchYaw, targetYaw);
                }
            }
        }
    }

    brain->client->moveHead(_searchPitch, _searchYaw);
    _timeLastCmd = curTime;

    brain->log->log(
        "CamFindBall/search",
        format("mode: %s pitch: %.3f commandedYaw: %.3f measuredYaw: %.3f direction: %.0f theta: %.3f",
               _searchMode == SearchMode::CONTINUE_TURN ? "CONTINUE_TURN" : "HEAD_SCAN",
               _searchPitch,
               _searchYaw,
               measuredHeadYaw,
               _searchDirection,
               theta));
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus Chase::tick()
{
    auto log = [=](string msg) {
        brain->log->debug("Chase4", msg);
    };
    log("ticked");
    
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);

    bool avoidObstacle = brain->config->get_avoid_during_chase();
    double oaSafeDist = brain->config->get_chase_ao_safe_dist();

    if (
        brain->config->get_limit_near_ball_speed()
        && brain->data->ball.range < brain->config->get_near_ball_range()
    ) {
        vxLimit = min(brain->config->get_near_ball_speed_limit(), vxLimit);
    }

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double kickDir = brain->data->kickDir;
    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x
    );
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;


    double vx, vy, vtheta;
    Pose2D target_f, target_r; 
    static string targetType = "direct"; 
    static double circleBackDir = 1.0; 
    double dirThreshold = M_PI / 2;
    if (targetType == "direct") dirThreshold *= 1.2;


    // Calculate target point
    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        log("targetType = direct");
        targetType = "direct";
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } else {
        targetType = "circle_back";
        double cbDirThreshold = 0.0; 
        cbDirThreshold -= 0.2 * circleBackDir; 
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
            
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = ballYaw;
    } else {
        vx = min(vxLimit, brain->data->ball.range);
        vy = 0;
        vtheta = targetDir;
        if (fabs(targetDir) < 0.1 && ballRange > 2.0) vtheta = 0.0;
        vx *= sigmoid((fabs(vtheta)), 1, 3); 
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    static double smoothVx = 0.0;
    static double smoothVy = 0.0;
    static double smoothVtheta = 0.0;
    smoothVx = smoothVx * 0.7 + vx * 0.3;
    smoothVy = smoothVy * 0.7 + vy * 0.3;
    smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, yTolerance, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("y_tolerance", yTolerance);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    double vx = 0.0;
    double vy = 0.0;
    setOutput("vx", vx);
    setOutput("vy", vy);

    if (!brain->tree->getEntry<bool>("ball_visible") ||
        !brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    const double ballRange = brain->data->ball.range;
    if (ballRange <= std::fabs(stopDist))
    {
        brain->log->log(
            "SimpleChase/vector",
            format("range: %.3f ballX: %.3f ballY: %.3f vx: %.3f vy: %.3f source: STOP_DIST",
                   ballRange,
                   brain->data->ball.posToRobot.x,
                   brain->data->ball.posToRobot.y,
                   vx,
                   vy));
        return NodeStatus::SUCCESS;
    }

    const double absVxLimit = std::fabs(vxLimit);
    const double absVyLimit = std::fabs(vyLimit);
    vx = cap(brain->data->ball.posToRobot.x, absVxLimit, -absVxLimit);
    vy = cap(brain->data->ball.posToRobot.y, absVyLimit, -absVyLimit);

    if (std::fabs(brain->data->ball.posToRobot.y) <= std::fabs(yTolerance))
    {
        vy = 0.0;
    }

    setOutput("vx", vx);
    setOutput("vy", vy);

    brain->log->log(
        "SimpleChase/vector",
        format("range: %.3f ballX: %.3f ballY: %.3f vx: %.3f vy: %.3f source: DEPTH",
               ballRange,
               brain->data->ball.posToRobot.x,
               brain->data->ball.posToRobot.y,
               vx,
               vy));
    return NodeStatus::SUCCESS;
}


NodeStatus ShootingAdjust::tick()
{
    double targetRange, targetYOffset, thetaOffset;
    double rangeTolerance, yTolerance, stopAngle;
    double rangeGain, yGain, ballYawGain;
    double vxLimit, vyLimit, vthetaLimit;
    double turnFirstThreshold, fixedHeadYaw, maxBallRange;

    getInput("target_range", targetRange);
    getInput("target_y_offset", targetYOffset);
    getInput("theta_offset", thetaOffset);
    getInput("range_tolerance", rangeTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("stop_angle", stopAngle);
    getInput("range_gain", rangeGain);
    getInput("y_gain", yGain);
    getInput("ball_yaw_gain", ballYawGain);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("turn_first_threshold", turnFirstThreshold);
    getInput("fixed_head_yaw", fixedHeadYaw);
    getInput("max_ball_range", maxBallRange);

    rangeTolerance = std::fabs(rangeTolerance);
    yTolerance = std::fabs(yTolerance);
    stopAngle = std::fabs(stopAngle);
    rangeGain = std::max(0.0, rangeGain);
    yGain = std::max(0.0, yGain);
    ballYawGain = std::max(0.0, ballYawGain);
    vxLimit = std::fabs(vxLimit);
    vyLimit = std::fabs(vyLimit);
    vthetaLimit = std::fabs(vthetaLimit);
    turnFirstThreshold = std::fabs(turnFirstThreshold);
    maxBallRange = std::fabs(maxBallRange);

    double vx = 0.0;
    double vy = 0.0;
    double theta = 0.0;
    setOutput("vx", vx);
    setOutput("vy", vy);
    setOutput("theta", theta);

    const bool bboxValid =
        brain->data->ball.boundingBox.xmax > brain->data->ball.boundingBox.xmin &&
        brain->data->ball.boundingBox.ymax > brain->data->ball.boundingBox.ymin;
    const bool visualBallUsable =
        brain->tree->getEntry<bool>("ball_visible") &&
        brain->data->ballDetected &&
        bboxValid;
    if (!visualBallUsable || !brain->data->headStateReceived.load(std::memory_order_acquire))
    {
        return NodeStatus::SUCCESS;
    }

    const double measuredHeadPitch = brain->data->headPitch.load(std::memory_order_relaxed);
    const double measuredHeadYaw = brain->data->headYaw.load(std::memory_order_relaxed);
    const double ballPixelX = mean(
        brain->data->ball.boundingBox.xmax,
        brain->data->ball.boundingBox.xmin);
    const double ballPixelY = mean(
        brain->data->ball.boundingBox.ymax,
        brain->data->ball.boundingBox.ymin);
    const double imageCenterX = brain->config->cameraImageWidth / 2.0;
    const double imageCenterY = brain->config->cameraImageHeight / 2.0;
    const double ballPixelDx = ballPixelX - imageCenterX;
    const double ballPixelDy = ballPixelY - imageCenterY;
    int visualSearchDirection = 0;
    if (ballPixelX > imageCenterX)
    {
        visualSearchDirection = -1;
    }
    else if (ballPixelX < imageCenterX)
    {
        visualSearchDirection = 1;
    }
    else if (std::fabs(measuredHeadYaw) > 0.0)
    {
        visualSearchDirection = measuredHeadYaw > 0.0 ? 1 : -1;
    }

    const auto ballTrackingGeneration =
        brain->data->ballTrackingGeneration.load(std::memory_order_relaxed);
    const auto previousVisualGeneration =
        brain->data->lastBallSearchGeneration.load(std::memory_order_relaxed);
    if (previousVisualGeneration != ballTrackingGeneration)
    {
        brain->data->lastBallSearchDirection.store(0, std::memory_order_relaxed);
    }
    if (visualSearchDirection != 0)
    {
        brain->data->lastBallSearchDirection.store(
            visualSearchDirection,
            std::memory_order_relaxed);
    }
    brain->data->lastBallPixelX.store(ballPixelX, std::memory_order_relaxed);
    brain->data->lastBallPixelY.store(ballPixelY, std::memory_order_relaxed);
    brain->data->lastBallPixelDx.store(ballPixelDx, std::memory_order_relaxed);
    brain->data->lastBallPixelDy.store(ballPixelDy, std::memory_order_relaxed);
    brain->data->lastBallObservedHeadYaw.store(measuredHeadYaw, std::memory_order_relaxed);
    brain->data->lastBallObservedHeadPitch.store(measuredHeadPitch, std::memory_order_relaxed);
    brain->data->lastBallSearchGeneration.store(
        ballTrackingGeneration,
        std::memory_order_release);

    const double hardwareYawLimit = fixedHeadYaw >= 0.0
        ? std::fabs(brain->config->get_head_yaw_limit_left())
        : std::fabs(brain->config->get_head_yaw_limit_right());
    const double commandedHeadYaw = cap(fixedHeadYaw, hardwareYawLimit, -hardwareYawLimit);
    const bool fixedHeadCommandSent = ballTrackingGeneration != _fixedHeadCommandGeneration;
    if (fixedHeadCommandSent)
    {
        brain->client->moveHead(measuredHeadPitch, commandedHeadYaw);
        _fixedHeadCommandGeneration = ballTrackingGeneration;
    }

    const double ballX = brain->data->ball.posToRobot.x;
    const double ballY = brain->data->ball.posToRobot.y;
    const double ballYaw = brain->data->ball.yawToRobot;
    const bool ballLocationKnown = brain->tree->getEntry<bool>("ball_location_known");
    const bool bodyUsable = ballLocationKnown && brain->data->ball.range <= maxBallRange;
    const double rangeError = bodyUsable ? ballX - targetRange : 0.0;
    const double yError = bodyUsable ? ballY - targetYOffset : 0.0;
    const double thetaError = bodyUsable ? toPInPI(ballYaw - thetaOffset) : 0.0;

    if (bodyUsable && std::fabs(rangeError) > rangeTolerance)
    {
        vx = cap(rangeError * rangeGain, vxLimit, -vxLimit);
    }
    if (bodyUsable && std::fabs(yError) > yTolerance)
    {
        vy = cap(yError * yGain, vyLimit, -vyLimit);
    }

    if (bodyUsable && std::fabs(thetaError) > stopAngle)
    {
        theta = thetaError * ballYawGain;
    }

    const char *thetaSource = !ballLocationKnown
        ? "NO_DEPTH"
        : (!bodyUsable ? "OUT_OF_RANGE" : (std::fabs(thetaError) > stopAngle ? "BALL_YAW" : "DEADBAND"));

    if (bodyUsable && turnFirstThreshold > 0.0 && std::fabs(thetaError) > turnFirstThreshold)
    {
        vx = 0.0;
        vy = 0.0;
    }

    theta = cap(theta, vthetaLimit, -vthetaLimit);
    setOutput("vx", vx);
    setOutput("vy", vy);
    setOutput("theta", theta);

    brain->log->log(
        "ShootingAdjust/vector",
        format("ballX: %.3f ballY: %.3f ballRange: %.3f maxBallRange: %.3f ballYaw: %.3f rangeError: %.3f yError: %.3f thetaError: %.3f vx: %.3f vy: %.3f theta: %.3f source: %s measuredHeadYaw: %.3f fixedHeadYaw: %.3f fixedHeadCommandSent: %d visualSearchDirection: %d",
               ballX,
               ballY,
               brain->data->ball.range,
               maxBallRange,
               ballYaw,
               rangeError,
               yError,
               thetaError,
               vx,
               vy,
               theta,
               thetaSource,
               measuredHeadYaw,
               commandedHeadYaw,
               fixedHeadCommandSent,
               visualSearchDirection));
    return NodeStatus::SUCCESS;
}


NodeStatus GoToFreekickPosition::onStart() {
    _isInFinalAdjust = false;
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {

    string side;
    getInput("side", side);
    if (side !="attack" && side != "defense") return NodeStatus::SUCCESS;
    
    Pose2D targetPose;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    double kickDir = brain->data->kickDir;
    double defenseDir = atan2(ballPos.y, ballPos.x + fd.length / 2);
    if (side == "attack") {
       double dist;
       getInput("attack_dist", dist);
        
       if (brain->data->myStrikerIDRank == 0) {
        targetPose.x = ballPos.x - dist * cos(kickDir);
        targetPose.y = ballPos.y - dist * sin(kickDir);
        targetPose.theta = kickDir;
       } else if (brain->data->myStrikerIDRank == 1) {
        targetPose.x = ballPos.x - 2.0 * cos(defenseDir);
        targetPose.y = ballPos.y - 2.0 * sin(defenseDir);
        targetPose.theta = defenseDir;
        } else if (brain->data->myStrikerIDRank == 2) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = fd.goalAreaWidth / 2.0;
        } else if (brain->data->myStrikerIDRank == 3) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = - fd.goalAreaWidth / 2.0;
        }
    } else if (side == "defense") {
        if (brain->data->myStrikerIDRank == 0) {
            targetPose.x = ballPos.x - 3.0 * cos(defenseDir);  // Changed from 2 to 3, defensive player is farther from the ball
            targetPose.y = ballPos.y - 2.5 * sin(defenseDir);
            targetPose.theta = defenseDir;
           } else if (brain->data->myStrikerIDRank == 1) {
            targetPose.x = ballPos.x - 3.5 * cos(defenseDir);
            targetPose.y = ballPos.y - 4.0 * sin(defenseDir);
            targetPose.theta = defenseDir;
            } else if (brain->data->myStrikerIDRank == 2) {
                targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
                targetPose.y = fd.goalAreaWidth / 2.0;
            } else if (brain->data->myStrikerIDRank == 3) {
                targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
                targetPose.y = - fd.goalAreaWidth / 2.0;
            }
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    double deltaDir = toPInPI(targetPose.theta - robotPose.theta);


    if ( // Considered to have reached the target position
        dist < 0.2 
        && fabs(deltaDir) < 0.1
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    
    if (!brain->config->get_enable_obstacle_avoidance() || dist < 1.0 || _isInFinalAdjust) {
        _isInFinalAdjust = true; // Entering the final adjustment phase
        auto targetPose_r = brain->data->field2robot(targetPose);

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        double vtheta = brain->data->ball.yawToRobot * 2.0; // The larger the multiplier, the faster the rotation

        double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); // When the distance is far, prioritize turning
        vx *= linearFactor;
        vy *= linearFactor;

        // Prevent collision with the ball
        Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
        if (
            pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.5
            && brain->data->ball.range < 1.0
        ) {
            vx = min(0.0, vx);
            vy = vy >= 0 ? vy + 0.1: vy - 0.1;
        }

        double vxLimit, vyLimit;
        getInput("vx_limit", vxLimit);
        getInput("vy_limit", vyLimit);
        vx = cap(vx, vxLimit, -0.4);     // Further limit speed
        vy = cap(vy, vyLimit, -vyLimit);     // Further limit speed
        

        brain->client->setVelocity(vx, vy, vtheta);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.4;
    double turnThreshold = 0.4;
    double vxLimit = 0.6;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    brain->client->moveToPoseOnField3(targetPose.x, targetPose.y, targetPose.theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, 0.2, 0.2, 0.1, avoidObstacle);

    return NodeStatus::RUNNING;
}
void GoToFreekickPosition::onHalted() {
}

NodeStatus GoToGoalBlockingPosition::tick() {
    
    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    string curRole = brain->tree->getEntry<string>("player_role");

    Pose2D targetPose;
    targetPose.x = curRole == "striker" ? (std::max(- fd.length / 2.0 + distToGoalline, ballPos.x - 1.5))
            : (- fd.length / 2.0 + distToGoalline);
    if (ballPos.x + fd.length / 2.0 < distToGoalline) {
        targetPose.y = curRole == "striker" ? (ballPos.y > 0 ? fd.goalWidth / 2.0 : -fd.goalWidth / 2.0)
            : (ballPos.y > 0 ? fd.goalWidth / 4.0 : -fd.goalWidth / 4.0);
    } else {
        targetPose.y = ballPos.y * distToGoalline / (ballPos.x + fd.length / 2.0);
        targetPose.y = curRole == "striker" ? (cap(targetPose.y, fd.goalWidth / 2.0, -fd.goalWidth / 2.0))
            : (cap(targetPose.y, fd.penaltyAreaWidth/ 2.0, -fd.penaltyAreaWidth / 2.0));
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( // Considered to have reached the target position
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    double vx = targetPose_r.x;
    double vy = targetPose_r.y;
    double vtheta = brain->data->ball.yawToRobot * 4.0; 


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -vxLimit);    
    vy = cap(vy, vyLimit, -vyLimit);    
    

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus Assist::tick() {
    auto log = [=](string msg) {
        brain->log->debug("Assist", msg);
    };
    log("ticked");

    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;
    string curRole = brain->tree->getEntry<string>("player_role");

    bool isSecondary = false; 
    bool has2Assists = false;
    int selfIdx = brain->config->get_player_id() - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; 

        auto tmStatus = brain->data->tmStatus[i];
        if (!tmStatus.isAlive) continue; 
        if (tmStatus.isLead) continue; 
        if (tmStatus.role != "striker") continue; 

        has2Assists = true;
        log("2 assists found");
        if (tmStatus.robotPoseToField.x > robotPose.x) {
            log("i am secondary");
            isSecondary = true; 
        }
    }
    log(format("has2Assists: %d, isSecondary: %d", has2Assists, isSecondary));


    Pose2D targetPose;
    targetPose.x = isSecondary ? ballPos.x - 4.0 : ballPos.x - 2.0;
    targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); 
    targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); 
    if (has2Assists) { 
        targetPose.y += isSecondary ? - 0.5 : 0.5;
    }


    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( 
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx, vy, vtheta;
    auto targetPose_r = brain->data->field2robot(targetPose);
    double targetDir = atan2(targetPose_r.y, targetPose_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);

    bool avoidObstacle = brain->config->get_avoid_during_chase();
    double oaSafeDist = brain->config->get_chase_ao_safe_dist();

    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = brain->data->ball.yawToRobot;
    } else {
        vx = targetPose_r.x;
        vy = targetPose_r.y;
        vtheta = brain->data->ball.yawToRobot * 4.0; 
    }


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -1.0);     
    vy = cap(vy, vyLimit, -vyLimit);     
    

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus Adjust::tick()
{
    auto log = [=](string msg) { 
        brain->log->debug("adjust5", msg); 
    };
    log("enter");
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f", brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, brain->data->ball.yawToRobot));
    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);


    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double st = st_far; 
    double R = ballRange; 
    double r = range;
    double sr = cap(R - r, 0.5, 0); 
    log(format("R: %.2f, r: %.2f, sr: %.2f", R, r, sr));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; 
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; 
    double thetar_r = dir_rb_f - theta_robot_f; 

    vx = st * cos(thetat_r) + sr * cos(thetar_r); 
    vy = st * sin(thetat_r) + sr * sin(thetar_r); 
    vtheta = ballYaw;
    vtheta *= vtheta_factor; 

    if (fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.; 
    if (
        fabs(ballYaw) > TURN_FIRST_THRESHOLD 
        && fabs(deltaDir) < M_PI / 4
    ) { 
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDir::tick()
{
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1;

    auto gpAngles = brain->getGoalPostAngles(0.0);
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;

    if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
        brain->data->kickType = "cross";
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - fd.penaltyDist/2 - bPos.x
        );
    }
    else if (brain->isDefensing()) {
        brain->data->kickType = "block";
        brain->data->kickDir = atan2(
            bPos.y,
            bPos.x + fd.length/2
        );

    } else { 
        brain->data->kickType = "shoot";
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - bPos.x
        );
        if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    }

    brain->log->log(
        "field/kick_dir",
        format("Kick direction: %.2f rad at ball position (%.2f, %.2f)", 
               brain->data->kickDir, brain->data->ball.posToField.x, brain->data->ball.posToField.y)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->debug("striker_decide", msg);
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.3; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    bool avoidPushing = brain->config->get_avoid_during_kick();
    double kickAoSafeDist = brain->config->get_kick_ao_safe_dist();
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
    } else if (
                brain->config->get_enable_auto_visual_kick() &&
                brain->data->tmImLead && 
                brain->data->tmMyCostRank == 0 && 
                !brain->tree->getEntry<bool>("ball_out") && 
                brain->data->lose_ball == false &&
                brain->data->tmMyCost < 7.0 &&
                brain->data->ball.range < brain->config->get_auto_visual_kick_enable_dist_max() &&
                brain->data->ball.range > brain->config->get_auto_visual_kick_enable_dist_min() &&
                fabs(brain->data->ball.yawToRobot) < brain->config->get_auto_visual_kick_enable_angle() * 1.3 &&
                brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2 - 14.3 &&
                fabs(brain->data->ball.posToField.y) < 5 &&
                brain->data->robotPoseToField.x > brain->config->fieldDimensions.length / 2 - 14.3 &&
                fabs(brain->data->robotPoseToField.y) < 5 
            ) {
        newDecision = "auto_visual_kick";
        brain->data->tmImInVisualKick = true;
    } else if (!brain->data->tmImLead) {
        newDecision = "assist";
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
    } else if (
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff) 
            || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 1.5
    ) {
        if (brain->data->kickType == "cross") newDecision = "cross";
        else newDecision = "kick";      
        brain->data->isFreekickKickingOff = false; 
    }
    else
    {
        newDecision = "adjust";
    }

    setOutput("decision_out", newDecision);
    
    // Publish player_decide message
    brain->visualizer->publishPlayerDecision(format("striker-%s", newDecision.c_str()));
    
    // Publish decision information through visualization_publisher
    auto decision_marker = brain->visualizer->createDecisionInfoMarker(
        "striker",
        newDecision,
        ballRange,
        ballYaw,
        kickDir,
        dir_rb_f,
        angleGoodForKick,
        brain->data->tmImLead,
        "map"
    );
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(decision_marker);
    brain->visualizer->publishMarkers(marker_array);
    
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1]; 
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
    }
    else
    {
        newDecision = "adjust";
    }

    setOutput("decision_out", newDecision);
    
    // Publish player_decide message
    brain->visualizer->publishPlayerDecision(format("goalie-%s", newDecision.c_str()));
    
    // Publish decision information through visualization_publisher
    auto decision_marker = brain->visualizer->createDecisionInfoMarker(
        "goalie",
        newDecision,
        ballRange,
        ballYaw,
        kickDir,
        dir_rb_f,
        angleIsGood,
        false,  // goalie does not need is_lead information
        "map"
    );
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(decision_marker);
    brain->visualizer->publishMarkers(marker_array);
    
    return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->get_vx_factor();   
    double yawOffset = brain->config->get_yaw_offset(); 


    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; 
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    { 
        vx = vxLimit;
        vy = 0.0;
    }
    else
    { 
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }


    double speed = norm(vx, vy);
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart()
{
    _minRange = brain->data->ball.range;
    _speed = 0.5;
    _startTime = brain->get_clock()->now();


    bool avoidPushing = brain->config->get_avoid_during_kick();
    double kickAoSafeDist = brain->config->get_kick_ao_safe_dist();
    string role = brain->tree->getEntry<string>("player_role");
    if (
        avoidPushing
        && (role != "goal_keeper")
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // Publish movement command
    double angle = brain->data->ball.yawToRobot;
    brain->client->crabWalk(angle, _speed);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    auto log = [=](string msg) {
        brain->log->debug("Kick", msg);
    };


    bool enableAbort = brain->config->get_abort_kick_when_ball_moved();
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;
    const double BALL_LOST_THRESHOLD = 1000;  
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) 
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD 
        )
    ) {
        log("ball moved, abort kick");
        return NodeStatus::SUCCESS;
    }


    if (ballRange < _minRange) _minRange = ballRange;    

    
    bool avoidPushing = brain->config->get_avoid_during_kick();
    double kickAoSafeDist = brain->config->get_kick_ao_safe_dist();
    if (
        avoidPushing
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }


    double msecs = getInput<double>("min_msec_kick").value();
    double speed = getInput<double>("speed_limit").value();
    msecs = msecs + brain->data->ball.range / speed * 1000;
    if (brain->msecsSince(_startTime) > msecs) { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }


    if (brain->data->ballDetected) { 
        double angle = brain->data->ball.yawToRobot;
        double speed = getInput<double>("speed_limit").value();
        _speed += 0.1; 
        speed = min(speed, _speed);
        brain->client->crabWalk(angle, speed);
    }

    return NodeStatus::RUNNING;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}


// Static variable definition
rclcpp::Time RLVisionKick::_lastExitTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

NodeStatus RLVisionKick::onStart()
{
    _startTime = brain->get_clock()->now();
    _isDecelerating = false;
    _visionKickStarted = false;
    _pendingRobocupWalk = false;
    
    // Start deceleration
    startDecelerate(500.0);
    stepDecelerate();
    
    return NodeStatus::RUNNING;
}

NodeStatus RLVisionKick::onRunning()
{
    // Check exit flag
    if (brain->data->shouldExitRLVisionKick) {
        brain->data->shouldExitRLVisionKick = false;
        brain->data->tmImInVisualKick = false;
        recordExitTime();
        return NodeStatus::SUCCESS;
    }
    
    // Handle deceleration phase
    if (_isDecelerating) {
        stepDecelerate();
        
        if (!_isDecelerating) {
            // Deceleration completed
            if (_pendingRobocupWalk) {
                brain->client->robocupWalk();
                _pendingRobocupWalk = false;
                return NodeStatus::SUCCESS;
            } else if (!_visionKickStarted) {
                // Start vision kick after deceleration
                brain->client->RLVisionKick();
                _headScanStartTime = brain->get_clock()->now();
                _visionKickStarted = true;
            }
        }
        return NodeStatus::RUNNING;
    }
    
    if (_visionKickStarted) {
        double headMsec = brain->msecsSince(_headScanStartTime);
        if (headMsec < 300.0) {
            brain->client->moveHead(0.4, 0.0);
        } else if (headMsec < 550.0) {
            brain->client->moveHead(0.7, 0.0);
        }
    }

    // Check exit conditions
    double elapsed = brain->msecsSince(_startTime);
    double minMsecKick = getInput<double>("min_msec_kick").value();
    
    // Check if ball is too far or cost is too high
    bool ballTooFar = brain->data->ballDetected && brain->data->ball.range > 5.0;
    bool shouldExit = (((ballTooFar || brain->data->tmMyCost > 8.0) && (elapsed > minMsecKick)) || brain->data->lose_ball || brain->tree->getEntry<bool>("ball_out"));
    
    if (shouldExit) {
        recordExitTime();
        startDecelerate(500.0);
        _pendingRobocupWalk = true;
        stepDecelerate();
        return NodeStatus::RUNNING;
    }

    return NodeStatus::RUNNING;
}

void RLVisionKick::onHalted()
{
    brain->data->tmImInVisualKick = false;
    brain->client->setVelocity(0.0, 0.0, 0.0);
    brain->client->robocupWalk();
    recordExitTime();
    
    _isDecelerating = false;
    _visionKickStarted = false;
    _pendingRobocupWalk = false;
}

bool RLVisionKick::isMinIntervalSatisfied(double minIntervalMsec)
{
    // This is a static method, so we can't access brain instance
    // For now, always return true. If needed, pass brain pointer as parameter
    return true;
}

void RLVisionKick::recordExitTime()
{
    _lastExitTime = brain->get_clock()->now();
}

void RLVisionKick::startDecelerate(double durationMs)
{
    if (_isDecelerating) {
        return;
    }
    
    _isDecelerating = true;
    _decelStartTime = brain->get_clock()->now();
    _decelDurationMs = durationMs;
}

bool RLVisionKick::stepDecelerate()
{
    if (!_isDecelerating) {
        return true;
    }
    
    double elapsed = brain->msecsSince(_decelStartTime);
    brain->client->setVelocity(0.0, 0.0, 0.0);
    
    if (elapsed >= _decelDurationMs) {
        _isDecelerating = false;
        return true;
    }
    
    return false;
}

NodeStatus StandStill::onStart()
{

    _startTime = brain->get_clock()->now();


    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::RUNNING;
}

NodeStatus StandStill::onRunning()
{
    double msecs;
    getInput("msecs", msecs);
    if (brain->msecsSince(_startTime) < msecs) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }


    return NodeStatus::SUCCESS;
}

void StandStill::onHalted()
{
    double msecs;
    getInput("msecs", msecs);
    _startTime -= rclcpp::Duration(- 2 * msecs, 0);
}


NodeStatus RobotFindBall::onStart()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;

    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit;
    getInput("vyaw_limit", vyawLimit);

    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    _turnDir = 1.0;
}

NodeStatus CamFastScan::onStart()
{
    _cmdIndex = 0;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus CamFastScan::onRunning()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(_timeLastCmd) < interval) return NodeStatus::RUNNING;

    // else 
    if (_cmdIndex >= 6) return NodeStatus::SUCCESS;

    // else
    _cmdIndex++;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onStart()
{
    _timeStart = brain->get_clock()->now();
    _lastAngle = brain->data->robotPoseToOdom.theta;
    _cumAngle = 0.0;

    bool towardsBall = false;
    _angle = getInput<double>("rad").value();
    getInput("towards_ball", towardsBall);
    if (towardsBall) {
        double ballPixX = (brain->data->ball.boundingBox.xmin + brain->data->ball.boundingBox.xmax) / 2;
        _angle = fabs(_angle) * (ballPixX < brain->config->cameraImageWidth / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    if (
        fabs(_cumAngle) - fabs(_angle) > -0.1
        || turnTime > _msecLimit
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // else 
    brain->client->setVelocity(0, 0, (_angle - _cumAngle)*2);
    return NodeStatus::RUNNING;
}

NodeStatus MoveToPoseOnField::tick()
{

    double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);
    bool avoidObstacle;
    getInput("avoid_obstacle", avoidObstacle);

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoToReadyPosition::tick()
{
    double distTolerance, thetaTolerance;
    getInput("dist_tolerance", distTolerance);
    getInput("theta_tolerance", thetaTolerance);
    string role = brain->tree->getEntry<string>("player_role");
    bool isKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
    auto fd = brain->config->fieldDimensions;

    // default values, override with different conditions
    double tx = 0, ty = 0, ttheta = 0; 
    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    if (brain->distToBorder() > - 1.0) { // near border
        vxLimit = 0.6;
        vyLimit = 0.4;
    }
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;

    if (role == "striker") {
        if (brain->data->myStrikerIDRank == 0) {
            tx = isKickoff ? - fd.circleRadius - 0.5 : - fd.circleRadius * 2;
            ty = 0.0;
        } else if (brain->data->myStrikerIDRank == 1) {
            tx = isKickoff ? - fd.circleRadius - 0.5 : - fd.circleRadius * 2;
            ty = -1.5;
        } else if (brain->data->myStrikerIDRank == 2) {
            tx = - fd.length / 2.0 + fd.penaltyAreaLength;
            ty = fd.circleRadius / 2.0;
        } else if (brain->data->myStrikerIDRank == 3) {
            tx = - fd.length / 2.0 + fd.penaltyDist;
            ty = - fd.circleRadius / 2.0;
        }
    } else if (role == "goal_keeper") {
        tx = -fd.length / 2.0 + fd.goalAreaLength;
        ty = 0;
        ttheta = 0;
    }

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick()
{
    auto log = [=](string msg) {
        brain->log->debug("GoBackInField", msg);
    };
    log("GoBackInField ticked");

    double valve;
    getInput("valve", valve);
    double vx = 0; 
    double vy = 0; 
    double dir = 0;
    auto fd = brain->config->fieldDimensions;
    if (brain->data->robotPoseToField.x > fd.length / 2.0 - valve) dir = - M_PI;
    else if (brain->data->robotPoseToField.x < - fd.length / 2.0 + valve) dir = 0;
    else if (brain->data->robotPoseToField.y > fd.width / 2.0 + valve) dir = - M_PI / 2.0;
    else if (brain->data->robotPoseToField.y < - fd.width / 2.0 - valve) dir = M_PI / 2.0;
    else { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus WaveHand::tick()
{
    string action;
    getInput("action", action);
    if (action == "start")
        brain->client->waveHand(true);
    else
        brain->client->waveHand(false);
    return NodeStatus::SUCCESS;
}

NodeStatus MoveHead::tick()
{
    double pitch, yaw;
    getInput("pitch", pitch);
    getInput("yaw", yaw);
    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus CheckAndStandUp::tick()
{
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 2) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->debug("recovery", "reset recovery");
        return NodeStatus::SUCCESS;
    }
    brain->log->debug("recovery", format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex));

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        brain->data->currentRobotModeIndex == 1 && 
        brain->data->recoveryPerformedRetryCount < brain->config->get_retry_max_count()) {
        brain->data->shouldExitRLVisionKick = true;
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->log->debug("recovery", format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->recoveryPerformed && brain->data->currentRobotModeIndex == 10) {
        brain->data->recoveryPerformedRetryCount +=1;
        brain->data->recoveryPerformed = false;
        brain->log->debug("recovery", format("Add retry count: %d", brain->data->recoveryPerformedRetryCount));
    }


    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        (brain->data->currentRobotModeIndex == 8 || brain->data->currentRobotModeIndex == 20)) { 
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->data->shouldExitRLVisionKick = false;
        brain->log->debug("recovery", "Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState)));
    }

    return NodeStatus::SUCCESS;
}


NodeStatus CalibrateOdom::tick()
{
    double x, y, theta;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    brain->calibrateOdom(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}
