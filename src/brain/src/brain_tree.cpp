#include <cmath>
#include <cstdlib>
#include "brain_tree.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>

/**
 * 这里使用宏定义来缩减 RegisterBuilder 的代码量
 * REGISTER_BUILDER(Test) 展开后的效果是
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
    REGISTER_BUILDER(RobotFindBall) // 机器人寻找球
    REGISTER_BUILDER(Chase) // 追球
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust) // 调整机器人位置
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(SelfLocateEnterField)
    REGISTER_BUILDER(SelfLocate1M)
    REGISTER_BUILDER(SelfLocateBorder)
    REGISTER_BUILDER(SelfLocate2T)
    REGISTER_BUILDER(SelfLocateLT)
    REGISTER_BUILDER(SelfLocatePT)
    REGISTER_BUILDER(SelfLocate2X)
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
    REGISTER_BUILDER(Assist)

    

    // Action Nodes for debug
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)
    REGISTER_BUILDER(PlaySound)
    REGISTER_BUILDER(Speak)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // 构造完成后，初始化 blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    // 开球，对方开球时球动了，或到了时间限制时，置为 true，表示我们可以动了。
    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("need_check_behind", false);

    // 双机通讯相关
    setEntry<bool>("is_lead", true); // true 时代表自己为控球, false 时代表自己不控球, 而是给其它队员打配合.
    setEntry<string>("goalie_mode", "attack"); // guard, attack

    setEntry<int>("test_choice", 0);
    setEntry<int>("control_state", 0);
    setEntry<bool>("assist_chase", false);
    setEntry<bool>("assist_kick", false);
    setEntry<bool>("go_manual", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    // 自动视觉校准相关
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
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
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
    double pitch, yaw, ballX, ballY, deltaX, deltaY;
    const double pixToleranceX = brain->config->camPixX / 4.; // 球距离视野中心的像素差小于这个容差, 则认为在视野中心了.
    const double pixToleranceY = brain->config->camPixY / 4.;
    const double xCenter = brain->config->camPixX / 2;
    const double yCenter = brain->config->camPixY / 2; // 用下视野 2/3 位置来追踪球, 以获得更多的场上信息.

    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/CamTrackBall", rerun::TextLog(msg));
    };
    auto logTrackingBox = [=](int color, string label) {
        brain->log->setTimeNow();
        vector<rerun::Vec2D> mins;
        vector<rerun::Vec2D> sizes;
        mins.push_back(rerun::Vec2D{xCenter - pixToleranceX, yCenter - pixToleranceY});
        sizes.push_back(rerun::Vec2D{pixToleranceX * 2, pixToleranceY * 2});
        brain->log->log(
            "image/track_ball",
            rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                .with_labels({label})
                .with_colors(color)
        );   

    };

    bool iSeeBall = brain->data->ballDetected;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
        return NodeStatus::SUCCESS;

    if (!iSeeBall)
    { // 没看见, 看向记忆中球的大致位置
        if (iKnowBallPos) {
            pitch = brain->data->ball.pitchToRobot;
            yaw = brain->data->ball.yawToRobot;
        } else if (tmBallPosReliable) {
            pitch = brain->data->tmBall.pitchToRobot;
            yaw = brain->data->tmBall.yawToRobot;
        } else {
            log("reached impossible condition");
        }
        logTrackingBox(0x000000FF, "ball not detected"); 
    }
    else { // 看见了, 视线跟踪球               
        ballX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin);
        ballY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin);
        deltaX = ballX - xCenter;
        deltaY = ballY - yCenter; 
        
        if (std::fabs(deltaX) < pixToleranceX && std::fabs(deltaY) < pixToleranceY)
        { // 认为已经在中心了
            auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f", ballX, ballY, deltaX, deltaY);
            logTrackingBox(0x00FF00FF, label);
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5; // 越大头部运动越平滑, 越小则越快, 小于 1.0 会超调震荡
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
        auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f, pitch: %.1f, yaw: %.1f", ballX, ballY, deltaX, deltaY, pitch, yaw);
        logTrackingBox(0xFF0000FF, label);
    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 1.0;
    double highPitch = 0.45;
    double leftYaw = 1.1;
    double rightYaw = -1.1;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 800;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    } // 目前全部节点都是返回 Success 的, 返回 failure 会影响后面节点的执行.

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    } // 没到下条指令的执行时间
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {                  // 超过一定时间, 认为这是重新从头执行
        _cmdIndex = 0; // 注意这里不 return
    }
    else
    { // 达到时间, 执行下一个指令, 同样不 return
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();
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
        brain->log->setTimeNow();
        brain->log->log("debug/Chase4", rerun::TextLog(msg));
    };
    log("ticked");
    
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);

    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (
        brain->config->limitNearBallSpeed
        && brain->data->ball.range < brain->config->nearBallRange
    ) {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
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
    Pose2D target_f, target_r; // 移动目标的目标, 分别相对于球场和机器人
    static string targetType = "direct"; // direct | circle_back
    static double circleBackDir = 1.0; // 1.0 代表走向球的左侧切向, 1.1 代表走向球的右侧切向.
    // 计算目标点
    double dirThreshold = M_PI / 2;
    if (targetType == "direct") dirThreshold *= 1.2; // 防止震荡, 防止球在机器人前方时, 机器人直接转向球, 导致震荡.
    /*防止第一脚震荡
    if (brain->data->isKickingOff && brain->tree->getEntry<bool>("gc_is_kickoff_side")) {
        targetType = "direct";
        log("Kickoff mode: force direct chase");
    }
    */
    // 计算目标点
    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        log("targetType = direct");
        targetType = "direct";
        // 直接走向球, 球在机器人前方时, 机器人直接转向球, 导致震荡.
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } else {
        targetType = "circle_back";
        double cbDirThreshold = 0.0; 
        // 防止震荡, 防止球在机器人前方时, 机器人直接转向球, 导致震荡.
        cbDirThreshold -= 0.2 * circleBackDir; // 防止震荡
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); // 球到目标切身交点的方向.
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
    brain->log->setTimeNow();
    brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}), 0xFFFFFFFF, false, false);
            
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
        vx *= sigmoid((fabs(vtheta)), 1, 3); // 角度大时减小转弯半径
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    static double smoothVx = 0.0;
    static double smoothVy = 0.0;
    static double smoothVtheta = 0.0;
    smoothVx = smoothVx * 0.8 + vx * 0.2;
    smoothVy = smoothVy * 0.8 + vy * 0.2;
    smoothVtheta = smoothVtheta * 0.8 + vtheta * 0.2;

    // brain->client->setVelocity(smoothVx, smoothVy, smoothVtheta, false, false, false);
    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); // 距离远时, 优先转向
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -0.1);     // 进一步限速
    vy = cap(vy, vyLimit, -vyLimit); // vy 进一步限速

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
        // if (fabs(brain->data->ball.yawToRobot) < stopAngle) vtheta = 0; // uncomment 这一行, 会站住. 现在站不太稳, 就让它一直动着吧.
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


NodeStatus GoToFreekickPosition::onStart() {
    // brain->log->log("debug/freekick_position/onStart", rerun::TextLog(format("stage onStart")));
    _isInFinalAdjust = false;
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
    };
    log("running");
    // if (!brain->tree->getEntry<bool>("ball_location_known")) {
    //     brain->client->setVelocity(0, 0, 0);
    //     return NodeStatus::SUCCESS;
    // }

    string side;
    getInput("side", side);
    if (side !="attack" && side != "defense") return NodeStatus::SUCCESS;
    
    Pose2D targetPose;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    if (side == "attack") {
        double targetDir = brain->data->kickDir;
    //    double targetDir = atan2(0 - ballPos.y, fd.length / 2 - ballPos.x); 
    //    if (fabs(targetDir > M_PI / 12 * 4)) targetDir = atan2(0 - ballPos.y, fd.length / 2 - fd.penaltyDist - ballPos.x);
       double dist;
       getInput("attack_dist", dist);

       targetPose.x = ballPos.x - dist * cos(targetDir);
       targetPose.y = ballPos.y - dist * sin(targetDir);
       targetPose.theta = targetDir;

        // 3v3 逻辑
        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (!brain->isPrimaryStriker()) {
                targetPose.y = 0;
                targetPose.x -= 1.5;
                if (targetPose.x < -fd.length / 2.0 + fd.goalAreaLength) targetPose.x = -fd.length / 2.0 + fd.goalAreaLength;
                // 如果计算出来的进攻位置接近底线
                auto buffer = 2.0;
                auto targetXPose = brain->config->fieldDimensions.length / 2 - buffer;
                if (targetPose.x > targetXPose) {
                    targetPose.x = targetXPose;
                    targetPose.theta = 0;
                }
            }
        }

        // 5v5 逻辑
        if (brain->config->numOfPlayers == 5 && brain->data->liveCount >= 3)
        {
            if (!brain->isPrimaryStriker()) {
                // 根据机器人ID分配不同的进攻位置
                int playerId = brain->config->playerId;
                switch (playerId) {
                    case 2: // 第二个机器人，左翼支援
                        targetPose.y = ballPos.y > 0 ? ballPos.y + 1.0 : 1.5;
                        targetPose.x -= 1.0;
                        break;
                    case 3: // 第三个机器人，右翼支援
                        targetPose.y = ballPos.y < 0 ? ballPos.y - 1.0 : -1.5;
                        targetPose.x -= 1.0;
                        break;
                    case 4: // 第四个机器人，中场支援
                        targetPose.y = 0;
                        targetPose.x -= 2.0;
                        break;
                    case 5: // 第五个机器人，后场支援
                        targetPose.y = ballPos.y > 0 ? 0.8 : -0.8;
                        targetPose.x -= 3.0;
                        break;
                    default:
                        // 默认位置
                        targetPose.y = 0;
                        targetPose.x -= 1.5;
                        break;
                }
                
                // 确保不超出场地边界
                if (targetPose.x < -fd.length / 2.0 + fd.goalAreaLength) 
                    targetPose.x = -fd.length / 2.0 + fd.goalAreaLength;
                if (targetPose.y > fd.width / 2.0 - 0.5) 
                    targetPose.y = fd.width / 2.0 - 0.5;
                if (targetPose.y < -fd.width / 2.0 + 0.5) 
                    targetPose.y = -fd.width / 2.0 + 0.5;
                    
                // 如果计算出来的进攻位置接近底线
                auto buffer = 2.0;
                auto targetXPose = brain->config->fieldDimensions.length / 2 - buffer;
                if (targetPose.x > targetXPose) {
                    targetPose.x = targetXPose;
                    targetPose.theta = 0;
                }
            }
        }

    } else if (side == "defense") {
        double targetDir = atan2(ballPos.y, ballPos.x + fd.length / 2);
        double dist;
        getInput("defense_dist", dist);
        targetPose.x = ballPos.x - dist * cos(targetDir);
        targetPose.y = ballPos.y - dist * sin(targetDir);
        targetPose.theta = targetDir;
        if (ballPos.x < -fd.length / 2 + 1.0)  targetPose.x = -fd.length / 2 + 1.5;

        // 3v3 逻辑
        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (!brain->isPrimaryStriker()) {
                targetPose.y = targetPose.y > 0 ? targetPose.y - 1.0 : targetPose.y + 1.0;
            }
        }

        // 5v5 逻辑
        if (brain->config->numOfPlayers == 5 && brain->data->liveCount >= 3)
        {
            if (!brain->isPrimaryStriker()) {
                // 根据机器人ID分配不同的防守位置
                int playerId = brain->config->playerId;
                switch (playerId) {
                    case 2: // 第二个机器人，左侧防守
                        targetPose.y = targetPose.y > 0 ? targetPose.y - 0.8 : targetPose.y + 1.5;
                        targetPose.x -= 0.5;
                        break;
                    case 3: // 第三个机器人，右侧防守
                        targetPose.y = targetPose.y < 0 ? targetPose.y + 0.8 : targetPose.y - 1.5;
                        targetPose.x -= 0.5;
                        break;
                    case 4: // 第四个机器人，中路防守
                        targetPose.y = targetPose.y > 0 ? targetPose.y - 0.5 : targetPose.y + 0.5;
                        targetPose.x -= 1.0;
                        break;
                    case 5: // 第五个机器人，深度防守
                        targetPose.y = 0;
                        targetPose.x = -fd.length / 2 + fd.goalAreaLength + 1.0;
                        targetPose.theta = atan2(ballPos.y, ballPos.x + fd.length / 2);
                        break;
                    default:
                        // 默认防守位置
                        targetPose.y = targetPose.y > 0 ? targetPose.y - 1.0 : targetPose.y + 1.0;
                        break;
                }
                
                // 确保不超出场地边界
                if (targetPose.y > fd.width / 2.0 - 0.5) 
                    targetPose.y = fd.width / 2.0 - 0.5;
                if (targetPose.y < -fd.width / 2.0 + 0.5) 
                    targetPose.y = -fd.width / 2.0 + 0.5;
            }
        }
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    double deltaDir = toPInPI(targetPose.theta - robotPose.theta);

    // brain->log->log("debug/freekick_position/dist", rerun::Scalar(dist));
    // brain->log->log("debug/freekick_position/deltaDir", rerun::Scalar(deltaDir));

    if ( // 认为到达了目标位置
        dist < 0.2 
        && fabs(deltaDir) < 0.1
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    if (!brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() || dist < 1.0 || _isInFinalAdjust) {
        // brain->log->log("debug/freekick_position/onFinalAdjust", rerun::TextLog(format("stage onFinalAdjust: %f", dist)));
        _isInFinalAdjust = true; // 进入最后的微调阶段
        auto targetPose_r = brain->data->field2robot(targetPose);

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        double vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快

        double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); // 距离远时, 优先转向
        vx *= linearFactor;
        vy *= linearFactor;

        // 防止撞到球
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
        vx = cap(vx, vxLimit, -0.4);     // 进一步限速
        vy = cap(vy, vyLimit, -vyLimit);     // 进一步限速
        

        brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit = 0.6;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    // brain->log->log("debug/freekick_position", rerun::TextLog(format("stage move: targetPose: (%.2f, %.2f, %.2f)", targetPose.x, targetPose.y, targetPose.theta)));
    brain->client->moveToPoseOnField3(targetPose.x, targetPose.y, targetPose.theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, 0.2, 0.2, 0.1, avoidObstacle);

    return NodeStatus::RUNNING;
}

void GoToFreekickPosition::onHalted() {
    // brain->log->log("debug/freekick_position/onHault", rerun::TextLog(format("stage OnHalted")));
}

NodeStatus GoToGoalBlockingPosition::tick() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToGoalBlockingPosition", rerun::TextLog(msg));
    };
    log("GoToGoalBlockingPosition ticked");

    // if (!brain->tree->getEntry<bool>("ball_location_known")) {
    //     brain->client->setVelocity(0, 0, 0);
    //     return NodeStatus::SUCCESS;
    // }
    // brain->log->setTimeNow();
    // brain->log->log("tree/GoToGoalBlockingPosition", rerun::TextLog("GoToGoalBlockingPosition tick"));
    
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
    if ( // 认为到达了目标位置
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    double vx = targetPose_r.x;
    double vy = targetPose_r.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -vxLimit);     // 进一步限速
    vy = cap(vy, vyLimit, -vyLimit);     // 进一步限速
    

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Assist::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Assist", rerun::TextLog(msg));
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
    int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; // 跳过自己

        auto tmStatus = brain->data->tmStatus[i];
        if (!tmStatus.isAlive) continue; // 跳过不在线的
        if (tmStatus.isLead) continue; // 跳过主攻, 只看助攻
        if (tmStatus.role != "striker") continue; // 不看守门员

        has2Assists = true;
        log("2 assists found");
        if (tmStatus.robotPoseToField.x > robotPose.x) {
            log("i am secondary");
            isSecondary = true; // 我为第二助攻
        }
    }
    log(format("has2Assists: %d, isSecondary: %d", has2Assists, isSecondary));


    Pose2D targetPose;
    targetPose.x = isSecondary ? ballPos.x - 4.0 : ballPos.x - 2.0;
    targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); // 不要太接近底线
    targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); // 可以挡住球的位置
    if (has2Assists) { // 有两个 assists 时, 位置错开
        targetPose.y += isSecondary ? - 0.5 : 0.5;
    }


    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( // 认为到达了目标位置
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

    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

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
        vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快
    }


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -0.15);     // 进一步限速, 不允许后退速度过快.
    vy = cap(vy, vyLimit, -vyLimit);     // 进一步限速
    

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Adjust::tick()
{
    auto log = [=](string msg) { 
        brain->log->setTimeNow();
        brain->log->log("debug/adjust5", rerun::TextLog(msg)); 
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
    double dir_rb_f = brain->data->robotBallAngleToField; // 机器人到球, field 坐标系中的方向
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    // 计算绕球转的速度指令, 意义更为明确的版本
    // double st = cap(fabs(deltaDir), st_far, st_near); // 切向速度
    double st = st_far; 
    // if (fabs(ballYaw) > turnThreshold) {
    //     st = 0.0; // 如果球的角度差大, 切向速度为 0, 只转向球
    //     log(format("large ball angle: %.2f, using zero tangential speed", ballYaw));
    // }
    double R = ballRange; // current Range
    double r = range; // target Range
    double sr = cap(R - r, 0.5, 0); // radial speed
    log(format("R: %.2f, r: %.2f, sr: %.2f", R, r, sr));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
        // sr = 0.;
        // vxLimit = 0.1;
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; // 机器人在 field 坐标系中的方向
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; // 机器人到球的切线方向, robot 坐标系
    double thetar_r = dir_rb_f - theta_robot_f; // 机器人到球的方向, robot 坐标系

    vx = st * cos(thetat_r) + sr * cos(thetar_r); // 切线速度 + 径向速度
    vy = st * sin(thetat_r) + sr * sin(thetar_r); // 切线速度 + 径向速度
    // vtheta = toPInPI(ballYaw + st / R * (deltaDir > 0 ? 1.0 : -1.0)); // 第二项是补偿因为切向速度而导致的角度偏差
    vtheta = ballYaw;
    vtheta *= vtheta_factor; // 后面的乘数越大, 转身越快

    if (fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.; // 角度比较小时不转身
    if (
        fabs(ballYaw) > TURN_FIRST_THRESHOLD 
        && fabs(deltaDir) < M_PI / 4
    ) { // 角度比较大时, 优先转身
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -vxLimit);// 进一步限速
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDir::tick()
{
    // 读取和处理参数
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1; //防止震荡

    auto gpAngles = brain->getGoalPostAngles(0.0);
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;
    auto color = 0xFFFFFFFF; // for log

    if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
        brain->data->kickType = "cross";
        color = 0xFF00FFFF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - fd.penaltyDist/2 - bPos.x
        );
    }
    else if (
        brain->data->isFreekickKickingOff 
        && brain->isPrimaryStriker() 
        && !brain->data->isDirectShoot
        && (bPos.x < fd.length/2.0 - fd.goalAreaWidth && bPos.x > -fd.length/2.0 + fd.penaltyAreaWidth)
    ) {
        // 在开球时, 决策要不要传中
        brain->data->kickType = "cross";
        color = 0xFF00FFFF;
        auto ballPos = brain->data->ball.posToField;
        auto fd = brain->config->fieldDimensions;
        if (ballPos.y > fd.width / 2.0  * 0.8) brain->data->kickDir = - M_PI / 2.0;
        if (ballPos.y < -fd.width / 2.0  * 0.8) brain->data->kickDir =  M_PI / 2.0;
    }
    else if (brain->data->isKickingOff && brain->tree->getEntry<bool>("gc_is_kickoff_side") && brain->tree->getEntry<string>("gc_game_state") == "SET") {
        // 正常比赛中第一次开球或进球后我方开球时: 直接走到球正前方并朝前射门
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = 0.0;
    }
    else if (brain->isDefensing()) {
        brain->data->kickType = "block";
        color = 0xFFFF00FF;
        brain->data->kickDir = atan2(
            bPos.y,
            bPos.x + fd.length/2
        );

    } else { // default to shoot
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - bPos.x
        );
        if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; // 已经过线了, 继续向前踢
    }

    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/striker_decide", rerun::TextLog(msg));
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; // 机器人到球, field 坐标系中的方向
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.3; // 计算角度时为门柱让出的距离
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing // 是否在踢球时避免碰撞
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    // 判断是否穿过了 KickDir
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
    auto color = 0xFFFFFFFF; // for log
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    // 移除assist限制，让所有前锋都能直接踢球
     } else if (!brain->data->tmImLead) {
         newDecision = "assist";
         color = 0x00FFFFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
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
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; // 只要进一次 kick, 就不算是 kickoff 阶段了.
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{
    // 读取和处理参数
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField; // 机器人到球, field 坐标系中的方向
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; // 球到左边门柱的角度(我们的左)
    double theta_r = goalPostAngles[1]; // 球到右边门柱的角度
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;

    // 读取参数
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;   // 用于调整 vx, vx *= vxFactor, 以补偿 x, y 方向的速度参数与实际速度比例的偏差, 使运动方向准确
    double yawOffset = brain->config->yawOffset; // 用于补偿定位角度的偏差

    // 计算速度指令
    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; // 移动的目标
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    { // 在可踢中的范围内, 尽量直走, 同时避免后面出现除 0 的问题.
        vx = vxLimit;
        vy = 0.0;
    }
    else
    { // 否则计算出要向哪个方向移动, 并给出可实现的速度指令
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    // 估算移动所需时间
    double speed = norm(vx, vy);
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart()
{
    _minRange = brain->data->ball.range;
    _speed = 0.5;
    _startTime = brain->get_clock()->now();

    // 处理避障, 如果需要避障, 则直接返回 success, 不执行踢球.
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
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

    // 发布运动指令
    double angle = brain->data->ball.yawToRobot;
    brain->client->crabWalk(angle, _speed);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Kick", rerun::TextLog(msg));
    };

    // 已经踢到球的话, 可以提前结束踢球动作
    bool enableAbort;
    brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;
    const double BALL_LOST_THRESHOLD = 1000;  // ms
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) // 球已经踢远
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD // 疑似丢球了
        )
    ) {
        log("ball moved, abort kick");
        return NodeStatus::SUCCESS;
    }

    // 没有踢到球, 则更新最小距离
    if (ballRange < _minRange) _minRange = ballRange;    

    // 处理避障, 如果需要避障, 则直接返回 success, 不执行踢球.
    bool avoidPushing;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    if (
        avoidPushing
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 估算踢球所需要的时间, 如果到了时间, 则认为踢球动作完成, 返回 success 退出
    double msecs = getInput<double>("min_msec_kick").value();
    double speed = getInput<double>("speed_limit").value();
    msecs = msecs + brain->data->ball.range / speed * 1000;
    if (brain->msecsSince(_startTime) > msecs) { // 完成踢球动作
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // else, 继续执行踢球动作. 如果还能看见球, 则修正方向
    if (brain->data->ballDetected) { 
        double angle = brain->data->ball.yawToRobot;
        double speed = getInput<double>("speed_limit").value();
        _speed += 0.1; // 使踢球速度逐渐增加, 以减少晃动
        speed = min(speed, _speed);
        brain->client->crabWalk(angle, speed);
    }

    return NodeStatus::RUNNING;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

NodeStatus StandStill::onStart()
{
    // 初始化 Node
    _startTime = brain->get_clock()->now();

    // 发布运动指令
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

    // else
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
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onStart");

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
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onRunning");

    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit;
    getInput("vyaw_limit", vyawLimit);

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    if (brain->data->ball.range < 0.3)
    { // 记忆中的球位置太近了, 后退一点
      // vx = cap(-brain->data->ball.posToRobot.x, 0.2, -0.2);
      // vy = cap(-brain->data->ball.posToRobot.y, 0.2, -0.2);
    }
    // vtheta = _turnDir > 0 ? vyawLimit : -vyawLimit;
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onHalted");
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
        _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle, false, false, true);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    // brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
    //     "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
    //     _angle, _cumAngle, deltaAngle, turnTime
    // )));
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

NodeStatus SelfLocate::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/SelfLocate", rerun::TextLog(msg));
    };
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    string mode = getInput<string>("mode").value();
    double xMin, xMax, yMin, yMax, thetaMin, thetaMax; // 结束条件
    auto markers = brain->data->getMarkersForLocator();

    // 计算约束条件
    if (mode == "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "trust_direction")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                      // m/s
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // 在这个时间内, odom 最多漂移了多少距离

        xMin = max(-brain->config->fieldDimensions.length / 2 - 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = min(brain->config->fieldDimensions.length / 2 + 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = max(-brain->config->fieldDimensions.width / 2 - 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = min(brain->config->fieldDimensions.width / 2 + 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }
    else if (mode == "fall_recovery")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                      // m/s
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // 在这个时间内, odom 最多漂移了多少距离

        xMin = -brain->config->fieldDimensions.length / 2 - 2;
        xMax = brain->config->fieldDimensions.length / 2 + 2;
        yMin = -brain->config->fieldDimensions.width / 2 - 2;
        yMax = brain->config->fieldDimensions.width / 2 + 2;
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }

    // TODO other modes

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    double residual;
    auto res = brain->locator->locateRobot(markers, constraints);

    brain->log->setTimeNow();
    string mstring = "";
    for (int i = 0; i < markers.size(); i++) {
        auto m = markers[i];
        mstring += format("type: %c  x: %.1f y: %.1f", m.type, m.x, m.y);
    }
    if (res.success) {
        
        brain->log->log(
            "field/recal",
            rerun::Arrows2D::from_vectors({{res.pose.x - brain->data->robotPoseToField.x, -res.pose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(res.success ? 0x00FF00FF : 0xFF0000FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"pf"})
        );
    }
    log(
        format(
            "success: %d  residual: %.2f  marker.size: %d  minMarkerCnt: %d  resTolerance: %.2f marker: %s",
            res.success,
            res.residual,
            markers.size(),
            brain->locator->minMarkerCnt,
            brain->locator->residualTolerance,
            mstring.c_str()
        )
    );
    
    // 定位失败
    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.

    // else 定位成功
    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("定位成功: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateEnterField::tick()
{
    auto log = [=](string msg, bool success) {
        brain->log->setTimeNow();
        brain->log->log("debug/SelfLocateEnterField", rerun::TextLog(msg).with_level(success? rerun::TextLogLevel::Info : rerun::TextLogLevel::Error));
    };
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    auto markers = brain->data->getMarkersForLocator();
    auto fd = brain->config->fieldDimensions;
    PoseBox2D cEnterLeft = {-fd.length / 2, -fd.circleRadius, fd.width / 2, fd.width / 2 + 1, -M_PI / 2 - M_PI / 6, -M_PI / 2 + M_PI / 6};
    PoseBox2D cEnterRight = {-fd.length / 2, -fd.circleRadius, -fd.width / 2 - 1, -fd.width / 2, M_PI / 2 - M_PI / 6, M_PI / 2 + M_PI / 6};


    auto resLeft = brain->locator->locateRobot(markers, cEnterLeft);
    auto resRight = brain->locator->locateRobot(markers, cEnterRight);
    LocateResult res;

    static string lastReport = "";
    string report = lastReport;
    if (resLeft.success && !resRight.success) {
        res = resLeft;
        report = "Entering Left";
    }
    else if (!resLeft.success && resRight.success) {
        res = resRight;
        report = "Entering Right";
    }
    else if (resLeft.success && resRight.success) {
        if (resLeft.residual < resRight.residual) {
            res = resLeft;
            report = "Entering Left";
        }
        else {
            res = resRight;
            report = "Entering Right";
        }
    } else res = resLeft;

    if (report != lastReport) {
        brain->speak(report);
        lastReport = report;
    }

    brain->log->setTimeNow();
    string logPath = res.success ? "debug/locator_enter_field/success" : "debug/locator_enter_field/fail";
    log(
            format(
                "%s left success: %d  left residual: %.2f  right success %d  right residual %.2f resTolerance: %.2f markers: %d minMarkerCnt: %d ",
                report.c_str(),
                resLeft.success, 
                resLeft.residual,
                resRight.success,
                resRight.residual,
                brain->locator->residualTolerance,
                markers.size(),
                brain->locator->minMarkerCnt
            ),
            res.success
        );

    brain->log->log(
        "field/recal_enter_field", 
        rerun::Arrows2D::from_vectors({{res.pose.x - brain->data->robotPoseToField.x, -res.pose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(res.success ? 0x00FF00FF: 0xFF0000FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"pfe"})
    );

    if (!res.success) return NodeStatus::SUCCESS; // 不阻塞后面的节点


    // else, 成功了.
    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("定位成功: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate1M::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/1m/success";
    string logPathF = "/locate/1m/fail";

    // 避免过于频繁定位
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    // find nearest marker
    int markerIndex = -1;
    GameObject marker;
    MapMarking mapMarker; 
    double minDist = 100;
    auto markings = brain->data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];

        // 排除掉容易误识别引起误判的点
        if (m.name == "LOLG" || m.name == "LORG" || m.name == "LSLG" || m.name == "LSRG") continue; 

        if (m.range < minDist) {
            minDist = m.range;
            markerIndex = i;
            marker = m;
        }
    }
    
    if (
        markerIndex < 0 || markerIndex >= markings.size()
        || marker.id < 0 || marker.id >= brain->config->mapMarkings.size()
    ) {
        log->log(logPathF, rerun::TextLog("Failed, No markings Found. Or marker id invalid."));
        return NodeStatus::SUCCESS;
    }
    mapMarker = brain->config->mapMarkings[marker.id];

    // 检查距离, 太远不用. (因为远了测距不准)
    if (marker.range > maxDist) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, min marker Dist(%.2f) > maxDist(%.2f)", marker.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    // 检查视野中的位置, 太偏的不用. (因为可能有扭曲, 导致测距有问题)
    if (!brain->isBoundingBoxInCenter(marker.boundingBox)) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, boundingbox is not in the center area"))
        );
        return NodeStatus::SUCCESS;
    }

    double dx, dy; // 偏移量, 理论 - 实际
    dx = mapMarker.x - marker.posToField.x;
    dy = mapMarker.y - marker.posToField.y;

    // 偏量太大, 不用. 可能是误识别导致的.
    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }
    
    // 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    brain->log->log(
        "field/recal/1m/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({marker.name})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2X::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/2x/success";
    string logPathF = "/locate/2x/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto points = brain->data->getMarkingsByType({"XCross"});
    if (points.size() != 2) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, point cnt(%d) != 2", points.size()))
        );
        return NodeStatus::SUCCESS;
    }

    auto p0 = points[0]; auto p1 = points[1];
    
    if (p0.range > maxDist || p1.range > maxDist) { // 太远
        log->log(logPathF,
            rerun::TextLog(format("Failed, p0 range (%.2f) or p1 range (%.2f) > maxDist(%.2f)", p0.range, p1.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    double xDist = fabs(p0.posToField.x - p1.posToField.x);
    if (xDist > 0.5) { // 方向不对
        log->log(logPathF,
            rerun::TextLog(format("Failed, xDist(%.2f) > maxDist(%.2f)", xDist, 0.5))
        );
        return NodeStatus::SUCCESS;
    }

    double yDist = fabs(p0.posToField.y - p1.posToField.y);
    double mapYDist = brain->config->fieldDimensions.circleRadius * 2.0;
    if (fabs(yDist - mapYDist) > 0.5) { // 距离不对
        log->log(logPathF,
            rerun::TextLog(format("Failed, yDist(%.2f) too far (%.2f) from mapYDist(%.2f)", yDist, 0.5, mapYDist))
        );
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值
    double dx = - (p0.posToField.x + p1.posToField.x) / 2.0;
    double dy = - (p1.posToField.y + p1.posToField.y) / 2.0;
    double drift = norm(dx, dy);

    if (drift > maxDrift) { // 修正量过大
        log->log(logPathF,
            rerun::TextLog(format("Failed, dirft(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/2x/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"1p"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2T::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/2t/success";
    string logPathF = "/locate/2t/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto markers = brain->data->getMarkingsByType({"TCross"});
    GameObject m1, m2;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < markers.size(); i++) {
        m1 = markers[i];
        
        if (m1.range > maxDist) continue; // 太远

        for (int j = i + 1; j < markers.size(); j++) {
            m2 = markers[j];

            if (m2.range > maxDist) continue; // 太远

            if (
                fabs(m1.posToField.x - m2.posToField.x) < 0.3
                && fabs(fabs(m1.posToField.y - m2.posToField.y) - fabs(fd.goalAreaWidth - fd.penaltyAreaWidth)/2.0)< 0.3
            ) {
                found = true;
                break;
            }
        }
        if (found) break;
    }


    if (!found) {
        log->log(logPathF, rerun::TextLog(format("Failed, No pattern within maxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        (m1.posToField.x + m2.posToField.x)/2,
        (m1.posToField.y + m2.posToField.y)/2
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0), 
                side * (fd.penaltyAreaWidth + fd.goalAreaWidth) / 4.0
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }

    if (!matched) {
        log->log(logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值, 理论 - 实际
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/2t/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateLT::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/lt/success";
    string logPathF = "/locate/lt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    auto lMarkers = brain->data->getMarkingsByType({"LCross"});

    GameObject t, l;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < tMarkers.size(); i++) {
        t = tMarkers[i];
        
        if (t.range > maxDist) continue; // 太远

        for (int j = i + 1; j < lMarkers.size(); j++) {
            l = lMarkers[j];

            if (l.range > maxDist) continue;

            if (
                fabs(t.posToField.y - l.posToField.y) < 0.3
                && fabs(fabs(t.posToField.x - l.posToField.x) - fd.goalAreaLength)< 0.3
            ) {
                found = true;
                break;
            }
        }

        if (found) break;
    }


    if (!found) {
        log->log(logPathF, rerun::TextLog(format("Failed, No pattern within MaxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        (t.posToField.x + l.posToField.x)/2,
        (t.posToField.y + l.posToField.y)/2
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0 - fd.goalAreaLength / 2.0), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        log->log(logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值, 理论 - 实际
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/lt/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocatePT::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/pt/success";
    string logPathF = "/locate/pt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto posts = brain->data->getGoalposts();
    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    
    GameObject p, t;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < posts.size(); i++) {
        p = posts[i];
        if (p.range > maxDist) continue;

        for (int j = i + 1; j < tMarkers.size(); j++) {
            t = tMarkers[j];
            if (t.range > maxDist) continue;
            if (
                fabs(t.posToField.x - p.posToField.x) < 0.5
                && fabs(fabs(t.posToField.x - p.posToField.x) - fabs(fd.goalAreaWidth - fd.goalWidth) / 2.0) < 0.3
            ) {
                found = true;
                break;
            }
        }
        
        if (found) break;
    }


    if (!found) {
        log->log(logPathF, rerun::TextLog(format("Failed, No pattern within maxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        t.posToField.x,
        t.posToField.y
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        log->log(logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值, 理论 - 实际
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/pt/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateBorder::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/border/success";
    string logPathF = "/locate/border/fail";

    // 避免过于频繁定位
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    // // 非静止状态线的定位不稳定
    // if (!brain->client->isStandingStill(1000)) {
    //     log->log(logPathF, rerun::TextLog(format("Failed, Not Standing Still")));
    //     return NodeStatus::SUCCESS;
    // }
    
    // find best touchline and best goalline
    bool touchLineFound = false;
    FieldLine touchLine;
    double bestConfidenceTouchline = 0.0;
    bool goalLineFound = false;
    FieldLine goalLine;
    double bestConfidenceGoalline = 0.0;
    bool middleLineFound = false;
    FieldLine middleLine;
    double bestConfidenceMiddleLine = 0.0;

    auto fieldLines = brain->data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (line.type != LineType::TouchLine && line.type != LineType::GoalLine && line.type != LineType::MiddleLine) continue;
        if (line.confidence < 0.8) continue;
        
        double dist = pointMinDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}), 
            line.posToField
        );
        if (dist > maxDist) continue;

        if (line.type == LineType::TouchLine) {
           if (line.confidence > bestConfidenceTouchline) {
               bestConfidenceTouchline = line.confidence;
               touchLine = line;
               touchLineFound = true;
           }
        } else if (line.type == LineType::GoalLine) {
            if (line.confidence > bestConfidenceGoalline) {
                bestConfidenceGoalline = line.confidence;
                goalLine = line;
                goalLineFound = true;
            }
        } else if (line.type == LineType::MiddleLine) {
            if (line.confidence > bestConfidenceMiddleLine) {
                bestConfidenceMiddleLine = line.confidence;
                middleLine = line;
                middleLineFound = true;
            }
        }
    }

    // 计算校正量
    double dx = 0; 
    double dy = 0; 
    auto fd = brain->config->fieldDimensions;
    if (touchLineFound) {
       double y_m = touchLine.side == LineSide::Left ? fd.width / 2.0 : - fd.width / 2.0;
       double perpDist = pointPerpDistToLine(
           Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
           touchLine.posToField
       );
       double y_o = touchLine.side == LineSide::Left ? 
           brain->data->robotPoseToField.y - perpDist :
           brain->data->robotPoseToField.y + perpDist;
       dy = y_m - y_o;
    }
    if (goalLineFound) {
        double x_m = goalLine.half == LineHalf::Opponent ? fd.length / 2.0: - fd.length / 2.0;
        double perpDist = pointPerpDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
            goalLine.posToField
        );
        double x_o = goalLine.half == LineHalf::Opponent?
            brain->data->robotPoseToField.x - perpDist :
            brain->data->robotPoseToField.x + perpDist;
        dx = x_m - x_o;
    } else if (middleLineFound) {
        double x_m = 0;
        auto linePos = middleLine.posToField;
        auto robotPose = brain->data->robotPoseToField;
        vector<double> pointA(2);
        vector<double> pointB(2);
        vector<double> pointR = {robotPose.x, robotPose.y};

        if (linePos.y0 > linePos.y1) {
            pointA = {linePos.x0, linePos.y0};
            pointB = {linePos.x1, linePos.y1};
        } else {
            pointA = {linePos.x1, linePos.y1};
            pointB = {linePos.x0, linePos.y0};
        }

        vector<double> vl = {pointB[0] - pointA[0], pointB[1] - pointA[1]};
        vector<double> vr = {pointR[0] - pointA[0], pointR[1] - pointA[1]};

        double normvl = norm(vl);
        double normvr = norm(vr);
        if (normvl < 1e-3 || normvr < 1e-3) {
            dx = 10000; // a large enough number that will certainly be bigger than max drift
        } else {
            double dist = crossProduct(vr, vl) / normvl;
            double x_o = robotPose.x + dist;
            dx = x_m - x_o;
        }
    }

    // 没找到
    if ((!touchLineFound && !goalLineFound && !middleLineFound)) {
        log->log(logPathF,
            rerun::TextLog("No touchline or goalline or middleLine found.")
        );
        return NodeStatus::SUCCESS;
    }

    // 偏量太大, 不用. 可能是误识别导致的.
    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }
    
    // 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    string label = "";
    if (touchLineFound) label += "TouchLine";
    if (touchLineFound && (goalLineFound || middleLineFound)) label += " ";
    if (goalLineFound) label += "GoalLine";
    if (middleLineFound) label += "MiddleLine";
    brain->log->log(
        "field/recal/border/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({label})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus MoveToPoseOnField::tick()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/Move", rerun::TextLog(msg));
    };
    log("Move ticked");

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
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToReadyPosition", rerun::TextLog(msg));
    };
    log("GoToReadyPosition ticked");

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
        vxLimit = 2.2;
        vyLimit = 1.2;
    }
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;

    if (role == "striker" && isKickoff) {
        tx = - max(fd.circleRadius, 1.5);
        ty = 0;
        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (brain->isPrimaryStriker()) {
                ty = 0.0;
            } else {
                ty = 0.0;
            }
        }
        
        // 5v5 开球逻辑
        if (brain->config->numOfPlayers == 5 && brain->data->liveCount >= 3)
        {
            int playerId = brain->config->playerId;
            if (brain->isPrimaryStriker()) {
                ty = 0.0;  // 主攻击手居中
            } else {
                switch (playerId) {
                    case 2: // 第二个机器人，左前方
                        ty = 1.8;
                        tx = - max(fd.circleRadius, 1.5) - 0.5;
                        break;
                    case 3: // 第三个机器人，右前方
                        ty = -1.8;
                        tx = - max(fd.circleRadius, 1.5) - 0.5;
                        break;
                    case 4: // 第四个机器人，左后方
                        ty = 1.2;
                        tx = - max(fd.circleRadius, 1.5) - 1.5;
                        break;
                    case 5: // 第五个机器人，右后方
                        ty = -1.2;
                        tx = - max(fd.circleRadius, 1.5) - 1.5;
                        break;
                    default:
                        ty = -1.5;
                        break;
                }
            }
        }
        ttheta = 0;
    } else if (role == "striker" && !isKickoff) {
        tx = - fd.circleRadius;
        ty = 0;
        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (brain->isPrimaryStriker()) {
                ty = 0.0;
            } else {
                ty = 0.0;
            }
        }
        
        // 5v5 非开球逻辑
        if (brain->config->numOfPlayers == 5 && brain->data->liveCount >= 3)
        {
            int playerId = brain->config->playerId;
            if (brain->isPrimaryStriker()) 
            {
                ty = 0.8;  // 主攻击手稍微偏移
            } 
            else 
            {
                switch (playerId) {
                    case 2: // 第二个机器人，左侧
                        ty = 1.5;
                        tx = - fd.circleRadius * 2 - 0.5;
                        break;
                    case 3: // 第三个机器人，右侧
                        ty = -1.5;
                        tx = - fd.circleRadius * 2 - 0.5;
                        break;
                    case 4: // 第四个机器人，左中场
                        ty = 0.8;
                        tx = - fd.circleRadius * 2 - 1.0;
                        break;
                    case 5: // 第五个机器人，右中场
                        ty = -0.8;
                        tx = - fd.circleRadius * 2 - 1.0;
                        break;
                    default:
                        ty = -0.6;
                        break;
                }
            }
        }
        ttheta = 0;
    } 
    else if (role == "goal_keeper") 
    {
        // 守门员位置
        int playerId = brain->config->playerId;
        if(playerId == 4)
        {
            tx = -6;
            ty = 2.5;
            ttheta = 0;

        }
        else
        {
            tx = -fd.length / 2.0 + fd.goalAreaLength;
            ty = 0;
            ttheta = 0;
        }

    }

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/GoBackInField", rerun::TextLog(msg));
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
    else { // 没出界
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 出界了, 往回走
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0, false, false, false);
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
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 1) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("reset recovery"));
        return NodeStatus::SUCCESS;
    }
    brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex)));

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        // brain->data->isRecoveryAvailable && // 倒了就直接尝试RL起身，（不需要关注是否recoveryAailable）
        brain->data->currentRobotModeIndex == 3 && // is damping
        brain->data->recoveryPerformedRetryCount < brain->get_parameter("recovery.retry_max_count").get_value<int>()) {
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->speak("Trying to stand up");
        brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount)));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->recoveryPerformed && brain->data->currentRobotModeIndex == 12) { // recover
        brain->data->recoveryPerformedRetryCount +=1;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog(format("Add retry count: %d", brain->data->recoveryPerformedRetryCount)));
    }

    // 机器人站着且是robocup步态，可以重置跌到爬起的状态
    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        brain->data->currentRobotModeIndex == 8) { // in robocup gait
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    return NodeStatus::SUCCESS;
}

/* ------------------------------------ 节点实现: 调试用 ------------------------------------*/

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

NodeStatus PlaySound::tick()
{
    string sound;
    getInput("sound", sound);
    bool allowRepeat;
    getInput("allow_repeat", allowRepeat);
    brain->playSound(sound, allowRepeat);
    return NodeStatus::SUCCESS;
}

NodeStatus Speak::tick()
{
    const string lastText;
    string text;
    getInput("text", text);
    if (text == lastText) return NodeStatus::SUCCESS;

    brain->speak(text, false);
    return NodeStatus::SUCCESS;
}
