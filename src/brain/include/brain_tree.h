#pragma once

#include <tuple>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();

    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value;
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    /**
     * 初始化 blackboard 里的 entry，注意新加字段，在这里设置个默认值
     */
    void initEntry();
};

// ------------------------------- 比赛用 -------------------------------

// 计算踢球的角度
class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("cross_threshold", 0.2, "可进门的角度范围小于这个值时, 则传中")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// Striker 的比赛决策, 决定 Striker 在比赛中什么时候执行哪种技术动作
class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<string>("position", "offense", "offense | defense, 决定了向哪个方向踢球"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir; // 上次运行时 kickAngle 与当前 angle 的差
    rclcpp::Time timeLastTick; 
};

// Goal-keeper 的比赛决策, 决定其在比赛中什么时候执行哪种技术动作
class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "小于这个角度, 认为 adjust 已经成功"),
            InputPort<double>("adjust_y_tolerance", 0.1, "y 方向偏移小于这个值, 认为 y 方向 adjust 成功"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<double>("auto_visual_kick_enable_dist_min", 2.0, "自动视觉踢球启用时球的最小距离"),
            InputPort<double>("auto_visual_kick_enable_dist_max", 3.0, "自动视觉踢球启用时球的最大距离"),
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "自动视觉踢球启用时球的角度范围"),
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 3.0, "自动视觉踢球障碍物距离阈值，如果该距离内有障碍物，则不执行自动视觉踢球"),
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "自动视觉踢球障碍物在前方角度范围内的阈值，如果该角度内有障碍物，则不执行自动视觉踢球"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// CamTrackBall, 摄像头跟随球运行, 保持球位于画面的中心
class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override;

private:
    Brain *brain;
};

// CamFindBall, 没看到球时，尝试找到球. 注意, 之所以没有做成 Stateful Node, 是因为 CamFindBall 后通常还要执行其它动作
class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    // 找球的动作序列， 依次看向这几个位置
    rclcpp::Time _timeLastCmd;    // 上一次执行命令的时间，用于确保命令之间具有时间间隔
    int _cmdIndex;                // 当前执行到 cmdSequence 中的哪一步了
    long _cmdIntervalMSec;        // 执行动作序列的时间间隔，单位毫秒
    long _cmdRestartIntervalMSec; // 距离上一次执行超过这个时间，则重新从 0 开始执行序列

    Brain *brain;
    // TODO, 暴露这些参数
};

// 机器人执行找球的动作, 需要与 CamFindBall 配合使用
class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "转向的速度上限"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _turnDir; // 1.0 向左 -1.0 向右
    Brain *brain;
};

// 摄像头以 6 点式扫描球场一周
class CamFastScan : public StatefulActionNode
{
public:
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 300, "在同一个位置停留多少毫秒"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _cmdSequence[7][2] = {
        {0.45, 1.1},
        {0.45, 0.0},
        {0.45, -1.1},
        {1.0, -1.1},
        {1.0, 0.0},
        {1.0, 1.1},
        {0.45, 0.0},
    };    // 动作序列， 依次看向这几个位置
    rclcpp::Time _timeLastCmd;    // 上一次执行命令的时间，用于确保命令之间具有时间间隔
    int _cmdIndex = 0;                // 当前执行到 cmdSequence 中的哪一步了
    Brain *brain;
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("rad", 0, "转多少弧度, 向左为正"),
            InputPort<bool>("towards_ball", false, "为 true 时, 不考虑 rad 的正负号, 而是转向上一次看到不球的方向.")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _lastAngle; // 上个 tick 的弧度
    double _angle; // 转多少弧度
    double _cumAngle; // 共转了多少弧度
    double _msecLimit = 5000;  // 最多执行多少毫秒 (防止卡死)
    rclcpp::Time _timeStart; // 进入节点的时间 
    Brain *brain;
};

// 追球, 如果球在自己的后面, 会绕到球的后面
class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 1.2, "追球的最大 x 速度"),
            InputPort<double>("vy_limit", 0.4, "追球的最大 y 速度"),
            InputPort<double>("vtheta_limit", 1.5, "追球时, 实时调整方向的速度不大于这个值"),
            InputPort<double>("dist", 1.0, "追球的目标是球后面多少距离"),
            InputPort<double>("safe_dist", 1.0, "circle back 时, 保持的安全距离"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     // circl_back, chase;
    double _dir = 1.0; // 1.0 从右侧 circle back, -1.0 从右侧 circle back
};

// 已经接近球后, 调整到合适进攻或防御的踢球角度
class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 1.0, "球的角度大于这个值, 机器人先转身面向球, 直线运动先暂停"),
            InputPort<double>("vx_limit", 0.1, "调整过过程中 vx 的限制 [-limit, limit]"),
            InputPort<double>("vy_limit", 0.1, "调整过过程中 vy 的限制 [-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.4, "调整过过程中 vtheta 的限制 [-limit, limit]"),
            InputPort<double>("range", 1.5, "ball  range 保持这个值"),
            InputPort<double>("vtheta_factor", 1.5, "调整角度时, vtheta 的乘数, 越大转向越快"),
            InputPort<double>("tangential_speed_far", 0.5, "调整角度时, 较远时的切线速度"),
            InputPort<double>("tangential_speed_near", 0.2, "调整角度时, 较近时的切线速度"),
            InputPort<double>("near_threshold", 0.5, "距离目标小于这个值时, 使用 near speed"),
            InputPort<double>("no_turn_threshold", 0.1, "角度差小于这个值时, 不转身"),
            InputPort<double>("turn_first_threshold", 0.5, "角度差大于这个值时, 先转身, 不移动"),

        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// 执行踢球动作
class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_msec_kick", 500, "踢球动作最少执行多少毫秒"),
            InputPort<double>("msecs_stablize", 1000, "稳定多少毫秒"),
            InputPort<double>("speed_limit", 1.2, "速度最大值"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; // 开始时间
    string _state = "kick"; // stablize | kick
    int _msecKick = 1000;    // 在开始时, 根据距离估算执行踢球动作的的持续时间
    double _speed; // 用于平滑 
    double _minRange; // 用于判断是否已经踢到球了
    tuple<double, double, double> _calcSpeed();
};

// StandStill, 站立不动, 用于稳定机器人
class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("msecs", 1000, "站立多少毫秒"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; // 开始时间
};

// CamScanField, 视角画圈扫视, 先抬头向一个方向, 再低头向另一个方向扫视, 此为一圈
class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("low_pitch", 0.45, "向下看时的最大 pitch"),
            InputPort<double>("high_pitch", 0.15, "向上看时的最小 pitch"),
            InputPort<double>("left_yaw", 0.8, "向左看时的最大 yaw"),
            InputPort<double>("right_yaw", -0.8, "向右看时的最小 yaw"),
            InputPort<int>("msec_cycle", 4000, "多少毫秒转一圈"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// SelfLocate, 利用粒子滤波对当前的位置进行校正, 纠正里程计的漂移
class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("mode", "enter_field", "must be one of [trust_direction, face_forward, fall_recovery]"),
            // trust_direction: 正常情况下使用，此时 odom 信息大体上是准确的（未摔倒过）
            // face_forward: 面向对方球门的方向, 主要用于测试
            // fall_recovery: 摔倒后的恢复
            InputPort<double>("msecs_interval", 10000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
        };
    };

private:
    Brain *brain;
};

// SelfLocateEnterField, 特化的进场前的定位, 可自动判断左侧和右侧进场
class SelfLocateEnterField : public SyncActionNode
{
public:
    SelfLocateEnterField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
        };
    };

private:
    Brain *brain;
};

// SelfLocate1M, 用单个 Marker 进行校正
class SelfLocate1M : public SyncActionNode
{
public:
    SelfLocate1M(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "marker 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

// SelfLocate2X, 用两个 X Cross 进行位置校正
class SelfLocate2X : public SyncActionNode
{
public:
    SelfLocate2X(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "penalty point 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

// 底线上同侧的两个 TCross
class SelfLocate2T : public SyncActionNode
{
public:
    SelfLocate2T(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "两个 TCross 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

// Goalarea 的同侧一个 LCross 和 一个 TCross
class SelfLocateLT : public SyncActionNode
{
public:
    SelfLocateLT(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "penalty point 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

// 一个门柱和一个 TCross
class SelfLocatePT : public SyncActionNode
{
public:
    SelfLocatePT(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "penalty point 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

// SelfLocateBorder, 用边界线进行位置修正
class SelfLocateBorder : public SyncActionNode
{
public:
    SelfLocateBorder(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "border 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

// 移动到 Field 坐标系中的一个 Pose, 包含最终目标方向. 最好与 CamScanField 和 SelfLocate 同时使用, 以使最终位置较为准确
class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "目标 x 坐标, Field 坐标系"),
            InputPort<double>("y", 0, "目标 y 坐标, Field 坐标系"),
            InputPort<double>("theta", 0, "目标最终朝向, Field 坐标系"),
            InputPort<double>("long_range_threshold", 1.5, "目标点的距离超过这个值时, 优先走过去, 而不是细调位置和方向"),
            InputPort<double>("turn_threshold", 0.4, "长距离时, 目标点的方向超这个数值时, 先转向目标点"),
            InputPort<double>("vx_limit", 1.0, "x 限速"),
            InputPort<double>("vy_limit", 0.5, "y 限速"),
            InputPort<double>("vtheta_limit", 0.4, "theta 限速"),
            InputPort<double>("x_tolerance", 0.2, "x 容差"),
            InputPort<double>("y_tolerance", 0.2, "y 容差"),
            InputPort<double>("theta_tolerance", 0.1, "theta 容差"),
            InputPort<bool>("avoid_obstacle", false, "是否避障")
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoToReadyPosition : public SyncActionNode
{
public:
    GoToReadyPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.3, "x tolerance"), // 距离容差, 在距离容差范围内, 认为到达目标点
            InputPort<double>("theta_tolerance", 0.1, "theta tolerance"), // 角度容差, 在角度容差范围内, 认为到达目标点
            InputPort<double>("vx_limit", 2.5, "vx limit"), // x 方向速度限制, 最大速度
            InputPort<double>("vy_limit", 1.5, "vy limit"), // y 方向速度限制, 最大速度
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// 守门员移动到接近球门的合适位置
class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.5, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.1, "theta tolerance, winin which considered arrived."),
            InputPort<double>("vx_limit", 0.5, "x speed limit"),
            InputPort<double>("vy_limit", 0.5, "y speed limit"),
            InputPort<double>("dist_to_goalline", 1.0, "机器人站在门前多少距离"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// 助攻
class Assist : public SyncActionNode
{
public:
    Assist(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.5, "dist tolerance, within which considered arrived."), // 距离容差, 在距离容差范围内, 认为到达目标点
            InputPort<double>("theta_tolerance", 0.1, "theta tolerance, winin which considered arrived."), // 角度容差, 在角度容差范围内, 认为到达目标点
            InputPort<double>("vx_limit", 1.0, "x speed limit"), // x 方向速度限制, 最大速度
            InputPort<double>("vy_limit", 0.6, "y speed limit"), // y 方向速度限制, 最大速度
            InputPort<double>("dist_to_goalline", 1.0, "机器人站在门前多少距离"), // 机器人站在门前多少距离
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


/**
 * @brief 设置机器人的速度
 *
 * @param x,y,theta double, 机器人在 x，y 方向上的速度（m/s）和逆时针转动的角速度（rad/s), 默认值为 0. 全为 0 时，即相当于给出站立不动指令
 *
 */
class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};

// 原地踏步
class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {};
    }

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("action", "start", "start | stop"),
        };
    }

private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("pitch", 0, "target head pitch"),
            InputPort<double>("yaw", 0, "target head yaw"),
        };
    }

private:
    Brain *brain;
};

// 条件起身
class CheckAndStandUp : public SyncActionNode
{
public:
CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// 移动到任意球的发球位置, for striker
class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("side", "attack", "attack | defense"),
            InputPort<double>("attack_dist", 0.7, "attack side target dist to ball"),
            InputPort<double>("defense_dist", 1.9, "defense side target dist to ball"),
            InputPort<double>("vx_limit", 1.2, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),

        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    bool _isInFinalAdjust = false; // 是否在最后的调整阶段
};

// 回到场地内
class GoBackInField : public SyncActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("valve", 0.5, "回到场内距离边界多远可以停止"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// ------------------------------- FOR DEMO -------------------------------

// 用于演示跟着球跑, 不是比赛时使用的结点. 与 Chase 不同在于, Simple Chase 只是不断向球走, 而不会绕到球背后, 也因此不需要球场\定位\视频也可以运行
class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "在距离球多远的距离, 就不再走向球了"),
            InputPort<double>("stop_angle", 0.1, "球的角度在多少时, 就不再转向球了"),
            InputPort<double>("vy_limit", 0.2, "限制 Y 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 0.4"),
            InputPort<double>("vx_limit", 0.6, "限制 X 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 1.2"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// ------------------------------- FOR DEBUG -------------------------------

// 直接指定 Robot Pose on Field 的真值, 对 odom 进行校准
class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "x"),
            InputPort<double>("y", 0, "y"),
            InputPort<double>("theta", 0, "theta"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


// 向 cout 打印文字
class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("msg")};
    }

private:
    Brain *brain;
};

// 播放一些预定义的声音
class PlaySound : public SyncActionNode
{
public:
    PlaySound(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("sound", "cheerful", "播放声音的名称"),
            InputPort<bool>("allow_repeat", false, "是否允许重复播放同一个声音"),
        };
    }

private:
    Brain *brain;
};

// 使用本地 tts (espeak) 朗读文本
class Speak : public SyncActionNode
{
public:
    Speak(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("text", "", "朗读的文本内容, 必须是英文"),
        };
    }

private:
    Brain *brain;
};
