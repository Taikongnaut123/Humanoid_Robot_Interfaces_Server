#ifndef HUMANOID_ROBOT_CONTROL_MODULE_H
#define HUMANOID_ROBOT_CONTROL_MODULE_H

#include "modules/module_base.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <functional>
#include <mutex>
#include <vector>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "motor_interface/msg/motor_drive.hpp"
#include "motor_interface/msg/motor_feedback.hpp"

// 通信库头文件
#include "communication_v2/net/CNode.hpp"

namespace humanoid_robot {
namespace server {
namespace modules {

using TwistMsg = geometry_msgs::msg::Twist;
using MotorDriveMsg = motor_interface::msg::MotorDrive;
using MotorFeedbackMsg = motor_interface::msg::MotorFeedback;
using namespace humanoid_robot::framework::net;

// 通信接口抽象类（统一ROS2和自定义通信库的调用方式）
class CommunicationInterface {
public:
    virtual ~CommunicationInterface() = default;

    // 初始化通信模块
    virtual bool Initialize() = 0;

    // 清理通信资源
    virtual void Cleanup() = 0;

    // 发布零速度命令
    virtual void PublishZeroVelocity() = 0;

    // 发布关节控制命令
    virtual void PublishJointCommand(const std::vector<std::string>& joints,
                                     const std::vector<int32_t>& modes,
                                     const std::vector<double>& positions,
                                     const std::vector<double>& velocities,
                                     const std::vector<double>& torques) = 0;

    // 设置电机反馈回调函数
    using MotorFeedbackCallback = std::function<void(const MotorFeedbackMsg::SharedPtr)>;
    virtual void SetMotorFeedbackCallback(MotorFeedbackCallback callback) = 0;
};

// ROS2通信实现类
class ROS2Communication : public CommunicationInterface {
public:
    ROS2Communication();
    ~ROS2Communication() override;

    bool Initialize() override;
    void Cleanup() override;
    void PublishZeroVelocity() override;
    void PublishJointCommand(const std::vector<std::string>& joints,
                             const std::vector<int32_t>& modes,
                             const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             const std::vector<double>& torques) override;
    void SetMotorFeedbackCallback(MotorFeedbackCallback callback) override;

private:
    rclcpp::Node::SharedPtr node_;
    std::thread ros_spin_thread_;
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<MotorDriveMsg>::SharedPtr joint_motion_pub_;
    rclcpp::Subscription<MotorFeedbackMsg>::SharedPtr motor_feedback_sub_;
    MotorFeedbackCallback feedback_callback_;
    bool ros_initialized_;
};

// 自定义通信库实现类
class FrameworkCommunication : public CommunicationInterface {
public:
    FrameworkCommunication();
    ~FrameworkCommunication() override;

    bool Initialize() override;
    void Cleanup() override;
    void PublishZeroVelocity() override;
    void PublishJointCommand(const std::vector<std::string>& joints,
                             const std::vector<int32_t>& modes,
                             const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             const std::vector<double>& torques) override;
    void SetMotorFeedbackCallback(MotorFeedbackCallback callback) override;

private:
    std::shared_ptr<CNode> node_;
    std::shared_ptr<CPublisher<TwistMsg>> cmd_vel_pub_;
    std::shared_ptr<CPublisher<MotorDriveMsg>> joint_motion_pub_;
    std::shared_ptr<CSubscriber<MotorFeedbackMsg>> motor_feedback_sub_;
    MotorFeedbackCallback feedback_callback_;
    bool initialized_;
};

class ControlModule : public ModuleBase {
public:
    ControlModule();
    virtual ~ControlModule();

protected:
    bool Initialize() override;
    void Cleanup() override;

    ModuleResult ProcessCommand(
        grpc::ServerContext* context, const int32_t& command_id,
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params) override;

private:
    // 紧急停止方法
    ModuleResult EmergencyStopROS(
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params);

    ModuleResult GetJointInfoROS(
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params);

    ModuleResult JointMotionROS(
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params);

    // 发布零速度命令（转发给通信接口）
    void PublishZeroVelocity();

    // 发布关节控制命令（转发给通信接口）
    void PublishJointCommand(const std::vector<std::string>& joints,
                             const std::vector<int32_t>& modes,
                             const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             const std::vector<double>& torques);

    // 处理电机反馈数据
    void HandleMotorFeedback(const MotorFeedbackMsg::SharedPtr msg);

private:
    class ControlImpl;
    std::unique_ptr<ControlImpl> pImpl_;
    
    ControlModule(const ControlModule&) = delete;
    ControlModule& operator=(const ControlModule&) = delete;
};

// 控制命令定义
namespace ControlCommandCode {
    constexpr int32_t MOTION_PLAN = 2001;
    constexpr int32_t JOINT_CONTROL = 2002;
    constexpr int32_t POSTURE_ADJUST = 2003;
    constexpr int32_t GAIT_CONTROL = 2004;
    constexpr int32_t FORCE_CONTROL = 2005;
    constexpr int32_t EMERGENCY_STOP = 2006;
    constexpr int32_t GET_CONTROL_STATUS = 2007;
    constexpr int32_t GET_JOINT_INFO = 2008;    
    constexpr int32_t JOINT_MOTION = 2009;
}

namespace MotionMode {
    constexpr int32_t ABSOLUTE_POSITION = 1;    // 绝对位置模式
    constexpr int32_t RELATIVE_POSITION = 2;    // 相对位置模式
    constexpr int32_t VELOCITY = 3;             // 速度模式
    constexpr int32_t TORQUE = 4;               // 力矩模式
}

} // namespace modules
} // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_CONTROL_MODULE_H
