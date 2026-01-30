#ifndef HUMANOID_ROBOT_CONTROL_MODULE_H
#define HUMANOID_ROBOT_CONTROL_MODULE_H

#include "modules/module_base.h"
#include <memory>
#include <vector>
#include <string>

namespace humanoid_robot {
namespace server {
namespace modules {

// 前向声明
class CommunicationInterface;

class ControlModule : public ModuleBase {
public:
    ControlModule();
    virtual ~ControlModule();

protected:
    bool Initialize() override;
    bool IsRunning() override;
    void Cleanup() override;

    ModuleResult ProcessCommand(
        grpc::ServerContext* context, const int32_t& command_id,
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params) override;

private:
    // 内部命令处理方法
    ModuleResult EmergencyStop(
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params);

    ModuleResult GetJointInfo(
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params);

    ModuleResult JointMotion(
        const humanoid_robot::PB::common::Dictionary& input_data,
        const humanoid_robot::PB::common::Dictionary& params);

    // 转发到通信接口的方法
    void PublishZeroVelocity();
    void PublishJointCommand(const std::vector<std::string>& joints,
                             const std::vector<int32_t>& modes,
                             const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             const std::vector<double>& torques);

    // 电机反馈处理
    void HandleMotorFeedback(const void* msg_ptr);  // 使用void*延迟类型绑定

private:
    class ControlImpl;
    std::unique_ptr<ControlImpl> pImpl_;
    
    ControlModule(const ControlModule&) = delete;
    ControlModule& operator=(const ControlModule&) = delete;
};

// 控制命令定义 - 使用匿名命名空间防止多重定义
// namespace ControlCommandCode {
//     constexpr int32_t MOTION_PLAN = 2001;
//     constexpr int32_t JOINT_CONTROL = 2002;
//     constexpr int32_t POSTURE_ADJUST = 2003;
//     constexpr int32_t GAIT_CONTROL = 2004;
//     constexpr int32_t FORCE_CONTROL = 2005;
//     constexpr int32_t EMERGENCY_STOP = 2006;
//     constexpr int32_t GET_CONTROL_STATUS = 2007;
//     constexpr int32_t GET_JOINT_INFO = 2008;    
//     constexpr int32_t JOINT_MOTION = 2009;
// }

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


