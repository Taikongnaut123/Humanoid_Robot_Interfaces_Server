/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Control Module - 控制模块
 * 处理运动控制、姿态调节、关节控制等控制相关命令
 */

#ifndef HUMANOID_ROBOT_CONTROL_MODULE_H
#define HUMANOID_ROBOT_CONTROL_MODULE_H

#include "modules/module_base.h"

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            /**
             * 控制模块
             * 支持的命令：
             * - motion_control: 运动控制
             * - posture_adjust: 姿态调节
             * - joint_control: 关节控制
             * - balance_control: 平衡控制
             * - force_control: 力控制
             */
            class ControlModule : public ModuleBase
            {
            public:
                ControlModule();
                virtual ~ControlModule() = default;

            protected:
                bool Initialize() override;
                void Cleanup() override;

                ModuleResult ProcessCommand(
                    grpc::ServerContext *context, const int32_t &command_id,
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params) override;

            private:
                // 运动控制
                ModuleResult ProcessMotionControl(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 姿态调节
                ModuleResult ProcessPostureAdjust(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 关节控制
                ModuleResult ProcessJointControl(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 平衡控制
                ModuleResult ProcessBalanceControl(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 力控制
                ModuleResult ProcessForceControl(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 创建控制状态结果
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> CreateControlResult(
                    const std::string &control_type,
                    const std::string &status,
                    const std::vector<std::pair<std::string, double>> &parameters) const;

                // 模拟控制执行延时
                void SimulateControlDelay(int min_ms = 50, int max_ms = 300) const;
            };

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_CONTROL_MODULE_H
