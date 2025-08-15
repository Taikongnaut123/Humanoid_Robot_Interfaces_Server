/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Hardware Module - 硬件模块
 * 处理传感器控制、电机控制、电源管理等硬件相关命令
 */

#ifndef HUMANOID_ROBOT_HARDWARE_MODULE_H
#define HUMANOID_ROBOT_HARDWARE_MODULE_H

#include "modules/module_base.h"

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            /**
             * 硬件模块
             * 支持的命令：
             * - sensor_control: 传感器控制
             * - motor_control: 电机控制
             * - power_management: 电源管理
             * - system_diagnostics: 系统诊断
             * - calibration: 硬件校准
             */
            class HardwareModule : public ModuleBase
            {
            public:
                HardwareModule();
                virtual ~HardwareModule() = default;

            protected:
                bool Initialize() override;
                void Cleanup() override;

                ModuleResult ProcessCommand(
                    grpc::ServerContext *context, const int32_t &command_id,
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params) override;

            private:
                // 传感器控制
                ModuleResult ProcessSensorControl(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 电机控制
                ModuleResult ProcessMotorControl(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 电源管理
                ModuleResult ProcessPowerManagement(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 系统诊断
                ModuleResult ProcessSystemDiagnostics(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 硬件校准
                ModuleResult ProcessCalibration(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 创建硬件状态结果
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> CreateHardwareResult(
                    const std::string &hardware_type,
                    const std::string &status,
                    const std::vector<std::pair<std::string, std::string>> &properties) const;

                // 模拟硬件操作延时
                void SimulateHardwareDelay(int min_ms = 10, int max_ms = 200) const;
            };

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_HARDWARE_MODULE_H
