/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Decision Module - 决策模块
 * 处理路径规划、任务调度、策略选择等决策相关命令
 */

#ifndef HUMANOID_ROBOT_DECISION_MODULE_H
#define HUMANOID_ROBOT_DECISION_MODULE_H

#include "modules/module_base.h"

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            /**
             * 决策模块
             * 支持的命令：
             * - path_planning: 路径规划
             * - task_scheduling: 任务调度
             * - strategy_select: 策略选择
             * - risk_assessment: 风险评估
             * - goal_optimization: 目标优化
             */
            class DecisionModule : public ModuleBase
            {
            public:
                DecisionModule();
                virtual ~DecisionModule() = default;

            protected:
                bool Initialize() override;
                void Cleanup() override;

                ModuleResult ProcessCommand(
                    grpc::ServerContext *context, const int32_t &command_id,
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params) override;

            private:
                // 路径规划
                ModuleResult ProcessPathPlanning(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 任务调度
                ModuleResult ProcessTaskScheduling(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 策略选择
                ModuleResult ProcessStrategySelect(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 风险评估
                ModuleResult ProcessRiskAssessment(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 目标优化
                ModuleResult ProcessGoalOptimization(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 创建路径点列表
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> CreatePathResult(
                    const std::vector<std::pair<double, double>> &waypoints,
                    double total_distance,
                    double estimated_time) const;

                // 模拟决策处理延时
                void SimulateDecisionDelay(int min_ms = 200, int max_ms = 1000) const;
            };

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_DECISION_MODULE_H
