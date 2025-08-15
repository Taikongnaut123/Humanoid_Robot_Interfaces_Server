/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Navigation Module - 导航模块
 * 处理定位、地图构建、路径跟踪等导航相关命令
 */

#ifndef HUMANOID_ROBOT_NAVIGATION_MODULE_H
#define HUMANOID_ROBOT_NAVIGATION_MODULE_H

#include "modules/module_base.h"

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            /**
             * 导航模块
             * 支持的命令：
             * - localization: 定位
             * - mapping: 地图构建
             * - path_following: 路径跟踪
             * - obstacle_avoidance: 避障
             * - waypoint_navigation: 航点导航
             */
            class NavigationModule : public ModuleBase
            {
            public:
                NavigationModule();
                virtual ~NavigationModule() = default;

            protected:
                bool Initialize() override;
                void Cleanup() override;

                ModuleResult ProcessCommand(
                    grpc::ServerContext *context, const int32_t &command_id,
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params) override;

            private:
                // 定位
                ModuleResult ProcessLocalization(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 地图构建
                ModuleResult ProcessMapping(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 路径跟踪
                ModuleResult ProcessPathFollowing(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 避障
                ModuleResult ProcessObstacleAvoidance(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 航点导航
                ModuleResult ProcessWaypointNavigation(
                    const humanoid_robot::PB::common::Dictionary &input_data,
                    const humanoid_robot::PB::common::Dictionary &params);

                // 创建导航结果
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> CreateNavigationResult(
                    const std::string &nav_type,
                    double x, double y, double theta,
                    const std::string &status) const;

                // 模拟导航处理延时
                void SimulateNavigationDelay(int min_ms = 100, int max_ms = 800) const;
            };

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_NAVIGATION_MODULE_H
