/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Module Manager - 模块管理器
 * 管理所有模块的生命周期和命令路由
 */

#ifndef HUMANOID_ROBOT_MODULE_MANAGER_H
#define HUMANOID_ROBOT_MODULE_MANAGER_H

#include "modules/module_base.h"
#include "modules/perception_module.h"
#include <grpcpp/server_context.h>

#include <unordered_map>
#include <memory>
#include <string>

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            /**
             * 模块管理器
             * 负责管理所有功能模块，根据命令ID路由到对应模块
             */
            class ModuleManager
            {
            public:
                ModuleManager();
                ~ModuleManager();

                /**
                 * 启动所有模块
                 */
                bool StartAllModules();

                /**
                 * 停止所有模块
                 */
                void StopAllModules();

                /**
                 * 检查所有模块是否正在运行
                 */
                bool AreAllModulesRunning() const;

                /**
                 * 根据命令ID执行命令
                 * @param command_id 命令ID
                 * @param input_data 输入数据
                 * @param params 参数
                 * @param timeout_ms 超时时间
                 * @return 执行结果
                 */
                ModuleResult ExecuteCommand(grpc::ServerContext *&context,
                                            const int32_t &command_id,
                                            humanoid_robot::PB::common::Dictionary &input_data,
                                            humanoid_robot::PB::common::Dictionary &params,
                                            int32_t timeout_ms = 30000);

                /**
                 * 异步执行命令
                 * @param command_id 命令ID
                 * @param input_data 输入数据
                 * @param params 参数
                 * @return 结果的future
                 */
                std::future<ModuleResult> ExecuteCommandAsync(grpc::ServerContext *&context,
                                                              const int32_t &command_id,
                                                              humanoid_robot::PB::common::Dictionary &input_data,
                                                              humanoid_robot::PB::common::Dictionary &params);

                /**
                 * 获取指定模块的状态
                 * @param module_name 模块名称
                 * @return 模块状态，如果模块不存在返回nullptr
                 */
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> GetModuleStatus(const std::string &module_name) const;

                /**
                 * 获取所有模块的状态
                 * @return 所有模块的状态信息
                 */
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> GetAllModulesStatus() const;

                /**
                 * 获取支持的命令列表
                 * @return 按模块分组的命令列表
                 */
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> GetSupportedCommands() const;

            private:
                // 模块实例
                std::unique_ptr<PerceptionModule> perception_module_;

                // 模块映射 (用于通过名称访问)
                std::unordered_map<std::string, ModuleBase *> modules_;

                // 命令到模块的映射
                std::unordered_map<int32_t, ModuleBase *> command_to_module_;

                /**
                 * 初始化模块映射
                 */
                void InitializeModuleMappings();

                /**
                 * 初始化命令映射
                 */
                void InitializeCommandMappings();

                /**
                 * 根据命令ID获取对应的模块
                 * @param command_id 命令ID
                 * @return 对应的模块，如果未找到返回nullptr
                 */
                ModuleBase *GetModuleForCommand(const int32_t &command_id) const;

                /**
                 * 复制Dictionary (用于传递参数)
                 */
                std::unique_ptr<humanoid_robot::PB::common::Dictionary> CopyDictionary(const humanoid_robot::PB::common::Dictionary &source) const;
            };

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_MODULE_MANAGER_H
