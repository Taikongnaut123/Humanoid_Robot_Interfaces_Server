/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Module Manager Implementation - 模块管理器实现
 */

#include "modules/module_manager.h"
#include "perception/perception_request_response.pb.h"
#include <iostream>
#include <grpcpp/server_context.h>
#include "Log/wlog.hpp"
using namespace humanoid_robot::PB::perception;

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            ModuleManager::ModuleManager()
            {
                // 创建所有模块实例
                perception_module_ = std::make_unique<PerceptionModule>();
                // decision_module_ = std::make_unique<DecisionModule>();
                // control_module_ = std::make_unique<ControlModule>();
                // navigation_module_ = std::make_unique<NavigationModule>();
                // hardware_module_ = std::make_unique<HardwareModule>();

                InitializeModuleMappings();
                InitializeCommandMappings();
            }

            ModuleManager::~ModuleManager()
            {
                StopAllModules();
            }

            bool ModuleManager::StartAllModules()
            {
                WLOG_DEBUG("[ModuleManager] Starting all modules...");

                bool all_started = true;

                // 启动感知模块
                if (perception_module_ && !perception_module_->Start())
                {
                    WLOG_ERROR("[ModuleManager] Failed to start Perception module");
                    all_started = false;
                }

                // TODO: 启动其他模块
                /*
                if (decision_module_ && !decision_module_->Start()) {
                    std::cerr << "[ModuleManager] Failed to start Decision module" );
                    all_started = false;
                }
                */

                if (all_started)
                {
                    WLOG_DEBUG("[ModuleManager] All modules started successfully");
                }
                else
                {
                    WLOG_ERROR("[ModuleManager] Some modules failed to start");
                }

                return all_started;
            }

            void ModuleManager::StopAllModules()
            {
                WLOG_DEBUG("[ModuleManager] Stopping all modules...");

                if (perception_module_)
                {
                    perception_module_->Stop();
                }

                // TODO: 停止其他模块

                WLOG_DEBUG("[ModuleManager] All modules stopped");
            }

            bool ModuleManager::AreAllModulesRunning() const
            {
                return perception_module_ && perception_module_->IsRunning();
                // TODO: 检查其他模块
            }

            ModuleResult ModuleManager::ExecuteCommand(
                grpc::ServerContext *&context, const int32_t &command_id,
                humanoid_robot::PB::common::Dictionary &input_data,
                humanoid_robot::PB::common::Dictionary &params,
                int32_t timeout_ms)
            {
                WLOG_DEBUG("[ModuleManager] Executing command: %d", command_id);

                ModuleBase *module = GetModuleForCommand(command_id);
                if (!module)
                {
                    return ModuleResult::Error("ModuleManager", command_id,
                                               -100, "No module found for command: " + command_id);
                }

                return module->ExecuteSync(context, command_id, input_data,
                                           params, timeout_ms);
            }

            std::future<ModuleResult> ModuleManager::ExecuteCommandAsync(
                grpc::ServerContext *&context, const int32_t &command_id,
                humanoid_robot::PB::common::Dictionary &input_data,
                humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[ModuleManager] Executing command async: %d", command_id);

                ModuleBase *module = GetModuleForCommand(command_id);
                if (!module)
                {
                    std::promise<ModuleResult> promise;
                    auto future = promise.get_future();
                    promise.set_value(ModuleResult::Error("ModuleManager", command_id,
                                                          -100, "No module found for command: " + command_id));
                    return future;
                }

                return module->ExecuteAsync(context, command_id, input_data, params);
            }

            std::unique_ptr<humanoid_robot::PB::common::Dictionary> ModuleManager::GetModuleStatus(const std::string &module_name) const
            {
                auto it = modules_.find(module_name);
                if (it != modules_.end() && it->second)
                {
                    return it->second->GetStatus();
                }
                return nullptr;
            }

            std::unique_ptr<humanoid_robot::PB::common::Dictionary> ModuleManager::GetAllModulesStatus() const
            {
                auto status = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto kv_map = status->mutable_keyvaluelist();

                for (const auto &[module_name, module] : modules_)
                {
                    if (module)
                    {
                        auto module_status = module->GetStatus();
                        if (module_status)
                        {
                            const auto &module_kv = module_status->keyvaluelist();
                            for (const auto &[key, value] : module_kv)
                            {
                                kv_map->insert({module_name + "_" + key, value});
                            }
                        }
                    }
                }

                return status;
            }

            std::unique_ptr<humanoid_robot::PB::common::Dictionary> ModuleManager::GetSupportedCommands() const
            {
                auto commands = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto kv_map = commands->mutable_keyvaluelist();

                // 感知模块命令
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_stringvalue("vision_detect,audio_analyze,tactile_sense,environment_scan,object_recognition");
                    kv_map->insert({"Perception_commands", var});
                }

                // TODO: 添加其他模块的命令

                return commands;
            }

            void ModuleManager::InitializeModuleMappings()
            {
                modules_["Perception"] = perception_module_.get();
                // TODO: 添加其他模块映射
            }

            void ModuleManager::InitializeCommandMappings()
            {
                // 感知模块命令映射
                command_to_module_[CommandCode::GET_PERCEPTION_RESULT] = perception_module_.get();
                command_to_module_[CommandCode::GET_DETECTION_RESULT] = perception_module_.get();
                command_to_module_[CommandCode::GET_DIVISION_RESULT] = perception_module_.get();

                // TODO: 添加其他模块的命令映射
            }

            ModuleBase *ModuleManager::GetModuleForCommand(const int32_t &command_id) const
            {
                auto it = command_to_module_.find(command_id);
                if (it != command_to_module_.end())
                {
                    return it->second;
                }
                return nullptr;
            }

            std::unique_ptr<humanoid_robot::PB::common::Dictionary> ModuleManager::CopyDictionary(const humanoid_robot::PB::common::Dictionary &source) const
            {
                auto copy = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                *copy = source;
                return copy;
            }

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot
