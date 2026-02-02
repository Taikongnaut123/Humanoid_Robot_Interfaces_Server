/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Module Manager Implementation - 模块管理器实现
 */

#include "modules/module_manager.h"

#include <grpcpp/server_context.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "Log/wlog.hpp"
#include "common/service.pb.h"
#include "modules/navigation_module.h"
#include "modules/perception_module.h"
#include "perception/perception_request_response.pb.h"
#include "sdk_service/common/service.pb.h"

namespace humanoid_robot {
namespace server {
namespace modules {
using namespace humanoid_robot::PB::common;

using NavigationCommandCode =
    humanoid_robot::PB::sdk_service::common::NavigationCommandCode;

using PerceptionCommandCode =
    humanoid_robot::PB::sdk_service::common::PerceptionCommandCode;

ModuleManager::ModuleManager()
    : stop_check_thread_(false) // ********** 初始化原子变量 **********
{
  // auto perception_module = std::make_shared<PerceptionModule>();
  // modules_.insert({"Perception", std::move(perception_module)});
  // command_to_module_[PerceptionCommandCode::kGetPerceptionResult] =
  //     perception_module;
  // command_to_module_[PerceptionCommandCode::kGetDetectionResult] =
  //     perception_module;
  // command_to_module_[PerceptionCommandCode::kGetDivisionResult] =
  //     perception_module;

  auto navigation_module = std::make_shared<NavigationModule>();
  if (!navigation_module) {
    WLOG_FATAL("[ModuleManager] Failed to create NavigationModule");
  }
  modules_.insert({"Navigation", navigation_module});
  std::vector<NavigationCommandCode> navigation_commands = {
      NavigationCommandCode::kGetCurrentPose,
      NavigationCommandCode::kGetGridMap2D,
      NavigationCommandCode::kNavigationTo,
      NavigationCommandCode::kCancelNavigationTask,
      NavigationCommandCode::kGetRemainingPathDistance,
      NavigationCommandCode::kStartCharging,
      NavigationCommandCode::kStopCharging,
  };
  for (auto command : navigation_commands) {
    command_to_module_[command] = navigation_module;
  }

  // ********** 启动定时检查线程 **********
  check_thread_ = std::thread(&ModuleManager::ModulesRunningCheck, this);
  // std::cout << "[ModuleManager] All modules initialized" << std::endl;
  WLOG_DEBUG("[ModuleManager] All modules initialized");
}

ModuleManager::~ModuleManager() {
  // ********** 停止定时检查线程 **********
  stop_check_thread_ = true;
  if (check_thread_.joinable()) {
    check_thread_.join();
    WLOG_DEBUG("[ModuleManager] Module check thread stopped");
  }

  StopAllModules();
}

// ********** 定时检查循环函数（核心逻辑）**********
void ModuleManager::ModulesRunningCheck() {
  // 检查间隔：5秒
  // return;
  const std::chrono::seconds check_interval(5);

  while (!stop_check_thread_) {
    try {
      WLOG_DEBUG("[ModuleManager] Checking all modules running status...");

      for (const auto &[module_name, module] : modules_) {
        if (module) {
          if (!module->IsRunning()) {
            WLOG_WARN("[ModuleManager] %s is not running, restarting... ",
                      module_name.c_str());
            bool restart_ok = module->Restart(); // 调用Restart
            if (restart_ok) {
              WLOG_INFO("[ModuleManager] %s restarted successfully",
                        module_name.c_str());
            } else {
              WLOG_ERROR("[ModuleManager] Failed to restart %s",
                         module_name.c_str());
            }
          }
        }
      }

    } catch (const std::exception &e) {
      WLOG_ERROR("[ModuleManager] Error in module check loop: %s", e.what());
    }

    // 等待5秒，期间若stop_check_thread_变为true，会立即退出循环
    std::this_thread::sleep_for(check_interval);
  }
}

bool ModuleManager::StartAllModules() {
  WLOG_DEBUG("[ModuleManager] Starting all modules...");

  bool all_started = true;

  for (const auto &[module_name, module] : modules_) {
    std::cout << "[ModuleManager] Starting module " << module_name.c_str()
              << std::endl;
    WLOG_INFO("[ModuleManager] Starting module %s.", module_name.c_str());
    if (!module->Start()) {
      WLOG_WARN("[ModuleManager] Failed to start %s, it may be restarted in "
                "future.",
                module_name.c_str());
      all_started = false;
    }
    WLOG_INFO("[ModuleManager] %s module is started.", module_name.c_str());
  }
  if (all_started) {
    WLOG_DEBUG("[ModuleManager] All modules started successfully.");
  } else {
    WLOG_ERROR("[ModuleManager] Some modules failed to start.");
  }

  return all_started;
}

void ModuleManager::StopAllModules() {
  WLOG_DEBUG("[ModuleManager] Stopping all modules...");
  int retry_count = 0;
  const int max_retry = 3;
  bool all_stopped = true;
  for (const auto &[module_name, module] : modules_) {
    if (module) {
      while (module->IsRunning() && retry_count < max_retry) {
        module->Stop();
        WLOG_WARN("[ModuleManager] Retry stopping %s, attempt %d",
                  module_name.c_str(), retry_count + 1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        retry_count++;
      }
      if (!module->IsRunning()) {
        WLOG_INFO("[ModuleManager] %s stopped successfully",
                  module_name.c_str());
      } else {
        WLOG_ERROR("[ModuleManager] Failed to stop %s", module_name.c_str());
        all_stopped = false;
      }
    }
  }
  if (all_stopped) {
    WLOG_DEBUG("[ModuleManager] All modules stopped successfully");
  } else {
    WLOG_ERROR("[ModuleManager] Some modules failed to stop");
  }
}

bool ModuleManager::AreAllModulesRunning() const {
  bool all_running = true;
  for (const auto &[module_name, module] : modules_) {
    if (module) {
      if (!module->IsRunning()) {
        all_running = false;
        break;
      }
    }
  }
  return all_running;
}

ModuleResult ModuleManager::ExecuteCommand(
    grpc::ServerContext *&context, const int32_t &command_id,
    humanoid_robot::PB::common::Dictionary &input_data,
    humanoid_robot::PB::common::Dictionary &params, int32_t timeout_ms) {
  WLOG_DEBUG("[ModuleManager] Executing command: %d", command_id);

  auto module = GetModuleForCommand(command_id);

  if (!module) {
    return ModuleResult::Error("ModuleManager", command_id, -100,
                               "No module found for command: " +
                                   std::to_string(command_id));
  }
  return module->ExecuteSync(context, command_id, input_data, params,
                             timeout_ms);
}

std::future<ModuleResult> ModuleManager::ExecuteCommandAsync(
    grpc::ServerContext *&context, const int32_t &command_id,
    humanoid_robot::PB::common::Dictionary &input_data,
    humanoid_robot::PB::common::Dictionary &params) {
  WLOG_DEBUG("[ModuleManager] Executing command async: %d", command_id);

  auto module = GetModuleForCommand(command_id);

  if (!module) {
    std::promise<ModuleResult> promise;
    auto future = promise.get_future();
    // ********** 修复：std::to_string(command_id) 避免类型错误 **********
    promise.set_value(ModuleResult::Error("ModuleManager", command_id, -100,
                                          "No module found for command: " +
                                              std::to_string(command_id)));
    return future;
  }

  return module->ExecuteAsync(context, command_id, input_data, params);
}

std::unique_ptr<humanoid_robot::PB::common::Dictionary>
ModuleManager::GetModuleStatus(const std::string &module_name) const {
  auto it = modules_.find(module_name);
  if (it != modules_.end() && it->second) {
    return it->second->GetStatus();
  }
  return nullptr;
}

std::unique_ptr<humanoid_robot::PB::common::Dictionary>
ModuleManager::GetAllModulesStatus() const {
  auto status = std::make_unique<humanoid_robot::PB::common::Dictionary>();
  auto kv_map = status->mutable_keyvaluelist();

  for (const auto &[module_name, module] : modules_) {
    if (module) {
      auto module_status = module->GetStatus();
      if (module_status) {
        const auto &module_kv = module_status->keyvaluelist();
        for (const auto &[key, value] : module_kv) {
          kv_map->insert({module_name + "_" + key, value});
        }
      }
    }
  }

  return status;
}

std::shared_ptr<ModuleBase>
ModuleManager::GetModuleForCommand(const int32_t &command_id) const {
  WLOG_DEBUG("[ModuleManager] GetModuleForCommand: %d", command_id);
  auto it = command_to_module_.find(command_id);
  if (it != command_to_module_.end()) {
    WLOG_DEBUG("[ModuleManager] GetModuleForCommand: %d ", command_id);
    return it->second;
  }
  WLOG_DEBUG("[ModuleManager] GetModuleForCommand: %d -> nullptr", command_id);
  return nullptr;
}

std::unique_ptr<humanoid_robot::PB::common::Dictionary>
ModuleManager::CopyDictionary(
    const humanoid_robot::PB::common::Dictionary &source) const {
  auto copy = std::make_unique<humanoid_robot::PB::common::Dictionary>();
  *copy = source;
  return copy;
}

} // namespace modules
} // namespace server
} // namespace humanoid_robot