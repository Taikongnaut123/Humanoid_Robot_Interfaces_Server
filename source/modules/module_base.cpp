/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Module Base Implementation - 模块基类实现
 */

#include "modules/module_base.h"
#include "Log/wlog.hpp"
#include <chrono>
#include <grpcpp/grpcpp.h>
#include <iostream>
#include <sstream>

namespace humanoid_robot {
namespace server {
namespace modules {

ModuleBase::ModuleBase(const std::string &module_name, size_t worker_threads)
    : module_name_(module_name), running_(false),
      worker_thread_count_(worker_threads), task_id_counter_(0) {}

ModuleBase::~ModuleBase() { Stop(); }

bool ModuleBase::Start() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (running_.load()) {
    return true; // 已经运行中
  }

  WLOG_DEBUG("[ %s ] Starting module...", module_name_.c_str());

  // 调用子类初始化
  if (!Initialize()) {
    WLOG_ERROR("[ %s ] Failed to initialize module", module_name_.c_str());
    return false;
  }

  running_.store(true);

  // 启动工作线程
  worker_threads_.reserve(worker_thread_count_);
  for (size_t i = 0; i < worker_thread_count_; ++i) {
    worker_threads_.emplace_back(&ModuleBase::WorkerThread, this);
  }

  WLOG_DEBUG("[%s] Module started with %zu worker threads",
             module_name_.c_str(), worker_thread_count_);
  return true;
}

void ModuleBase::Stop() {
  bool was_running = false;
  std::vector<std::thread> threads_to_join;
  
  // 第一步：获取必要的信息并设置停止标志，然后释放锁
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!running_.load()) {
      return; // 已经停止
    }
    
    was_running = true;
    running_.store(false);
    
    // 唤醒所有工作线程
    queue_condition_.notify_all();
    
    // 移动工作线程到局部变量，以便在锁外等待
    threads_to_join.swap(worker_threads_);
  }
  
  if (!was_running) {
    return;
  }
  
  WLOG_DEBUG("[%s] Stopping module...", module_name_.c_str());
  
  // 第二步：在锁外等待所有工作线程结束
  for (auto &thread : threads_to_join) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  
  // 第三步：清理剩余任务
  std::vector<std::unique_ptr<ModuleTask>> tasks_to_clean;
  {
    std::lock_guard<std::mutex> lock_queue(queue_mutex_);
    while (!task_queue_.empty()) {
      tasks_to_clean.push_back(std::move(task_queue_.front()));
      task_queue_.pop();
    }
  }
  
  // 第四步：在锁外处理任务结果
  for (auto &task : tasks_to_clean) {
    task->result_promise.set_value(ModuleResult::Error(
        module_name_, task->command_id, -1, "Module stopped"));
  }
  
  // 第五步：调用子类清理
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Cleanup();
  }

  WLOG_DEBUG("[%s] Module stopped", module_name_.c_str());
}

// 重启模块
bool ModuleBase::Restart() {
  WLOG_DEBUG("[%s] Restarting module...", module_name_.c_str());
  
  // 先等待模块停止 - Stop()已经是线程安全的，不会长时间持有锁
  Stop();
  
  // 短暂休眠，确保模块完全停止
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // 启动模块 - Start()已经是线程安全的
  return Start();
}

std::future<ModuleResult>
ModuleBase::ExecuteAsync(grpc::ServerContext *&context,
                         const int32_t &command_id,
                         humanoid_robot::PB::common::Dictionary &input_data,
                         humanoid_robot::PB::common::Dictionary &params) {
  if (!running_.load()) {
    std::promise<ModuleResult> promise;
    auto future = promise.get_future();
    promise.set_value(ModuleResult::Error(module_name_, command_id, -2,
                                          "Module not running"));
    return future;
  }

  auto task = std::make_unique<ModuleTask>(command_id, GenerateTaskId());
  task->input_data = input_data;
  task->params = params;
  auto future = task->result_promise.get_future();

  // 添加任务到队列
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.push(std::move(task));
  }
  queue_condition_.notify_one();

  return future;
}

ModuleResult ModuleBase::ExecuteSync(
    grpc::ServerContext *&context, const int32_t &command_id,
    humanoid_robot::PB::common::Dictionary &input_data,
    humanoid_robot::PB::common::Dictionary &params, int32_t timeout_ms) {
  auto future = ExecuteAsync(context, command_id, input_data, params);

  if (future.wait_for(std::chrono::milliseconds(timeout_ms)) ==
      std::future_status::timeout) {
    return ModuleResult::Error(module_name_, command_id, -3, "Command timeout");
  }

  return future.get();
}

std::unique_ptr<humanoid_robot::PB::common::Dictionary>
ModuleBase::GetStatus() const {
  auto status = std::make_unique<humanoid_robot::PB::common::Dictionary>();
  auto kv_map = status->mutable_keyvaluelist();

  // 模块名称
  {
    humanoid_robot::PB::common::Variant var;
    var.set_stringvalue(module_name_);
    kv_map->insert({"module_name", var});
  }

  // 运行状态
  {
    humanoid_robot::PB::common::Variant var;
    var.set_boolvalue(running_.load());
    kv_map->insert({"running", var});
  }

  // 工作线程数
  {
    humanoid_robot::PB::common::Variant var;
    var.set_int32value(static_cast<int32_t>(worker_thread_count_));
    kv_map->insert({"worker_threads", var});
  }

  // 队列中任务数
  {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(queue_mutex_));
    humanoid_robot::PB::common::Variant var;
    var.set_int32value(static_cast<int32_t>(task_queue_.size()));
    kv_map->insert({"pending_tasks", var});
  }

  return status;
}

void ModuleBase::WorkerThread() {
  WLOG_DEBUG("[%s] Worker thread started", module_name_.c_str());

  while (running_.load()) {
    std::unique_ptr<ModuleTask> task;

    // 从队列获取任务
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait(
          lock, [this] { return !task_queue_.empty() || !running_.load(); });

      if (!running_.load()) {
        break;
      }

      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());
        task_queue_.pop();
      }
    }

    if (task) {
      try {
        WLOG_DEBUG("[%s] Processing command: %d (task: %d)",
                   module_name_.c_str(), task->command_id, task->task_id);
        grpc::ServerContext context;
        // 执行命令
        ModuleResult result = ProcessCommand(&context, task->command_id,
                                             task->input_data, task->params);

        // 设置结果
        task->result_promise.set_value(std::move(result));
      } catch (const std::exception &e) {
        WLOG_FATAL("[%s] Exception in command %d: %s", module_name_.c_str(),
                   task->command_id, e.what());
        task->result_promise.set_value(
            ModuleResult::Error(module_name_, task->command_id, -4,
                                "Exception: " + std::string(e.what())));
      } catch (...) {
        WLOG_FATAL("[%s] Unknown exception in command %d", module_name_.c_str(),
                   task->command_id);
        task->result_promise.set_value(ModuleResult::Error(
            module_name_, task->command_id, -5, "Unknown exception"));
      }
    }
  }

  WLOG_DEBUG("[%s] Worker thread stopped", module_name_.c_str());
}

int32_t ModuleBase::GenerateTaskId() { return task_id_counter_.fetch_add(1); }

} // namespace modules
} // namespace server
} // namespace humanoid_robot