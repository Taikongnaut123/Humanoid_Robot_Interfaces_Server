/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Module Base - 模块基类定义
 * 提供统一的模块接口和多线程支持
 */
#ifndef SDK_SERVER_MODULE_BASE_H
#define SDK_SERVER_MODULE_BASE_H

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "interfaces/interfaces_request_response.pb.h"

namespace humanoid_robot {
namespace server {
namespace modules {

/**
 * 模块执行结果
 */
struct ModuleResult {
  bool success;
  std::string error_message;
  int32_t error_code;
  std::string module_name;
  int32_t command_id;

  // 输出数据 (使用protobuf Dictionary格式)
  std::unique_ptr<humanoid_robot::PB::common::Dictionary> output_data;

  ModuleResult() : success(false), error_code(-1) {}

  static ModuleResult Success(
      const std::string &module, const int32_t &cmd,
      std::unique_ptr<humanoid_robot::PB::common::Dictionary> data = nullptr) {
    ModuleResult result;
    result.success = true;
    result.error_code = 0;
    result.module_name = module;
    result.command_id = cmd;
    result.output_data = std::move(data);
    return result;
  }

  static ModuleResult Error(const std::string &module, const int32_t &cmd,
                            int32_t code, const std::string &message) {
    ModuleResult result;
    result.success = false;
    result.error_code = code;
    result.error_message = message;
    result.module_name = module;
    result.command_id = cmd;
    return result;
  }
};

/**
 * 模块任务
 */
struct ModuleTask {
  int32_t command_id;
  int32_t task_id;
  humanoid_robot::PB::common::Dictionary input_data;
  humanoid_robot::PB::common::Dictionary params;
  std::promise<ModuleResult> result_promise;

  ModuleTask(const int32_t &cmd_id, const int32_t &t_id)
      : command_id(cmd_id), task_id(t_id) {}
};

/**
 * 模块基类
 * 提供统一的模块接口和多线程任务处理
 */
class ModuleBase {
public:
  ModuleBase(const std::string &module_name, size_t worker_threads = 2);
  virtual ~ModuleBase();

  /**
   * 启动模块
   */
  virtual bool Start();

  /**
   * 停止模块
   */
  virtual void Stop();

  /**
   * 重启模块
   */
  virtual bool Restart();

  /**
   * 检查模块是否运行中
   */
  virtual bool IsRunning() { return running_.load(); }

  /**
   * 获取模块名称
   */
  const std::string &GetModuleName() const { return module_name_; }

  /**
   * 异步执行命令
   * @param command_id 命令ID
   * @param input_data 输入数据
   * @param params 参数
   * @return 结果的future
   */
  std::future<ModuleResult>
  ExecuteAsync(grpc::ServerContext *&context, const int32_t &command_id,
               humanoid_robot::PB::common::Dictionary &input_data,
               humanoid_robot::PB::common::Dictionary &params);

  /**
   * 同步执行命令 (带超时)
   * @param command_id 命令ID
   * @param input_data 输入数据
   * @param params 参数
   * @param timeout_ms 超时时间(毫秒)
   * @return 执行结果
   */
  ModuleResult ExecuteSync(grpc::ServerContext *&context,
                           const int32_t &command_id,
                           humanoid_robot::PB::common::Dictionary &input_data,
                           humanoid_robot::PB::common::Dictionary &params,
                           int32_t timeout_ms = 30000);

  /**
   * 获取模块状态信息
   */
  virtual std::unique_ptr<humanoid_robot::PB::common::Dictionary>
  GetStatus() const;

protected:
  /**
   * 子类需要实现的命令处理函数
   * @param command_id 命令ID
   * @param input_data 输入数据
   * @param params 参数
   * @return 执行结果
   */
  virtual ModuleResult
  ProcessCommand(grpc::ServerContext *context, const int32_t &command_id,
                 const humanoid_robot::PB::common::Dictionary &input_data,
                 const humanoid_robot::PB::common::Dictionary &params) = 0;

  /**
   * 子类可选实现的初始化函数
   */
  virtual bool Initialize() { return true; }

  /**
   * 子类可选实现的清理函数
   */
  virtual void Cleanup() {}

private:
  std::string module_name_;
  std::atomic<bool> running_;
  size_t worker_thread_count_;

  // 任务队列
  std::queue<std::unique_ptr<ModuleTask>> task_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;

  // 工作线程
  std::vector<std::thread> worker_threads_;

  // 任务ID生成器
  std::atomic<uint64_t> task_id_counter_;

  /**
   * 工作线程函数
   */
  void WorkerThread();

  /**
   * 生成任务ID
   */
  int32_t GenerateTaskId();
};

} // namespace modules
} // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_MODULE_BASE_H
