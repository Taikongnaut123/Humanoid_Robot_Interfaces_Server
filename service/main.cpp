/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Interfaces Server Main - 独立进程运行的中间件服务器
 * 负责：
 * - 接收Client-SDK的请求
 * - 解析请求参数
 * - 转发给各个子模块(如perception_pipeline_cpp)
 * - 返回处理结果
 */

#include "Log/wlog.hpp"
#include "interfaces_server.h"
#include <iostream>
#include <memory>
#include <signal.h>

using namespace humanoid_robot::server;

// 全局服务器实例
std::unique_ptr<InterfacesServer> g_server;

// 信号处理函数
void SignalHandler(int signal) {
  WLOG_DEBUG("\nReceived signal %d, shutting down server...", signal);
  if (g_server) {
    g_server->Stop();
  }
}

int main(int argc, char **argv) {

  std::string config_path = "config/software.yaml";
  // 配置管理器
  std::unique_ptr<humanoid_robot::framework::common::ConfigManager> config_manager_;
  humanoid_robot::framework::common::ConfigNode loaded_config_ = config_manager_->LoadFromFile(config_path);
  std::string log_base_path = loaded_config_["software"]["general"]["sdk_server"]["log_base_path"];
  
  WLogSetPath(GetExeDir() + log_base_path);
  WLogInit();
  // 设置信号处理
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  WLOG_DEBUG("=== Humanoid Robot gRPC Interface Server ===");
  WLOG_DEBUG(
      "Multi-threaded callback architecture with persistent subscriptions");
  WLOG_DEBUG(
      "Acting as middleware between Client-SDK and perception_pipeline_cpp");

  try {
    // 创建服务器实例
    g_server = std::make_unique<InterfacesServer>();

    // 启动服务器 - 监听Client-SDK连接
    std::string server_address = loaded_config_["software"]["communication"]["grpc_server"]["server_address"];
    if (server_address.empty()) {
      WLOG_ERROR("server_address is empty");
      return 1;
    }
    if (!g_server->Start(server_address)) {
      WLOG_ERROR("Failed to start server!");
      return 1;
    }

    WLOG_INFO("🚀 Interfaces-Server ready! Client-SDK can connect now!");
    WLOG_INFO("📡 Listening on: %s", server_address.c_str());
    WLOG_INFO("🔗 Connected to perception_pipeline_cpp at localhost:50052");
    WLOG_INFO("🔧 Multi-threaded callback notifications enabled");
    WLOG_INFO("💬 Subscribe service creates persistent connections");
    WLOG_INFO("Press Ctrl+C to stop...");

    // 等待服务器关闭
    g_server->Wait();
  } catch (const std::exception &e) {
    WLOG_FATAL("Server error: %s", e.what());
    return 1;
  }
  // 停止日志模块
  WLogStop();
  WLOG_DEBUG("Interfaces-Server shutdown completed.");
  return 0;
}