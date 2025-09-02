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

#include <iostream>
#include <signal.h>
#include <memory>
#include "interfaces_server.h"
#include "Log/wlog.hpp"

using namespace humanoid_robot::server;

// 全局服务器实例
std::unique_ptr<InterfacesServer> g_server;

// 信号处理函数
void SignalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ", shutting down server..." << std::endl;
    if (g_server)
    {
        g_server->Stop();
    }
}

int main(int argc, char **argv)
{
    WLogSetPath(GetExeDir() + "/SDK-Server/logs");
    WLogInit();
    // 设置信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    std::cout << "=== Humanoid Robot gRPC Interface Server ===" << std::endl;
    std::cout << "Multi-threaded callback architecture with persistent subscriptions" << std::endl;
    std::cout << "Acting as middleware between Client-SDK and perception_pipeline_cpp" << std::endl;

    try
    {
        // 创建服务器实例
        g_server = std::make_unique<InterfacesServer>();

        // 启动服务器 - 监听Client-SDK连接
        std::string server_address = "0.0.0.0:50051"; // 对外提供服务的地址
        if (!g_server->Start(server_address))
        {
            std::cerr << "Failed to start server!" << std::endl;
            return 1;
        }

        std::cout << "🚀 Interfaces-Server ready! Client-SDK can connect now!" << std::endl;
        std::cout << "📡 Listening on: " << server_address << std::endl;
        std::cout << "🔗 Connected to perception_pipeline_cpp at localhost:50052" << std::endl;
        std::cout << "🔧 Multi-threaded callback notifications enabled" << std::endl;
        std::cout << "💬 Subscribe service creates persistent connections" << std::endl;
        std::cout << "\nPress Ctrl+C to stop...\n"
                  << std::endl;

        // 等待服务器关闭
        g_server->Wait();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Server error: " << e.what() << std::endl;
        return 1;
    }
    // 停止日志模块
    WLogStop();
    std::cout << "Interfaces-Server shutdown completed." << std::endl;
    return 0;
}