/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * gRPC Server Example - 多线程回调架构演示
 * 特性：持久订阅、非阻塞回调、线程安全管理
 */

#include <iostream>
#include <signal.h>
#include <memory>
#include "interfaces_server.h"

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

int main()
{
    // 设置信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    std::cout << "=== Humanoid Robot gRPC Interface Server ===" << std::endl;
    std::cout << "Multi-threaded callback architecture with persistent subscriptions" << std::endl;

    try
    {
        // 创建服务器实例
        g_server = std::make_unique<InterfacesServer>();

        // 启动服务器
        std::string server_address = "127.0.0.1:50051";
        if (!g_server->Start(server_address))
        {
            std::cerr << "Failed to start server!" << std::endl;
            return 1;
        }

        std::cout << "🚀 Server ready! Client-SDK can connect now!" << std::endl;
        std::cout << "📡 Listening on: " << server_address << std::endl;
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

    std::cout << "Server shutdown completed." << std::endl;
    return 0;
}
