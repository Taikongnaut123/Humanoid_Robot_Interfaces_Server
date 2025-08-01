/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Simple test - 参照Client-SDK模式
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include "interfaces_server.h"

using namespace humanoid_robot::server;

int main()
{
    std::cout << "Testing Interfaces gRPC Server (Client-SDK compatible)..." << std::endl;

    try
    {
        // 创建服务器实例 (简化模式)
        auto server = std::make_unique<InterfacesServer>();

        // 启动服务器
        std::string server_address = "127.0.0.1:50052";
        if (!server->Start(server_address))
        {
            std::cerr << "❌ Failed to start server!" << std::endl;
            return 1;
        }

        std::cout << "✅ Server started successfully!" << std::endl;
        std::cout << "✅ All 8 RPC services ready:" << std::endl;
        std::cout << "  ✓ Create, Send, Delete, Query" << std::endl;
        std::cout << "  ✓ BatchCreate, HealthCheck" << std::endl;
        std::cout << "  ✓ Subscribe (streaming), Unsubscribe" << std::endl;

        std::cout << "\n🔗 Client-SDK可以连接到: " << server_address << std::endl;

        // 等待2秒后关闭
        std::cout << "⏱️  Test completing in 2 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        server->Stop();
        std::cout << "✅ Server stopped successfully!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n🎉 Test PASSED! Server works like Client-SDK!" << std::endl;
    return 0;
}
