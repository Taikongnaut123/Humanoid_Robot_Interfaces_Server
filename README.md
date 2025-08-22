# Interfaces-Server

Interfaces-Server 是 Humanoid Robot 系统的 gRPC 服务端实现，作为 Client-SDK 与 perception_pipeline_cpp 的中间件，支持多线程回调、持久订阅和高性能异步处理。

## 最新特性

- **独立进程运行**：支持通过 `SDK-Server` 单独启动服务进程，便于部署和调试
- **多模块架构**：集成决策、硬件、控制、导航、感知等模块，统一管理
- **异步管道**：与 perception_pipeline_cpp 后端异步通信，支持高并发推理
- **线程安全队列**：TSQueue 支持超时等待和高效数据流转
- **详细日志**：全链路 debug 输出，便于定位问题

## 目录结构

```
Interfaces-Server/
├── include/
│   └── interfaces_server.h          # 主要服务器接口
│   └── modules/                    # 各功能模块头文件
├── source/
│   ├── interfaces_server.cpp        # 服务器实现
│   └── modules/                    # 各功能模块实现
│   └── CMakeLists.txt               # 源码构建配置
├── service/
│   ├── main.cpp                    # 独立进程入口
│   └── CMakeLists.txt              # 服务进程构建配置
├── examples/
│   ├── server_example.cpp           # 完整服务器示例
│   ├── simple_test.cpp              # 快速测试示例
│   └── CMakeLists.txt               # 示例构建配置
├── CMakeLists.txt                   # 主构建配置
└── README.md                        # 本文档
```

## 构建与运行

### 构建所有模块
```bash
mkdir build && cd build
cmake ..
make -j4
```

### 启动独立服务进程
```bash
# 推荐方式，直接运行 SDK-Server
./build/Interfaces-Server/service/SDK-Server
```

### 启动示例/测试
```bash
./build/Interfaces-Server/examples/server_example
./build/Interfaces-Server/examples/simple_test
```

## 主要模块说明

- **main.cpp**：服务进程入口，支持信号处理和多线程回调
- **interfaces_server.h/cpp**：核心 gRPC 服务实现，负责请求分发、订阅管理、回调通知
- **modules/**：各功能模块（决策、感知、导航等），通过 ModuleManager 统一调度
- **TSQueue**：线程安全队列，支持超时 pop/push，保障异步管道数据流

## 常见问题与调试

- 启动时如遇 "Failed to connect to perception service at localhost:50052"，请先启动 perception_pipeline_cpp 后端
- 编译报错 `fatal error: interfaces_server.h: 没有那个文件或目录`，请确认 include 目录已加入 CMakeLists.txt
- 链接错误 `undefined reference to ModuleManager`，请确保 modules 源文件已加入 target_sources
- 日志输出可用于定位异步管道、回调线程等问题

## 依赖要求

- C++17 或更高版本
- gRPC 1.71.0+
- Protocol Buffers 5.29.3+
- OpenCV 4.5+
- Boost (circular_buffer)

## 进阶用法

- 支持多客户端并发订阅与回调
- 可扩展自定义模块，继承 ModuleBase 并注册到 ModuleManager
- 支持模拟消息发布与事件推送

## 许可证

版权所有 (c) 2025 Humanoid Robot, Inc. 保留所有权利。

---

*最后更新: 2025年8月21日*
*版本: 独立进程与多模块架构版*
