# Interfaces-Server

Interfaces-Server 是 Humanoid Robot 系统的 gRPC 服务端实现，提供完整的接口服务和多线程回调架构。

## 核心特性

### 🚀 多线程回调架构
- **非阻塞通知**: 回调在独立的 detached 线程中执行
- **线程安全**: 订阅管理使用互斥锁保护
- **并发处理**: 支持多个订阅同时接收回调
- **自动清理**: 心跳监控自动清理失效订阅

### 📡 持久化订阅管理
- **长期连接**: 订阅建立后保持持久连接
- **心跳监控**: 每30秒检查订阅状态
- **自动超时**: 超过心跳间隔2倍时间自动清理
- **参数兼容**: 支持 `client_endpoint` 和 `callbackurl` 参数

### 🔧 完整的 gRPC 服务
- **Send**: 发送消息到服务器
- **Query**: 查询资源信息
- **Action**: 执行动作操作（支持流式响应）
- **Subscribe**: 订阅事件流（支持回调通知）
- **Unsubscribe**: 取消订阅

## 目录结构

```
Interfaces-Server/
├── include/
│   └── interfaces_server.h          # 主要服务器接口
├── source/
│   ├── interfaces_server.cpp        # 服务器实现
│   └── CMakeLists.txt               # 源码构建配置
├── examples/
│   ├── server_example.cpp           # 完整服务器示例
│   ├── simple_test.cpp              # 快速测试示例
│   └── CMakeLists.txt               # 示例构建配置
├── CMakeLists.txt                   # 主构建配置
└── README.md                        # 本文档
```

## 构建和运行

### 构建
```bash
# 在项目根目录下
mkdir build && cd build
cmake ..
make -j4
```

### 运行服务器
```bash
# 运行完整服务器示例
./build/Interfaces-Server/examples/server_example

# 运行快速测试
./build/Interfaces-Server/examples/simple_test
```

## 使用方式

### 启动服务器
```cpp
#include "interfaces_server.h"
using namespace humanoid_robot::server;

// 创建服务器实例
auto server = std::make_unique<InterfacesServer>();

// 启动服务器
if (server->Start("127.0.0.1:50051")) {
    std::cout << "Server started successfully!" << std::endl;
    server->Wait(); // 等待服务器关闭
}
```

### 订阅回调工作流程

1. **客户端发送订阅请求**
   ```
   Subscribe request with:
   - client_endpoint: "127.0.0.1:50052"
   - object_id: "robot_001" (可选)
   - event_types: ["status", "alerts"] (可选)
   ```

2. **服务器创建持久订阅**
   - 生成唯一订阅ID
   - 创建到客户端的 gRPC 连接
   - 存储到线程安全的订阅映射中

3. **多线程回调通知**
   - 在独立的 detached 线程中发送通知
   - 每个回调都在自己的线程中执行
   - 不会阻塞主服务或其他订阅

4. **自动清理和监控**
   - 心跳监控每30秒检查一次
   - 超时订阅自动移除
   - 线程安全的资源管理

## API 参考

### InterfacesServer

**主要方法:**
- `Start(server_address)` - 启动服务器
- `Stop()` - 停止服务器
- `Wait()` - 等待服务器关闭
- `GetService()` - 获取服务实例
- `SimulatePublishMessage(objectId, eventType, content)` - 模拟发布消息

### InterfaceServiceImpl

**gRPC 服务方法:**
- `Send()` - 处理发送请求
- `Query()` - 处理查询请求
- `Action()` - 处理动作请求（流式响应）
- `Subscribe()` - 处理订阅请求（创建持久订阅）
- `Unsubscribe()` - 处理取消订阅请求

**内部方法:**
- `PublishMessage()` - 发布消息到匹配订阅
- `SendSimulatedNotification()` - 发送模拟通知（测试用）
- `CheckHeartbeats()` - 检查心跳状态
- `StartHeartbeatMonitor()` - 启动心跳监控
- `StopHeartbeatMonitor()` - 停止心跳监控

## 配置选项

### 订阅参数
- **client_endpoint**: 客户端回调服务器地址（必需）
- **object_id**: 对象ID，用于消息过滤（可选）
- **event_types**: 事件类型列表，用于事件过滤（可选）
- **heartbeat_interval**: 心跳间隔，毫秒（可选，默认10000ms）

### 服务器配置
- **默认监听地址**: 127.0.0.1:50051
- **心跳检查间隔**: 30秒
- **订阅超时**: 心跳间隔的2倍时间
- **模拟通知数量**: 每个订阅发送10次测试消息

## 线程安全

### 订阅管理
- 所有订阅操作都有 `std::mutex` 保护
- 支持并发的订阅创建和删除
- 线程安全的订阅查找和遍历

### 回调处理
- 每个回调在独立的 detached 线程中执行
- 回调失败不会影响其他订阅
- 自动资源清理和错误隔离

## 依赖要求

- C++17 或更高版本
- gRPC 1.71.0+
- Protocol Buffers 5.29.3+
- 预编译的 interfaces protobuf 库

## 构建依赖

- `PB::interfaces` - 接口 protobuf 定义
- `gRPC::grpc++` - gRPC C++ 库
- `protobuf::libprotobuf` - Protocol Buffers 库

## 示例和测试

### server_example.cpp
完整的服务器示例，演示：
- 服务器启动和信号处理
- 多线程回调架构
- 持久订阅管理

### simple_test.cpp
快速测试示例，验证：
- 服务器启动和停止
- 基本功能正常工作
- 服务注册成功

## 最佳实践

1. **资源管理**: 确保正确启动和停止服务器
2. **错误处理**: 监控服务器状态和回调错误
3. **性能优化**: 
   - 合理设置心跳间隔
   - 避免创建过多订阅
   - 及时清理不需要的订阅
4. **线程安全**: 
   - 不要在外部直接操作订阅映射
   - 使用提供的 API 进行订阅管理
5. **监控和调试**:
   - 观察控制台输出了解服务器状态
   - 监控订阅数量和心跳状态
   - 使用日志记录回调执行情况

## 许可证

版权所有 (c) 2025 Humanoid Robot, Inc. 保留所有权利。

---

*最后更新: 2025年8月7日*
*版本: 多线程回调架构版本*
