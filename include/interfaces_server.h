/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Interfaces Server - gRPC服务端实现
 * 特性：
 * - 多线程回调架构，支持非阻塞通知
 * - 持久化订阅管理，自动心跳监控
 * - 线程安全的订阅存储和访问控制
 * - 完整的CRUD操作和流式服务支持
 */

#ifndef SDK_SERVER_INTERFACES_SERVER_H
#define SDK_SERVER_INTERFACES_SERVER_H

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

// 引入proto生成的文件
#include "interfaces/interfaces_callback.grpc.pb.h"
#include "interfaces/interfaces_grpc.grpc.pb.h"
#include "interfaces/interfaces_request_response.pb.h"

// 引入模块管理器
#include "modules/module_manager.h"

namespace humanoid_robot {
namespace server {

/**
 * 持久订阅信息
 */
struct PersistentSubscription {
  std::string subscriptionId;
  std::string objectId;
  std::string clientEndpoint;
  std::vector<std::string> eventTypes;
  int64_t heartbeatInterval;
  std::chrono::steady_clock::time_point lastHeartbeat;
  std::unique_ptr<humanoid_robot::PB::interfaces::ClientCallbackService::Stub>
      clientStub;

  PersistentSubscription(const std::string &subId, const std::string &objId,
                         const std::string &endpoint,
                         const std::vector<std::string> &events,
                         int64_t interval)
      : subscriptionId(subId), objectId(objId), clientEndpoint(endpoint),
        eventTypes(events), heartbeatInterval(interval),
        lastHeartbeat(std::chrono::steady_clock::now()) {}
};

/**
 * InterfaceService服务实现类
 * 多线程回调架构，支持持久订阅和非阻塞通知
 */
class InterfaceServiceImpl final
    : public humanoid_robot::PB::interfaces::InterfaceService::Service {
public:
  InterfaceServiceImpl();
  ~InterfaceServiceImpl();

  // 同步操作
  grpc::Status Send(
      ::grpc::ServerContext *context,
      ::grpc::ServerReaderWriter<::humanoid_robot::PB::interfaces::SendResponse,
                                 ::humanoid_robot::PB::interfaces::SendRequest>
          *stream) override;

  grpc::Status
  Query(grpc::ServerContext *context,
        const humanoid_robot::PB::interfaces::QueryRequest *request,
        humanoid_robot::PB::interfaces::QueryResponse *response) override;

  grpc::Status
  Action(::grpc::ServerContext *context,
         const ::humanoid_robot::PB::interfaces::ActionRequest *request,
         ::grpc::ServerWriter<::humanoid_robot::PB::interfaces::ActionResponse>
             *writer) override;

  // 支持异步回调
  grpc::Status Subscribe(
      grpc::ServerContext *context,
      const humanoid_robot::PB::interfaces::SubscribeRequest *request,
      humanoid_robot::PB::interfaces::SubscribeResponse *response) override;

  grpc::Status Unsubscribe(
      grpc::ServerContext *context,
      const humanoid_robot::PB::interfaces::UnsubscribeRequest *request,
      humanoid_robot::PB::interfaces::UnsubscribeResponse *response) override;

public:
  /**
   * 向指定订阅发布消息
   * @param objectId 对象ID，用于匹配订阅
   * @param eventType 事件类型，用于过滤订阅
   * @param notification 要发送的通知消息
   */
  void PublishMessage(
      const std::string &objectId, const std::string &eventType,
      const humanoid_robot::PB::interfaces::Notification &notification);

private:
  std::unordered_map<std::string, std::unique_ptr<PersistentSubscription>>
      subscriptions_;
  std::mutex subscriptions_mutex_;
  std::thread heartbeat_thread_;
  bool running_;

  // 模块管理器
  std::unique_ptr<modules::ModuleManager> module_manager_;

  void StartHeartbeatMonitor();
  void StopHeartbeatMonitor();
  void CheckHeartbeats();

  /**
   * 发送模拟通知消息
   * 在独立线程中运行，发送10次测试消息到指定订阅
   * @param subscriptionId 订阅ID
   */
  void SendSimulatedNotification(const std::string &subscriptionId);
};

/**
 * gRPC服务器类
 */
class InterfacesServer {
public:
  InterfacesServer();
  ~InterfacesServer();

  // 启动服务器
  bool Start(const std::string &server_address = "127.0.0.1:50051");

  // 停止服务器
  void Stop();

  // 等待服务器关闭
  void Wait();

  // 获取服务实例（用于发布消息）
  InterfaceServiceImpl *GetService() { return service_.get(); }

  /**
   * 模拟发布消息到匹配的订阅
   * @param objectId 目标对象ID
   * @param eventType 事件类型
   * @param messageContent 消息内容
   */
  void SimulatePublishMessage(const std::string &objectId,
                              const std::string &eventType,
                              const std::string &messageContent);

private:
  std::unique_ptr<grpc::Server> server_;
  std::unique_ptr<InterfaceServiceImpl> service_;
  // 配置管理器
  std::unique_ptr<humanoid_robot::framework::common::ConfigManager> config_manager_;
  humanoid_robot::framework::common::ConfigNode loaded_config_;
};

} // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_INTERFACES_SERVER_H
