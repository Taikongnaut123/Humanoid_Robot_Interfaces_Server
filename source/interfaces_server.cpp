/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Interfaces Server 实现 - 支持持久订阅和多线程回调
 * 特性：
 * - 持久化订阅管理，支持心跳监控
 * - 多线程回调机制，避免阻塞主服务
 * - 线程安全的订阅存储和访问
 * - 支持多种参数名兼容性（client_endpoint/callbackurl）
 * - 基于命令ID的多模块架构支持
 */

#include "interfaces_server.h"
#include "Log/wlog.hpp"
#include "grpc/status.h"
#include "modules/module_manager.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;

using namespace humanoid_robot::PB::interfaces;
using namespace humanoid_robot::PB::common;
using namespace grpc;

namespace humanoid_robot {
namespace server {

// =============================================================================
// InterfaceServiceImpl 实现
// =============================================================================

InterfaceServiceImpl::InterfaceServiceImpl() : running_(false) {
  // 创建模块管理器
  module_manager_ = std::make_unique<modules::ModuleManager>();

  // 启动所有模块
  if (!module_manager_->StartAllModules()) {
    WLOG_ERROR("Failed to start all modules");
  } else {
    WLOG_INFO("All modules started successfully");
  }

  StartHeartbeatMonitor();
}

InterfaceServiceImpl::~InterfaceServiceImpl() {
  StopHeartbeatMonitor();

  // 停止所有模块
  if (module_manager_) {
    module_manager_->StopAllModules();
    WLOG_INFO("All modules stopped");
  }
}

grpc::Status InterfaceServiceImpl::Send(
    ::grpc::ServerContext *context,
    ::grpc::ServerReaderWriter<::humanoid_robot::PB::interfaces::SendResponse,
                               ::humanoid_robot::PB::interfaces::SendRequest>
        *stream) {
  WLOG_DEBUG("Send service called");

  if (stream == nullptr) {
    WLOG_ERROR("Stream is null");
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Stream is null");
  }
  // context->
  SendRequest request;
  SendRequest stable_request;
  // 读取请求数据
  if (stream->Read(&request)) {
    // 检查上下文是否被取消
    if (context->IsCancelled()) {
      return grpc::Status(grpc::StatusCode::DEADLINE_EXCEEDED,
                          "Stream timeout");
    }

    // 使用 swap 避免内存拷贝，确保数据稳定性
    stable_request = request;

    // 解析请求中的 command_id 和 data -
    // 这里采用拷贝的方法获取request中input的副本，避免多线程（grpc的线程）访问冲突
    auto &inputMap = stable_request.input().keyvaluelist();

    WLOG_DEBUG("Input map size: %d", (int)inputMap.size());
    WLOG_DEBUG("Request size: %d bytes", stable_request.ByteSize());

    int32_t command_id = -1;
    ::humanoid_robot::PB::common::Dictionary data_ref;

    // 使用稳定的map进行查找
    auto command_id_iter = inputMap.find("command_id");
    if (command_id_iter == inputMap.end()) {
      WLOG_ERROR("send input map not contain command_id");
      // 再次确认map内容
      WLOG_ERROR("Map contains %d entries:", (int)inputMap.size());
      for (const auto &pair : inputMap) {
        WLOG_ERROR("  Key: '%s'", pair.first.c_str());
      }
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                          "command_id not exist");
    }

    if (command_id_iter->second.value_case() !=
        ::humanoid_robot::PB::common::Variant::kInt32Value) {
      WLOG_ERROR(
          "send input map's command_id type is invalid, expect(%d), actual(%d)",
          static_cast<int>(::humanoid_robot::PB::common::Variant::kInt32Value),
          static_cast<int>(command_id_iter->second.value_case()));
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                          "command_id type is invalid");
    }
    command_id = command_id_iter->second.int32value();
    WLOG_DEBUG("Parsed command_id: %d", command_id);

    // 查找 data
    auto data_iter = inputMap.find("data");
    if (data_iter == inputMap.end()) {
      WLOG_ERROR("send input map not contain data");
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "data not exist");
    }

    if (!data_iter->second.has_dictvalue()) {
      WLOG_ERROR("data does not contain dictvalue");
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                          "data type invalid");
    }
    data_ref = data_iter->second.dictvalue();
    WLOG_DEBUG("Parsed data with %d entries", data_ref.keyvaluelist_size());

    auto params_ptr = stable_request.mutable_params();
    // 调用模块管理器执行命令
    auto result = module_manager_->ExecuteCommand(context, command_id, data_ref,
                                                  *params_ptr, 30000);
    SendResponse response;

    auto ret = response.mutable_ret();
    auto codeMsg = ret->mutable_code();
    *codeMsg = std::to_string(result.error_code);
    auto messageMsg = ret->mutable_message();
    *messageMsg = result.error_message.c_str();
    // 填充响应数据
    if (result.output_data) {

      response.mutable_output()->Swap(result.output_data.get());
    } else {
      WLOG_DEBUG("No output data from module, using default response");
      // 可选：设置默认/错误码
    }
    // 写入响应数据
    if (stream->Write(std::move(response))) {
      WLOG_DEBUG("Response sent successfully");
    } else {
      WLOG_ERROR("Failed to send response");
      return grpc::Status(grpc::StatusCode::UNKNOWN, "Failed to send response");
    }
  }

  return grpc::Status::OK;
}

grpc::Status InterfaceServiceImpl::Query(
    grpc::ServerContext *context,
    const humanoid_robot::PB::interfaces::QueryRequest *request,
    humanoid_robot::PB::interfaces::QueryResponse *response) {
  WLOG_DEBUG("Query service called");
  auto &inputMap = request->input().keyvaluelist();

  auto iter = inputMap.find("command_id");
  if (iter == inputMap.end()) {
    WLOG_ERROR("query input map not contain command_id");
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "command_id not exist");
  }
  if (iter->second.value_case() !=
      ::humanoid_robot::PB::common::Variant::kInt32Value) {
    WLOG_ERROR(
        "query input map's command_id type is invalid, expect(%d), actual(%d)",
        static_cast<int>(::humanoid_robot::PB::common::Variant::kInt32Value),
        static_cast<int>(iter->second.value_case()));
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "command_id type is invalid");
  }
  auto command_id = iter->second.int32value();
  iter = inputMap.find("data");
  if (iter == inputMap.end()) {
    WLOG_ERROR("query input map not contain data");
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "data not exist");
  }
  auto data = iter->second.dictvalue();
  auto params = request->params();
  // 调用模块管理器执行查询
  auto result =
      module_manager_->ExecuteCommand(context, command_id, data, params, 10000);
  // 填充响应数据
  auto outputMap = response->mutable_output()->mutable_keyvaluelist();
  for (const auto &pair : result.output_data->keyvaluelist()) {
    (*outputMap)[pair.first] = pair.second;
  }
  WLOG_DEBUG("Query service called end");
  return grpc::Status::OK;
}

grpc::Status InterfaceServiceImpl::Action(
    ::grpc::ServerContext *context,
    const humanoid_robot::PB::interfaces::ActionRequest *request,
    ::grpc::ServerWriter<::humanoid_robot::PB::interfaces::ActionResponse>
        *writer) {
  WLOG_DEBUG("Action service called");

  auto &inputMap = request->input().keyvaluelist();
  auto iter = inputMap.find("command_id");
  if (iter == inputMap.end()) {
    WLOG_ERROR("query input map not contain command_id");
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "command_id not exist");
  }
  if (iter->second.value_case() !=
      ::humanoid_robot::PB::common::Variant::kInt32Value) {
    WLOG_ERROR(
        "query input map's command_id type is invalid, expect(%d), actual(%d)",
        static_cast<int>(::humanoid_robot::PB::common::Variant::kInt32Value),
        static_cast<int>(iter->second.value_case()));
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "command_id type is invalid");
  }
  auto command_id = iter->second.int32value();
  iter = inputMap.find("data");
  if (iter == inputMap.end()) {
    WLOG_ERROR("query input map not contain data");
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "data not exist");
  }
  auto data = iter->second.dictvalue();

  auto params = request->params();
  // 调用模块管理器执行动作
  auto result =
      module_manager_->ExecuteCommand(context, command_id, data, params, 10000);

  WLOG_DEBUG("Action service called end");
  return grpc::Status::OK;
}

grpc::Status InterfaceServiceImpl::Subscribe(
    grpc::ServerContext *context,
    const humanoid_robot::PB::interfaces::SubscribeRequest *request,
    humanoid_robot::PB::interfaces::SubscribeResponse *response) {
  WLOG_DEBUG("Subscribe service called");

  std::string subscriptionId;
  // 检查是否有传入的订阅ID，如果没有则基于object_id生成
  auto &input_map = request->input().keyvaluelist();
  auto it = input_map.find("subscription_id");
  if (it != input_map.end()) {
    subscriptionId = it->second.stringvalue();
    WLOG_DEBUG("Using provided subscription ID: %s", subscriptionId.c_str());
  } else {
    // 基于object_id生成订阅ID
    it = input_map.find("object_id");
    if (it == input_map.end()) {
      WLOG_DEBUG("No object_id provided, using default subscription ID");
      subscriptionId = "sub_default_" + std::to_string(std::time(nullptr));
    } else {
      if (it->second.has_int32value()) {
        subscriptionId = "sub_" + std::to_string(it->second.int32value()) +
                         "_" + std::to_string(std::time(nullptr));
      } else if (it->second.has_stringvalue()) {
        subscriptionId = "sub_" + it->second.stringvalue() + "_" +
                         std::to_string(std::time(nullptr));
      } else {
        subscriptionId = "sub_default_" + std::to_string(std::time(nullptr));
      }
    }
    WLOG_DEBUG("Generated subscription ID: %s", subscriptionId.c_str());
  }
  std::vector<std::string> eventTypes;
  it = input_map.find("event_types");
  if (it == input_map.end()) {
    WLOG_DEBUG("No event types provided, using default empty list");
    eventTypes.push_back("default_event");
  } else {
    for (const auto &eventType : it->second.stringarrayvalue().values()) {
      eventTypes.push_back(eventType);
    }
  }

  std::string clientEndpoint;
  it = input_map.find("client_endpoint");
  if (it == input_map.end()) {
    // 尝试fallback到旧的参数名
    it = input_map.find("callbackurl");
    if (it == input_map.end()) {
      WLOG_ERROR("No client endpoint provided (tried client_endpoint and "
                 "callbackurl), cannot create subscription");
      clientEndpoint = "localhost:50052"; // 使用默认值
    } else {
      clientEndpoint = it->second.stringvalue();
      WLOG_DEBUG("Using fallback parameter callbackurl: %s",
                 clientEndpoint.c_str());
    }
  } else {
    clientEndpoint = it->second.stringvalue();
    WLOG_DEBUG("Using passed Client endpoint: %s", clientEndpoint.c_str());
  }

  int64_t heartbeatInterval = 0;
  it = input_map.find("heartbeat_interval");
  if (it == input_map.end()) {
    WLOG_DEBUG("No heartbeat interval provided, using default 10000 ms");
    heartbeatInterval = 10000; // 默认10秒
  } else {
    heartbeatInterval = it->second.int64value();
  }

  // 获取objectId用于订阅匹配
  std::string objectId = "default_object"; // 默认对象ID
  auto obj_it = input_map.find("object_id");
  if (obj_it != input_map.end()) {
    if (obj_it->second.has_int32value()) {
      objectId = "object_" + std::to_string(obj_it->second.int32value());
    } else if (obj_it->second.has_stringvalue()) {
      objectId = obj_it->second.stringvalue();
    }
  }

  auto subscription = std::make_unique<PersistentSubscription>(
      subscriptionId, objectId, clientEndpoint, eventTypes, heartbeatInterval);

  // 创建客户端stub
  auto channel =
      grpc::CreateChannel(clientEndpoint, grpc::InsecureChannelCredentials());
  subscription->clientStub = ClientCallbackService::NewStub(channel);

  // 存储订阅
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    subscriptions_[subscriptionId] = std::move(subscription);
  }
  WLOG_DEBUG("Persistent subscription created: %s", subscriptionId.c_str());

  // 返回订阅信息给客户端
  auto output_map = response->mutable_output()->mutable_keyvaluelist();
  {
    Variant var;
    var.set_stringvalue(subscriptionId);
    output_map->insert({"subscription_id", var});
  }
  {
    Variant var;
    var.set_stringvalue("Subscription created successfully");
    output_map->insert({"status_desc", var});
  }
  {
    Variant var;
    var.set_int32value(0); // 0表示成功
    output_map->insert({"status_code", var});
  }

  // 在独立线程中发送模拟通知，避免阻塞订阅请求
  std::thread([this, subscriptionId]() {
    std::this_thread::sleep_for(
        std::chrono::seconds(5)); // 等待5秒后发送模拟通知
    SendSimulatedNotification(subscriptionId);
  }).detach();
  return grpc::Status::OK;
}

grpc::Status InterfaceServiceImpl::Unsubscribe(
    grpc::ServerContext *context,
    const humanoid_robot::PB::interfaces::UnsubscribeRequest *request,
    humanoid_robot::PB::interfaces::UnsubscribeResponse *response) {
  WLOG_DEBUG("Unsubscribe service called for subscription");
  auto &input_map = request->input().keyvaluelist();
  auto output_map = response->mutable_output()->mutable_keyvaluelist();

  auto it = input_map.find("subscription_id");
  if (it == input_map.end()) {
    WLOG_DEBUG("No subscription_id provided, cannot unsubscribe");
    {
      Variant var;
      var.set_stringvalue("No subscription_id provided");
      output_map->insert({"status_desc", var});
    }
    {
      Variant var;
      var.set_int32value(-0600010001); // 错误码：缺少订阅ID参数
      output_map->insert({"status_code", var});
    }
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "No subscription_id provided");
  }
  std::string subscriptionId = it->second.stringvalue();
  // 移除持久订阅
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    auto it = subscriptions_.find(subscriptionId);
    if (it != subscriptions_.end()) {
      subscriptions_.erase(it);
      WLOG_DEBUG("Persistent subscription removed: %s", subscriptionId.c_str());
    }
  }
  {
    Variant var;
    var.set_stringvalue("Persistent subscription removed: " + subscriptionId);
    output_map->insert({"status_desc", var});
  }
  {
    Variant var;
    var.set_int32value(0); // 0表示成功
    output_map->insert({"status_code", var});
  }
  {
    Variant var;
    var.set_stringvalue("Unsubscribe service executed successfully");
    output_map->insert({"message", var});
  }
  return grpc::Status::OK;
}

void InterfaceServiceImpl::PublishMessage(
    const std::string &objectId, const std::string &eventType,
    const humanoid_robot::PB::interfaces::Notification &notification) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  for (const auto &[subId, subscription] : subscriptions_) {
    // 检查订阅是否匹配
    if (subscription->objectId == objectId) {
      // 检查事件类型
      if (subscription->eventTypes.empty() ||
          std::find(subscription->eventTypes.begin(),
                    subscription->eventTypes.end(),
                    eventType) != subscription->eventTypes.end()) {
        // 发送消息到客户端
        grpc::ClientContext context;
        NotificationAck response;

        auto status = subscription->clientStub->OnSubscriptionMessage(
            &context, notification, &response);

        if (status.ok()) {
          WLOG_DEBUG("Message published to subscription: %s", subId.c_str());
          subscription->lastHeartbeat = std::chrono::steady_clock::now();
        } else {
          WLOG_ERROR("Failed to publish message to subscription: %s Error: %s",
                     subId.c_str(), status.error_message().c_str());
        }
      }
    }
  }
}

void InterfaceServiceImpl::StartHeartbeatMonitor() {
  running_ = true;
  heartbeat_thread_ = std::thread([this]() {
    while (running_) {
      CheckHeartbeats();
      std::this_thread::sleep_for(std::chrono::seconds(30)); // 每30秒检查一次
    }
  });
}

void InterfaceServiceImpl::StopHeartbeatMonitor() {
  running_ = false;
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
}

void InterfaceServiceImpl::CheckHeartbeats() {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  auto now = std::chrono::steady_clock::now();

  for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
    auto &subscription = it->second;
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - subscription->lastHeartbeat)
                       .count();

    // 如果超过心跳间隔的2倍时间没有响应，认为客户端断开
    if (elapsed > subscription->heartbeatInterval * 2) {
      WLOG_DEBUG("Subscription timeout, removing: %s",
                 subscription->subscriptionId.c_str());
      it = subscriptions_.erase(it);
    } else {
      ++it;
    }
  }
}

void InterfaceServiceImpl::SendSimulatedNotification(
    const std::string &subscriptionId) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  auto it = subscriptions_.find(subscriptionId);
  if (it == subscriptions_.end()) {
    WLOG_ERROR("Subscription not found: %s", subscriptionId.c_str());
    return;
  }

  auto &subscription = it->second;
  if (!subscription->clientStub) {
    WLOG_ERROR("Client stub not available for subscription: %s",
               subscriptionId.c_str());
    return;
  }

  int sendCount = 0;
  while (sendCount < 10) // 发送10次模拟通知消息
  {
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒发送一次
    WLOG_DEBUG("Sending simulated notification for subscription: %s",
               subscriptionId.c_str());
    WLOG_DEBUG("Send count: %d", sendCount + 1);
    sendCount++;
    // 创建模拟通知消息
    Notification notification;
    auto message = notification.mutable_notifymessage()->mutable_keyvaluelist();

    {
      Variant var;
      var.set_stringvalue("subscription_callback : " +
                          std::to_string(sendCount));
      message->insert({"event_type", var});
    }
    {
      Variant var;
      var.set_stringvalue(subscriptionId);
      message->insert({"subscription_id", var});
    }
    {
      Variant var;
      var.set_stringvalue(
          "Hello from server! This is a simulated notification message.");
      message->insert({"message_content", var});
    }
    {
      Variant var;
      var.set_stringvalue("robot_001");
      message->insert({"object_id", var});
    }
    {
      Variant var;
      auto timestamp = var.mutable_timestampvalue();
      timestamp->set_seconds(std::chrono::system_clock::to_time_t(
          std::chrono::system_clock::now()));
      timestamp->set_nanos(0);
      message->insert({"timestamp", var});
    }

    // 发送通知到客户端
    grpc::ClientContext context;
    NotificationAck response;

    WLOG_DEBUG("Sending simulated notification to client endpoint: %s",
               subscription->clientEndpoint.c_str());

    grpc::Status status = subscription->clientStub->OnSubscriptionMessage(
        &context, notification, &response);

    if (status.ok()) {
      WLOG_DEBUG(
          "Successfully sent notification to client for subscription: %s",
          subscriptionId.c_str());
      WLOG_DEBUG("Client response return code: %d", response.ret());
    } else {
      WLOG_ERROR("Failed to send notification to client: %d - %s",
                 status.error_code(), status.error_message().c_str());
    }
  }
}

// =============================================================================
// InterfacesServer 实现 - 简化版本
// =============================================================================

InterfacesServer::InterfacesServer() {
  service_ = std::make_unique<InterfaceServiceImpl>();
  std::string config_path = "config/software.yaml";
  try {
    loaded_config_ = config_manager_->LoadFromFile(config_path);
    if (loaded_config_.IsEmpty()) {
      WLOG_ERROR("[InterfacesServer] Config file loaded but empty: %s", config_path.c_str());
    } else {
      WLOG_INFO("[InterfacesServer] Config file loaded successfully: %s", config_path.c_str());
    }
  } catch (const std::exception& e) {
    WLOG_ERROR("[InterfacesServer] Failed to load config file: %s, error: %s", config_path.c_str(), e.what());
    loaded_config_ = humanoid_robot::framework::common::ConfigNode(); // 初始化为空节点
  }
}

InterfacesServer::~InterfacesServer() { Stop(); }

bool InterfacesServer::Start(const std::string &server_address) {
  try {

    grpc::ServerBuilder builder;
    auto connection_config = loaded_config_["software"]["communication"]["grpc_server"]["connection"];
    int max_connection_idle_ms = std::stoi(connection_config["max_connection_idle_ms"]);
    builder.AddChannelArgument(GRPC_ARG_MAX_CONNECTION_IDLE_MS, max_connection_idle_ms);

    int max_connection_age_ms = std::stoi(connection_config["max_connection_age_ms"]);
    builder.AddChannelArgument(GRPC_ARG_MAX_CONNECTION_AGE_MS,
                               max_connection_age_ms); // 设置最大连接年龄为20分钟
    int keepalive_time_ms = std::stoi(connection_config["keepalive_time_ms"]);                          
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS,
                               keepalive_time_ms); // 设置keepalive时间为30秒
    int keepalive_timeout_ms = std::stoi(connection_config["keepalive_timeout_ms"]);                        
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS,
                               keepalive_timeout_ms); // 设置keepalive超时时e为10秒
    int max_pings_without_data = std::stoi(connection_config["max_pings_without_data"]);
    builder.AddChannelArgument(GRPC_ARG_HTTP2_MAX_PINGS_WITHOUT_DATA,
                               max_pings_without_data); // 设置最大无数据ping次数为5
    int min_recv_ping_interval_ms = std::stoi(connection_config["min_recv_ping_interval_ms"]);
    builder.AddChannelArgument(
        GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, min_recv_ping_interval_ms);

    auto thread_config = loaded_config_["software"]["communication"]["grpc_server"]["thread_pool"];
    int num_cqs = std::stoi(thread_config["num_cqs"]);
    builder.SetSyncServerOption(grpc::ServerBuilder::SyncServerOption::NUM_CQS,
                                num_cqs); // 设置4个工作线程
    int max_pollers = std::stoi(thread_config["max_pollers"]);                            
    builder.SetSyncServerOption(
        grpc::ServerBuilder::SyncServerOption::MAX_POLLERS, max_pollers); // 设置4个轮询器
    int min_pollers = std::stoi(thread_config["min_pollers"]);
    builder.SetSyncServerOption(
        grpc::ServerBuilder::SyncServerOption::MIN_POLLERS,
        min_pollers); // 设置最小轮询器为1
    int cq_timeout_ms = std::stoi(thread_config["cq_timeout_ms"]);
    builder.SetSyncServerOption(
        grpc::ServerBuilder::SyncServerOption::CQ_TIMEOUT_MSEC,
        cq_timeout_ms); // 设置CQ超时时间为10秒

    // 4. 消息大小限制
    auto message_size_config = loaded_config_["software"]["communication"]["grpc_server"]["message_size"];
    int max_receive_mb = std::stoi(message_size_config["max_receive_mb"]);
    builder.SetMaxReceiveMessageSize(max_receive_mb * 1024 * 1024); // 4MB
    int max_send_mb = std::stoi(message_size_config["max_send_mb"]);
    builder.SetMaxSendMessageSize(max_send_mb * 1024 * 1024);    // 4MB

    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    // 加上复杂配置，使用健康检查和反射
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(service_.get());
    // 启动gRPC服务器
    WLOG_DEBUG("Starting gRPC server at ");
    server_ = builder.BuildAndStart();

    if (server_) {
      WLOG_DEBUG("gRPC Server listening on %s", server_address.c_str());
      return true;
    } else {
      WLOG_ERROR("Failed to start gRPC server!");
      return false;
    }
  } catch (const std::exception &e) {
    WLOG_ERROR("Error starting server: %s", e.what());
    return false;
  }
}

void InterfacesServer::Stop() {
  if (server_) {
    WLOG_DEBUG("Shutting down gRPC server...");
    server_->Shutdown(std::chrono::system_clock::now() + std::chrono::seconds(2)); // 停止接受新连接
  }
}

void InterfacesServer::Wait() {
  if (server_) {
    server_->Wait();
  }
}

void InterfacesServer::SimulatePublishMessage(
    const std::string &objectId, const std::string &eventType,
    const std::string &messageContent) {
  // 创建通知消息，使用传入的参数而不是硬编码值
  Notification notification;
  auto message = notification.mutable_notifymessage()->mutable_keyvaluelist();
  {
    Variant var;
    var.set_stringvalue(objectId);
    message->insert({"objectid", var});
  }
  {
    Variant var;
    var.set_stringvalue(eventType);
    message->insert({"event_type", var});
  }
  {
    Variant var;
    var.set_stringvalue(messageContent);
    message->insert({"message_content", var});
  }
  {
    Variant var;
    auto timestamp = var.mutable_timestampvalue();
    timestamp->set_seconds(
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    timestamp->set_nanos(0);
    message->insert({"timestamp", var});
  }
  {
    Variant var;
    var.set_stringvalue(
        "msg_" +
        std::to_string(
            std::chrono::system_clock::now().time_since_epoch().count()));
    message->insert({"messageid", var});
  }

  WLOG_DEBUG("Publishing message for objectId: %s, eventType: %s, content: %s",
             objectId.c_str(), eventType.c_str(), messageContent.c_str());

  // 发布消息到匹配的订阅
  service_->PublishMessage(objectId, eventType, notification);
}

} // namespace server
} // namespace humanoid_robot
