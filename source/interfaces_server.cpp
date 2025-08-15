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
#include "modules/module_manager.h"
#include <iostream>
#include <chrono>
#include "Log/wlog.hpp"
#include <algorithm>
#include "grpc/status.h"

using namespace humanoid_robot::PB::interfaces;
using namespace humanoid_robot::PB::common;
using namespace grpc;

namespace humanoid_robot
{
    namespace server
    {

        // =============================================================================
        // InterfaceServiceImpl 实现
        // =============================================================================

        InterfaceServiceImpl::InterfaceServiceImpl() : running_(false)
        {
            // 创建模块管理器
            module_manager_ = std::make_unique<modules::ModuleManager>();

            // 启动所有模块
            if (!module_manager_->StartAllModules())
            {
                std::cerr << "Failed to start all modules" << std::endl;
            }
            else
            {
                std::cout << "All modules started successfully" << std::endl;
            }

            StartHeartbeatMonitor();
        }

        InterfaceServiceImpl::~InterfaceServiceImpl()
        {
            StopHeartbeatMonitor();

            // 停止所有模块
            if (module_manager_)
            {
                module_manager_->StopAllModules();
                std::cout << "All modules stopped" << std::endl;
            }
        }

        grpc::Status InterfaceServiceImpl::Send(::grpc::ServerContext *context,
                                                ::grpc::ServerReaderWriter<::humanoid_robot::PB::interfaces::SendResponse, ::humanoid_robot::PB::interfaces::SendRequest> *stream)
        {
            std::cout << "Send service called" << std::endl;

            if (stream == nullptr)
            {
                WLOG_ERROR("Stream is null");
                return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Stream is null");
            }
            SendRequest request;
            if (stream->Read(&request))
            {
                auto &inputMap = request.input().keyvaluelist();
                auto iter = inputMap.find("commandID");
                if (iter == inputMap.end())
                {
                    WLOG_ERROR("send input map not contain commandID");
                    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "commandID not exist");
                }
                if (iter->second.type() != humanoid_robot::PB::common::Variant_Type_KInt32Value)
                {
                    WLOG_ERROR("send input map's commandID type is invalid, expect(%d), actual(%d)",
                               humanoid_robot::PB::common::Variant_Type_KInt32Value, iter->second.type());
                    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "commandID type is invalid");
                }
                auto commandID = iter->second.int32value();
                iter = inputMap.find("data");
                if (iter == inputMap.end())
                {
                    WLOG_ERROR("send input map not contain data");
                    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "data not exist");
                }
                auto data = iter->second.dictvalue();
                auto params = request.params();
                auto result = module_manager_->ExecuteCommand(context, commandID, data,
                                                              params, 10000);
                SendResponse response;
                response.mutable_output()->CopyFrom(*result.output_data);
                stream->Write(std::move(response));
            }

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Query(grpc::ServerContext *context,
                                                 const humanoid_robot::PB::interfaces::QueryRequest *request,
                                                 humanoid_robot::PB::interfaces::QueryResponse *response)
        {
            // std::cout << "Query service called" << std::endl;

            // auto output_map = response->mutable_output()->mutable_keyvaluelist();

            // // 检查模块管理器是否可用
            // if (!module_manager_)
            // {
            //     {
            //         Variant var;
            //         var.set_stringvalue("Module manager not available");
            //         output_map->insert({"status_desc", var});
            //     }
            //     {
            //         Variant var;
            //         var.set_int32value(-1);
            //         output_map->insert({"status_code", var});
            //     }
            //     return grpc::Status::OK;
            // }

            // // 从请求中提取查询类型
            // const auto &input_map = request->input().keyvaluelist();
            // auto query_it = input_map.find("query_type");

            // if (query_it != input_map.end())
            // {
            //     std::string query_type = query_it->second.stringvalue();
            //     std::cout << "Query type: " << query_type << std::endl;

            //     if (query_type == "module_status")
            //     {
            //         // 查询模块状态
            //         auto module_name_it = input_map.find("module_name");
            //         if (module_name_it != input_map.end())
            //         {
            //             // 查询特定模块状态
            //             std::string module_name = module_name_it->second.stringvalue();
            //             auto status = module_manager_->GetModuleStatus(module_name);

            //             if (status)
            //             {
            //                 const auto &status_map = status->keyvaluelist();
            //                 for (const auto &[key, value] : status_map)
            //                 {
            //                     output_map->insert({key, value});
            //                 }
            //             }
            //             else
            //             {
            //                 {
            //                     Variant var;
            //                     var.set_stringvalue("Module not found: " + module_name);
            //                     output_map->insert({"status_desc", var});
            //                 }
            //                 {
            //                     Variant var;
            //                     var.set_int32value(-3);
            //                     output_map->insert({"status_code", var});
            //                 }
            //                 return grpc::Status::OK;
            //             }
            //         }
            //         else
            //         {
            //             // 查询所有模块状态
            //             auto all_status = module_manager_->GetAllModulesStatus();
            //             if (all_status)
            //             {
            //                 const auto &all_status_map = all_status->keyvaluelist();
            //                 for (const auto &[key, value] : all_status_map)
            //                 {
            //                     output_map->insert({key, value});
            //                 }
            //             }
            //         }
            //     }
            //     else if (query_type == "supported_commands")
            //     {
            //         // 查询支持的命令列表
            //         auto commands = module_manager_->GetSupportedCommands();
            //         if (commands)
            //         {
            //             const auto &commands_map = commands->keyvaluelist();
            //             for (const auto &[key, value] : commands_map)
            //             {
            //                 output_map->insert({key, value});
            //             }
            //         }
            //     }
            //     else
            //     {
            //         // 尝试将查询作为命令执行
            //         auto input_data = std::make_unique<Dictionary>();
            //         *input_data = request->input();

            //         auto params = std::make_unique<Dictionary>();
            //         *params = request->params();

            //         auto result = module_manager_->ExecuteCommand(context, query_type, std::move(input_data),
            //                                                       std::move(params), 10000);

            //         if (result.success && result.output_data)
            //         {
            //             const auto &module_output = result.output_data->keyvaluelist();
            //             for (const auto &[key, value] : module_output)
            //             {
            //                 output_map->insert({key, value});
            //             }
            //         }
            //         else
            //         {
            //             {
            //                 Variant var;
            //                 var.set_stringvalue("Unknown query type: " + query_type);
            //                 output_map->insert({"status_desc", var});
            //             }
            //             {
            //                 Variant var;
            //                 var.set_int32value(-2);
            //                 output_map->insert({"status_code", var});
            //             }
            //             return grpc::Status::OK;
            //         }
            //     }
            // }
            // else
            // {
            //     // 默认返回系统状态
            //     auto all_status = module_manager_->GetAllModulesStatus();
            //     if (all_status)
            //     {
            //         const auto &all_status_map = all_status->keyvaluelist();
            //         for (const auto &[key, value] : all_status_map)
            //         {
            //             output_map->insert({key, value});
            //         }
            //     }
            // }

            // // 成功响应
            // {
            //     Variant var;
            //     var.set_stringvalue("Query executed successfully");
            //     output_map->insert({"status_desc", var});
            // }
            // {
            //     Variant var;
            //     var.set_int32value(0);
            //     output_map->insert({"status_code", var});
            // }

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Action(::grpc::ServerContext *context,
                                                  const humanoid_robot::PB::interfaces::ActionRequest *request,
                                                  ::grpc::ServerWriter<::humanoid_robot::PB::interfaces::ActionResponse> *writer)
        {
            std::cout << "Support Action service" << std::endl;

            for (int i = 0; i < 3; ++i)
            {
                humanoid_robot::PB::interfaces::ActionResponse response;
                auto output = response.mutable_output()->mutable_keyvaluelist();
                {
                    Variant var;
                    var.set_int32value(i + 1); // 示例使用整数值
                    output->insert({"action_id", var});
                }
                {
                    Variant var;
                    var.set_stringvalue("action " + std::to_string(i + 1)); // 示例使用字符串值
                    output->insert({"action_name", var});
                }
                {
                    Variant var;
                    var.set_stringvalue("Action service executed successfully - action " + std::to_string(i + 1));
                    output->insert({"message", var});
                }

                if (!writer->Write(response))
                {
                    break;
                }
            }

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Subscribe(grpc::ServerContext *context,
                                                     const humanoid_robot::PB::interfaces::SubscribeRequest *request,
                                                     humanoid_robot::PB::interfaces::SubscribeResponse *response)
        {
            std::cout << "Subscribe service called" << std::endl;

            std::string subscriptionId;
            // 检查是否有传入的订阅ID，如果没有则基于object_id生成
            auto &input_map = request->input().keyvaluelist();
            auto it = input_map.find("subscription_id");
            if (it != input_map.end())
            {
                subscriptionId = it->second.stringvalue();
                std::cout << "Using provided subscription ID: " << subscriptionId << std::endl;
            }
            else
            {
                // 基于object_id生成订阅ID
                it = input_map.find("object_id");
                if (it == input_map.end())
                {
                    std::cout << "No object_id provided, using default subscription ID" << std::endl;
                    subscriptionId = "sub_default_" + std::to_string(std::time(nullptr));
                }
                else
                {
                    if (it->second.has_int32value())
                    {
                        subscriptionId = "sub_" + std::to_string(it->second.int32value()) + "_" + std::to_string(std::time(nullptr));
                    }
                    else if (it->second.has_stringvalue())
                    {
                        subscriptionId = "sub_" + it->second.stringvalue() + "_" + std::to_string(std::time(nullptr));
                    }
                    else
                    {
                        subscriptionId = "sub_default_" + std::to_string(std::time(nullptr));
                    }
                }
                std::cout << "Generated subscription ID: " << subscriptionId << std::endl;
            }
            std::vector<std::string> eventTypes;
            it = input_map.find("event_types");
            if (it == input_map.end())
            {
                std::cout << "No event types provided, using default empty list" << std::endl;
                eventTypes.push_back("default_event");
            }
            else
            {
                for (const auto &eventType : it->second.stringarrayvalue().values())
                {
                    eventTypes.push_back(eventType);
                }
            }

            std::string clientEndpoint;
            it = input_map.find("client_endpoint");
            if (it == input_map.end())
            {
                // 尝试fallback到旧的参数名
                it = input_map.find("callbackurl");
                if (it == input_map.end())
                {
                    std::cerr << "No client endpoint provided (tried client_endpoint and callbackurl), cannot create subscription" << std::endl;
                    clientEndpoint = "localhost:50052"; // 使用默认值
                }
                else
                {
                    clientEndpoint = it->second.stringvalue();
                    std::cout << "Using fallback parameter callbackurl: " << clientEndpoint << std::endl;
                }
            }
            else
            {
                clientEndpoint = it->second.stringvalue();
                std::cout << "using passed Client endpoint: " << clientEndpoint << std::endl;
            }

            int64_t heartbeatInterval = 0;
            it = input_map.find("heartbeat_interval");
            if (it == input_map.end())
            {
                std::cout << "No heartbeat interval provided, using default 10000 ms" << std::endl;
                heartbeatInterval = 10000; // 默认10秒
            }
            else
            {
                heartbeatInterval = it->second.int64value();
            }

            // 获取objectId用于订阅匹配
            std::string objectId = "default_object"; // 默认对象ID
            auto obj_it = input_map.find("object_id");
            if (obj_it != input_map.end())
            {
                if (obj_it->second.has_int32value())
                {
                    objectId = "object_" + std::to_string(obj_it->second.int32value());
                }
                else if (obj_it->second.has_stringvalue())
                {
                    objectId = obj_it->second.stringvalue();
                }
            }

            auto subscription = std::make_unique<PersistentSubscription>(
                subscriptionId, objectId, clientEndpoint,
                eventTypes, heartbeatInterval);

            // 创建客户端stub
            auto channel = grpc::CreateChannel(clientEndpoint, grpc::InsecureChannelCredentials());
            subscription->clientStub = ClientCallbackService::NewStub(channel);

            // 存储订阅
            {
                std::lock_guard<std::mutex> lock(subscriptions_mutex_);
                subscriptions_[subscriptionId] = std::move(subscription);
            }
            std::cout << "Persistent subscription created: " << subscriptionId << std::endl;

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
            std::thread([this, subscriptionId]()
                        {
                std::this_thread::sleep_for(std::chrono::seconds(5)); // 等待5秒后发送模拟通知
                SendSimulatedNotification(subscriptionId); })
                .detach();
            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Unsubscribe(grpc::ServerContext *context,
                                                       const humanoid_robot::PB::interfaces::UnsubscribeRequest *request,
                                                       humanoid_robot::PB::interfaces::UnsubscribeResponse *response)
        {
            std::cout << "Unsubscribe service called for subscription" << std::endl;
            auto &input_map = request->input().keyvaluelist();
            auto output_map = response->mutable_output()->mutable_keyvaluelist();

            auto it = input_map.find("subscription_id");
            if (it == input_map.end())
            {
                std::cout << "No subscription_id provided, cannot unsubscribe" << std::endl;
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
                return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "No subscription_id provided");
            }
            std::string subscriptionId = it->second.stringvalue();
            // 移除持久订阅
            {
                std::lock_guard<std::mutex> lock(subscriptions_mutex_);
                auto it = subscriptions_.find(subscriptionId);
                if (it != subscriptions_.end())
                {
                    subscriptions_.erase(it);
                    std::cout << "Persistent subscription removed: " << subscriptionId << std::endl;
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

        void InterfaceServiceImpl::PublishMessage(const std::string &objectId, const std::string &eventType,
                                                  const humanoid_robot::PB::interfaces::Notification &notification)
        {
            std::lock_guard<std::mutex> lock(subscriptions_mutex_);

            for (const auto &[subId, subscription] : subscriptions_)
            {
                // 检查订阅是否匹配
                if (subscription->objectId == objectId)
                {
                    // 检查事件类型
                    if (subscription->eventTypes.empty() ||
                        std::find(subscription->eventTypes.begin(), subscription->eventTypes.end(), eventType) != subscription->eventTypes.end())
                    {
                        // 发送消息到客户端
                        grpc::ClientContext context;
                        NotificationAck response;

                        auto status = subscription->clientStub->OnSubscriptionMessage(&context, notification, &response);

                        if (status.ok())
                        {
                            std::cout << "Message published to subscription: " << subId << std::endl;
                            subscription->lastHeartbeat = std::chrono::steady_clock::now();
                        }
                        else
                        {
                            std::cerr << "Failed to publish message to subscription: " << subId
                                      << " Error: " << status.error_message() << std::endl;
                        }
                    }
                }
            }
        }

        void InterfaceServiceImpl::StartHeartbeatMonitor()
        {
            running_ = true;
            heartbeat_thread_ = std::thread([this]()
                                            {
                while (running_)
                {
                    CheckHeartbeats();
                    std::this_thread::sleep_for(std::chrono::seconds(30)); // 每30秒检查一次
                } });
        }

        void InterfaceServiceImpl::StopHeartbeatMonitor()
        {
            running_ = false;
            if (heartbeat_thread_.joinable())
            {
                heartbeat_thread_.join();
            }
        }

        void InterfaceServiceImpl::CheckHeartbeats()
        {
            std::lock_guard<std::mutex> lock(subscriptions_mutex_);
            auto now = std::chrono::steady_clock::now();

            for (auto it = subscriptions_.begin(); it != subscriptions_.end();)
            {
                auto &subscription = it->second;
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - subscription->lastHeartbeat)
                                   .count();

                // 如果超过心跳间隔的2倍时间没有响应，认为客户端断开
                if (elapsed > subscription->heartbeatInterval * 2)
                {
                    std::cout << "Subscription timeout, removing: " << subscription->subscriptionId << std::endl;
                    it = subscriptions_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        void InterfaceServiceImpl::SendSimulatedNotification(const std::string &subscriptionId)
        {
            std::lock_guard<std::mutex> lock(subscriptions_mutex_);
            auto it = subscriptions_.find(subscriptionId);
            if (it == subscriptions_.end())
            {
                std::cerr << "Subscription not found: " << subscriptionId << std::endl;
                return;
            }

            auto &subscription = it->second;
            if (!subscription->clientStub)
            {
                std::cerr << "Client stub not available for subscription: " << subscriptionId << std::endl;
                return;
            }

            int sendCount = 0;
            while (sendCount < 10) // 发送10次模拟通知消息
            {
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒发送一次
                std::cout << "Sending simulated notification for subscription: " << subscriptionId << std::endl;
                std::cout << "Send count: " << sendCount + 1 << std::endl;
                sendCount++;
                // 创建模拟通知消息
                Notification notification;
                auto message = notification.mutable_notifymessage()->mutable_keyvaluelist();

                {
                    Variant var;
                    var.set_stringvalue("subscription_callback : " + std::to_string(sendCount));
                    message->insert({"event_type", var});
                }
                {
                    Variant var;
                    var.set_stringvalue(subscriptionId);
                    message->insert({"subscription_id", var});
                }
                {
                    Variant var;
                    var.set_stringvalue("Hello from server! This is a simulated notification message.");
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
                    timestamp->set_seconds(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                    timestamp->set_nanos(0);
                    message->insert({"timestamp", var});
                }

                // 发送通知到客户端
                grpc::ClientContext context;
                NotificationAck response;

                std::cout << "Sending simulated notification to client endpoint: "
                          << subscription->clientEndpoint << std::endl;

                grpc::Status status = subscription->clientStub->OnSubscriptionMessage(&context, notification, &response);

                if (status.ok())
                {
                    std::cout << "Successfully sent notification to client for subscription: "
                              << subscriptionId << std::endl;
                    std::cout << "Client response return code: " << response.ret() << std::endl;
                }
                else
                {
                    std::cerr << "Failed to send notification to client: "
                              << status.error_code() << " - " << status.error_message() << std::endl;
                }
            }
        }

        // =============================================================================
        // InterfacesServer 实现 - 简化版本
        // =============================================================================

        InterfacesServer::InterfacesServer()
        {
            service_ = std::make_unique<InterfaceServiceImpl>();
        }

        InterfacesServer::~InterfacesServer()
        {
            Stop();
        }

        bool InterfacesServer::Start(const std::string &server_address)
        {
            try
            {
                grpc::EnableDefaultHealthCheckService(true);
                grpc::reflection::InitProtoReflectionServerBuilderPlugin();

                grpc::ServerBuilder builder;

                // 加上复杂配置，使用健康检查和反射
                builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
                builder.RegisterService(service_.get());

                server_ = builder.BuildAndStart();

                if (server_)
                {
                    std::cout << "gRPC Server listening on " << server_address << std::endl;
                    return true;
                }
                else
                {
                    std::cerr << "Failed to start gRPC server!" << std::endl;
                    return false;
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error starting server: " << e.what() << std::endl;
                return false;
            }
        }

        void InterfacesServer::Stop()
        {
            if (server_)
            {
                server_->Shutdown();
                std::cout << "gRPC Server stopped." << std::endl;
            }
        }

        void InterfacesServer::Wait()
        {
            if (server_)
            {
                server_->Wait();
            }
        }

        void InterfacesServer::SimulatePublishMessage(const std::string &objectId, const std::string &eventType,
                                                      const std::string &messageContent)
        {
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
                timestamp->set_seconds(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                timestamp->set_nanos(0);
                message->insert({"timestamp", var});
            }
            {
                Variant var;
                var.set_stringvalue("msg_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
                message->insert({"messageid", var});
            }

            std::cout << "Publishing message for objectId: " << objectId
                      << ", eventType: " << eventType
                      << ", content: " << messageContent << std::endl;

            // 发布消息到匹配的订阅
            service_->PublishMessage(objectId, eventType, notification);
        }

    } // namespace server
} // namespace humanoid_robot
