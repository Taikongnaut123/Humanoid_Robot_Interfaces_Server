/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Interfaces Server 实现 - 支持持久订阅
 * 参照Client-SDK模式，避免复杂的gRPC初始化
 */

#include "interfaces_server.h"
#include <iostream>
#include <chrono>
#include <algorithm>

namespace humanoid_robot
{
    namespace server
    {

        // =============================================================================
        // InterfaceServiceImpl 实现
        // =============================================================================

        InterfaceServiceImpl::InterfaceServiceImpl() : running_(false)
        {
            StartHeartbeatMonitor();
        }

        InterfaceServiceImpl::~InterfaceServiceImpl()
        {
            StopHeartbeatMonitor();
        }

        grpc::Status InterfaceServiceImpl::Send(grpc::ServerContext *context,
                                                const interfaces::SendRequest *request,
                                                interfaces::SendResponse *response)
        {
            std::cout << "Support Send service" << std::endl;
            auto output_map = response->mutable_output()->mutable_keyvaluelist();
            {
                base_types::Variant var;
                var.set_stringvalue("send request success");
                output_map->insert({"status_desc", var});
            }
            {
                base_types::Variant var;
                var.set_int32value(0); // 0表示成功
                output_map->insert({"status_code", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue("Send service executed successfully");
                output_map->insert({"message", var});
            }

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Query(grpc::ServerContext *context,
                                                 const interfaces::QueryRequest *request,
                                                 interfaces::QueryResponse *response)
        {
            std::cout << "Support Query service" << std::endl;
            auto output_map = response->mutable_output()->mutable_keyvaluelist();
            {
                base_types::Variant var;
                var.set_stringvalue("query request success");
                output_map->insert({"status_desc", var});
            }
            {
                base_types::Variant var;
                var.set_int32value(0); // 示例返回查询结果
                output_map->insert({"status_code", var});
            }
            {
                base_types::Variant var;
                var.set_int32value(1); // 示例返回1条记录
                output_map->insert({"count", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue("Query service executed successfully");
                output_map->insert({"message", var});
            }

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Action(::grpc::ServerContext *context,
                                                  const ::interfaces::ActionRequest *request,
                                                  ::grpc::ServerWriter<::interfaces::ActionResponse> *writer)
        {
            std::cout << "Support Action service" << std::endl;

            for (int i = 0; i < 3; ++i)
            {
                ::interfaces::ActionResponse response;
                auto output = response.mutable_output()->mutable_keyvaluelist();
                {
                    base_types::Variant var;
                    var.set_int32value(i + 1); // 示例使用整数值
                    output->insert({"action_id", var});
                }
                {
                    base_types::Variant var;
                    var.set_stringvalue("action " + std::to_string(i + 1)); // 示例使用字符串值
                    output->insert({"action_name", var});
                }
                {
                    base_types::Variant var;
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
                                                     const interfaces::SubscribeRequest *request,
                                                     interfaces::SubscribeResponse *response)
        {
            std::cout << "Subscribe service called" << std::endl;

            std::string subscriptionId;
            // 检查是否有传入的订阅ID
            auto &input_map = request->input().keyvaluelist();
            auto it = input_map.find("object_id");
            if (it == input_map.end())
            {
                std::cout << "object_id ID already exists: " << it->second.stringvalue() << std::endl;
                std::cout << "using default subscription ID" << std::endl;
                subscriptionId = "sub_default_" + std::to_string(std::time(nullptr));
            }
            else
            {
                // 创建持久订阅
                subscriptionId = "sub_" + std::to_string(it->second.int32value()) + "_" + std::to_string(std::time(nullptr));
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
                std::cerr << "No client endpoint provided, cannot create subscription" << std::endl;
                clientEndpoint = "localhost:50052"; // 使用默认值或抛出异常
            }
            else
            {
                clientEndpoint = it->second.stringvalue();
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

            auto subscription = std::make_unique<PersistentSubscription>(
                subscriptionId, subscriptionId, clientEndpoint,
                eventTypes, heartbeatInterval);

            // 创建客户端stub
            auto channel = grpc::CreateChannel(clientEndpoint, grpc::InsecureChannelCredentials());
            subscription->clientStub = interfaces::ClientCallbackService::NewStub(channel);

            // 存储订阅
            {
                std::lock_guard<std::mutex> lock(subscriptions_mutex_);
                subscriptions_[subscriptionId] = std::move(subscription);
            }

            auto output_map = response->mutable_output()->mutable_keyvaluelist();
            {
                base_types::Variant var;
                var.set_stringvalue("persistent subscription created successfully");
                output_map->insert({"status_desc", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue(subscriptionId);
                output_map->insert({"subscription_id", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue(clientEndpoint);
                output_map->insert({"client_endpoint", var});
            }
            {
                base_types::Variant var;
                var.set_int64value(heartbeatInterval);
                output_map->insert({"heartbeat_interval", var});
            }
            {
                base_types::Variant var;
                var.set_int32value(0);
                output_map->insert({"status_code", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue("Persistent subscription created: " + subscriptionId);
                output_map->insert({"message", var});
            }
            std::cout << "Persistent subscription created: " << subscriptionId << std::endl;
            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Unsubscribe(grpc::ServerContext *context,
                                                       const interfaces::UnsubscribeRequest *request,
                                                       interfaces::UnsubscribeResponse *response)
        {
            std::cout << "Unsubscribe service called for subscription" << std::endl;
            auto &input_map = request->input().keyvaluelist();
            auto output_map = response->mutable_output()->mutable_keyvaluelist();

            auto it = input_map.find("subscription_id");
            if (it == input_map.end())
            {
                std::cout << "No subscription_id provided, cannot unsubscribe" << std::endl;
                {
                    base_types::Variant var;
                    var.set_stringvalue("No subscription_id provided");
                    output_map->insert({"status_desc", var});
                }
                {
                    base_types::Variant var;
                    var.set_int32value(-0600010001); // 1表示错误
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
                base_types::Variant var;
                var.set_stringvalue("Persistent subscription removed: " + subscriptionId);
                output_map->insert({"status_desc", var});
            }
            {
                base_types::Variant var;
                var.set_int32value(0); // 0表示成功
                output_map->insert({"status_code", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue("Unsubscribe service executed successfully");
                output_map->insert({"message", var});
            }
            return grpc::Status::OK;
        }

        void InterfaceServiceImpl::PublishMessage(const std::string &objectId, const std::string &eventType,
                                                  const interfaces::Notification &notification)
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
                        interfaces::NotificationAck response;

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
            // 创建订阅通知消息
            interfaces::Notification notification;
            auto message = notification.mutable_notifymessage()->mutable_keyvaluelist();
            {
                base_types::Variant var;
                var.set_stringvalue("null");
                message->insert({"subscriptionid", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue("null");
                message->insert({"objectid", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue("news");
                message->insert({"topicid", var});
            }
            {
                base_types::Variant var;
                auto timestamp = var.mutable_timestampvalue();
                timestamp->set_seconds(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                timestamp->set_nanos(0);
                message->insert({"timestamp", var});
            }
            {
                base_types::Variant var;
                var.set_stringvalue("msg_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
                message->insert({"messageid", var});
            }

            std::cout << "Publishing message for objectId: " << objectId
                      << ", eventType: " << eventType
                      << ", content: " << messageContent << std::endl;

            // 发布消息
            service_->PublishMessage(objectId, eventType, notification);
        }

    } // namespace server
} // namespace humanoid_robot
