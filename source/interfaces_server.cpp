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

        grpc::Status InterfaceServiceImpl::Create(grpc::ServerContext *context,
                                                  const interfaces::CreateRequest *request,
                                                  interfaces::CreateResponse *response)
        {
            std::cout << "Support Create service" << std::endl;

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_message("Create service executed successfully");

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Send(grpc::ServerContext *context,
                                                const interfaces::SendRequest *request,
                                                interfaces::SendResponse *response)
        {
            std::cout << "Support Send service" << std::endl;

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_message("Send service executed successfully");

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Delete(grpc::ServerContext *context,
                                                  const interfaces::DeleteRequest *request,
                                                  interfaces::DeleteResponse *response)
        {
            std::cout << "Support Delete service" << std::endl;

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_message("Delete service executed successfully");

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Query(grpc::ServerContext *context,
                                                 const interfaces::QueryRequest *request,
                                                 interfaces::QueryResponse *response)
        {
            std::cout << "Support Query service" << std::endl;

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_totalcount(1);
            response->set_hasmore(false);

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

            std::cout << "Creating persistent subscription for objectId: " << request->objectid() << std::endl;

            // 创建持久订阅
            std::string subscriptionId = "sub_" + request->objectid() + "_" + std::to_string(std::time(nullptr));

            std::vector<std::string> eventTypes;
            for (const auto &eventType : request->eventtypes())
            {
                eventTypes.push_back(eventType);
            }

            auto subscription = std::make_unique<PersistentSubscription>(
                subscriptionId, request->objectid(), request->clientendpoint(),
                eventTypes, request->heartbeatinterval());

            // 创建客户端stub
            auto channel = grpc::CreateChannel(request->clientendpoint(), grpc::InsecureChannelCredentials());
            subscription->clientStub = interfaces::ClientCallbackService::NewStub(channel);

            // 存储订阅
            {
                std::lock_guard<std::mutex> lock(subscriptions_mutex_);
                subscriptions_[subscriptionId] = std::move(subscription);
            }

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_message("Persistent subscription created: " + subscriptionId);
            response->set_subscriptionid(subscriptionId);
            std::cout << "Persistent subscription created: " << subscriptionId << std::endl;
            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Unsubscribe(grpc::ServerContext *context,
                                                       const interfaces::UnsubscribeRequest *request,
                                                       interfaces::UnsubscribeResponse *response)
        {
            std::cout << "Unsubscribe service called for subscription: " << request->subscriptionid() << std::endl;

            // 移除持久订阅
            {
                std::lock_guard<std::mutex> lock(subscriptions_mutex_);
                auto it = subscriptions_.find(request->subscriptionid());
                if (it != subscriptions_.end())
                {
                    subscriptions_.erase(it);
                    std::cout << "Persistent subscription removed: " << request->subscriptionid() << std::endl;
                }
            }

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_message("Unsubscribe service executed successfully");

            return grpc::Status::OK;
        }

        void InterfaceServiceImpl::PublishMessage(const std::string &objectId, const std::string &eventType,
                                                  const interfaces::SubscriptionNotification &notification)
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
            interfaces::SubscriptionNotification notification;
            notification.set_subscriptionid(""); // 将由PublishMessage填充
            notification.set_objectid(objectId);
            notification.set_topicid(eventType);
            notification.set_timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::system_clock::now().time_since_epoch())
                                           .count());
            notification.set_messageid("msg_" + std::to_string(notification.timestamp()));

            // 设置消息内容
            auto messageData = notification.mutable_messagedata();
            base_types::Variant contentVariant;
            contentVariant.set_stringvalue(messageContent);
            (*messageData->mutable_keyvaluelist())["content"] = contentVariant;

            std::cout << "Publishing message for objectId: " << objectId
                      << ", eventType: " << eventType
                      << ", content: " << messageContent << std::endl;

            // 发布消息
            service_->PublishMessage(objectId, eventType, notification);
        }

    } // namespace server
} // namespace humanoid_robot
