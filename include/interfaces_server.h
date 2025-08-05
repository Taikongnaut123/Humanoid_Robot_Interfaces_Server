/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Interfaces Server - gRPC服务端实现
 * 基于Client-SDK模式构建，避免复杂的gRPC初始化
 */

#ifndef HUMANOID_ROBOT_INTERFACES_SERVER_H
#define HUMANOID_ROBOT_INTERFACES_SERVER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

// 引入proto生成的文件
#include "interfaces/interfaces_grpc.grpc.pb.h"
#include "interfaces/interfaces_request_response.pb.h"
#include "interfaces/interfaces_callback.grpc.pb.h"

namespace humanoid_robot
{
    namespace server
    {

        /**
         * 持久订阅信息
         */
        struct PersistentSubscription
        {
            std::string subscriptionId;
            std::string objectId;
            std::string clientEndpoint;
            std::vector<std::string> eventTypes;
            int64_t heartbeatInterval;
            std::chrono::steady_clock::time_point lastHeartbeat;
            std::unique_ptr<interfaces::ClientCallbackService::Stub> clientStub;

            PersistentSubscription(const std::string &subId, const std::string &objId,
                                   const std::string &endpoint, const std::vector<std::string> &events,
                                   int64_t interval)
                : subscriptionId(subId), objectId(objId), clientEndpoint(endpoint),
                  eventTypes(events), heartbeatInterval(interval),
                  lastHeartbeat(std::chrono::steady_clock::now()) {}
        };

        /**
         * InterfaceService服务实现类
         * 简化实现，参照Client-SDK模式
         */
        class InterfaceServiceImpl final : public interfaces::InterfaceService::Service
        {
        public:
            InterfaceServiceImpl();
            ~InterfaceServiceImpl();

            // 基本CRUD操作
            grpc::Status Create(grpc::ServerContext *context,
                                const interfaces::CreateRequest *request,
                                interfaces::CreateResponse *response) override;

            grpc::Status Send(grpc::ServerContext *context,
                              const interfaces::SendRequest *request,
                              interfaces::SendResponse *response) override;

            grpc::Status Delete(grpc::ServerContext *context,
                                const interfaces::DeleteRequest *request,
                                interfaces::DeleteResponse *response) override;

            grpc::Status Query(grpc::ServerContext *context,
                               const interfaces::QueryRequest *request,
                               interfaces::QueryResponse *response) override;

            grpc::Status Action(::grpc::ServerContext *context,
                                const ::interfaces::ActionRequest *request,
                                ::grpc::ServerWriter<::interfaces::ActionResponse> *writer) override;

            // 流式服务
            grpc::Status Subscribe(grpc::ServerContext *context,
                                   const interfaces::SubscribeRequest *request,
                                   interfaces::SubscribeResponse *response) override;

            grpc::Status Unsubscribe(grpc::ServerContext *context,
                                     const interfaces::UnsubscribeRequest *request,
                                     interfaces::UnsubscribeResponse *response) override;

            // 持久订阅管理
            void PublishMessage(const std::string &objectId, const std::string &eventType,
                                const interfaces::SubscriptionNotification &notification);

        private:
            std::unordered_map<std::string, std::unique_ptr<PersistentSubscription>> subscriptions_;
            std::mutex subscriptions_mutex_;
            std::thread heartbeat_thread_;
            bool running_;

            void StartHeartbeatMonitor();
            void StopHeartbeatMonitor();
            void CheckHeartbeats();
        };

        /**
         * gRPC服务器类
         */
        class InterfacesServer
        {
        public:
            InterfacesServer();
            ~InterfacesServer();

            // 启动服务器 (简化版本)
            bool Start(const std::string &server_address = "127.0.0.1:50051");

            // 停止服务器
            void Stop();

            // 等待服务器关闭
            void Wait();

            // 获取服务实例（用于发布消息）
            InterfaceServiceImpl *GetService() { return service_.get(); }

            // 示例：模拟发布消息
            void SimulatePublishMessage(const std::string &objectId, const std::string &eventType,
                                        const std::string &messageContent);

        private:
            std::unique_ptr<grpc::Server> server_;
            std::unique_ptr<InterfaceServiceImpl> service_;
        };

    } // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_INTERFACES_SERVER_H
