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
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

// 引入proto生成的文件
#include "interfaces/interfaces_grpc.grpc.pb.h"
#include "interfaces/interfaces_request_response.pb.h"

namespace humanoid_robot
{
    namespace server
    {

        /**
         * InterfaceService服务实现类
         * 简化实现，参照Client-SDK模式
         */
        class InterfaceServiceImpl final : public interfaces::InterfaceService::Service
        {
        public:
            InterfaceServiceImpl() = default;
            ~InterfaceServiceImpl() = default;

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

            grpc::Status BatchCreate(grpc::ServerContext *context,
                                     const interfaces::BatchCreateRequest *request,
                                     interfaces::BatchCreateResponse *response) override;

            grpc::Status HealthCheck(grpc::ServerContext *context,
                                     const interfaces::HealthCheckRequest *request,
                                     interfaces::HealthCheckResponse *response) override;

            // 流式服务
            grpc::Status Subscribe(grpc::ServerContext *context,
                                   const interfaces::SubscribeRequest *request,
                                   grpc::ServerWriter<interfaces::SubscribeResponse> *writer) override;

            grpc::Status Unsubscribe(grpc::ServerContext *context,
                                     const interfaces::UnsubscribeRequest *request,
                                     interfaces::UnsubscribeResponse *response) override;
        };

        /**
         * 简化的gRPC服务器类
         * 参照Client-SDK的简洁模式
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

        private:
            std::unique_ptr<grpc::Server> server_;
            std::unique_ptr<InterfaceServiceImpl> service_;
        };

    } // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_INTERFACES_SERVER_H
