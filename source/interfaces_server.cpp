/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Interfaces Server 实现 - 简化版本
 * 参照Client-SDK模式，避免复杂的gRPC初始化
 */

#include "interfaces_server.h"
#include <iostream>

namespace humanoid_robot
{
    namespace server
    {

        // =============================================================================
        // InterfaceServiceImpl 实现
        // =============================================================================

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

        grpc::Status InterfaceServiceImpl::BatchCreate(grpc::ServerContext *context,
                                                       const interfaces::BatchCreateRequest *request,
                                                       interfaces::BatchCreateResponse *response)
        {
            std::cout << "Support BatchCreate service" << std::endl;

            response->set_overallstatus(interfaces::STATUS_SUCCESS);
            response->set_successcount(1);
            response->set_failedcount(0);

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::HealthCheck(grpc::ServerContext *context,
                                                       const interfaces::HealthCheckRequest *request,
                                                       interfaces::HealthCheckResponse *response)
        {
            std::cout << "Support HealthCheck service" << std::endl;

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_message("HealthCheck service executed successfully");
            response->set_service("InterfaceService");

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Subscribe(grpc::ServerContext *context,
                                                     const interfaces::SubscribeRequest *request,
                                                     grpc::ServerWriter<interfaces::SubscribeResponse> *writer)
        {
            std::cout << "Support Subscribe service" << std::endl;

            // 简单的流式响应
            for (int i = 0; i < 3; ++i)
            {
                interfaces::SubscribeResponse response;
                response.set_status(interfaces::STATUS_SUCCESS);
                response.set_message("Subscribe service stream response " + std::to_string(i + 1));

                if (!writer->Write(response))
                {
                    break;
                }
            }

            return grpc::Status::OK;
        }

        grpc::Status InterfaceServiceImpl::Unsubscribe(grpc::ServerContext *context,
                                                       const interfaces::UnsubscribeRequest *request,
                                                       interfaces::UnsubscribeResponse *response)
        {
            std::cout << "Support Unsubscribe service" << std::endl;

            response->set_status(interfaces::STATUS_SUCCESS);
            response->set_message("Unsubscribe service executed successfully");

            return grpc::Status::OK;
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
                // 重新加上复杂的gRPC配置
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

    } // namespace server
} // namespace humanoid_robot
