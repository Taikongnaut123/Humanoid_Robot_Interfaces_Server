/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Perception Module Implementation - 感知模块实现
 */

#include "modules/perception_module.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include "grpcpp/grpcpp.h"
#include "perception/perception_service.grpc.pb.h"
#include "perception/perception_request_response.pb.h"
#include "Log/wlog.hpp"
using namespace humanoid_robot;
using namespace humanoid_robot::PB::perception;
using namespace humanoid_robot::PB::common;

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            class PerceptionModule::PerceptionImpl
            {
            public:
                std::shared_ptr<grpc::Channel> channel_;
                std::unique_ptr<PerceptionService::Stub> stub_;
                std::string target_;
                bool connected_;

                PerceptionImpl() : connected_(false) {}

                ~PerceptionImpl()
                {
                    if (connected_)
                    {
                        // Graceful shutdown could be implemented here
                    }
                }
            };

            PerceptionModule::PerceptionModule()
                : ModuleBase("Perception", 3), // 3个工作线程
                  pImpl_(std::make_unique<PerceptionImpl>())
            {
            }

            PerceptionModule::~PerceptionModule() = default;

            bool PerceptionModule::Initialize()
            {
                try
                {
                    std::string target = "localhost:52001"; // 5为默认，2为感知序列号，000-999为感知自行分配区间
                    pImpl_->target_ = target;

                    // Create gRPC channel with default credentials
                    grpc::ChannelArguments args;
                    args.SetMaxReceiveMessageSize(100 * 1024 * 1024); // 100MB max message size
                    args.SetMaxSendMessageSize(100 * 1024 * 1024);

                    pImpl_->channel_ = grpc::CreateCustomChannel(
                        target,
                        grpc::InsecureChannelCredentials(),
                        args);

                    if (!pImpl_->channel_)
                    {
                        return false;
                    }

                    // Create service stub
                    pImpl_->stub_ = PerceptionService::NewStub(pImpl_->channel_);

                    if (!pImpl_->stub_)
                    {
                        return false;
                    }

                    pImpl_->connected_ = true;
                    return true;
                }
                catch (const std::exception &e)
                {
                    return false;
                }
                return true;
            }

            void PerceptionModule::Cleanup()
            {
                std::cout << "[Perception] Cleaning up perception resources..." << std::endl;

                pImpl_->stub_.reset();
                pImpl_->channel_.reset();
                pImpl_->connected_ = false;
                WLOG_INFO("Perception module cleanup completed");
            }

            ModuleResult PerceptionModule::ProcessCommand(
                grpc::ServerContext *context, const int32_t &command_id,
                const humanoid_robot::PB::common::Dictionary &input_data,
                const humanoid_robot::PB::common::Dictionary &params)
            {
                std::cout << "[Perception] Processing command: " << command_id << std::endl;

                if (command_id == GET_PERCEPTION_RESULT)
                {
                    return perceive(input_data, params);
                    // return perceive(context, input_data, params);
                }
                else if (command_id == GET_DETECTION_RESULT)
                {
                    return detect(input_data, params);
                    // return detect(context, input_data, params);
                }
                else if (command_id == GET_DIVISION_RESULT)
                {
                    return divide(input_data, params);
                    // return divide(context, input_data, params);
                }
                else
                {
                    return ModuleResult::Error("Perception", command_id,
                                               -100, "Unknown perception command: " + command_id);
                }
            }

            ModuleResult PerceptionModule::perceive(
                // grpc::ServerContext *&context,
                const humanoid_robot::PB::common::Dictionary &input_data,
                const humanoid_robot::PB::common::Dictionary &params)
            {
                SimulateProcessingDelay(200, 800);
                ::grpc::ClientContext context;
                auto readwrite = pImpl_->stub_->GetPerceptionResult(&context);

                // 模拟视觉检测结果
                std::vector<std::pair<std::string, std::string>> detected_objects = {
                    {"person", "human_1"},
                    {"chair", "furniture_1"},
                    {"table", "furniture_2"}};
                ::humanoid_robot::PB::perception::Perception response;
                auto result = readwrite->Read(&response);
                Dictionary result_data;
                return ModuleResult::Success("Perception", GET_PERCEPTION_RESULT,
                                             std::make_unique<humanoid_robot::PB::common::Dictionary>(std::move(result_data)));
            }

            ModuleResult PerceptionModule::detect(
                // grpc::ServerContext *&context,
                const humanoid_robot::PB::common::Dictionary &input_data,
                const humanoid_robot::PB::common::Dictionary &params)
            {
                SimulateProcessingDelay(150, 600);

                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto kv_map = result_data->mutable_keyvaluelist();

                // 音频分析结果
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_stringvalue("speech");
                    kv_map->insert({"audio_type", var});
                }
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_stringvalue("Hello, how are you?");
                    kv_map->insert({"recognized_text", var});
                }
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_doublevalue(0.89);
                    kv_map->insert({"confidence", var});
                }
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_stringvalue("english");
                    kv_map->insert({"language", var});
                }

                return ModuleResult::Success("Perception", GET_DETECTION_RESULT, std::move(result_data));
            }

            ModuleResult PerceptionModule::divide(
                // grpc::ServerContext *&context,
                const humanoid_robot::PB::common::Dictionary &input_data,
                const humanoid_robot::PB::common::Dictionary &params)
            {
                SimulateProcessingDelay(50, 200);

                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto kv_map = result_data->mutable_keyvaluelist();

                // 触觉感知结果
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_doublevalue(25.6); // 温度
                    kv_map->insert({"temperature", var});
                }
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_doublevalue(1.2); // 压力
                    kv_map->insert({"pressure", var});
                }
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_stringvalue("smooth"); // 质地
                    kv_map->insert({"texture", var});
                }
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_boolvalue(true); // 接触状态
                    kv_map->insert({"contact", var});
                }

                return ModuleResult::Success("Perception", GET_DIVISION_RESULT, std::move(result_data));
            }

            void PerceptionModule::SimulateProcessingDelay(int min_ms, int max_ms) const
            {
                static thread_local std::random_device rd;
                static thread_local std::mt19937 gen(rd());
                std::uniform_int_distribution<> dis(min_ms, max_ms);

                int delay = dis(gen);
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            }

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot
