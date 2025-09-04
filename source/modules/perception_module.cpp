/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Perception Module Implementation - 感知模块实现
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include "modules/perception_module.h"
#include "perception/perception_service.grpc.pb.h"
#include "perception/perception_request_response.pb.h"
#include "grpcpp/grpcpp.h"
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
                    std::string target = "localhost:50052"; // 连接到perception_pipeline_cpp服务
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
                        WLOG_ERROR("[Perception] Failed to create gRPC channel to %s", target.c_str());
                        return false;
                    }

                    // Create service stub
                    pImpl_->stub_ = PerceptionService::NewStub(pImpl_->channel_);

                    if (!pImpl_->stub_)
                    {
                        WLOG_ERROR("[Perception] Failed to create service stub");
                        return false;
                    }

                    // 测试连接
                    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(5);
                    if (pImpl_->channel_->WaitForConnected(deadline))
                    {
                        WLOG_DEBUG("[Perception] Successfully connected to perception service at %s", target.c_str());
                        pImpl_->connected_ = true;
                    }
                    else
                    {
                        WLOG_ERROR("[Perception] Failed to connect to perception service at %s", target.c_str());
                        return false;
                    }

                    return true;
                }
                catch (const std::exception &e)
                {
                    WLOG_FATAL("[Perception] Exception during initialization: %s", e.what());
                    return false;
                }
                return true;
            }

            void PerceptionModule::Cleanup()
            {
                WLOG_INFO("Cleaning up perception resources...");

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
                WLOG_DEBUG("Processing command in PerceptionModule: %d", command_id);

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
                WLOG_DEBUG("Processing perception command");
                if (!pImpl_->connected_ || !pImpl_->stub_)
                {
                    return ModuleResult::Error("Perception", GET_PERCEPTION_RESULT,
                                               -1, "Perception service not connected");
                }

                try
                {
                    // 从input_data中提取图像数据
                    auto &kv_map = input_data.keyvaluelist();
                    auto image_it = kv_map.find("image");
                    if (image_it == kv_map.end())
                    {
                        return ModuleResult::Error("Perception", GET_PERCEPTION_RESULT,
                                                   -2, "No image data provided");
                    }

                    // 构造图像请求
                    humanoid_robot::PB::common::Image image_request;

                    // 如果是Image类型的Variant
                    if (image_it->second.value_case() == humanoid_robot::PB::common::Variant::KImageValue)
                    {
                        image_request = image_it->second.imagevalue();
                    }
                    else
                    {
                        // 假设是字符串格式的图像数据
                        image_request.set_img(image_it->second.bytevalue());
                        image_request.set_requiresmasks(true);

                        // 设置时间戳
                        auto now = std::chrono::system_clock::now();
                        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                                             now.time_since_epoch())
                                             .count();
                        image_request.set_timestamp(std::to_string(timestamp));
                    }

                    // 创建客户端上下文并设置超时
                    grpc::ClientContext context;
                    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(10);
                    context.set_deadline(deadline);

                    // 调用GetPerceptionResult (流式接口)
                    auto stream = pImpl_->stub_->GetPerceptionResult(&context);

                    // 发送图像
                    if (!stream->Write(image_request))
                    {
                        return ModuleResult::Error("Perception", GET_PERCEPTION_RESULT,
                                                   -3, "Failed to send image to perception service");
                    }

                    // 完成写入
                    stream->WritesDone();

                    // 读取响应
                    humanoid_robot::PB::perception::Perception response;
                    std::vector<humanoid_robot::PB::perception::Perception> responses;

                    while (stream->Read(&response))
                    {
                        responses.push_back(response);
                    }

                    // 检查状态
                    grpc::Status status = stream->Finish();
                    if (!status.ok())
                    {
                        return ModuleResult::Error("Perception", GET_PERCEPTION_RESULT,
                                                   status.error_code(),
                                                   "Perception service error: " + status.error_message());
                    }

                    // 构造返回结果
                    auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                    auto result_kv_map = result_data->mutable_keyvaluelist();

                    // 添加检测结果数量
                    {
                        humanoid_robot::PB::common::Variant var;
                        var.set_int32value(responses.size());
                        result_kv_map->insert({"perception_count", var});
                    }

                    // 添加感知结果
                    for (size_t i = 0; i < responses.size(); ++i)
                    {
                        std::string key = "perception_" + std::to_string(i);
                        humanoid_robot::PB::common::Variant var;
                        // var.set_type(humanoid_robot::PB::common::Variant::KPerceptionRowValue);

                        // 这里需要根据实际的Perception消息结构来转换
                        // 暂时用字符串表示
                        var.set_stringvalue("perception_result_" + std::to_string(i));
                        result_kv_map->insert({key, var});

                        {
                            auto timestamp = responses[i].timestamp();
                            humanoid_robot::PB::common::Variant timestamp_var;
                            timestamp_var.set_stringvalue(timestamp);
                            result_kv_map->insert({"timestamp_" + std::to_string(i), timestamp_var});
                        }

                        {
                            auto rows = responses[i].rows();
                            for (auto row : rows)
                            {
                                {

                                    auto bbox = row.bbox();
                                    humanoid_robot::PB::common::Variant bbox_var;
                                    // bbox_var.set_type(humanoid_robot::PB::common::Variant::KBBoxValue);
                                    *bbox_var.mutable_bboxvalue() = bbox;
                                    result_kv_map->insert({"bbox_" + std::to_string(i), bbox_var});
                                }

                                {
                                    auto track_id = row.trackid();
                                    humanoid_robot::PB::common::Variant track_id_var;
                                    track_id_var.set_stringvalue(track_id);
                                    result_kv_map->insert({"track_id_" + std::to_string(i), track_id_var});
                                }

                                {
                                    auto conf = row.conf();
                                    humanoid_robot::PB::common::Variant conf_var;
                                    conf_var.set_floatvalue(conf);
                                    result_kv_map->insert({"conf_" + std::to_string(i), conf_var});
                                }

                                {
                                    auto cls = row.cls();
                                    humanoid_robot::PB::common::Variant cls_var;
                                    cls_var.set_stringvalue(cls);
                                    result_kv_map->insert({"cls_" + std::to_string(i), cls_var});
                                }
                                {
                                    auto ismove = row.ismove();
                                    humanoid_robot::PB::common::Variant ismove_var;
                                    ismove_var.set_boolvalue(ismove);
                                    result_kv_map->insert({"ismove_" + std::to_string(i), ismove_var});
                                }
                            }
                        }
                    }

                    WLOG_DEBUG("[Perception] Successfully processed perception request, got %d results", responses.size());

                    return ModuleResult::Success("Perception", GET_PERCEPTION_RESULT, std::move(result_data));
                }
                catch (const std::exception &e)
                {
                    return ModuleResult::Error("Perception", GET_PERCEPTION_RESULT,
                                               -4, "Exception in perception processing: " + std::string(e.what()));
                }
            }

            ModuleResult PerceptionModule::detect(
                // grpc::ServerContext *&context,
                const humanoid_robot::PB::common::Dictionary &input_data,
                const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("Processing detection command");
                if (!pImpl_->connected_ || !pImpl_->stub_)
                {
                    return ModuleResult::Error("Perception", GET_DETECTION_RESULT,
                                               -1, "Perception service not connected");
                }

                try
                {
                    // 从input_data中提取图像数据
                    auto &kv_map = input_data.keyvaluelist();
                    auto image_it = kv_map.find("image");
                    if (image_it == kv_map.end())
                    {
                        return ModuleResult::Error("Perception", GET_DETECTION_RESULT,
                                                   -2, "No image data provided");
                    }
                    WLOG_DEBUG("Image data found in input_data");
                    // 构造图像请求
                    humanoid_robot::PB::common::Image image_request;

                    // 如果是Image类型的Variant
                    if (image_it->second.value_case() == humanoid_robot::PB::common::Variant::KImageValue)
                    {
                        image_request = image_it->second.imagevalue();
                    }
                    else
                    {
                        // 假设是字符串格式的图像数据
                        image_request.set_img(image_it->second.bytevalue());
                        WLOG_DEBUG("Image data set in image_request， image size : %d bytes",
                                   image_request.img().size());
                        image_request.set_requiresmasks(false); // 检测不需要masks

                        // 设置时间戳
                        auto now = std::chrono::system_clock::now();
                        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                                             now.time_since_epoch())
                                             .count();
                        image_request.set_timestamp(std::to_string(timestamp));
                    }

                    // 创建客户端上下文并设置超时
                    grpc::ClientContext context;
                    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(10);
                    context.set_deadline(deadline);

                    // 调用GetDetectionResult
                    humanoid_robot::PB::perception::Detection response;
                    grpc::Status status = pImpl_->stub_->GetDetectionResult(&context, image_request, &response);

                    if (!status.ok())
                    {
                        return ModuleResult::Error("Perception", GET_DETECTION_RESULT,
                                                   status.error_code(),
                                                   "Detection service error: " + status.error_message());
                    }
                    WLOG_DEBUG("[Perception] Detection request processed successfully");
                    // 构造返回结果
                    auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                    auto result_kv_map = result_data->mutable_keyvaluelist();
                    auto response_timestamp = response.timestamp();
                    auto response_rows_size = response.rows_size();
                    auto response_rows = response.rows();
                    WLOG_DEBUG("[Perception] Response timestamp: %s", response_timestamp.c_str());
                    WLOG_DEBUG("[Perception] Response rows size: %d", response_rows_size);
                    // 添加检测结果数量
                    {
                        humanoid_robot::PB::common::Variant var;
                        var.set_int32value(response.rows_size());
                        result_kv_map->insert({"rows_size", var});
                    }

                    // 添加检测结果
                    for (int i = 0; i < response.rows_size(); ++i)
                    {
                        const auto &detection = response.rows(i);
                        std::string prefix = "detection_" + std::to_string(i) + "_";

                        // 检测框
                        {
                            humanoid_robot::PB::common::Variant var;
                            // var.set_type(humanoid_robot::PB::common::Variant::KBBoxValue);
                            *var.mutable_bboxvalue() = detection.bbox();
                            result_kv_map->insert({prefix + "bbox", var});
                        }

                        // 跟踪ID
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_stringvalue(detection.trackid());
                            result_kv_map->insert({prefix + "track_id", var});
                        }

                        // 置信度
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_doublevalue(detection.conf());
                            result_kv_map->insert({prefix + "confidence", var});
                        }

                        // 类别
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_stringvalue(detection.cls());
                            result_kv_map->insert({prefix + "class", var});
                        }

                        // 运动状态
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_boolvalue(detection.ismove());
                            result_kv_map->insert({prefix + "is_moving", var});
                        }
                    }

                    WLOG_DEBUG("[Perception] Successfully processed detection request, got %d detections", response.rows_size());

                    return ModuleResult::Success("Perception", GET_DETECTION_RESULT, std::move(result_data));
                }
                catch (const std::exception &e)
                {
                    return ModuleResult::Error("Perception", GET_DETECTION_RESULT,
                                               -4, "Exception in detection processing: " + std::string(e.what()));
                }
            }

            ModuleResult PerceptionModule::divide(
                // grpc::ServerContext *&context,
                const humanoid_robot::PB::common::Dictionary &input_data,
                const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("Processing division command");
                if (!pImpl_->connected_ || !pImpl_->stub_)
                {
                    return ModuleResult::Error("Perception", GET_DIVISION_RESULT,
                                               -1, "Perception service not connected");
                }

                try
                {
                    // 从input_data中提取图像数据
                    auto &kv_map = input_data.keyvaluelist();
                    auto image_it = kv_map.find("image");
                    if (image_it == kv_map.end())
                    {
                        WLOG_ERROR("No image data provided in input_data for division");
                        return ModuleResult::Error("Perception", GET_DIVISION_RESULT,
                                                   -2, "No image data provided");
                    }

                    WLOG_DEBUG("Image data found in input_data for division");
                    // 构造图像请求
                    humanoid_robot::PB::common::Image image_request;

                    // 如果是Image类型的Variant
                    if (image_it->second.value_case() == humanoid_robot::PB::common::Variant::KImageValue)
                    {
                        image_request = image_it->second.imagevalue();
                    }
                    else
                    {
                        // 假设是字符串格式的图像数据
                        image_request.set_img(image_it->second.bytevalue());
                        image_request.set_requiresmasks(true); // 分割需要masks

                        // 设置时间戳
                        auto now = std::chrono::system_clock::now();
                        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                                             now.time_since_epoch())
                                             .count();
                        image_request.set_timestamp(std::to_string(timestamp));
                    }

                    // 创建客户端上下文并设置超时
                    grpc::ClientContext context;
                    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(10);
                    context.set_deadline(deadline);

                    // 调用GetDivisionResult
                    humanoid_robot::PB::perception::Division response;
                    grpc::Status status = pImpl_->stub_->GetDivisionResult(&context, image_request, &response);

                    if (!status.ok())
                    {
                        return ModuleResult::Error("Perception", GET_DIVISION_RESULT,
                                                   status.error_code(),
                                                   "Division service error: " + status.error_message());
                    }

                    // 构造返回结果
                    auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                    auto result_kv_map = result_data->mutable_keyvaluelist();

                    // 添加分割结果数量
                    {
                        humanoid_robot::PB::common::Variant var;
                        var.set_int32value(response.rows_size());
                        result_kv_map->insert({"division_count", var});
                    }

                    // 添加分割结果
                    for (int i = 0; i < response.rows_size(); ++i)
                    {
                        const auto &division = response.rows(i);
                        std::string prefix = "division_" + std::to_string(i) + "_";

                        // 跟踪ID
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_stringvalue(division.trackid());
                            result_kv_map->insert({prefix + "track_id", var});
                        }

                        // 置信度
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_doublevalue(division.conf());
                            result_kv_map->insert({prefix + "confidence", var});
                        }

                        // 类别
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_stringvalue(division.cls());
                            result_kv_map->insert({prefix + "class", var});
                        }

                        // 运动状态
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_boolvalue(division.ismove());
                            result_kv_map->insert({prefix + "is_moving", var});
                        }

                        // 分割掩码数量
                        {
                            humanoid_robot::PB::common::Variant var;
                            var.set_int32value(division.masks_size());
                            result_kv_map->insert({prefix + "mask_count", var});
                        }

                        // 分割掩码点
                        for (int j = 0; j < division.masks_size(); ++j)
                        {
                            const auto &mask = division.masks(j);
                            std::string mask_prefix = prefix + "mask_" + std::to_string(j) + "_";

                            {
                                humanoid_robot::PB::common::Variant var;
                                var.set_int32value(mask.x());
                                result_kv_map->insert({mask_prefix + "x", var});
                            }

                            {
                                humanoid_robot::PB::common::Variant var;
                                var.set_int32value(mask.y());
                                result_kv_map->insert({mask_prefix + "y", var});
                            }
                        }
                    }

                    WLOG_DEBUG("[Perception] Successfully processed division request, got %d divisions", response.rows_size());

                    return ModuleResult::Success("Perception", GET_DIVISION_RESULT, std::move(result_data));
                }
                catch (const std::exception &e)
                {
                    return ModuleResult::Error("Perception", GET_DIVISION_RESULT,
                                               -4, "Exception in division processing: " + std::string(e.what()));
                }
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
