/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Perception Module Implementation - 感知模块实现
 */

#include "modules/perception_module.h"

#include <chrono>
#include <iostream>
#include <random>
#include <thread>

#include "Log/wlog.hpp"
#include "common/service.pb.h"
#include "grpcpp/grpcpp.h"
#include "sdk_service/common/service.pb.h"
#include "sdk_service/perception/perception_service.grpc.pb.h"
#include "sdk_service/perception/request_detection.pb.h"
#include "sdk_service/perception/request_division.pb.h"
#include "sdk_service/perception/request_perception.pb.h"
#include "sdk_service/perception/response_detection.pb.h"
#include "sdk_service/perception/response_division.pb.h"
#include "sdk_service/perception/response_perception.pb.h"
#include "sdk_service/perception/response_status.pb.h"

using namespace humanoid_robot::PB::sdk_service::common;
namespace PB_common = humanoid_robot::PB::common;

namespace humanoid_robot {
namespace server {
namespace modules {

using ResponseDetection = humanoid_robot::PB::sdk_service::perception::ResponseDetection;
using ReDetection = humanoid_robot::PB::sdk_service::perception::Detection;

using ResponseDivision = humanoid_robot::PB::sdk_service::perception::ResponseDivision;
using ReDivision = humanoid_robot::PB::sdk_service::perception::Division;

using ResponsePerception = humanoid_robot::PB::sdk_service::perception::ResponsePerception;
using RePerception = humanoid_robot::PB::sdk_service::perception::Perception;

using PerceptionResStatus = humanoid_robot::PB::sdk_service::perception::ResponseStatus;
using PerceptionCommandCode = humanoid_robot::PB::sdk_service::common::PerceptionCommandCode;
using PerceptionService = humanoid_robot::PB::sdk_service::perception::PerceptionService;

class PerceptionModule::PerceptionImpl {
public:
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<PerceptionService::Stub> stub_;
  std::string target_;
  bool connected_;

  PerceptionImpl() : connected_(false) {}

  ~PerceptionImpl() {
    if (connected_) {
      // Graceful shutdown could be implemented here
    }
  }
};

PerceptionModule::PerceptionModule()
    : ModuleBase("Perception", 3), // 3个工作线程
      pImpl_(std::make_unique<PerceptionImpl>()) {}

PerceptionModule::~PerceptionModule() = default;

bool PerceptionModule::Initialize() {
  try {
    std::string target = "localhost:50052"; // 连接到perception_pipeline_cpp服务
    pImpl_->target_ = target;

    // Create gRPC channel with default credentials
    grpc::ChannelArguments args;
    args.SetMaxReceiveMessageSize(100 * 1024 * 1024); // 100MB max message size
    args.SetMaxSendMessageSize(100 * 1024 * 1024);

    pImpl_->channel_ = grpc::CreateCustomChannel(
        target, grpc::InsecureChannelCredentials(), args);

    if (!pImpl_->channel_) {
      WLOG_ERROR("[Perception] Failed to create gRPC channel to %s",
                 target.c_str());
      return false;
    }

    // Create service stub
    pImpl_->stub_ = PerceptionService::NewStub(pImpl_->channel_);

    if (!pImpl_->stub_) {
      WLOG_ERROR("[Perception] Failed to create service stub");
      return false;
    }

    // 测试连接
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(5);
    if (pImpl_->channel_->WaitForConnected(deadline)) {
      WLOG_DEBUG(
          "[Perception] Successfully connected to perception service at %s",
          target.c_str());
      pImpl_->connected_ = true;
    } else {
      WLOG_ERROR("[Perception] Failed to connect to perception service at %s",
                 target.c_str());
      return false;
    }

    return true;
  } catch (const std::exception &e) {
    WLOG_FATAL("[Perception] Exception during initialization: %s", e.what());
    return false;
  }
  return true;
}

// bool PerceptionModule::IsRunning() {
//   if (!pImpl_->service_client_)
//     return false;
//   auto client_connected =
//       pImpl_->service_client_->WaitForService(std::chrono::seconds(3));
//   return client_connected;
//   return true;
// }

void PerceptionModule::Cleanup() {
  WLOG_INFO("Cleaning up perception resources...");

  pImpl_->stub_.reset();
  pImpl_->channel_.reset();
  pImpl_->connected_ = false;
  WLOG_INFO("Perception module cleanup completed");
}

ModuleResult PerceptionModule::ProcessCommand(
    grpc::ServerContext *context, const int32_t &command_id,
    const humanoid_robot::PB::common::Dictionary &input_data,
    const humanoid_robot::PB::common::Dictionary &params) {
  WLOG_DEBUG("Processing command in PerceptionModule: %d", command_id);

  if (command_id == PerceptionCommandCode::kPerception) {
    return Perception(input_data, params);
    // return perceive(context, input_data, params);
  } else if (command_id ==
             PerceptionCommandCode::kDetection) {
    return Detection(input_data, params);
    // return detect(context, input_data, params);
  } else if (command_id ==
             PerceptionCommandCode::kDivision) {
    return Division(input_data, params);
    // return divide(context, input_data, params);
  } else {
    return ModuleResult::Error("Perception", command_id, -100,
                               "Unknown perception command: " + command_id);
  }
}

ModuleResult PerceptionModule::Perception(
    // grpc::ServerContext *&context,
    const humanoid_robot::PB::common::Dictionary &input_data,
    const humanoid_robot::PB::common::Dictionary &params) {
  WLOG_DEBUG("Processing perception command");
  if (!pImpl_->connected_ || !pImpl_->stub_) {
    return ModuleResult::Error(
        "Perception", PerceptionCommandCode::kPerception, -1,
        "Perception service not connected");
  }

  try {
    // 从input_data中提取图像数据
    auto data_it = input_data.keyvaluelist().find("request_perception");
    if (data_it == input_data.keyvaluelist().end()) {
        WLOG_ERROR("[Perception] Failed to find data in input_data");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kPerception, -100,
                                "Failed to find data in input_data");
    }
    const PB_common::Variant &data_var = data_it->second;
    humanoid_robot::PB::sdk_service::perception::RequestPerception request_perception;
    auto protobuf_convert_status = request_perception.ParseFromString(data_var.bytevalue());
    if (!protobuf_convert_status) {
        WLOG_ERROR("[Perception] Failed to parse data to RequestPerception");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kPerception, -100,
                                "Failed to parse data to RequestPerception");
    }
    // auto image_it = data_it.image();
   const humanoid_robot::PB::common::Image &image_from_request = request_perception.image();
    
    if (!request_perception.has_image()) {
        WLOG_ERROR("[Perception] RequestPerception has no valid Image field");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kPerception, -101,
                                "RequestPerception missing Image field");
    }

    humanoid_robot::PB::common::Image image_request;
    image_request.CopyFrom(image_from_request); // 拷贝原始 Image 中的所有有效数据

    image_request.set_requiresmasks(true);

    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count();
    image_request.set_timestamp(std::to_string(timestamp));


    // 创建客户端上下文并设置超时
    grpc::ClientContext context;
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(10);
    context.set_deadline(deadline);

    // 调用GetPerceptionResult (流式接口)
    auto stream = pImpl_->stub_->GetPerceptionResult(&context);

    // 发送图像
    if (!stream->Write(image_request)) {
      return ModuleResult::Error(
          "Perception", PerceptionCommandCode::kPerception,
          -3, "Failed to send image to perception service");
    }

    // 完成写入
    stream->WritesDone();

    // 读取响应
    RePerception response;
    std::vector<RePerception> responses;
    ResponsePerception final_response;

    while (stream->Read(&response)) {
      responses.push_back(response);
    }

    // 检查状态
    grpc::Status status = stream->Finish();
    if (!status.ok()) {
      return ModuleResult::Error(
          "Perception", PerceptionCommandCode::kPerception,
          status.error_code(),
          "Perception service error: " + status.error_message());
    }

    auto result_data = std::make_unique<PB_common::Dictionary>();
    auto result_kv = result_data->mutable_keyvaluelist();
    PB_common::Variant pb_response_str;

    for (size_t i = 0; i < responses.size(); ++i) {
      RePerception* temp_perception = final_response.add_singel_perception();
      temp_perception->CopyFrom(responses[i]);
    } 
    std::string pb_response_serialized_str;
    auto pb_serialize_result = final_response.SerializeToString(&pb_response_serialized_str);
    if (!pb_serialize_result) {
        WLOG_ERROR("[Perception] Failed to serialize response_perception");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kPerception, -100,
                                "Failed to serialize response_perception");
    }

    pb_response_str.set_bytevalue(pb_response_serialized_str);
    result_kv->insert({"data", pb_response_str});
    WLOG_DEBUG("[Perception] Successfully processed perception request, got %zd "
               "results",
               responses.size());

    return ModuleResult::Success(
        "Perception", PerceptionCommandCode::kPerception,
        std::move(result_data));
  } catch (const std::exception &e) {
    return ModuleResult::Error(
        "Perception", PerceptionCommandCode::kPerception, -4,
        "Exception in perception processing: " + std::string(e.what()));
  }
}

ModuleResult PerceptionModule::Detection(
    // grpc::ServerContext *&context,
    const humanoid_robot::PB::common::Dictionary &input_data,
    const humanoid_robot::PB::common::Dictionary &params) {
  WLOG_DEBUG("Processing detection command");
  if (!pImpl_->connected_ || !pImpl_->stub_) {
    return ModuleResult::Error(
        "Perception", PerceptionCommandCode::kDetection, -1,
        "Perception service not connected");
  }

  try {
    // 从input_data中提取图像数据
    auto data_it = input_data.keyvaluelist().find("request_detection");
    if (data_it == input_data.keyvaluelist().end()) {
        WLOG_ERROR("[Perception] Failed to find data in input_data");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDetection, -100,
                                "Failed to find data in input_data");
    }
    const PB_common::Variant &data_var = data_it->second;
    humanoid_robot::PB::sdk_service::perception::RequestDetection request_detection;
    auto protobuf_convert_status = request_detection.ParseFromString(data_var.bytevalue());
    if (!protobuf_convert_status) {
        WLOG_ERROR("[Perception] Failed to parse data to RequestDetection");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDetection, -100,
                                "Failed to parse data to RequestDetection");
    }
    // auto image_it = data_it.image();
   const humanoid_robot::PB::common::Image &image_from_request = request_detection.image();
    
    if (!request_detection.has_image()) {
        WLOG_ERROR("[Perception] RequestDetection has no valid Image field");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDetection, -101,
                                "RequestDetection missing Image field");
    }
    WLOG_DEBUG("Image data found in input_data");
    // 构造图像请求
    
    humanoid_robot::PB::common::Image image_request;
    image_request.CopyFrom(image_from_request); // 拷贝原始 Image 中的所有有效数据

    image_request.set_requiresmasks(false);

    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count();
    image_request.set_timestamp(std::to_string(timestamp));
    
    // 创建客户端上下文并设置超时
    grpc::ClientContext context;
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(10);
    context.set_deadline(deadline);

    // 调用GetDetectionResult
    ReDetection response;
    grpc::Status status =
        pImpl_->stub_->GetDetectionResult(&context, image_request, &response);

    if (!status.ok()) {
      return ModuleResult::Error(
          "Perception", PerceptionCommandCode::kDetection,
          status.error_code(),
          "Detection service error: " + status.error_message());
    }
    WLOG_DEBUG("[Perception] Detection request processed successfully");
    // 构造返回结果
    
    ResponseDetection pb_response;
    pb_response.set_ret(0);
    pb_response.mutable_detection()->CopyFrom(response);

    auto result_data = std::make_unique<PB_common::Dictionary>();
    auto result_kv = result_data->mutable_keyvaluelist();
    PB_common::Variant pb_response_str;

    std::string pb_response_serialized_str;
    auto pb_serialize_result = pb_response.SerializeToString(&pb_response_serialized_str);
    if (!pb_serialize_result) {
        WLOG_ERROR("[Perception] Failed to serialize response_detection");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDetection, -100,
                                "Failed to serialize response_detection");
    }

    pb_response_str.set_bytevalue(pb_response_serialized_str);
    result_kv->insert({"data", pb_response_str});
    
    WLOG_DEBUG("[Perception] Successfully processed detection request, got %d "
               "detections",
               response.rows_size());

    return ModuleResult::Success(
        "Perception", PerceptionCommandCode::kDetection,
        std::move(result_data));
  } catch (const std::exception &e) {
    return ModuleResult::Error(
        "Perception", PerceptionCommandCode::kDetection, -4,
        "Exception in detection processing: " + std::string(e.what()));
  }
}

ModuleResult PerceptionModule::Division(
    // grpc::ServerContext *&context,
    const humanoid_robot::PB::common::Dictionary &input_data,
    const humanoid_robot::PB::common::Dictionary &params) {
  WLOG_DEBUG("Processing division command");
  if (!pImpl_->connected_ || !pImpl_->stub_) {
    return ModuleResult::Error(
        "Perception", PerceptionCommandCode::kDivision, -1,
        "Perception service not connected");
  }

  try {
    // 从input_data中提取图像数据
    auto data_it = input_data.keyvaluelist().find("request_division");
    if (data_it == input_data.keyvaluelist().end()) {
        WLOG_ERROR("[Perception] Failed to find data in input_data");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDivision, -100,
                                "Failed to find data in input_data");
    }
    const PB_common::Variant &data_var = data_it->second;
    humanoid_robot::PB::sdk_service::perception::RequestDivision request_division;
    auto protobuf_convert_status = request_division.ParseFromString(data_var.bytevalue());
    if (!protobuf_convert_status) {
        WLOG_ERROR("[Perception] Failed to parse data to RequestDivision");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDivision, -100,
                                "Failed to parse data to RequestDivision");
    }
    // auto image_it = data_it.image();
   const humanoid_robot::PB::common::Image &image_from_request = request_division.image();
    
    if (!request_division.has_image()) {
        WLOG_ERROR("[Perception] RequestDivision has no valid Image field");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDivision, -101,
                                "RequestDivision missing Image field");
    }

    WLOG_DEBUG("Image data found in input_data for division");
    // 构造图像请求
    humanoid_robot::PB::common::Image image_request;
    image_request.CopyFrom(image_from_request); // 拷贝原始 Image 中的所有有效数据

    image_request.set_requiresmasks(true);

    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count();
    image_request.set_timestamp(std::to_string(timestamp));
    // 创建客户端上下文并设置超时
    grpc::ClientContext context;
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(10);
    context.set_deadline(deadline);

    // 调用GetDivisionResult
    ReDivision response;
    grpc::Status status =
        pImpl_->stub_->GetDivisionResult(&context, image_request, &response);

    if (!status.ok()) {
      return ModuleResult::Error(
          "Perception", PerceptionCommandCode::kDivision,
          status.error_code(),
          "Division service error: " + status.error_message());
    }

    // 构造返回结果
    auto result_data = std::make_unique<PB_common::Dictionary>();
    auto result_kv = result_data->mutable_keyvaluelist();
    PB_common::Variant pb_response_str;
    ResponseDivision pb_response;
    pb_response.set_ret(0);
    pb_response.mutable_division()->CopyFrom(response);

    std::string pb_response_serialized_str;
    auto pb_serialize_result = pb_response.SerializeToString(&pb_response_serialized_str);
    if (!pb_serialize_result) {
        WLOG_ERROR("[Perception] Failed to serialize response_division");
        return ModuleResult::Error("Perception",
                                PerceptionCommandCode::kDivision, -100,
                                "Failed to serialize response_division");
    }

    pb_response_str.set_bytevalue(pb_response_serialized_str);
    result_kv->insert({"data", pb_response_str});
    WLOG_DEBUG("[Perception] Successfully processed division request, got %d "
               "divisions",
               response.rows_size());

    return ModuleResult::Success(
        "Perception", PerceptionCommandCode::kDivision,
        std::move(result_data));
  } catch (const std::exception &e) {
    return ModuleResult::Error(
        "Perception", PerceptionCommandCode::kDivision, -4,
        "Exception in division processing: " + std::string(e.what()));
  }
}

void PerceptionModule::SimulateProcessingDelay(int min_ms, int max_ms) const {
  static thread_local std::random_device rd;
  static thread_local std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(min_ms, max_ms);

  int delay = dis(gen);
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

} // namespace modules
} // namespace server
} // namespace humanoid_robot
