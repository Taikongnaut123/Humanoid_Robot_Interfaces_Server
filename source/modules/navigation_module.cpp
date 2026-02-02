#include "modules/navigation_module.h"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <unordered_map>

#include "Log/wlog.hpp"
#include "absl/strings/escaping.h"
#include "common/service.pb.h"
#include "communication/communication_service.pb.h"
#include "communication_v2/net/CNode.h"
#include "communication_v2/net/CNode.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "loc_msg/msg/req_pose_msg.hpp"
#include "loc_msg/msg/res_remaining_distance.hpp"
#include "loc_msg/msg/res_start_nav.hpp"
#include "loc_msg/msg/res_status.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "ros2/action_msgs/GoalStatus.pb.h"
#include "ros2/geometry_msgs/Pose.pb.h"
#include "ros2/nav_msgs/Goals.pb.h"
#include "ros2/nav_msgs/OccupancyGrid.pb.h"
#include "sdk_service/common/service.pb.h"
#include "sdk_service/navigation/req_pose_msg.pb.h"
#include "sdk_service/navigation/request_cancel_navigation.pb.h"
#include "sdk_service/navigation/request_charging.pb.h"
#include "sdk_service/navigation/request_grid_map.pb.h"
#include "sdk_service/navigation/request_remaining_distance.pb.h"
#include "sdk_service/navigation/res_start_nav.pb.h"
#include "sdk_service/navigation/res_status.pb.h"
#include "std_msgs/msg/empty.hpp"
#include "utils/message_serializer/RosMessageSerializer.hpp"
#include "utils/ros_proto_converter/ros_proto_converter.h"

namespace humanoid_robot {
namespace server {
namespace modules {
using UniversalRequest = humanoid_robot::PB::communication::UniversalRequest;
using UniversalResponse = humanoid_robot::PB::communication::UniversalResponse;
using CNode = humanoid_robot::framework::net::CNode;
using NavigationCommandCode =
    humanoid_robot::PB::sdk_service::common::NavigationCommandCode;
using CServiceClient =
    humanoid_robot::framework::net::CServiceClient<UniversalRequest,
                                                   UniversalResponse>;
using CServiceServer =
    humanoid_robot::framework::net::CServiceServer<UniversalRequest,
                                                   UniversalResponse>;
using Dictionary = humanoid_robot::PB::common::Dictionary;
using Variant = humanoid_robot::PB::common::Variant;

using ReflectionConverter =
    humanoid_robot::framework::utils::ros_proto_converter::ReflectionConverter;
using ConvertResult =
    humanoid_robot::framework::utils::ros_proto_converter::ConvertResult;
using ConvertOptions =
    humanoid_robot::framework::utils::ros_proto_converter::ConvertOptions;
namespace message_serializer =
    humanoid_robot::framework::utils::message_serializer;

using RequestGridMap =
    humanoid_robot::PB::sdk_service::navigation::RequestGridMap;
using OccupancyGridPb = humanoid_robot::PB::ros2::nav_msgs::OccupancyGrid;
using PosePb = humanoid_robot::PB::ros2::geometry_msgs::Pose;
using ReqPoseMsgPb = humanoid_robot::PB::sdk_service::navigation::ReqPoseMsg;
using ResStartNavPb = humanoid_robot::PB::sdk_service::navigation::ResStartNav;
using GoalsPb = humanoid_robot::PB::ros2::nav_msgs::Goals;

using RequestRemainingDistancePb =
    humanoid_robot::PB::sdk_service::navigation::RequestRemainingDistance;
using ResponseRemainingDistancePb =
    humanoid_robot::PB::sdk_service::navigation::ResponseRemainingDistance;
using RequestCancelNavigationPb =
    humanoid_robot::PB::sdk_service::navigation::RequestCancelNavigation;
using ResponseCancelNavigationPb =
    humanoid_robot::PB::sdk_service::navigation::ResponseCancelNavigation;
using RequestStartChargingPb =
    humanoid_robot::PB::sdk_service::navigation::RequestStartCharging;
using ResponseStartChargingPb =
    humanoid_robot::PB::sdk_service::navigation::ResponseStartCharging;
using RequestStopChargingPb =
    humanoid_robot::PB::sdk_service::navigation::RequestStopCharging;
using ResponseStopChargingPb =
    humanoid_robot::PB::sdk_service::navigation::ResponseStopCharging;

namespace {
// 通用常量定义 - 消除硬编码/魔法值
constexpr char* const kNavigationServiceTarget = "navigation_service";
constexpr char* const kRequestDataKey = "request_data";
constexpr char* const kResponseDataKey = "data";
constexpr char* const kStatusFieldName = "status";
constexpr char* const kPayloadFieldName = "payload";
constexpr int32_t kDefaultErrorCode = -100;
constexpr std::chrono::milliseconds kRealServerTimeoutMs{30000};
constexpr std::chrono::seconds kServiceWaitTimeout = std::chrono::seconds(5);
constexpr std::chrono::seconds kRunningCheckTimeout = std::chrono::seconds(3);

// 错误信息模板 - 统一日志/返回信息
constexpr char* const kErrMsgMissingInputData =
    "Failed to find %s in input_data";
constexpr char* const kErrMsgParsePbFailed =
    "Failed to parse data to PB message";
constexpr char* const kErrMsgConvertRosToPb =
    "Failed to convert ROS to PB message";
constexpr char* const kErrMsgConvertPbToRos =
    "Failed to convert PB to ROS message";
constexpr char* const kErrMsgSerializePb = "Failed to serialize PB message";
constexpr char* const kErrMsgNoResponse = "No response from navigation service";
constexpr char* const kErrMsgInvalidResponse =
    "Invalid response structure from navigation service";
constexpr char* const kErrMsgServiceError =
    "Navigation service returned error code: 0x%x, msg: %s";
constexpr char* const kErrMsgDeserializeRos =
    "Failed to deserialize ROS message from payload, size: %d";
constexpr char* const kErrMsgClientNotConnected =
    "Navigation service client not connected";
constexpr char* const kErrMsgUnknownCommand = "Unknown Navigation command: %s";

using EmptyRequestRos = std_msgs::msg::Empty;
using EmptyRequestPb = google::protobuf::Message;

// 序列化器单例管理 - 统一管理，避免静态变量散列
class NavigationSerializers {
 public:
  // EmptyRequestRos
  static message_serializer::RosMessageSerializer<EmptyRequestRos>&
  GetEmptyRequestRosSerializer() {
    static message_serializer::RosMessageSerializer<EmptyRequestRos> serializer;
    return serializer;
  }
  static message_serializer::RosMessageSerializer<loc_msg::msg::ReqPoseMsg>&
  GetReqPoseSerializer() {
    static message_serializer::RosMessageSerializer<loc_msg::msg::ReqPoseMsg>
        serializer;
    return serializer;
  }

  // ResStatus
  static message_serializer::RosMessageSerializer<loc_msg::msg::ResStatus>&
  GetResStatusSerializer() {
    static message_serializer::RosMessageSerializer<loc_msg::msg::ResStatus>
        serializer;
    return serializer;
  }

  static message_serializer::RosMessageSerializer<geometry_msgs::msg::Pose>&
  GetGeometryPoseSerializer() {
    static message_serializer::RosMessageSerializer<geometry_msgs::msg::Pose>
        serializer;
    return serializer;
  }

  static message_serializer::RosMessageSerializer<nav_msgs::msg::OccupancyGrid>&
  GetGridMapSerializer() {
    static message_serializer::RosMessageSerializer<
        nav_msgs::msg::OccupancyGrid>
        serializer;
    return serializer;
  }

  static message_serializer::RosMessageSerializer<nav_msgs::msg::Goals>&
  GetGoalsSerializer() {
    static message_serializer::RosMessageSerializer<nav_msgs::msg::Goals>
        serializer;
    return serializer;
  }

  static message_serializer::RosMessageSerializer<loc_msg::msg::ResStartNav>&
  GetResStartNavSerializer() {
    static message_serializer::RosMessageSerializer<loc_msg::msg::ResStartNav>
        serializer;
    return serializer;
  }
  // ResponseRemainingDistance
  static message_serializer::RosMessageSerializer<
      loc_msg::msg::ResRemainingDistance>&
  GetResRemainingDistanceSerializer() {
    static message_serializer::RosMessageSerializer<
        loc_msg::msg::ResRemainingDistance>
        serializer;
    return serializer;
  }
};
// 序列化器类型推导辅助函数 - 简化模板调用
template <typename T>
auto& GetSerializer();

template <>
auto& GetSerializer<loc_msg::msg::ResStatus>() {
  return NavigationSerializers::GetResStatusSerializer();
}

template <>
auto& GetSerializer<geometry_msgs::msg::Pose>() {
  return NavigationSerializers::GetGeometryPoseSerializer();
}

template <>
auto& GetSerializer<nav_msgs::msg::OccupancyGrid>() {
  return NavigationSerializers::GetGridMapSerializer();
}

template <>
auto& GetSerializer<loc_msg::msg::ReqPoseMsg>() {
  return NavigationSerializers::GetReqPoseSerializer();
}

template <>
auto& GetSerializer<nav_msgs::msg::Goals>() {
  return NavigationSerializers::GetGoalsSerializer();
}
template <>
auto& GetSerializer<loc_msg::msg::ResStartNav>() {
  return NavigationSerializers::GetResStartNavSerializer();
}
template <>
auto& GetSerializer<EmptyRequestRos>() {
  return NavigationSerializers::GetEmptyRequestRosSerializer();
}
template <>
auto& GetSerializer<loc_msg::msg::ResRemainingDistance>() {
  return NavigationSerializers::GetResRemainingDistanceSerializer();
}
// 通用导航请求模板函数
template <typename RequestRosType, typename RequestPbType,
          typename ResponseRosType, typename ResponsePbType>
ModuleResult SendNavigationRequest(
    const std::shared_ptr<CServiceClient>& service_client,
    NavigationCommandCode command_code, const Dictionary& input_data,
    bool has_request_params = true) {  // 新增：是否有请求参数，默认true
  // 1. 构造通用请求体
  UniversalRequest request;
  request.set_command(command_code);
  request.set_sendrequesttimestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());

  // 2. 有请求参数：解析+转换+设置payload；无参数：跳过此步骤
  if (has_request_params) {
    // 解析input_data中的request_data
    auto data_it = input_data.keyvaluelist().find(kRequestDataKey);
    if (data_it == input_data.keyvaluelist().end()) {
      char err_msg[256] = {0};
      snprintf(err_msg, sizeof(err_msg), kErrMsgMissingInputData,
               kRequestDataKey);
      WLOG_ERROR("[Navigation] %s", err_msg);
      return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                                 err_msg);
    }
    const Variant& data_var = data_it->second;

    // PB反序列化
    RequestPbType req_pb;
    if (!req_pb.ParseFromString(data_var.bytevalue())) {
      WLOG_ERROR("[Navigation] %s", kErrMsgParsePbFailed);
      return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                                 kErrMsgParsePbFailed);
    }

    // PB→ROS转换
    RequestRosType req_ros;
    ConvertOptions convert_options;
    // convert_options.log_conversion_details = true;
    auto request_convert_result =
        ReflectionConverter::PbToRos(req_pb, req_ros, convert_options);
    if (request_convert_result != ConvertResult::SUCCESS) {
      WLOG_ERROR("[Navigation] %s", kErrMsgConvertPbToRos);
      return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                                 kErrMsgConvertPbToRos);
    }

    // 设置请求payload
    auto& request_serializer = GetSerializer<RequestRosType>();
    request.set_payload(request_serializer.SerializeToString(req_ros));
    request.set_payloadsize(request.payload().size());  // 设置payload长度
  } else {
    // 无请求参数：payload置空（也可根据服务端要求不设置）
    request.set_payload("");
    request.set_payloadsize(0);
    WLOG_DEBUG("[Navigation] No request params for command: %d", command_code);
  }

  // 3. 同步调用服务（有/无参数逻辑一致）
  auto response = service_client->Call(request, kRealServerTimeoutMs);
  if (!response) {
    WLOG_ERROR("[Navigation] %s", kErrMsgNoResponse);
    return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                               kErrMsgNoResponse);
  }

  // 4. 验证响应结构合法性（有/无参数逻辑一致）
  const google::protobuf::Descriptor* desc = response->GetDescriptor();
  const google::protobuf::FieldDescriptor* status_field =
      desc->FindFieldByName(kStatusFieldName);
  const google::protobuf::FieldDescriptor* payload_field =
      desc->FindFieldByName(kPayloadFieldName);
  if (!(status_field && payload_field)) {
    WLOG_ERROR("[Navigation] %s", kErrMsgInvalidResponse);
    return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                               kErrMsgInvalidResponse);
  }

  // 5. 检查服务端返回状态（有/无参数逻辑一致）
  int32_t navi_svr_status = response->status();
  if (navi_svr_status != 0) {
    char err_msg[256] = {0};
    snprintf(err_msg, sizeof(err_msg), kErrMsgServiceError, navi_svr_status,response->payload().c_str());

    WLOG_ERROR("[Navigation] %s", err_msg);
    return ModuleResult::Error("Navigation", command_code, navi_svr_status,
                               err_msg);
  }

  // 6. 反序列化响应为ROS消息（有/无参数逻辑一致）
  ResponseRosType ros_response;
  auto& response_serializer = GetSerializer<ResponseRosType>();
  bool deserialize_ret = response_serializer.DeserializeFromString(response->payload(), ros_response);
  if (!deserialize_ret) {
    char err_msg[256] = {0};
    snprintf(err_msg, sizeof(err_msg), kErrMsgDeserializeRos, response->payloadsize());
    WLOG_ERROR("[Navigation] %s", err_msg);
    return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                               err_msg);
  }

  // 7. ROS→PB转换（有/无参数逻辑一致）
  ResponsePbType pb_response;
  ConvertOptions convert_options;
  // convert_options.log_conversion_details = true;
  auto convert_result =
      ReflectionConverter::RosToPb(ros_response, pb_response, convert_options);
  if (convert_result != ConvertResult::SUCCESS) {
    WLOG_ERROR("[Navigation] %s", kErrMsgConvertRosToPb);
    return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                               kErrMsgConvertRosToPb);
  }

  // 8. 构造返回结果（有/无参数逻辑一致）
  std::string pb_serialized_str;
  if (!pb_response.SerializeToString(&pb_serialized_str)) {
    WLOG_ERROR("[Navigation] %s", kErrMsgSerializePb);
    return ModuleResult::Error("Navigation", command_code, kDefaultErrorCode,
                               kErrMsgSerializePb);
  }
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();
  Variant pb_var;
  pb_var.set_bytevalue(pb_serialized_str);
  result_kv_map->insert({kResponseDataKey, pb_var});

  return ModuleResult::Success("Navigation", command_code,
                               std::move(result_data));
}
// 命令处理器接口 - 解耦业务逻辑与模块核心类
class INavigationCommandHandler {
 public:
  virtual ~INavigationCommandHandler() = default;
  virtual ModuleResult Handle(
      const Dictionary& input_data, const Dictionary& params,
      const std::shared_ptr<CServiceClient>& service_client) = 0;
};

// 获取当前位姿命令处理器
class GetCurrentPoseHandler : public INavigationCommandHandler {
 public:
  ModuleResult Handle(
      const Dictionary& input_data, const Dictionary&,
      const std::shared_ptr<CServiceClient>& service_client) override {
    WLOG_DEBUG("[Navigation] Handling GetCurrentPose command");
    return SendNavigationRequest<loc_msg::msg::ReqPoseMsg, ReqPoseMsgPb,
                                 geometry_msgs::msg::Pose, PosePb>(
        service_client, NavigationCommandCode::kGetCurrentPose, input_data);
  }
};

// 获取2D网格地图命令处理器
class GetGridMap2DHandler : public INavigationCommandHandler {
 public:
  ModuleResult Handle(
      const Dictionary& input_data, const Dictionary&,
      const std::shared_ptr<CServiceClient>& service_client) override {
    WLOG_DEBUG("[Navigation] Handling GetGridMap2D command");
    return SendNavigationRequest<EmptyRequestRos, RequestGridMap,
                                 nav_msgs::msg::OccupancyGrid, OccupancyGridPb>(
        service_client, NavigationCommandCode::kGetGridMap2D, input_data,
        false /*no request_params*/);
  }
};

// 导航到目标点命令处理器
class NavigationToHandler : public INavigationCommandHandler {
 public:
  ModuleResult Handle(
      const Dictionary& input_data, const Dictionary&,
      const std::shared_ptr<CServiceClient>& service_client) override {
    WLOG_DEBUG("[Navigation] Handling NavigationTo command");
    return SendNavigationRequest<nav_msgs::msg::Goals, GoalsPb,
                                 loc_msg::msg::ResStartNav, ResStartNavPb>(
        service_client, NavigationCommandCode::kNavigationTo, input_data);
  }
};

// 取消导航命令处理器
class CancelNavigationTaskHandler : public INavigationCommandHandler {
 public:
  ModuleResult Handle(
      const Dictionary& input_data, const Dictionary&,
      const std::shared_ptr<CServiceClient>& service_client) override {
    WLOG_DEBUG("[Navigation] Handling CancelNavigationTask command");
    return SendNavigationRequest<EmptyRequestRos, RequestCancelNavigationPb,
                                 loc_msg::msg::ResStatus,
                                 ResponseCancelNavigationPb>(
        service_client, NavigationCommandCode::kCancelNavigationTask,
        input_data, false /*no request_params*/);
  }
};

// 获取剩余路径距离命令处理器
class GetRemainingDistanceHandler : public INavigationCommandHandler {
 public:
  ModuleResult Handle(
      const Dictionary& input_data, const Dictionary&,
      const std::shared_ptr<CServiceClient>& service_client) override {
    WLOG_DEBUG("[Navigation] Handling GetRemainingPathDistance command");
    return SendNavigationRequest<EmptyRequestRos, RequestRemainingDistancePb,
                                 loc_msg::msg::ResRemainingDistance,
                                 ResponseRemainingDistancePb>(
        service_client, NavigationCommandCode::kGetRemainingPathDistance,
        input_data, false /*no request_params*/);
  }
};
// 开始充电命令处理器
class StartChargingHandler : public INavigationCommandHandler {
 public:
  ModuleResult Handle(
      const Dictionary& input_data, const Dictionary&,
      const std::shared_ptr<CServiceClient>& service_client) override {
    WLOG_DEBUG("[Navigation] Handling StartCharging command");
    return SendNavigationRequest<EmptyRequestRos, RequestStartChargingPb,
                                 loc_msg::msg::ResStatus,
                                 ResponseStartChargingPb>(
        service_client, NavigationCommandCode::kStartCharging, input_data,
        false /*no request_params*/);
  }
};

// 停止充电命令处理器
class StopChargingHandler : public INavigationCommandHandler {
 public:
  ModuleResult Handle(
      const Dictionary& input_data, const Dictionary&,
      const std::shared_ptr<CServiceClient>& service_client) override {
    WLOG_DEBUG("[Navigation] Handling StopCharging command");
    return SendNavigationRequest<EmptyRequestRos, RequestStopChargingPb,
                                 loc_msg::msg::ResStatus,
                                 ResponseStopChargingPb>(
        service_client, NavigationCommandCode::kStopCharging, input_data,
        false /*no request_params*/);
  }
};

}  // namespace
// ************************ 原有PImpl类（最小改动）************************
class NavigationModule::NavigationImpl {
 public:
  CNode node_;
  std::shared_ptr<CServiceClient> service_client_;
  std::string server_target_;
  bool connected_;

  NavigationImpl() : connected_(false), node_("navigation_node_sdk") {};
  ~NavigationImpl() = default;
};

// 命令处理器映射成员变量（类内声明）
std::unordered_map<int32_t, std::unique_ptr<INavigationCommandHandler>>
    command_handlers_;
// ************************
// 模块核心类（仅保留生命周期+命令分发）************************
NavigationModule::NavigationModule()
    : ModuleBase("Navigation", 3),  // 3个工作线程
      pImpl_(std::make_unique<NavigationImpl>()) {
  // 初始化命令处理器映射 - 新增命令仅需在此添加，符合开闭原则
  command_handlers_[NavigationCommandCode::kGetCurrentPose] =
      std::make_unique<GetCurrentPoseHandler>();
  command_handlers_[NavigationCommandCode::kGetGridMap2D] =
      std::make_unique<GetGridMap2DHandler>();
  command_handlers_[NavigationCommandCode::kNavigationTo] =
      std::make_unique<NavigationToHandler>();
  command_handlers_[NavigationCommandCode::kCancelNavigationTask] =
      std::make_unique<CancelNavigationTaskHandler>();
  command_handlers_[NavigationCommandCode::kGetRemainingPathDistance] =
      std::make_unique<GetRemainingDistanceHandler>();
  command_handlers_[NavigationCommandCode::kStartCharging] =
      std::make_unique<StartChargingHandler>();
  command_handlers_[NavigationCommandCode::kStopCharging] =
      std::make_unique<StopChargingHandler>();
}

NavigationModule::~NavigationModule() = default;

bool NavigationModule::Initialize() {
  try {
    pImpl_->server_target_ = kNavigationServiceTarget;
    // 创建 Service Client（会自动从 DDS 发现 Server）
    humanoid_robot::framework::net::CClientOptions client_options;
    client_options.enable_dds_discovery =
        true;                          // 从 DDS 自动发现 Service Server
    client_options.prefer_uds = true;  // 同机通信优先使用 UDS
    client_options.default_timeout = kServiceWaitTimeout;

    pImpl_->service_client_ =
        pImpl_->node_.CreateServiceClient<UniversalRequest, UniversalResponse>(
            pImpl_->server_target_, client_options);
    if (!pImpl_->service_client_) {
      WLOG_ERROR("[Navigation] Failed to create service client for %s",
                 pImpl_->server_target_.c_str());
      return false;
    }

    if (!pImpl_->service_client_->WaitForService(kServiceWaitTimeout)) {
      WLOG_ERROR("[Navigation] Failed to connect to %s",
                 pImpl_->server_target_.c_str());
      return false;
    }

    pImpl_->connected_ = true;
    WLOG_INFO("[Navigation] Navigation module initialize success");
    return true;
  } catch (const std::exception& e) {
    WLOG_FATAL("[Navigation] Exception during initialization: %s", e.what());
    return false;
  }
}

bool NavigationModule::IsRunning() {
  if (!pImpl_->service_client_) return false;
  pImpl_->connected_ = pImpl_->service_client_->WaitForService(kRunningCheckTimeout);
  return pImpl_->connected_;
}

void NavigationModule::Cleanup() {
  WLOG_INFO("Cleaning up navigation resources...");
  if (pImpl_->connected_) {
    pImpl_->connected_ = false;
    pImpl_->service_client_.reset();  // 显式释放客户端，避免内存泄漏
  }
  WLOG_INFO("[Navigation] Navigation module cleanup completed");
}

ModuleResult NavigationModule::ProcessCommand(grpc::ServerContext* /*context*/,
                                              const int32_t& command_id,
                                              const Dictionary& input_data,
                                              const Dictionary& params) {
  WLOG_DEBUG("[Navigation] Processing command in NavigationModule: %d",
             command_id);

  // 1. 查找命令处理器
  auto handler_it = command_handlers_.find(command_id);
  if (handler_it == command_handlers_.end()) {
    char err_msg[256] = {0};
    snprintf(err_msg, sizeof(err_msg), kErrMsgUnknownCommand, command_id);
    WLOG_WARN("[Navigation] %s", err_msg);
    return ModuleResult::Error("Navigation", command_id, kDefaultErrorCode,
                               err_msg);
  }

  // 2. 检查客户端连接状态
  if (!pImpl_->service_client_ || !pImpl_->connected_) {
    WLOG_ERROR("[Navigation] %s", kErrMsgClientNotConnected);
    return ModuleResult::Error("Navigation", command_id, kDefaultErrorCode,
                               kErrMsgClientNotConnected);
  }

  // 3. 分发命令到对应处理器执行
  return handler_it->second->Handle(input_data, params,
                                    pImpl_->service_client_);
}

}  // namespace modules
}  // namespace server
}  // namespace humanoid_robot