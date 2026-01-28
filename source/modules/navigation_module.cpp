#include "modules/navigation_module.h"

#include <chrono>
#include <iostream>
#include <random>
#include <thread>

#include "Log/wlog.hpp"
#include "absl/strings/escaping.h"
#include "common/service.pb.h"
#include "communication/communication_service.pb.h"
#include "communication_v2/net/CNode.h"
#include "communication_v2/net/CNode.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "loc_msg/msg/req_pose_msg.hpp"
#include "loc_msg/msg/res_start_nav.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "ros2/action_msgs/GoalStatus.pb.h"
#include "ros2/geometry_msgs/Pose.pb.h"
#include "sdk_service/common/service.pb.h"
#include "sdk_service/navigation/req_pose_msg.pb.h"
#include "sdk_service/navigation/res_start_nav.pb.h"
#include "sdk_service/navigation/res_status.pb.h"
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
namespace message_serializer =
    humanoid_robot::framework::utils::message_serializer;

class NavigationModule::NavigationImpl {
public:
  CNode node_;

  // 发布订阅
  // std::shared_ptr<humanoid_robot::framework::net::CPublisher<NavigationTopicMessage>>
  // publisher_; std::unique_ptr<NavigationService::Stub> publisher_stub_;

  // std::shared_ptr<humanoid_robot::framework::net::CSubscriber<PoseMessage>>
  // pose_subscriber_;
  // std::shared_ptr<humanoid_robot::framework::net::CSubscriber<GridMap2DMessage>>
  // map_subscriber_;
  // std::shared_ptr<humanoid_robot::framework::net::CSubscriber<PathMessage>>
  // path_subscriber_;

  // 请求响应
  // std::shared_ptr<CServiceServer<NavigationRequest,
  // NavigationResponse>> service_server_;

  std::shared_ptr<CServiceClient> service_client_;

  std::string server_target_;
  bool connected_;

  NavigationImpl() : connected_(false), node_("navigation_node_sdk"){};

  ~NavigationImpl() {
    if (connected_) {
      // Graceful shutdown could be implemented here
    }
  };
  static message_serializer::RosMessageSerializer<loc_msg::msg::ReqPoseMsg>
      req_pose_serializer;
  static message_serializer::RosMessageSerializer<geometry_msgs::msg::Pose>
      geometry_pose_serializer;
};

NavigationModule::NavigationModule()
    : ModuleBase("Navigation", 3), // 3个工作线程
      pImpl_(std::make_unique<NavigationImpl>()) {}

NavigationModule::~NavigationModule() = default;

bool NavigationModule::Initialize() {
  try {
    pImpl_->server_target_ = "navigation_service";
    // 创建 Service Client（会自动从 DDS 发现 Server）
    humanoid_robot::framework::net::CClientOptions client_options;
    // client_options.service_address = "192.168.20.190:50052";  //
    // 手动配置（可选）
    client_options.enable_dds_discovery =
        true;                         // 从 DDS 自动发现 Service Server
    client_options.prefer_uds = true; // 同机通信优先使用 UDS
    // client_options.prefer_uds = false;           // 同机通信优先使用 UDS
    client_options.default_timeout = std::chrono::seconds(5);

    // std::cout << "Creating service client for 'echo_service'..." <<
    // std::endl; std::cout << "Waiting for service discovery via DDS..." <<
    // std::endl;

    pImpl_->service_client_ =
        pImpl_->node_.CreateServiceClient<UniversalRequest, UniversalResponse>(
            pImpl_->server_target_, client_options);
    if (!pImpl_->service_client_) {
      WLOG_ERROR("[Navigation] Failed to create service client for %s",
                 pImpl_->server_target_.c_str());
      return false;
    }

    if (!pImpl_->service_client_->WaitForService(std::chrono::seconds(5))) {
      WLOG_ERROR("[Navigation] Failed to connect to %s",
                 pImpl_->server_target_.c_str());
      return false;
    }

    pImpl_->connected_ = true;
    WLOG_INFO("[Navigation] Navigation module initialize");
    return true;
  }

  catch (const std::exception &e) {
    WLOG_FATAL("[Navigation] Exception during initialization: %s", e.what());
    return false;
  }
}

bool NavigationModule::IsRunning() {
  if (!pImpl_->service_client_)
    return false;
  auto client_connected =
      pImpl_->service_client_->WaitForService(std::chrono::seconds(3));
  return client_connected;
}

void NavigationModule::Cleanup() {
  WLOG_INFO("Cleaning up navigation resources...");
  if (pImpl_->connected_) {
    pImpl_->connected_ = false;
    // Graceful shutdown could be implemented here

    // 还未实现
    // pImpl_->service_client_.reset();

    // pImpl_->connected_ = false;
  }
  WLOG_INFO("[Navigation] Navigation module cleanup completed");
}

ModuleResult NavigationModule::ProcessCommand(grpc::ServerContext *context,
                                              const int32_t &command_id,
                                              const Dictionary &input_data,
                                              const Dictionary &params) {
  WLOG_DEBUG("[Navigation] Processing command in NavigationModule: %d",
             command_id);
  switch (command_id) {
  // 获取当前位姿
  case NavigationCommandCode::kGetCurrentPose:
    return GetCurrentPose(input_data, params);
  // 获取2D网格地图
  case NavigationCommandCode::kGetGridMap2D:
    return GetGridMap2D(input_data, params);
  // 导航到目标点
  case NavigationCommandCode::kNavigationTo:
    return NavigationTo(input_data, params);
  // 取消导航任务
  case NavigationCommandCode::KCancelNavigationTask:
    return CancelNavigationTask(input_data, params);
  // 获取剩余路径距离
  case NavigationCommandCode::kGetRemainingPathDistance:
    return GetRemainingPathDistance(input_data, params);
  // 开始充电
  case NavigationCommandCode::kStartCharging:
    return StartCharging(input_data, params);
  // 停止充电
  case NavigationCommandCode::kStopCharging:
    return StopCharging(input_data, params);
  // 非法指令
  default:
    WLOG_WARN("[Navigation] Unknown Navigation command: %d", command_id);
    return ModuleResult::Error("Navigation", command_id, -100,
                               "Unknown Navigation command: " + command_id);
  }
}

ModuleResult NavigationModule::GetCurrentPose(const Dictionary &input_data,
                                              const Dictionary &params) {
  WLOG_DEBUG("[Navigation] GetCurrentPose");
  // 构造返回结果
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();

  // 构造请求
  UniversalRequest request;
  request.set_command(NavigationCommandCode::kGetCurrentPose);
  // request.set_version(1);
  // request.set_requestid(1);
  request.set_sendrequesttimestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  // request.set_checksum(1);

  // 将inputdata种中的data反序列化为PB消息
  auto data_it = input_data.keyvaluelist().find("pose_request");
  if (data_it == input_data.keyvaluelist().end()) {
    WLOG_ERROR("[Navigation] Failed to find data in input_data");
    return ModuleResult::Error("Navigation",
                               NavigationCommandCode::kGetCurrentPose, -100,
                               "Failed to find data in input_data");
  }
  const Variant &data_var = data_it->second;
  humanoid_robot::PB::sdk_service::navigation::ReqPoseMsg req_pb;
  auto protobuf_convert_status = req_pb.ParseFromString(data_var.bytevalue());
  if (!protobuf_convert_status) {
    WLOG_ERROR("[Navigation] Failed to parse data to ReqPoseMsg");
    return ModuleResult::Error("Navigation",
                               NavigationCommandCode::kGetCurrentPose, -100,
                               "Failed to parse data to ReqPoseMsg");
  }

  // 转换为ROS消息
  loc_msg::msg::ReqPoseMsg req_ros;

  auto result = ReflectionConverter::PbToRos(req_pb, req_ros);
  if (result != ConvertResult::SUCCESS) {
    WLOG_ERROR("[Navigation] Failed to convert PbToRos");
    return ModuleResult::Error("Navigation",
                               NavigationCommandCode::kGetCurrentPose, -100,
                               "Failed to convert PbToRos");
  }

  std::string serialized_string =
      pImpl_->req_pose_serializer.SerializeToString(req_ros);

  request.set_payload(serialized_string);
  request.set_payloadsize(serialized_string.size());

  // 同步调用服务
  auto response = pImpl_->service_client_->Call(request);

  if (!response) {
    WLOG_ERROR("[Navigation] Failed to get current pose");
    return ModuleResult::Error("Navigation",
                               NavigationCommandCode::kGetCurrentPose, -100,
                               "Failed to get current pose, no response.");
  }

  const google::protobuf::Descriptor *desc = response->GetDescriptor();
  // 查找名为 "status" 的字段（区分大小写）
  const google::protobuf::FieldDescriptor *status_field =
      desc->FindFieldByName("status");
  const google::protobuf::FieldDescriptor *payload_field =
      desc->FindFieldByName("payload");
  if (!(status_field && payload_field)) {
    WLOG_ERROR("[Navigation] Failed to get current pose");
    return ModuleResult::Error(
        "Navigation", NavigationCommandCode::kGetCurrentPose, -100,
        "Failed to get current pose, no valid response.");
  }

  auto navi_svr_status = response->status();
  if (navi_svr_status != 0) {
    WLOG_ERROR("[Navigation] Failed to get current pose");
    return ModuleResult::Error(
        "Navigation", NavigationCommandCode::kGetCurrentPose, navi_svr_status,
        "Failed to get current pose, navigation server error.");
  }

  // 映射返回结果
  geometry_msgs::msg::Pose ros_pose;
  Variant pb_pose_str;

  pImpl_->geometry_pose_serializer.DeserializeFromString(response->payload(),
                                                         ros_pose);

  humanoid_robot::PB::ros2::geometry_msgs::Pose pb_pose;
  auto ros_to_pb_result = ReflectionConverter::RosToPb(ros_pose, pb_pose);

  // std::cout << "ros: \n " << std::endl;
  // std::cout << "    position.x: " << ros_pose.position.x << std::endl;
  // std::cout << "    position.y: " << ros_pose.position.y << std::endl;
  // std::cout << "    position.z: " << ros_pose.position.z << std::endl;
  // std::cout << "    orientation.x: " << ros_pose.orientation.x << std::endl;
  // std::cout << "    orientation.y: " << ros_pose.orientation.y << std::endl;
  // std::cout << "    orientation.z: " << ros_pose.orientation.z << std::endl;
  // std::cout << "    orientation.w: " << ros_pose.orientation.w << std::endl;
  // std::cout << "pb: \n " << pb_pose.DebugString() << std::endl;

  if (ros_to_pb_result != ConvertResult::SUCCESS) {
    WLOG_ERROR("[Navigation] Failed to convert RosToPb");
    return ModuleResult::Error("Navigation",
                               NavigationCommandCode::kGetCurrentPose, -100,
                               "Failed to convert RosToPb");
  }

  std::string pb_pose_serialized_str;
  auto pb_serialize_result = pb_pose.SerializeToString(&pb_pose_serialized_str);
  if (!pb_serialize_result) {
    WLOG_ERROR("[Navigation] Failed to serialize pb_pose");
    return ModuleResult::Error("Navigation",
                               NavigationCommandCode::kGetCurrentPose, -100,
                               "Failed to serialize pb_pose");
  }

  pb_pose.SerializeToString(&pb_pose_serialized_str);

  pb_pose_str.set_bytevalue(pb_pose_serialized_str);
  result_kv_map->insert({"data", pb_pose_str});

  return ModuleResult::Success("Navigation",
                               NavigationCommandCode::kGetCurrentPose,
                               std::move(result_data));
}

ModuleResult NavigationModule::GetGridMap2D(const Dictionary &input_data,
                                            const Dictionary &params) {
  WLOG_DEBUG("[Navigation] GetGridMap2D");
  // 构造返回结果
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();
  return ModuleResult::Success("Navigation",
                               NavigationCommandCode::kGetGridMap2D,
                               std::move(result_data));
}

ModuleResult NavigationModule::NavigationTo(const Dictionary &input_data,
                                            const Dictionary &params) {
  WLOG_DEBUG("[Navigation] NavigationTo");
  // 构造返回结果
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();
  return ModuleResult::Success("Navigation",
                               NavigationCommandCode::kNavigationTo,
                               std::move(result_data));
}

ModuleResult
NavigationModule::CancelNavigationTask(const Dictionary &input_data,
                                       const Dictionary &params) {
  WLOG_DEBUG("[Navigation] CancelNavigationTask");
  // 构造返回结果
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();
  return ModuleResult::Success("Navigation",
                               NavigationCommandCode::KCancelNavigationTask,
                               std::move(result_data));
}
ModuleResult
NavigationModule::GetRemainingPathDistance(const Dictionary &input_data,
                                           const Dictionary &params) {
  WLOG_DEBUG("[Navigation] GetRemainingPathDistance");
  // 构造返回结果
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();
  return ModuleResult::Success("Navigation",
                               NavigationCommandCode::kGetRemainingPathDistance,
                               std::move(result_data));
}
ModuleResult NavigationModule::StartCharging(const Dictionary &input_data,
                                             const Dictionary &params) {
  WLOG_DEBUG("[Navigation] StartCharging");
  // 构造返回结果
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();
  return ModuleResult::Success("Navigation",
                               NavigationCommandCode::kStartCharging,
                               std::move(result_data));
}
ModuleResult NavigationModule::StopCharging(const Dictionary &input_data,
                                            const Dictionary &params) {
  WLOG_DEBUG("[Navigation] StopCharging");
  // 构造返回结果
  auto result_data = std::make_unique<Dictionary>();
  auto result_kv_map = result_data->mutable_keyvaluelist();
  return ModuleResult::Success("Navigation",
                               NavigationCommandCode::kStopCharging,
                               std::move(result_data));
}

} // namespace modules
} // namespace server
} // namespace humanoid_robot