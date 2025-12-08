#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include "modules/navigation_module.h"
// #include "grpcpp/grpcpp.h"
#include "Log/wlog.hpp"

#include "CNode.hpp"
// #include "communication/rpc_service.pb.h"
#include "communication/communication_service.pb.h"
// #include "common/pose.pb.h"

#include "loc_msg/msg/req_pose_msg.hpp"
#include "loc_msg/msg/res_start_nav.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "common/service.pb.h"

#include "common/RosMessageSerializer.h"
#include "absl/strings/escaping.h"

using namespace humanoid_robot::framework::net;

using namespace humanoid_robot::PB::common;
using namespace humanoid_robot::framework::communication;
namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {
            class NavigationModule::NavigationImpl
            {
            public:
                CNode node_;

                // 发布订阅
                // std::shared_ptr<humanoid_robot::framework::net::CPublisher<NavigationTopicMessage>> publisher_;
                // std::unique_ptr<NavigationService::Stub> publisher_stub_;

                // std::shared_ptr<humanoid_robot::framework::net::CSubscriber<PoseMessage>> pose_subscriber_;
                // std::shared_ptr<humanoid_robot::framework::net::CSubscriber<GridMap2DMessage>> map_subscriber_;
                // std::shared_ptr<humanoid_robot::framework::net::CSubscriber<PathMessage>> path_subscriber_;

                // 请求响应
                // std::shared_ptr<humanoid_robot::framework::net::CServiceServer<NavigationRequest, NavigationResponse>> service_server_;

                std::shared_ptr<humanoid_robot::framework::net::CServiceClient<UniversalRequest, UniversalResponse>> service_client_;

                std::string server_target_;
                bool connected_;

                NavigationImpl() : connected_(false), node_("navigation_node_sdk") {};

                ~NavigationImpl()
                {
                    if (connected_)
                    {
                        // Graceful shutdown could be implemented here
                    }
                };
                static RosMessageSerializer<loc_msg::msg::ReqPoseMsg> req_pose_serializer;
                static RosMessageSerializer<geometry_msgs::msg::Pose> geometry_pose_serializer;
            };

            NavigationModule::NavigationModule()
                : ModuleBase("Navigation", 3), // 3个工作线程
                  pImpl_(std::make_unique<NavigationImpl>())
            {
            }

            NavigationModule::~NavigationModule() = default;

            bool NavigationModule::Initialize()
            {
                try
                {
                    pImpl_->server_target_ = "navigation_service";
                    // 创建 Service Client（会自动从 DDS 发现 Server）
                    humanoid_robot::framework::net::CClientOptions client_options;
                    // client_options.service_address = "localhost:50052";  // 手动配置（可选）
                    client_options.enable_dds_discovery = true; // 从 DDS 自动发现 Service Server
                    client_options.prefer_uds = true;           // 同机通信优先使用 UDS
                    // client_options.prefer_uds = false;           // 同机通信优先使用 UDS
                    client_options.default_timeout = std::chrono::seconds(5);

                    // std::cout << "Creating service client for 'echo_service'..." << std::endl;
                    // std::cout << "Waiting for service discovery via DDS..." << std::endl;

                    pImpl_->service_client_ = pImpl_->node_.CreateServiceClient<UniversalRequest, UniversalResponse>(
                        pImpl_->server_target_,
                        client_options);
                    if (!pImpl_->service_client_->WaitForService(std::chrono::seconds(5)))
                    {
                        WLOG_ERROR("[Navigation] Failed to connect to %s", pImpl_->server_target_.c_str());
                        return false;
                    }

                    pImpl_->connected_ = true;
                    WLOG_INFO("[Navigation] Navigation module initialize");
                    return true;
                }

                catch (const std::exception &e)
                {
                    WLOG_FATAL("[Navigation] Exception during initialization: %s", e.what());
                    return false;
                }
            }

            void NavigationModule::Cleanup()
            {
                WLOG_INFO("Cleaning up navigation resources...");
                if (pImpl_->connected_)
                {
                    pImpl_->connected_ = false;
                    // Graceful shutdown could be implemented here

                    // 还未实现
                    // pImpl_->service_client_.reset();

                    pImpl_->connected_ = false;
                }
                WLOG_INFO("[Navigation] Navigation module cleanup completed");
            }

            ModuleResult NavigationModule::ProcessCommand(
                grpc::ServerContext *context, const int32_t &command_id,
                const humanoid_robot::PB::common::Dictionary &input_data,
                const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] Processing command in NavigationModule: %d", command_id);

                if (command_id == humanoid_robot::PB::common::CommandCode::GET_CURRENT_POSE)
                {
                    return GetCurrentPose(input_data, params);
                }
                else if (command_id == humanoid_robot::PB::common::CommandCode::GET_GRID_MAP_2D)
                {
                    return GetGridMap2D(input_data, params);
                }
                else if (command_id == humanoid_robot::PB::common::CommandCode::NAVIGATION_TO)
                {
                    return NavigationTo(input_data, params);
                }
                else if (command_id == humanoid_robot::PB::common::CommandCode::CANCEL_NAVIGATION_TASK)
                {
                    return CancelNavigationTask(input_data, params);
                }
                else if (command_id == humanoid_robot::PB::common::CommandCode::GET_REMAINING_PATH_DISTANCE)
                {
                    return GetRemainingPathDistance(input_data, params);
                }
                else if (command_id == humanoid_robot::PB::common::CommandCode::START_CHARGING)
                {
                    return StartCharging(input_data, params);
                }
                else if (command_id == humanoid_robot::PB::common::CommandCode::STOP_CHARGING)
                {
                    return StopCharging(input_data, params);
                }
                else
                {
                    WLOG_WARN("[Navigation] Unknown Navigation command: %d", command_id);
                    return ModuleResult::Error("Navigation", command_id,
                                               -100, "Unknown Navigation command: " + command_id);
                }
            }

            ModuleResult NavigationModule::GetCurrentPose(const humanoid_robot::PB::common::Dictionary &input_data,
                                                          const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] GetCurrentPose");
                // 构造返回结果
                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto result_kv_map = result_data->mutable_keyvaluelist();

                // 构造请求
                humanoid_robot::PB::communication::UniversalRequest request;
                request.set_command(humanoid_robot::PB::common::CommandCode::GET_CURRENT_POSE);
                request.set_version(1);
                request.set_requestid(1);
                request.set_sendrequesttimestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                     std::chrono::system_clock::now().time_since_epoch())
                                                     .count());
                request.set_checksum(1);

                // humanoid_robot::PB::navigation::PoseRequest pose_message;

                // auto frame_id = pose_message.mutable_frame_id();
                // auto child_frame_id = pose_message.mutable_child_frame_id();

                loc_msg::msg::ReqPoseMsg pose_message;
                bool frame_id_found = false;
                bool child_frame_id_found = false;

                // 方法1：范围 for 循环（C++11+，简洁推荐）
                for (const auto &kv : input_data.keyvaluelist())
                {
                    const std::string &key = kv.first; // 取出 key（string 类型）
                    const Variant &value = kv.second;  // 取出 value（Variant 类型）

                    // ------------- 关键：解析 Variant 类型的值 -------------
                    // 假设 Variant 定义了常见类型（如 int32、double、string），需通过 HasXXX() 判断类型
                    if (key == "frame_id" && value.has_stringvalue())
                    {
                        pose_message.frame_id = value.stringvalue();
                        frame_id_found = true;
                    }
                    else if (key == "child_frame_id" && value.has_stringvalue())
                    {
                        pose_message.child_frame_id = value.stringvalue();
                        child_frame_id_found = true;
                    }
                    else
                    {
                        WLOG_WARN("[Navigation] Unknown key: %s", key.c_str());
                    }
                }


                if (!frame_id_found)
                {
                    WLOG_WARN("[Navigation] frame_id is empty");
                    return ModuleResult::Error("Navigation", humanoid_robot::PB::common::CommandCode::GET_CURRENT_POSE,
                                               -100, "frame_id is empty");
                }
                if (!child_frame_id_found)
                {
                    WLOG_WARN("[Navigation] child_frame_id is empty");
                    return ModuleResult::Error("Navigation", humanoid_robot::PB::common::CommandCode::GET_CURRENT_POSE,
                                               -100, "child_frame_id is empty");
                }

                std::string serialized_string = pImpl_->req_pose_serializer.SerializeToString(pose_message);
                
                request.set_payload(serialized_string);
                // request.set_payload(serialized_msg);

                // request.set_payloadtype(humanoid_robot::PB::navigation::POSE_REQUEST);
                // request.set_payloadsize(serialized_bytes.size());
                request.set_payloadsize(serialized_string.size());

                std::cout << "[Navigation] Sending request: " << request.payload() << std::endl;

                // 同步调用服务
                auto response = pImpl_->service_client_->Call(request);

                if (response)
                {
                    std::cout << "[Navigation] Received response:" << std::endl;
                    std::cout << "  Payload: " << response->payload() << std::endl;
                    std::cout << "  Sequence: " << response->requestid() << std::endl;
                }
                else
                {
                    WLOG_ERROR("[Navigation] Failed to get current pose");
                    return ModuleResult::Error("Navigation", humanoid_robot::PB::common::CommandCode::GET_CURRENT_POSE,
                                               -100, "Failed to get current pose, no response.");
                }

                const google::protobuf::Descriptor *desc = response->GetDescriptor();
                // 查找名为 "status" 的字段（区分大小写）
                const google::protobuf::FieldDescriptor *status_field = desc->FindFieldByName("status");
                const google::protobuf::FieldDescriptor *payload_field = desc->FindFieldByName("payload");
                if (!(status_field && payload_field))
                {
                    WLOG_ERROR("[Navigation] Failed to get current pose");
                    return ModuleResult::Error("Navigation", humanoid_robot::PB::common::CommandCode::GET_CURRENT_POSE,
                                               -100, "Failed to get current pose, no response.");
                }

                // 映射返回结果
                humanoid_robot::PB::common::Variant pose_var;

                geometry_msgs::msg::Pose pose;
                pImpl_->geometry_pose_serializer.DeserializeFromString(response->payload(), pose);
                std::string payload = pImpl_->geometry_pose_serializer.SerializeToString(pose);

                auto pose_value = pose_var.mutable_stringvalue();
                
                *pose_value = absl::Base64Escape(payload);
                std::cout << "[Navigation] Received pose: " << *pose_value << std::endl;
                result_kv_map->insert({"pose", pose_var});

                humanoid_robot::PB::common::Variant status_var;
                status_var.set_uint32value(response->status());
                result_kv_map->insert({"status", status_var});

                return ModuleResult::Success("Navigation", humanoid_robot::PB::common::CommandCode::GET_CURRENT_POSE, std::move(result_data));
            }

            ModuleResult NavigationModule::GetGridMap2D(const humanoid_robot::PB::common::Dictionary &input_data,
                                                        const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] GetGridMap2D");
                // 构造返回结果
                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto result_kv_map = result_data->mutable_keyvaluelist();
                return ModuleResult::Success("Navigation", humanoid_robot::PB::common::CommandCode::GET_GRID_MAP_2D, std::move(result_data));
            }

            ModuleResult NavigationModule::NavigationTo(const humanoid_robot::PB::common::Dictionary &input_data,
                                                        const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] NavigationTo");
                // 构造返回结果
                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto result_kv_map = result_data->mutable_keyvaluelist();
                return ModuleResult::Success("Navigation", humanoid_robot::PB::common::CommandCode::NAVIGATION_TO, std::move(result_data));
            }

            ModuleResult NavigationModule::CancelNavigationTask(const humanoid_robot::PB::common::Dictionary &input_data,
                                                                const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] CancelNavigationTask");
                // 构造返回结果
                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto result_kv_map = result_data->mutable_keyvaluelist();
                return ModuleResult::Success("Navigation", humanoid_robot::PB::common::CommandCode::CANCEL_NAVIGATION_TASK, std::move(result_data));
            }
            ModuleResult NavigationModule::GetRemainingPathDistance(const humanoid_robot::PB::common::Dictionary &input_data,
                                                                    const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] GetRemainingPathDistance");
                // 构造返回结果
                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto result_kv_map = result_data->mutable_keyvaluelist();
                return ModuleResult::Success("Navigation", humanoid_robot::PB::common::CommandCode::GET_REMAINING_PATH_DISTANCE, std::move(result_data));
            }
            ModuleResult NavigationModule::StartCharging(const humanoid_robot::PB::common::Dictionary &input_data,
                                                         const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] StartCharging");
                // 构造返回结果
                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto result_kv_map = result_data->mutable_keyvaluelist();
                return ModuleResult::Success("Navigation", humanoid_robot::PB::common::CommandCode::START_CHARGING, std::move(result_data));
            }
            ModuleResult NavigationModule::StopCharging(const humanoid_robot::PB::common::Dictionary &input_data,
                                                        const humanoid_robot::PB::common::Dictionary &params)
            {
                WLOG_DEBUG("[Navigation] StopCharging");
                // 构造返回结果
                auto result_data = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto result_kv_map = result_data->mutable_keyvaluelist();
                return ModuleResult::Success("Navigation", humanoid_robot::PB::common::CommandCode::STOP_CHARGING, std::move(result_data));
            }

        }
    }
}