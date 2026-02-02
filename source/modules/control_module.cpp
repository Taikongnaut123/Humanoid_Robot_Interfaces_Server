#include "modules/control_module.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <functional>
#include <mutex>
#include <vector>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "motor_interface/msg/motor_drive.hpp"
#include "motor_interface/msg/motor_feedback.hpp"
#include "sdk_service/common/service.pb.h"
#include "sdk_service/control/responce_emergency_stop.pb.h"
#include "sdk_service/control/responce_get_joint_info.pb.h"
#include "sdk_service/control/request_joint_motion.pb.h"
#include "sdk_service/control/responce_joint_motion.pb.h"
#include "sdk_service/control/responce_status.pb.h"

#include "communication_v2/net/CNode.hpp"
#include "Log/wlog.hpp"
using namespace humanoid_robot::PB::sdk_service::common;

namespace PB = humanoid_robot::PB;
namespace PB_common = humanoid_robot::PB::common;

namespace humanoid_robot {
namespace server {
namespace modules {

using ResponceEmergencyStop = humanoid_robot::PB::sdk_service::control::ResponceEmergencyStop;
using ResponceGetJointInfo = humanoid_robot::PB::sdk_service::control::ResponceGetJointInfo;
using ResponceJointMotion = humanoid_robot::PB::sdk_service::control::ResponceJointMotion;
using ControlResStatus = humanoid_robot::PB::sdk_service::control::ResponceStatus;
using ControlCommandCode = humanoid_robot::PB::sdk_service::common::ControlCommandCode;

// ==================== 通信接口抽象类 ====================
class CommunicationInterface {
public:
    virtual ~CommunicationInterface() = default;

    virtual bool Initialize() = 0;
    virtual void Cleanup() = 0;
    virtual void PublishZeroVelocity() = 0;
    virtual void PublishJointCommand(const std::vector<std::string>& joints,
                                     const std::vector<int32_t>& modes,
                                     const std::vector<double>& positions,
                                     const std::vector<double>& velocities,
                                     const std::vector<double>& torques) = 0;

    // 使用void*作为回调参数，避免在头文件中暴露具体类型
    using MotorFeedbackCallback = std::function<void(const void*)>;
    virtual void SetMotorFeedbackCallback(MotorFeedbackCallback callback) = 0;
};

// ==================== ROS2通信实现类 ====================
class ROS2Communication : public CommunicationInterface {
public:
    ROS2Communication();
    ~ROS2Communication() override;

    bool Initialize() override;
    void Cleanup() override;
    void PublishZeroVelocity() override;
    void PublishJointCommand(const std::vector<std::string>& joints,
                             const std::vector<int32_t>& modes,
                             const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             const std::vector<double>& torques) override;
    void SetMotorFeedbackCallback(MotorFeedbackCallback callback) override;

private:
    rclcpp::Node::SharedPtr node_;
    std::thread ros_spin_thread_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<motor_interface::msg::MotorDrive>::SharedPtr joint_motion_pub_;
    rclcpp::Subscription<motor_interface::msg::MotorFeedback>::SharedPtr motor_feedback_sub_;
    MotorFeedbackCallback feedback_callback_;
    bool ros_initialized_;
};

// ==================== 自定义通信库实现类 ====================
class FrameworkCommunication : public CommunicationInterface {
public:
    FrameworkCommunication();
    ~FrameworkCommunication() override;

    bool Initialize() override;
    void Cleanup() override;
    void PublishZeroVelocity() override;
    void PublishJointCommand(const std::vector<std::string>& joints,
                             const std::vector<int32_t>& modes,
                             const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             const std::vector<double>& torques) override;
    void SetMotorFeedbackCallback(MotorFeedbackCallback callback) override;

private:
    std::shared_ptr<humanoid_robot::framework::net::CNode> node_;
    std::shared_ptr<humanoid_robot::framework::net::CPublisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
    std::shared_ptr<humanoid_robot::framework::net::CPublisher<motor_interface::msg::MotorDrive>> joint_motion_pub_;
    std::shared_ptr<humanoid_robot::framework::net::CSubscriber<motor_interface::msg::MotorFeedback>> motor_feedback_sub_;
    MotorFeedbackCallback feedback_callback_;
    bool initialized_;
};

// ==================== ControlImpl实现 ====================
class ControlModule::ControlImpl {
public:
    // 通信接口实例
#ifdef USE_ROS2_COMMUNICATION
    std::unique_ptr<CommunicationInterface> comm_ = std::make_unique<ROS2Communication>();
#else
    std::unique_ptr<CommunicationInterface> comm_ = std::make_unique<FrameworkCommunication>();
#endif

    // 电机反馈数据缓存
    std::mutex feedback_mutex_;
    std::vector<std::string> motor_joint_names_;
    bool feedback_received_;

    ControlImpl() : feedback_received_(false) {}
    ~ControlImpl() = default;
};

// ==================== ROS2Communication实现 ====================
ROS2Communication::ROS2Communication() : ros_initialized_(false) {}

ROS2Communication::~ROS2Communication() {
    Cleanup();
}

bool ROS2Communication::Initialize() {
    try {
        WLOG_DEBUG("[ROS2Communication] Initializing ROS2 communication");
        
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
        
        node_ = std::make_shared<rclcpp::Node>("humanoid_control_module");
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        joint_motion_pub_ = node_->create_publisher<motor_interface::msg::MotorDrive>("/motor_drive", 10);
        
        // 创建电机反馈订阅者
        if (feedback_callback_) {
            motor_feedback_sub_ = node_->create_subscription<motor_interface::msg::MotorFeedback>(
                "/motorFbk_topic", 10,
                [this](const motor_interface::msg::MotorFeedback::SharedPtr msg) {
                    this->feedback_callback_(msg.get());
                });
        }
        
        ros_spin_thread_ = std::thread([this]() {
            WLOG_DEBUG("[ROS2Communication] Starting ROS2 spin thread");
            rclcpp::spin(node_);
            WLOG_DEBUG("[ROS2Communication] ROS2 spin thread finished");
        });
        
        WLOG_DEBUG("[ROS2Communication] ROS2 communication initialized successfully");
        ros_initialized_ = true;
        return true;
        
    } catch (const std::exception& e) {
        WLOG_FATAL("[ROS2Communication] Initialization exception: %s", e.what());
        return false;
    }
}

void ROS2Communication::Cleanup() {
    WLOG_INFO("[ROS2Communication] Cleaning up ROS2 communication");
    
    if (ros_initialized_) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        
        if (ros_spin_thread_.joinable()) {
            ros_spin_thread_.join();
        }
        
        ros_initialized_ = false;
    }
    
    WLOG_INFO("[ROS2Communication] ROS2 communication cleanup completed");
}

void ROS2Communication::PublishZeroVelocity() {
    if (!ros_initialized_) {
        WLOG_ERROR("[ROS2Communication] Not initialized, cannot publish zero velocity");
        return;
    }
    
    auto zero_twist = std::make_unique<geometry_msgs::msg::Twist>();
    zero_twist->linear.x = 0.0;
    zero_twist->linear.y = 0.0;
    zero_twist->linear.z = 0.0;
    zero_twist->angular.x = 0.0;
    zero_twist->angular.y = 0.0;
    zero_twist->angular.z = 0.0;
    
    cmd_vel_pub_->publish(std::move(zero_twist));
    WLOG_DEBUG("[ROS2Communication] Published zero velocity command to /cmd_vel");
}

void ROS2Communication::PublishJointCommand(const std::vector<std::string>& joints,
                                           const std::vector<int32_t>& modes,
                                           const std::vector<double>& positions,
                                           const std::vector<double>& velocities,
                                           const std::vector<double>& torques) {
    if (!ros_initialized_) {
        WLOG_ERROR("[ROS2Communication] Not initialized, cannot publish joint command");
        return;
    }
    
    auto motor_drive_msg = std::make_unique<motor_interface::msg::MotorDrive>();
    motor_drive_msg->joint = joints;
    motor_drive_msg->mode = modes;
    motor_drive_msg->pos = positions;
    motor_drive_msg->vel = velocities;
    motor_drive_msg->tq = torques;
    
    joint_motion_pub_->publish(std::move(motor_drive_msg));
    
    WLOG_DEBUG("[ROS2Communication] Published joint motion command for %zu joints", joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        WLOG_DEBUG("[ROS2Communication] Joint: %s, Mode: %d, Pos: %.3f, Vel: %.3f, Tq: %.3f",
                   joints[i].c_str(), modes[i], positions[i], velocities[i], torques[i]);
    }
}

void ROS2Communication::SetMotorFeedbackCallback(MotorFeedbackCallback callback) {
    feedback_callback_ = std::move(callback);
    
    if (ros_initialized_ && node_) {
        motor_feedback_sub_ = node_->create_subscription<motor_interface::msg::MotorFeedback>(
            "/motorFbk_topic", 10,
            [this](const motor_interface::msg::MotorFeedback::SharedPtr msg) {
                this->feedback_callback_(msg.get());
            });
    }
}

// ==================== FrameworkCommunication实现 ====================
FrameworkCommunication::FrameworkCommunication() : initialized_(false) {}

FrameworkCommunication::~FrameworkCommunication() {
    Cleanup();
}

bool FrameworkCommunication::Initialize() {
    try {
        WLOG_DEBUG("[FrameworkCommunication] Initializing framework communication");
        
        humanoid_robot::framework::net::CNodeOptions node_options;
        node_options.enable_dds_discovery = true;
        node_options.enable_uds = true;
        node_options.heartbeat_interval_ms = 1000;
        
        node_ = std::make_shared<humanoid_robot::framework::net::CNode>("humanoid_control_module", node_options);
        
        // 配置发布者选项
        humanoid_robot::framework::net::CPublisherOptions pub_options;
        pub_options.grpc_address = "0.0.0.0:0";  // 自动分配TCP端口
        pub_options.enable_dds_discovery = true;
        pub_options.enable_dual_server = true;   // 启用TCP+UDS双服务
        
        // 创建cmd_vel发布者
        cmd_vel_pub_ = node_->CreatePublisher<geometry_msgs::msg::Twist>("/cmd_vel", pub_options);
        if (!cmd_vel_pub_) {
            WLOG_ERROR("[FrameworkCommunication] Failed to create cmd_vel publisher");
            return false;
        }
        
        // 创建motor_drive发布者
        pub_options.uds_path = "/tmp/humanoid_motor_drive.sock";
        joint_motion_pub_ = node_->CreatePublisher<motor_interface::msg::MotorDrive>("/motor_drive", pub_options);
        if (!joint_motion_pub_) {
            WLOG_ERROR("[FrameworkCommunication] Failed to create joint_motion publisher");
            return false;
        }
        
        // 创建电机反馈订阅者
        if (feedback_callback_) {
            humanoid_robot::framework::net::CSubscriberOptions sub_options;
            sub_options.enable_dds_discovery = true;
            sub_options.prefer_uds = true;         // 优先使用UDS
            sub_options.thread_pool_size = 2;
            sub_options.reconnect_interval = std::chrono::seconds(2);
            
            motor_feedback_sub_ = node_->CreateSubscriber<motor_interface::msg::MotorFeedback>(
                "/motorFbk_topic",
                [this](const motor_interface::msg::MotorFeedback& msg) {
                    auto msg_ptr = std::make_shared<motor_interface::msg::MotorFeedback>(msg);
                    this->feedback_callback_(msg_ptr.get());
                },
                sub_options);
            
            if (!motor_feedback_sub_) {
                WLOG_ERROR("[FrameworkCommunication] Failed to create motor_feedback subscriber");
                return false;
            }
        }
        
        // 启动节点（后台线程运行）
        std::thread spin_thread([this]() {
            WLOG_DEBUG("[FrameworkCommunication] Starting node spin thread");
            node_->Spin();
        });
        spin_thread.detach();
        
        WLOG_DEBUG("[FrameworkCommunication] Framework communication initialized successfully");
        initialized_ = true;
        return true;
        
    } catch (const std::exception& e) {
        WLOG_FATAL("[FrameworkCommunication] Initialization exception: %s", e.what());
        return false;
    }
}

void FrameworkCommunication::Cleanup() {
    WLOG_INFO("[FrameworkCommunication] Cleaning up framework communication");
    
    if (initialized_) {
        // 关闭节点
        if (node_) {
            node_->Shutdown();
        }
        
        // 释放资源
        cmd_vel_pub_.reset();
        joint_motion_pub_.reset();
        motor_feedback_sub_.reset();
        
        initialized_ = false;
    }
    
    WLOG_INFO("[FrameworkCommunication] Framework communication cleanup completed");
}

void FrameworkCommunication::PublishZeroVelocity() {
    if (!initialized_) {
        WLOG_ERROR("[FrameworkCommunication] Not initialized, cannot publish zero velocity");
        return;
    }
    
    geometry_msgs::msg::Twist zero_twist;
    zero_twist.linear.x = 0.0;
    zero_twist.linear.y = 0.0;
    zero_twist.linear.z = 0.0;
    zero_twist.angular.x = 0.0;
    zero_twist.angular.y = 0.0;
    zero_twist.angular.z = 0.0;
    
    if (cmd_vel_pub_->Publish(zero_twist)) {
        WLOG_DEBUG("[FrameworkCommunication] Published zero velocity command to /cmd_vel");
    } else {
        WLOG_ERROR("[FrameworkCommunication] Failed to publish zero velocity command");
    }
}

void FrameworkCommunication::PublishJointCommand(const std::vector<std::string>& joints,
                                               const std::vector<int32_t>& modes,
                                               const std::vector<double>& positions,
                                               const std::vector<double>& velocities,
                                               const std::vector<double>& torques) {
    if (!initialized_) {
        WLOG_ERROR("[FrameworkCommunication] Not initialized, cannot publish joint command");
        return;
    }
    
    motor_interface::msg::MotorDrive motor_drive_msg;
    motor_drive_msg.joint = joints;
    motor_drive_msg.mode = modes;
    motor_drive_msg.pos = positions;
    motor_drive_msg.vel = velocities;
    motor_drive_msg.tq = torques;
    
    if (joint_motion_pub_->Publish(motor_drive_msg)) {
        WLOG_DEBUG("[FrameworkCommunication] Published joint motion command for %zu joints", joints.size());
        for (size_t i = 0; i < joints.size(); ++i) {
            WLOG_DEBUG("[FrameworkCommunication] Joint: %s, Mode: %d, Pos: %.3f, Vel: %.3f, Tq: %.3f",
                       joints[i].c_str(), modes[i], positions[i], velocities[i], torques[i]);
        }
    } else {
        WLOG_ERROR("[FrameworkCommunication] Failed to publish joint motion command");
    }
}

void FrameworkCommunication::SetMotorFeedbackCallback(MotorFeedbackCallback callback) {
    feedback_callback_ = std::move(callback);
    
    // 如果已初始化，创建订阅者
    if (initialized_ && node_) {
        humanoid_robot::framework::net::CSubscriberOptions sub_options;
        sub_options.enable_dds_discovery = true;
        sub_options.prefer_uds = true;
        sub_options.thread_pool_size = 2;
        sub_options.reconnect_interval = std::chrono::seconds(2);
        
        motor_feedback_sub_ = node_->CreateSubscriber<motor_interface::msg::MotorFeedback>(
            "/motorFbk_topic",
            [this](const motor_interface::msg::MotorFeedback& msg) {
                auto msg_ptr = std::make_shared<motor_interface::msg::MotorFeedback>(msg);
                this->feedback_callback_(msg_ptr.get());
            },
            sub_options);
        
        if (!motor_feedback_sub_) {
            WLOG_ERROR("[FrameworkCommunication] Failed to create motor_feedback subscriber");
        }
    }
}

// ==================== ControlModule核心实现 ====================
ControlModule::ControlModule()
    : ModuleBase("Control", 2),
      pImpl_(std::make_unique<ControlImpl>()) {
}

ControlModule::~ControlModule() = default;

bool ControlModule::Initialize() {
    try {
        WLOG_DEBUG("[Control] Initializing control module");
        
        // 设置电机反馈回调（使用lambda处理具体类型转换）
        pImpl_->comm_->SetMotorFeedbackCallback([this](const void* msg_ptr) {
            this->HandleMotorFeedback(msg_ptr);
        });
        
        if (!pImpl_->comm_->Initialize()) {
            WLOG_FATAL("[Control] Communication module initialization failed");
            return false;
        }
        
        WLOG_DEBUG("[Control] Control module initialized successfully");
        return true;
        
    } catch (const std::exception& e) {
        WLOG_FATAL("[Control] Initialization exception: %s", e.what());
        return false;
    }
}

bool ControlModule::IsRunning() {
    if (!pImpl_) {
        WLOG_ERROR("[Control] IsRunning: pImpl_ is null, module is not running");
        return false;
    }

    if (!pImpl_->comm_) {
        WLOG_WARN("[Control] IsRunning: Communication interface is null, module is not running");
        return false;
    }

    // std::lock_guard<std::mutex> lock(pImpl_->feedback_mutex_);
    // if (!pImpl_->initialized_) {
    //     WLOG_WARN("[Control] IsRunning: Module not initialized yet");
    //     return false;
    // }

    bool comm_is_running = false;
#ifdef USE_ROS2_COMMUNICATION
    comm_is_running = true;
#else
    comm_is_running = true;
#endif
    return comm_is_running;
}

void ControlModule::Cleanup() {
    WLOG_INFO("[Control] Cleaning up control module");
    
    // 清理通信模块
    pImpl_->comm_->Cleanup();
    
    WLOG_INFO("[Control] Control module cleanup completed");
}

ModuleResult ControlModule::ProcessCommand(
    grpc::ServerContext* context, const int32_t& command_id,
    const PB_common::Dictionary& input_data,
    const PB_common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Processing command: %d", command_id);

    if (!pImpl_->comm_) {
        return ModuleResult::Error("Control", command_id, -1, "Communication module not initialized");
    }

    switch (command_id) {
        case ControlCommandCode::kEmergencyStop:
            return EmergencyStop(input_data, params);
        case ControlCommandCode::kGetJointInfo:
            return GetJointInfo(input_data, params);
        case ControlCommandCode::kJointMotion:    
            return JointMotion(input_data, params);
        default:
            return ModuleResult::Error("Control", command_id, -100, "Unknown control command");
    }
}

ModuleResult ControlModule::EmergencyStop(
    const PB_common::Dictionary& input_data,
    const PB_common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Executing emergency stop");
    // std::cout << "emergency stop received data" << std::endl;
    try {
        // 发布零速度命令
        PublishZeroVelocity();
        
        // 构造返回结果
        auto result_data = std::make_unique<PB_common::Dictionary>();
        auto result_kv = result_data->mutable_keyvaluelist();
        PB_common::Variant pb_responce_str;
        
        ResponceEmergencyStop responce_emergency_stop;
        responce_emergency_stop.set_stop_feedback("急停执行成功");
        
        auto now = std::chrono::system_clock::now();
        auto now_s = std::chrono::duration_cast<std::chrono::duration<double>>(
            now.time_since_epoch()
        );
        // 调用 set_stop_timestamp() 传入时间戳值
        responce_emergency_stop.set_stop_timestamp(now_s.count());

        std::string pb_responce_serialized_str;
        auto pb_serialize_result = responce_emergency_stop.SerializeToString(&pb_responce_serialized_str);
        if (!pb_serialize_result) {
            WLOG_ERROR("[Control] Failed to serialize responce_emergency_stop");
            return ModuleResult::Error("Control",
                                    ControlCommandCode::kEmergencyStop, -100,
                                    "Failed to serialize responce_emergency_stop");
        }

        pb_responce_str.set_bytevalue(pb_responce_serialized_str);
        result_kv->insert({"data", pb_responce_str});
        
        WLOG_DEBUG("[Control] Emergency stop completed successfully");
        return ModuleResult::Success("Control", ControlCommandCode::kEmergencyStop, 
                                   std::move(result_data));
        
    } catch (const std::exception& e) {
        return ModuleResult::Error("Control", ControlCommandCode::kEmergencyStop,
                                 -4, "Exception in emergency stop: " + std::string(e.what()));
    }
}

ModuleResult ControlModule::GetJointInfo(
    const PB_common::Dictionary& input_data,
    const PB_common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Getting joint information");
    
    try {
        // 检查是否收到反馈数据
        {
            std::lock_guard<std::mutex> lock(pImpl_->feedback_mutex_);
            if (!pImpl_->feedback_received_) {
                return ModuleResult::Error("Control", ControlCommandCode::kGetJointInfo,
                                         -1, "No motor feedback received yet");
            }
        }
        
        std::vector<std::string> joint_names;
        {
            std::lock_guard<std::mutex> lock(pImpl_->feedback_mutex_);
            joint_names = pImpl_->motor_joint_names_;
        }
        
        if (joint_names.empty()) {
            return ModuleResult::Error("Control", ControlCommandCode::kGetJointInfo,
                                     -2, "No joint information available");
        }
        
        // 构造返回结果
        auto result_data = std::make_unique<PB_common::Dictionary>();
        auto result_kv = result_data->mutable_keyvaluelist();
        PB_common::Variant pb_responce_str;
        
        ResponceGetJointInfo responce_get_joint_info;
        
        auto get_joint_supported_modes = [](const std::string& joint_name) -> std::vector<int32_t> {
            return {MotionMode::ABSOLUTE_POSITION, MotionMode::RELATIVE_POSITION, MotionMode::VELOCITY};
        };
        
        auto get_joint_limits = [](const std::string& joint_name) -> std::pair<double, double> {
            return {-3.14, 3.14}; // (min_limit, max_limit)
        };
        
        for (const auto& joint_name : joint_names) {
            humanoid_robot::PB::sdk_service::control::JointDetail* joint_detail = responce_get_joint_info.add_joint_details();
            
            joint_detail->set_joint_name(joint_name);
            
            std::vector<int32_t> supported_modes = get_joint_supported_modes(joint_name);
            for (int32_t mode : supported_modes) {
                joint_detail->add_support_modes(mode);
            }
            
            auto [min_limit, max_limit] = get_joint_limits(joint_name);
            joint_detail->set_min_limit(min_limit);
            joint_detail->set_max_limit(max_limit);
        }
        
        std::string pb_responce_serialized_str;
        auto pb_serialize_result = responce_get_joint_info.SerializeToString(&pb_responce_serialized_str);
        if (!pb_serialize_result) {
            WLOG_ERROR("[Control] Failed to serialize responce_get_joint_info");
            return ModuleResult::Error("Control",
                                    ControlCommandCode::kGetJointInfo, -100,
                                    "Failed to serialize responce_get_joint_info");
        }
        
        // 将序列化后的 Protobuf 数据存入 Variant 的 bytevalue
        pb_responce_str.set_bytevalue(pb_responce_serialized_str);
        result_kv->insert({"data", pb_responce_str});

        WLOG_DEBUG("[Control] Get joint info completed, found %zu joints", joint_names.size());
        return ModuleResult::Success("Control", ControlCommandCode::kGetJointInfo, 
                                   std::move(result_data));
        
    } catch (const std::exception& e) {
        return ModuleResult::Error("Control", ControlCommandCode::kGetJointInfo,
                                 -4, "Exception: " + std::string(e.what()));
    }
}

ModuleResult ControlModule::JointMotion(
    const PB_common::Dictionary& input_data,
    const PB_common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Executing joint motion control");
    
    try {
        auto data_it = input_data.keyvaluelist().find("request_joint_motion");
        if (data_it == input_data.keyvaluelist().end()) {
            WLOG_ERROR("[Control] Failed to find data in input_data");
            return ModuleResult::Error("Control",
                                    ControlCommandCode::kJointMotion, -100,
                                    "Failed to find data in input_data");
        }
        const PB_common::Variant &data_var = data_it->second;
        humanoid_robot::PB::sdk_service::control::RequestJointMotion request_joint_motion;
        auto protobuf_convert_status = request_joint_motion.ParseFromString(data_var.bytevalue());
        if (!protobuf_convert_status) {
            WLOG_ERROR("[Control] Failed to parse data to RequestJointMotion");
            return ModuleResult::Error("Control",
                                    ControlCommandCode::kJointMotion, -100,
                                    "Failed to parse data to RequestJointMotion");
        }
        
        std::vector<std::string> joints;
        std::vector<int32_t> modes;
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> torques;
        
        for (int i = 0; i < request_joint_motion.joint_commands_size(); ++i) {
            const humanoid_robot::PB::sdk_service::control::JointCommand& joint_cmd = request_joint_motion.joint_commands(i);
            
            if (joint_cmd.joint_name().empty()) {
                std::string error_msg = "Joint command at index " + std::to_string(i) + " has empty joint name";
                WLOG_ERROR("[Control] %s", error_msg.c_str());
                return ModuleResult::Error("Control",
                                        ControlCommandCode::kJointMotion, -2,
                                        error_msg);
            }
            if (joint_cmd.motion_mode() == humanoid_robot::PB::sdk_service::control::JOINT_MODE_UNSPECIFIED) {
                std::string error_msg = "Joint command for '" + joint_cmd.joint_name() + "' has unspecified motion mode";
                WLOG_ERROR("[Control] %s", error_msg.c_str());
                return ModuleResult::Error("Control",
                                        ControlCommandCode::kJointMotion, -2,
                                        error_msg);
            }
            
            joints.push_back(joint_cmd.joint_name());
            modes.push_back(static_cast<int32_t>(joint_cmd.motion_mode())); // 枚举转 int32_t
            
            double target_value = joint_cmd.target_value();
            switch (joint_cmd.motion_mode()) {
                case humanoid_robot::PB::sdk_service::control::JOINT_MODE_ABSOLUTE_POSITION:
                case humanoid_robot::PB::sdk_service::control::JOINT_MODE_RELATIVE_POSITION:
                    positions.push_back(target_value);
                    velocities.push_back(joint_cmd.max_velocity()); // 最大速度
                    torques.push_back(0.0); // 位置/速度模式下力矩为 0
                    break;
                case humanoid_robot::PB::sdk_service::control::JOINT_MODE_VELOCITY:
                    positions.push_back(0.0); // 速度模式下位置为 0
                    velocities.push_back(target_value);
                    torques.push_back(0.0);
                    break;
                case humanoid_robot::PB::sdk_service::control::JOINT_MODE_TORQUE:
                    positions.push_back(0.0); // 力矩模式下位置为 0
                    velocities.push_back(0.0); // 力矩模式下速度为 0
                    torques.push_back(target_value);
                    break;
                default:
                    positions.push_back(0.0);
                    velocities.push_back(0.0);
                    torques.push_back(0.0);
                    break;
            }
        }
        
        if (joints.size() != modes.size() || joints.size() != positions.size() ||
            joints.size() != velocities.size() || joints.size() != torques.size()) {
            WLOG_ERROR("[Control] Parameter count mismatch after parsing joint commands");
            return ModuleResult::Error("Control", ControlCommandCode::kJointMotion,
                                     -3, "Parameter count mismatch");
        }
        // 发布关节命令
        PublishJointCommand(joints, modes, positions, velocities, torques);
        
        // 构造返回结果
        auto result_data = std::make_unique<PB_common::Dictionary>();
        auto result_kv = result_data->mutable_keyvaluelist();
        PB_common::Variant pb_responce_str;
        
        ResponceJointMotion responce_joint_motion;
        std::string execute_info = "All " + std::to_string(joints.size()) + " joints executed successfully";
        responce_joint_motion.set_execute_info(execute_info);

        std::string pb_responce_serialized_str;
        auto pb_serialize_result = responce_joint_motion.SerializeToString(&pb_responce_serialized_str);
        if (!pb_serialize_result) {
            WLOG_ERROR("[Control] Failed to serialize responce_joint_motion");
            return ModuleResult::Error("Control",
                                    ControlCommandCode::kJointMotion, -101,
                                    "Failed to serialize responce_joint_motion");
        }
        
        
        pb_responce_str.set_bytevalue(pb_responce_serialized_str);
        result_kv->insert({"data", pb_responce_str});
        
        WLOG_DEBUG("[Control] Joint motion completed for %zu joints", joints.size());
        return ModuleResult::Success("Control", ControlCommandCode::kJointMotion, 
                                   std::move(result_data));
        
    } catch (const std::exception& e) {
        return ModuleResult::Error("Control", ControlCommandCode::kJointMotion,
                                 -4, "Exception: " + std::string(e.what()));
    }
}

void ControlModule::PublishZeroVelocity() {
    if (pImpl_->comm_) {
        pImpl_->comm_->PublishZeroVelocity();
    } else {
        WLOG_ERROR("[Control] Communication module not initialized");
    }
}

void ControlModule::PublishJointCommand(const std::vector<std::string>& joints,
                                      const std::vector<int32_t>& modes,
                                      const std::vector<double>& positions,
                                      const std::vector<double>& velocities,
                                      const std::vector<double>& torques) {
    if (pImpl_->comm_) {
        pImpl_->comm_->PublishJointCommand(joints, modes, positions, velocities, torques);
    } else {
        WLOG_ERROR("[Control] Communication module not initialized");
    }
}

void ControlModule::HandleMotorFeedback(const void* msg_ptr) {
    try {
        const motor_interface::msg::MotorFeedback* typed_msg = 
            reinterpret_cast<const motor_interface::msg::MotorFeedback*>(msg_ptr);

        const auto& joints = typed_msg->joint;
        if (joints.empty()) {
            WLOG_WARN("[Control] Received motor feedback with empty joint list");
            return;
        }
        
        std::lock_guard<std::mutex> lock(pImpl_->feedback_mutex_);
        pImpl_->motor_joint_names_ = joints;
        pImpl_->feedback_received_ = true;
        
        WLOG_DEBUG("[Control] Received motor feedback for %zu joints", joints.size());
        
    } catch (const std::exception& e) {
        WLOG_ERROR("[Control] Exception in HandleMotorFeedback: %s", e.what());
    }
}

} // namespace modules
} // namespace server
} // namespace humanoid_robot
