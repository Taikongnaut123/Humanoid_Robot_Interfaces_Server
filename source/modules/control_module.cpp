#include "modules/control_module.h"
// #include "control/control_request_response.pb.h"
#include "Log/wlog.hpp"

// using namespace humanoid_robot::PB::control;
using namespace humanoid_robot::PB::common;
using namespace humanoid_robot::server::modules;

namespace humanoid_robot {
namespace server {
namespace modules {

// ControlImpl实现（包含通信接口和数据缓存）
class ControlModule::ControlImpl {
public:
    // 通信接口实例（通过宏定义切换实现）
#ifdef USE_ROS2_COMMUNICATION
    std::unique_ptr<CommunicationInterface> comm_ = std::make_unique<ROS2Communication>();
#else
    std::unique_ptr<CommunicationInterface> comm_ = std::make_unique<FrameworkCommunication>();
#endif

    // 电机反馈数据缓存
    std::mutex feedback_mutex_;
    std::vector<std::string> motor_joint_names_;  // 从motor_feedback中获取的关节名称
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
        
        // 初始化ROS2
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
        
        // 创建ROS2节点
        node_ = std::make_shared<rclcpp::Node>("humanoid_control_module");
        cmd_vel_pub_ = node_->create_publisher<TwistMsg>("/cmd_vel", 10);
        joint_motion_pub_ = node_->create_publisher<MotorDriveMsg>("/motor_drive", 10);
        
        // 创建电机反馈订阅者（如果已设置回调）
        if (feedback_callback_) {
            motor_feedback_sub_ = node_->create_subscription<MotorFeedbackMsg>(
                "/motorFbk_topic", 10,
                [this](const MotorFeedbackMsg::SharedPtr msg) {
                    this->feedback_callback_(msg);
                });
        }
        
        // 启动ROS2 spin线程
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
        // 关闭ROS2
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        
        // 等待spin线程结束
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
    
    auto zero_twist = std::make_unique<TwistMsg>();
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
    
    auto motor_drive_msg = std::make_unique<MotorDriveMsg>();
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
    
    // 如果已初始化，创建订阅者
    if (ros_initialized_ && node_) {
        motor_feedback_sub_ = node_->create_subscription<MotorFeedbackMsg>(
            "/motorFbk_topic", 10,
            [this](const MotorFeedbackMsg::SharedPtr msg) {
                this->feedback_callback_(msg);
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
        
        // 创建通信库节点
        CNodeOptions node_options;
        node_options.enable_dds_discovery = true;
        node_options.enable_uds = true;
        node_options.heartbeat_interval_ms = 1000;
        node_ = std::make_shared<CNode>("humanoid_control_module", node_options);
        
        // 配置发布者选项
        CPublisherOptions pub_options;
        pub_options.grpc_address = "0.0.0.0:0";  // 自动分配TCP端口
        pub_options.enable_dds_discovery = true;
        pub_options.enable_dual_server = true;   // 启用TCP+UDS双服务
        // 创建cmd_vel发布者
        cmd_vel_pub_ = node_->CreatePublisher<TwistMsg>("/cmd_vel", pub_options);
        if (!cmd_vel_pub_) {
            WLOG_ERROR("[FrameworkCommunication] Failed to create cmd_vel publisher");
            return false;
        }
        
        // 创建motor_drive发布者
        pub_options.uds_path = "/tmp/humanoid_motor_drive.sock";
        joint_motion_pub_ = node_->CreatePublisher<MotorDriveMsg>("/motor_drive", pub_options);
        if (!joint_motion_pub_) {
            WLOG_ERROR("[FrameworkCommunication] Failed to create joint_motion publisher");
            return false;
        }
        
        // 创建电机反馈订阅者（如果已设置回调）
        if (feedback_callback_) {
            CSubscriberOptions sub_options;
            sub_options.enable_dds_discovery = true;
            sub_options.prefer_uds = true;         // 优先使用UDS
            sub_options.thread_pool_size = 2;
            sub_options.reconnect_interval = std::chrono::seconds(2);
            
            motor_feedback_sub_ = node_->CreateSubscriber<MotorFeedbackMsg>(
                "/motorFbk_topic",
                [this](const MotorFeedbackMsg& msg) {
                    // 转换为shared_ptr适配原有回调接口
                    auto msg_ptr = std::make_shared<MotorFeedbackMsg>(msg);
                    this->feedback_callback_(msg_ptr);
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
    
    TwistMsg zero_twist;
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
    
    MotorDriveMsg motor_drive_msg;
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
        CSubscriberOptions sub_options;
        sub_options.enable_dds_discovery = true;
        sub_options.prefer_uds = true;
        sub_options.thread_pool_size = 2;
        sub_options.reconnect_interval = std::chrono::seconds(2);
        
        motor_feedback_sub_ = node_->CreateSubscriber<MotorFeedbackMsg>(
            "/motorFbk_topic",
            [this](const MotorFeedbackMsg& msg) {
                auto msg_ptr = std::make_shared<MotorFeedbackMsg>(msg);
                this->feedback_callback_(msg_ptr);
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
        
        // 设置电机反馈回调
        pImpl_->comm_->SetMotorFeedbackCallback([this](const MotorFeedbackMsg::SharedPtr msg) {
            this->HandleMotorFeedback(msg);
        });
        
        // 初始化通信模块
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

void ControlModule::Cleanup() {
    WLOG_INFO("[Control] Cleaning up control module");
    
    // 清理通信模块
    pImpl_->comm_->Cleanup();
    
    WLOG_INFO("[Control] Control module cleanup completed");
}

ModuleResult ControlModule::ProcessCommand(
    grpc::ServerContext* context, const int32_t& command_id,
    const humanoid_robot::PB::common::Dictionary& input_data,
    const humanoid_robot::PB::common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Processing command: %d", command_id);

    if (!pImpl_->comm_) {
        return ModuleResult::Error("Control", command_id, -1, "Communication module not initialized");
    }

    switch (command_id) {
        case ControlCommandCode::EMERGENCY_STOP:
            return EmergencyStopROS(input_data, params);
        case ControlCommandCode::GET_JOINT_INFO:
            return GetJointInfoROS(input_data, params);
        case ControlCommandCode::JOINT_MOTION:    
            return JointMotionROS(input_data, params);
        default:
            return ModuleResult::Error("Control", command_id, -100, "Unknown control command");
    }
}

ModuleResult ControlModule::EmergencyStopROS(
    const humanoid_robot::PB::common::Dictionary& input_data,
    const humanoid_robot::PB::common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Executing emergency stop");
    
    try {
        // 发布零速度命令
        PublishZeroVelocity();
        
        // 构造返回结果
        auto result_data = std::make_unique<Dictionary>();
        auto result_kv = result_data->mutable_keyvaluelist();
        
        {
            Variant var;
            var.set_boolvalue(true);
            result_kv->insert({"stop_success", var});
        }
        
        {
            Variant var;
            const auto& kv_map = input_data.keyvaluelist();
            auto reason_it = kv_map.find("reason");
            std::string stop_reason = reason_it != kv_map.end() ? 
                reason_it->second.stringvalue() : "Emergency stop command received";
            var.set_stringvalue(stop_reason);
            result_kv->insert({"stop_reason", var});
        }
        
        {
            Variant var;
            var.set_int64value(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            result_kv->insert({"stop_timestamp", var});
        }
        
        WLOG_DEBUG("[Control] Emergency stop completed successfully");
        return ModuleResult::Success("Control", ControlCommandCode::EMERGENCY_STOP, 
                                   std::move(result_data));
        
    } catch (const std::exception& e) {
        return ModuleResult::Error("Control", ControlCommandCode::EMERGENCY_STOP,
                                 -4, "Exception in emergency stop: " + std::string(e.what()));
    }
}

ModuleResult ControlModule::GetJointInfoROS(
    const humanoid_robot::PB::common::Dictionary& input_data,
    const humanoid_robot::PB::common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Getting joint information");
    
    try {
        // 检查是否收到反馈数据
        {
            std::lock_guard<std::mutex> lock(pImpl_->feedback_mutex_);
            if (!pImpl_->feedback_received_) {
                return ModuleResult::Error("Control", ControlCommandCode::GET_JOINT_INFO,
                                         -1, "No motor feedback received yet");
            }
        }
        
        std::vector<std::string> joint_names;
        {
            std::lock_guard<std::mutex> lock(pImpl_->feedback_mutex_);
            joint_names = pImpl_->motor_joint_names_;
        }
        
        if (joint_names.empty()) {
            return ModuleResult::Error("Control", ControlCommandCode::GET_JOINT_INFO,
                                     -2, "No joint information available");
        }
        
        // 构造返回结果
        auto result_data = std::make_unique<Dictionary>();
        auto result_kv = result_data->mutable_keyvaluelist();
        
        // 关节名称（逗号分隔）
        std::string joint_names_str;
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (i > 0) joint_names_str += ",";
            joint_names_str += joint_names[i];
        }
        {
            Variant var;
            var.set_stringvalue(joint_names_str);
            result_kv->insert({"joint_names", var});
        }
        
        // 关节数量
        {
            Variant var;
            var.set_int32value(static_cast<int32_t>(joint_names.size()));
            result_kv->insert({"joint_count", var});
        }
        
        // 支持的模式
        const std::vector<int32_t> supported_modes = {
            MotionMode::VELOCITY, MotionMode::ABSOLUTE_POSITION, MotionMode::RELATIVE_POSITION
        };
        std::string modes_str;
        for (size_t i = 0; i < supported_modes.size(); ++i) {
            if (i > 0) modes_str += ",";
            modes_str += std::to_string(supported_modes[i]);
        }
        
        // 每个关节的模式
        for (const auto& joint : joint_names) {
            Variant var;
            var.set_stringvalue(modes_str);
            result_kv->insert({"modes_" + joint, var});
        }
        
        // 模式说明
        {
            Variant var;
            var.set_stringvalue("1=绝对位置,2=相对位置,3=速度");
            result_kv->insert({"mode_descriptions", var});
        }
        
        WLOG_DEBUG("[Control] Get joint info completed, found %zu joints", joint_names.size());
        return ModuleResult::Success("Control", ControlCommandCode::GET_JOINT_INFO, 
                                   std::move(result_data));
        
    } catch (const std::exception& e) {
        return ModuleResult::Error("Control", ControlCommandCode::GET_JOINT_INFO,
                                 -4, "Exception: " + std::string(e.what()));
    }
}

ModuleResult ControlModule::JointMotionROS(
    const humanoid_robot::PB::common::Dictionary& input_data,
    const humanoid_robot::PB::common::Dictionary& params) {
    
    WLOG_DEBUG("[Control] Executing joint motion control");
    
    try {
        const auto& kv_map = input_data.keyvaluelist();
        
        // 解析关节名称
        auto joints_it = kv_map.find("joints");
        if (joints_it == kv_map.end()) {
            return ModuleResult::Error("Control", ControlCommandCode::JOINT_MOTION,
                                     -1, "No joints specified");
        }
        std::vector<std::string> joints;
        std::string joints_str = joints_it->second.stringvalue();
        size_t pos = 0;
        while ((pos = joints_str.find(',')) != std::string::npos) {
            joints.push_back(joints_str.substr(0, pos));
            joints_str.erase(0, pos + 1);
        }
        if (!joints_str.empty()) joints.push_back(joints_str);
        
        // 解析模式
        auto modes_it = kv_map.find("modes");
        if (modes_it == kv_map.end()) {
            return ModuleResult::Error("Control", ControlCommandCode::JOINT_MOTION,
                                     -2, "No motion modes specified");
        }
        std::vector<int32_t> modes;
        std::string modes_str = modes_it->second.stringvalue();
        pos = 0;
        while ((pos = modes_str.find(',')) != std::string::npos) {
            modes.push_back(std::stoi(modes_str.substr(0, pos)));
            modes_str.erase(0, pos + 1);
        }
        if (!modes_str.empty()) modes.push_back(std::stoi(modes_str));
        
        // 解析位置
        std::vector<double> positions;
        auto pos_it = kv_map.find("positions");
        if (pos_it != kv_map.end()) {
            std::string pos_str = pos_it->second.stringvalue();
            pos = 0;
            while ((pos = pos_str.find(',')) != std::string::npos) {
                positions.push_back(std::stod(pos_str.substr(0, pos)));
                pos_str.erase(0, pos + 1);
            }
            if (!pos_str.empty()) positions.push_back(std::stod(pos_str));
        }
        
        // 解析速度
        std::vector<double> velocities(joints.size(), 0.0);
        auto vel_it = kv_map.find("velocities");
        if (vel_it != kv_map.end()) {
            std::string vel_str = vel_it->second.stringvalue();
            pos = 0;
            size_t idx = 0;
            while ((pos = vel_str.find(',')) != std::string::npos && idx < joints.size()) {
                velocities[idx++] = std::stod(vel_str.substr(0, pos));
                vel_str.erase(0, pos + 1);
            }
            if (!vel_str.empty() && idx < joints.size()) {
                velocities[idx] = std::stod(vel_str);
            }
        }
        
        // 解析力矩
        std::vector<double> torques(joints.size(), 0.0);
        auto tq_it = kv_map.find("torques");
        if (tq_it != kv_map.end()) {
            std::string tq_str = tq_it->second.stringvalue();
            pos = 0;
            size_t idx = 0;
            while ((pos = tq_str.find(',')) != std::string::npos && idx < joints.size()) {
                torques[idx++] = std::stod(tq_str.substr(0, pos));
                tq_str.erase(0, pos + 1);
            }
            if (!tq_str.empty() && idx < joints.size()) {
                torques[idx] = std::stod(tq_str);
            }
        }
        
        // 验证参数数量
        if (joints.size() != modes.size() || joints.size() != positions.size() ||
            joints.size() != velocities.size() || joints.size() != torques.size()) {
            return ModuleResult::Error("Control", ControlCommandCode::JOINT_MOTION,
                                     -3, "Parameter count mismatch");
        }
        
        // 发布关节命令
        PublishJointCommand(joints, modes, positions, velocities, torques);
        
        // 构造返回结果
        auto result_data = std::make_unique<Dictionary>();
        auto result_kv = result_data->mutable_keyvaluelist();
        
        {
            Variant var;
            var.set_boolvalue(true);
            result_kv->insert({"motion_success", var});
        }
        {
            Variant var;
            var.set_int32value(static_cast<int32_t>(joints.size()));
            result_kv->insert({"controlled_joints_count", var});
        }
        
        WLOG_DEBUG("[Control] Joint motion completed for %zu joints", joints.size());
        return ModuleResult::Success("Control", ControlCommandCode::JOINT_MOTION, 
                                   std::move(result_data));
        
    } catch (const std::exception& e) {
        return ModuleResult::Error("Control", ControlCommandCode::JOINT_MOTION,
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

void ControlModule::HandleMotorFeedback(const MotorFeedbackMsg::SharedPtr msg) {
    try {
        const auto& joints = msg->joint;
        if (joints.empty()) {
            WLOG_WARN("[Control] Received motor feedback with empty joint list");
            return;
        }
        
        std::lock_guard<std::mutex> lock(pImpl_->feedback_mutex_);
        pImpl_->motor_joint_names_ = joints;
        pImpl_->feedback_received_ = true;
        
        // 日志输出
        const auto& positions = msg->pos;
        const auto& velocities = msg->vel;
        const auto& torques = msg->tq;
        const auto& errors = msg->err;
        
        WLOG_DEBUG("[Control] Received motor feedback for %zu joints", joints.size());
        for (size_t i = 0; i < joints.size(); ++i) {
            if (i < positions.size() && i < velocities.size() && i < torques.size() && i < errors.size()) {
                WLOG_DEBUG("[Control] Joint %s: pos=%.3f, vel=%.3f, tq=%.3f, err=%d",
                          joints[i].c_str(), positions[i], velocities[i], torques[i], errors[i]);
            }
        }
        
    } catch (const std::exception& e) {
        WLOG_ERROR("[Control] Exception in HandleMotorFeedback: %s", e.what());
    }
}

} // namespace modules
} // namespace server
} // namespace humanoid_robot
