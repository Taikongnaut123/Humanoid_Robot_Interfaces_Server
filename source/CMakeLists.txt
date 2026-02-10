cmake_minimum_required(VERSION 3.8)
project(chric_interfaces_server VERSION 1.0.0.0 DESCRIPTION "CHRIC Interfaces Server" LANGUAGES CXX)

set(TARGET_NAME chric_interfaces_server)

# 创建服务器库 (与Client-SDK模式一致)
add_library(${TARGET_NAME} SHARED
    interfaces_server.cpp
    modules/module_base.cpp
    # modules/perception_module.cpp
    modules/module_manager.cpp
    modules/navigation_module.cpp
    modules/control_module.cpp
)

message(STATUS "COMMON_INCLUDE_DIR: ${COMMON_INCLUDE_DIR}")

# find_package(grpc++ REQUIRED)
find_package(loc_msg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(chric_common_log REQUIRED)
find_package(chric_protobuf_interfaces REQUIRED)
find_package(chric_protobuf_ros2 REQUIRED)
find_package(chric_protobuf_sdk_service REQUIRED)

find_package(chric_communication_v2_net REQUIRED)
find_package(chric_communication_v2_dds_discovery REQUIRED)
find_package(chric_protobuf_perception REQUIRED)
find_package(chric_utils_ros_proto_converter REQUIRED)
find_package(chric_utils_message_serializer REQUIRED)
find_package(motor_interface REQUIRED)
find_package(chric_config_manager REQUIRED)
ament_target_dependencies(${TARGET_NAME}
    chric_common_log
    chric_protobuf_interfaces
    chric_protobuf_perception
    chric_protobuf_ros2
    chric_protobuf_sdk_service

    loc_msg
    rclcpp
    std_msgs
    chric_communication_v2_net
    chric_communication_v2_dds_discovery
    chric_utils_ros_proto_converter
    chric_utils_message_serializer
    motor_interface
    chric_config_manager
)

target_link_libraries(${TARGET_NAME}
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
)

# 设置目标属性
set_target_properties(${TARGET_NAME}
    PROPERTIES
    OUTPUT_NAME "interfaces_server"
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "interfaces_server.h"
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
)

# 增加调试选项
target_compile_options(${TARGET_NAME} PRIVATE -g)

# 安装库文件到 sdk_server/lib
install(TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION ${CHRIC_CLOUD_FOLDER}/sdk_server/lib
    ARCHIVE DESTINATION ${CHRIC_CLOUD_FOLDER}/sdk_server/lib
    RUNTIME DESTINATION ${CHRIC_CLOUD_FOLDER}/sdk_server/bin
)
# 安装配置文件到 sdk_server/config
install(DIRECTORY ../config DESTINATION ${CHRIC_CLOUD_FOLDER}/sdk_server)


