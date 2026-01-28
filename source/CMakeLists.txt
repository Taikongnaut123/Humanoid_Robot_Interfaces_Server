cmake_minimum_required(VERSION 3.8)

set(TARGET_NAME CHRIC_InterfaceServer)

# 创建服务器库 (与Client-SDK模式一致)
add_library(${TARGET_NAME} SHARED
    interfaces_server.cpp
    modules/module_base.cpp
    # modules/perception_module.cpp
    modules/module_manager.cpp
    modules/navigation_module.cpp
)

message(STATUS "COMMON_INCLUDE_DIR: ${COMMON_INCLUDE_DIR}")

# find_package(grpc++ REQUIRED)
find_package(chric_common_log REQUIRED)
find_package(chric_protobuf_interfaces REQUIRED)
find_package(chric_protobuf_ros2 REQUIRED)
find_package(chric_protobuf_sdk_service REQUIRED)
find_package(loc_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chric_communication_v2_net REQUIRED)
find_package(chric_communication_v2_dds_discovery REQUIRED)
find_package(chric_protobuf_perception REQUIRED)
find_package(chric_utils REQUIRED)


ament_target_dependencies(${TARGET_NAME}
    chric_common_log
    chric_protobuf_interfaces
    chric_protobuf_perception
    chric_protobuf_ros2
    chric_protobuf_sdk_service

    loc_msg
    rclcpp
    chric_communication_v2_net
    chric_communication_v2_dds_discovery
    chric_utils
)

target_link_libraries(${TARGET_NAME}
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
    OUTPUT_NAME "interfaces_server"
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "interfaces_server.h"
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
)

# 安装库文件到 sdk_server/lib
install(TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION sdk_server/lib
    ARCHIVE DESTINATION sdk_server/lib
    RUNTIME DESTINATION sdk_server/bin
)
