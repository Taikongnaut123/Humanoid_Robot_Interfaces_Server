cmake_minimum_required(VERSION 3.8)

set(TARGET_NAME CHRIC_InterfaceServer)

# 创建服务器库 (与Client-SDK模式一致)
add_library(${TARGET_NAME} SHARED
    interfaces_server.cpp
    modules/module_base.cpp
    modules/perception_module.cpp
    modules/module_manager.cpp
    modules/navigation_module.cpp
)

message(STATUS "COMMON_INCLUDE_DIR: ${COMMON_INCLUDE_DIR}")
# 设置包含目录
target_include_directories(${TARGET_NAME}
    PUBLIC
    ${INTERFACES_SERVER_INCLUDE_DIR}
    ${PB_INCLUDE_DIR}

    PRIVATE
    CHRIC_CommonLog

    ${COMMON_INCLUDE_DIR}/communication_v2/net
    ${COMMON_INCLUDE_DIR}/communication_v2/dds
    ${COMMON_INCLUDE_DIR}/communication

    ${loc_msg_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
    PRIVATE 
    CHRIC_CommonLog
    PB::CHRIC_interfacesPB
    PB::CHRIC_commonPB
    PB::CHRIC_perceptionPB

    CHRIC_net_communication
    libCHRIC_communicationPB
    CHRIC_dds_discovery
    libCHRIC_interfacesPB
    rclcpp::rclcpp
    ${loc_msg_LIBRARIES}

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
