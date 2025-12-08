cmake_minimum_required(VERSION 3.8)
set(TARGET_NAME CHRIC_SDKServerService)
project(${TARGET_NAME} VERSION 1.0.0.0 DESCRIPTION "SDK Server Service" LANGUAGES CXX)


# 生成 Interfaces-Server 独立进程
add_executable(${TARGET_NAME} 
    main.cpp
    ../source/interfaces_server.cpp
    ../source/modules/module_manager.cpp
    ../source/modules/module_base.cpp
    ../source/modules/perception_module.cpp
    ../source/modules/navigation_module.cpp
)


# 设置包含目录
target_include_directories(${TARGET_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${PB_INCLUDE_DIR}
    ${COMMON_INCLUDE_DIR}/communication_v2/net
    ${COMMON_INCLUDE_DIR}/communication_v2/dds
    ${COMMON_INCLUDE_DIR}/communication

    ${loc_msg_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${CHRIC_CLOUD_FOLDER}/framework/common/include/communication


)

target_link_libraries(${TARGET_NAME}
    PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
    PRIVATE 
    CHRIC_CommonLog
    # PB::CHRIC_interfacesPB
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
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
    # 使用全局 CMAKE_INSTALL_RPATH 配置
)

# 可选：安装规则
install(TARGETS ${TARGET_NAME} DESTINATION ${CHRIC_CLOUD_FOLDER}/bin/)