cmake_minimum_required(VERSION 3.8)
set(TARGET_NAME CHRIC_SDKServerService)
project(${TARGET_NAME} VERSION 1.0.0.0 DESCRIPTION "SDK Server Service" LANGUAGES CXX)


# 生成 Interfaces-Server 独立进程
add_executable(${TARGET_NAME} 
    main.cpp
    ../source/interfaces_server.cpp
    ../source/modules/module_manager.cpp
    ../source/modules/module_base.cpp
    # ../source/modules/perception_module.cpp
    ../source/modules/navigation_module.cpp

)

# 添加调试选项
# target_compile_options(${TARGET_NAME} PRIVATE -g)

# find_package(grpc++ REQUIRED)

find_package(loc_msg REQUIRED)
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
    chric_utils_ros_proto_converter
    chric_utils_message_serializer
)

target_link_libraries(${TARGET_NAME}
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
)
set_target_properties(${TARGET_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
    # 使用全局 CMAKE_INSTALL_RPATH 配置
)

# 可选：安装规则
install(TARGETS ${TARGET_NAME} DESTINATION ${CHRIC_CLOUD_FOLDER}/bin/)