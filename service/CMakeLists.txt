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
)


# 设置包含目录
target_include_directories(${TARGET_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${PB_INCLUDE_DIR}
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
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
)

# 可选：安装规则
install(TARGETS ${TARGET_NAME} DESTINATION ${CHRIC_CLOUD_FOLDER})