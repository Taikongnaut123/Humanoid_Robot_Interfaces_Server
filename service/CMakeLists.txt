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

# # 链接依赖库（如 protobuf、gRPC、pthread 等，按需补充）
# target_link_libraries(${TARGET_NAME}
#     protobuf::libprotobuf
#     gRPC::grpc++
#     pthread
# )
# 使用特定的PB源文件而不是动态库，避免gRPC冲突
set(PB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PB/source/interfaces/interfaces_grpc.grpc.pb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PB/source/interfaces/interfaces_request_response.pb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PB/source/interfaces/interfaces_callback.grpc.pb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PB/source/interfaces/interfaces_callback.pb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PB/source/common/variant.pb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PB/source/perception/perception_service.grpc.pb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PB/source/perception/perception_request_response.pb.cc
)

# 链接库 - 使用源文件而不是动态库
target_sources(${TARGET_NAME} PRIVATE ${PB_SOURCES})
target_link_libraries(${TARGET_NAME}
    PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
    PRIVATE CHRIC_CommonLog
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
)

# 可选：安装规则
install(TARGETS ${TARGET_NAME} DESTINATION ${CHRIC_CLOUD_FOLDER})