cmake_minimum_required(VERSION 3.8)

# 创建服务器库 (与Client-SDK模式一致)
add_library(interfaces_server SHARED
    interfaces_server.cpp
)

# 添加模块子目录
add_subdirectory(modules)

# 设置包含目录
target_include_directories(interfaces_server
    PUBLIC
    ${INTERFACES_SERVER_INCLUDE_DIR}
    ${PB_INCLUDE_DIR}

    PRIVATE
    CHRIC_Common_Log
)

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
target_sources(interfaces_server PRIVATE ${PB_SOURCES})
target_link_libraries(interfaces_server
    PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
    PRIVATE CHRIC_Common_Log
)
