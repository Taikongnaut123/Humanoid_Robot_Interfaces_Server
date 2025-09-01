cmake_minimum_required(VERSION 3.8)

set(TARGET_NAME CHRIC_InterfaceServer)

# 创建服务器库 (与Client-SDK模式一致)
add_library(${TARGET_NAME} SHARED
    interfaces_server.cpp
    modules/module_base.cpp
    modules/perception_module.cpp
    modules/module_manager.cpp
)

# 设置包含目录
target_include_directories(${TARGET_NAME}
    PUBLIC
    ${INTERFACES_SERVER_INCLUDE_DIR}
    ${PB_INCLUDE_DIR}

    PRIVATE
    CHRIC_CommonLog
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
    PRIVATE 
    CHRIC_CommonLog
    PB::interfacesPB
    PB::commonPB
    PB::perceptionPB
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
    OUTPUT_NAME "interfaces_server"
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "interfaces_server.h"
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
)
