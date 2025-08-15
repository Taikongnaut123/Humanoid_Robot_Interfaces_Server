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
)

# 链接库 (与Client-SDK保持一致的链接方式)
target_link_libraries(interfaces_server
    PUBLIC
    libinterfacesPB
    libcommonPB
    libperceptionPB
    protobuf::libprotobuf
    gRPC::grpc++
    PRIVATE CHRIC_Common_Log
)
