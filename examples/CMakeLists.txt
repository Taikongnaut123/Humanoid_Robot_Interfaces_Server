cmake_minimum_required(VERSION 3.8)

# 创建服务器示例
add_executable(server_example
    server_example.cpp
)

target_link_libraries(server_example
    PUBLIC
    CHRIC_InterfaceServer
    protobuf::libprotobuf
    gRPC::grpc++
    PRIVATE 
    CHRIC_CommonLog
)

target_include_directories(server_example
    PRIVATE
    ${INTERFACES_SERVER_INCLUDE_DIR}
    ${PB_INCLUDE_DIR}
)

# 创建简单测试
add_executable(simple_test
    simple_test.cpp
)

target_link_libraries(simple_test
    PUBLIC
    CHRIC_InterfaceServer
    protobuf::libprotobuf
    gRPC::grpc++
    PRIVATE 
    CHRIC_CommonLog
)
