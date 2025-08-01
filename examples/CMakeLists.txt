cmake_minimum_required(VERSION 3.8)

# 创建服务器示例
add_executable(server_example
    server_example.cpp
)

target_link_libraries(server_example
    interfaces_server
)

# 创建简单测试
add_executable(simple_test
    simple_test.cpp
)

target_link_libraries(simple_test
    interfaces_server
)
