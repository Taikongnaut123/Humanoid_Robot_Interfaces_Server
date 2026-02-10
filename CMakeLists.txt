cmake_minimum_required(VERSION 3.8)
project(interfaces_server)
find_package(loc_msg REQUIRED)
find_package(rclcpp REQUIRED)

message(STATUS "loc_msg_INCLUDE_DIRS: ${loc_msg_INCLUDE_DIRS}")
# message(STATUS "loc_msg_LIBRARIES: ${loc_msg_LIBRARIES}")
message(STATUS "rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 设置项目变量
set(INTERFACES_SERVER_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(INTERFACES_SERVER_INCLUDE_DIR "${INTERFACES_SERVER_ROOT_PATH}/include")
set(INTERFACES_SERVER_SOURCE_DIR "${INTERFACES_SERVER_ROOT_PATH}/source")
set(INTERFACES_SERVER_EXAMPLES_DIR "${INTERFACES_SERVER_ROOT_PATH}/examples")

include_directories(${INTERFACES_SERVER_INCLUDE_DIR})


# 添加子目录
add_subdirectory(source)
add_subdirectory(service)
