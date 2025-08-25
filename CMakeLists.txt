cmake_minimum_required(VERSION 3.8)
project(interfaces_server)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖 (与Client-SDK保持一致)
find_package(PkgConfig REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

# 设置项目变量
set(INTERFACES_SERVER_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(INTERFACES_SERVER_INCLUDE_DIR "${INTERFACES_SERVER_ROOT_PATH}/include")
set(INTERFACES_SERVER_SOURCE_DIR "${INTERFACES_SERVER_ROOT_PATH}/source")
set(INTERFACES_SERVER_EXAMPLES_DIR "${INTERFACES_SERVER_ROOT_PATH}/examples")



# 添加子目录
add_subdirectory(source)
add_subdirectory(examples)
add_subdirectory(service)
