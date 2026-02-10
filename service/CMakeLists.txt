cmake_minimum_required(VERSION 3.8)
set(TARGET_NAME CHRIC_SDKServerService)
project(${TARGET_NAME} VERSION 1.0.0.0 DESCRIPTION "SDK Server Service" LANGUAGES CXX)


# 生成 Interfaces-Server 独立进程
add_executable(${TARGET_NAME} 
    main.cpp
)

# # 添加调试选项
# target_compile_options(${TARGET_NAME} PRIVATE -g)

# find_package(grpc++ REQUIRED)

target_link_libraries(${TARGET_NAME}
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
    chric_interfaces_server
)
set_target_properties(${TARGET_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BIN_DIR}
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    INSTALL_RPATH "\$ORIGIN/lib"
    INSTALL_RPATH_USE_LINK_PATH ON
)

# 安装可执行文件
install(TARGETS ${TARGET_NAME} DESTINATION ${CHRIC_CLOUD_FOLDER}/sdk_server)


