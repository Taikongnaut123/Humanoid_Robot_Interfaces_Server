# 模块源文件 CMakeLists.txt

# 收集所有模块源文件
set(MODULE_SOURCES
    modules/module_base.cpp
    modules/perception_module.cpp
    modules/module_manager.cpp
)

# 将模块源文件添加到主库中
target_sources(interfaces_server PRIVATE ${MODULE_SOURCES})

# 添加模块头文件目录
target_include_directories(interfaces_server PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
