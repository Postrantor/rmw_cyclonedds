# Copyright 2018 ADLINK Technology Limited.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# 设置CMake最低版本要求为3.5
cmake_minimum_required(VERSION 3.5)

# 定义项目名称为rmw_cyclonedds_cpp
project(rmw_cyclonedds_cpp)

# 默认使用C++14标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 如果编译器是GNU或Clang，添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wimplicit-fallthrough)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(tracetools REQUIRED)

find_package(CycloneDDS QUIET CONFIG)
if(CycloneDDS_FOUND)
  # RMW中的共享内存支持取决于Cyclone DDS中是否编译了共享内存支持以及iceoryx_binding_c是否可用
  get_target_property(_cyclonedds_has_shm CycloneDDS::ddsc
                      SHM_SUPPORT_IS_AVAILABLE)
  if(_cyclonedds_has_shm)
    find_package(iceoryx_binding_c REQUIRED)
  else()
    message(STATUS "Cyclone DDS is NOT compiled with support for shared memory")
  endif()
else()
  message(
    WARNING "Could not find Eclipse Cyclone DDS - skipping '${PROJECT_NAME}'")
  ament_package()
  return()
endif()

find_package(rmw REQUIRED)
find_package(rmw_dds_common REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

if(_cyclonedds_has_shm)
  ament_export_dependencies(iceoryx_binding_c)
endif()
ament_export_dependencies(CycloneDDS)
ament_export_dependencies(rcutils)
ament_export_dependencies(rcpputils)
ament_export_dependencies(rmw)
ament_export_dependencies(rosidl_runtime_c)
ament_export_dependencies(rmw_dds_common)
ament_export_dependencies(rosidl_typesupport_introspection_c)
ament_export_dependencies(rosidl_typesupport_introspection_cpp)
ament_export_dependencies(tracetools)

add_library(
  rmw_cyclonedds_cpp
  src/rmw_get_network_flow_endpoints.cpp
  src/rmw_node.cpp
  src/serdata.cpp
  src/serdes.cpp
  src/u16string.cpp
  src/exception.cpp
  src/demangle.cpp
  src/deserialization_exception.cpp
  src/Serialization.cpp
  src/TypeSupport2.cpp
  src/TypeSupport.cpp)

# 链接CycloneDDS::ddsc库到rmw_cyclonedds_cpp目标
target_link_libraries(rmw_cyclonedds_cpp PRIVATE CycloneDDS::ddsc)

# 如果_cyclonedds_has_shm为真，则链接iceoryx_binding_c::iceoryx_binding_c库到rmw_cyclonedds_cpp目标
if(_cyclonedds_has_shm)
  target_link_libraries(rmw_cyclonedds_cpp
                        PRIVATE iceoryx_binding_c::iceoryx_binding_c)
endif()

# 链接rmw::rmw库到rmw_cyclonedds_cpp目标，作为公共依赖
target_link_libraries(rmw_cyclonedds_cpp PUBLIC rmw::rmw)

# 链接以下私有依赖库到rmw_cyclonedds_cpp目标
target_link_libraries(
  rmw_cyclonedds_cpp
  PRIVATE
    rcutils::rcutils
    rcpputils::rcpputils
    rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
    rmw_dds_common::rmw_dds_common_library
    rosidl_runtime_c::rosidl_runtime_c
    tracetools::tracetools)

# 设置TARGET_ARCH变量
if(CMAKE_GENERATOR_PLATFORM)
  set(TARGET_ARCH "${CMAKE_GENERATOR_PLATFORM}")
else()
  set(TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 如果使用GNU编译器且目标架构为RISC-V，则链接libatomic库到rmw_cyclonedds_cpp目标
if(CMAKE_COMPILER_IS_GNUCXX AND TARGET_ARCH MATCHES "^(riscv|RISCV)64$")
  target_link_libraries(rmw_cyclonedds_cpp PUBLIC -latomic)
endif()

# 配置rmw_cyclonedds_cpp库
configure_rmw_library(rmw_cyclonedds_cpp)

# 为${PROJECT_NAME}添加私有编译定义
target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE RMW_VERSION_MAJOR=${rmw_VERSION_MAJOR}
          RMW_VERSION_MINOR=${rmw_VERSION_MINOR}
          RMW_VERSION_PATCH=${rmw_VERSION_PATCH})

# 导出目标，使其他包可以使用 rmw_cyclonedds_cpp
ament_export_targets(export_rmw_cyclonedds_cpp)

# 注册 RMW 实现，提供 C 和 C++ 类型支持
register_rmw_implementation(
  # C 类型支持
  "c:rosidl_typesupport_c:rosidl_typesupport_introspection_c"
  # C++ 类型支持
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_introspection_cpp")

# 如果构建测试，则执行以下操作
if(BUILD_TESTING)
  # 查找 ament_lint_auto 包
  find_package(ament_lint_auto REQUIRED)
  # 自动查找测试依赖项
  ament_lint_auto_find_test_dependencies()
endif()

# 打包当前项目
ament_package()

# 安装 rmw_cyclonedds_cpp 目标
install(
  # 指定要安装的目标
  TARGETS rmw_cyclonedds_cpp
  # 导出目标
  EXPORT export_rmw_cyclonedds_cpp
  # 指定库文件的安装位置
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib # 指定可执行文件的安装位置
  RUNTIME DESTINATION bin)

# 包含 get_rmw_cyclonedds_output_filter.cmake 文件
include(cmake/get_rmw_cyclonedds_output_filter.cmake)
# 获取 rmw_cyclonedds 的输出过滤器模式
get_rmw_cyclonedds_output_filter(rmw_cyclonedds_output_patterns)
# 注册资源，使其他包可以访问输出过滤器模式
ament_index_register_resource("rmw_output_patterns" CONTENT
                              "${rmw_cyclonedds_output_patterns}")
