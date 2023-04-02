// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "TypeSupport.hpp"

#include "rosidl_runtime_c/string.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "serdes.hpp"

namespace rmw_cyclonedds_cpp {

/**
 * @brief 获取消息的大小
 *
 * @param type_supports 消息类型支持的指针
 * @return size_t 返回消息的大小
 */
size_t get_message_size(const rosidl_message_type_support_t *type_supports) {
  // 处理 C++ 类型支持
  const rosidl_message_type_support_t *ts = get_message_typesupport_handle(
      type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (ts != nullptr) {
    // 获取 C++ 类型支持的成员信息
    auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
    return members->size_of_;
  } else {
    // 处理 C 类型支持
    const rosidl_message_type_support_t *ts_c = get_message_typesupport_handle(
        type_supports, rosidl_typesupport_introspection_c__identifier);
    if (ts_c != nullptr) {
      // 获取 C 类型支持的成员信息
      auto members =
          static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts_c->data);
      return members->size_of_;
    } else {
      throw std::runtime_error("get_message_size, unsupported typesupport");
    }
  }
}

/**
 * @brief 初始化消息
 *
 * @param type_supports 消息类型支持的指针
 * @param message 需要初始化的消息指针
 */
void init_message(const rosidl_message_type_support_t *type_supports, void *message) {
  // 处理 C++ 类型支持
  const rosidl_message_type_support_t *ts = get_message_typesupport_handle(
      type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (ts != nullptr) {
    // 获取 C++ 类型支持的成员信息
    auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
    members->init_function(message, rosidl_runtime_cpp::MessageInitialization::ALL);
  } else {
    // 处理 C 类型支持
    const rosidl_message_type_support_t *ts_c = get_message_typesupport_handle(
        type_supports, rosidl_typesupport_introspection_c__identifier);
    if (ts_c != nullptr) {
      // 获取 C 类型支持的成员信息
      auto members =
          static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts_c->data);
      members->init_function(message, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
    } else {
      throw std::runtime_error("get_message_size, unsupported typesupport");
    }
  }
}

/**
 * @brief 释放消息资源
 *
 * @param type_supports 消息类型支持的指针
 * @param message 需要释放的消息指针
 */
void fini_message(const rosidl_message_type_support_t *type_supports, void *message) {
  // 处理 C++ 类型支持
  const rosidl_message_type_support_t *ts = get_message_typesupport_handle(
      type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (ts != nullptr) {
    // 获取 C++ 类型支持的成员信息
    auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
    // 调用成员的 fini 函数，释放消息资源
    members->fini_function(message);
  } else {
    // 处理 C 类型支持
    const rosidl_message_type_support_t *ts_c = get_message_typesupport_handle(
        type_supports, rosidl_typesupport_introspection_c__identifier);
    if (ts_c != nullptr) {
      // 获取 C 类型支持的成员信息
      auto members =
          static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts_c->data);
      // 调用成员的 fini 函数，释放消息资源
      members->fini_function(message);
    } else {
      // 抛出运行时错误，不支持的类型支持
      throw std::runtime_error("get_message_size, unsupported typesupport");
    }
  }
}

}  // namespace rmw_cyclonedds_cpp
