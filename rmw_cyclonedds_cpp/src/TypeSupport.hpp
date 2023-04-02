// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright 2018 ADLINK Technology
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

#ifndef TYPESUPPORT_HPP_
#define TYPESUPPORT_HPP_

#include <cassert>
#include <functional>
#include <string>

#include "rcutils/logging_macros.h"
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "serdes.hpp"

namespace rmw_cyclonedds_cpp {

// 辅助类，使用模板特化来实现从 cycser/cycdeser 读取/写入字符串类型
template <typename MembersType>
struct StringHelper;

// 对于 C 语言内省类型支持，我们创建 std::string 的中间实例，
// 以便 cycser/cycdeser 可以正确处理字符串。
template <>
struct StringHelper<rosidl_typesupport_introspection_c__MessageMembers> {
  // 定义类型为 rosidl_runtime_c__String
  using type = rosidl_runtime_c__String;

  // 将 void * 数据转换为 std::string
  static std::string convert_to_std_string(void *data) {
    // 将数据强制转换为 rosidl_runtime_c__String 类型
    auto c_string = static_cast<rosidl_runtime_c__String *>(data);
    // 如果转换失败，记录错误并返回空字符串
    if (!c_string) {
      RCUTILS_LOG_ERROR_NAMED("rmw_cyclonedds_cpp",
                              "Failed to cast data as rosidl_runtime_c__String");
      return "";
    }
    // 如果字符串数据无效，记录错误并返回空字符串
    if (!c_string->data) {
      RCUTILS_LOG_ERROR_NAMED("rmw_cyclonedds_cpp", "rosidl_generator_c_String had invalid data");
      return "";
    }
    // 返回转换后的 std::string
    return std::string(c_string->data);
  }

  // 将 rosidl_runtime_c__String 数据转换为 std::string
  static std::string convert_to_std_string(rosidl_runtime_c__String &data) {
    return std::string(data.data);
  }

  // 将 cycdeser 中的数据分配给 field
  static void assign(cycdeser &deser, void *field) {
    // 创建一个 std::string 对象
    std::string str;
    // 从 deser 中读取字符串
    deser >> str;
    // 将 field 强制转换为 rosidl_runtime_c__String 类型
    rosidl_runtime_c__String *c_str = static_cast<rosidl_runtime_c__String *>(field);
    // 将 std::string 分配给 rosidl_runtime_c__String
    rosidl_runtime_c__String__assign(c_str, str.c_str());
  }
};

// 为 C++ 内省类型支持，我们直接重用相同的 std::string。
template <>
struct StringHelper<rosidl_typesupport_introspection_cpp::MessageMembers> {
  // 定义 type 为 std::string 类型
  using type = std::string;

  // 将 void 指针转换为 std::string 引用
  static std::string &convert_to_std_string(void *data) {
    return *(static_cast<std::string *>(data));
  }

  // 为字段分配值
  static void assign(cycdeser &deser, void *field) {
    std::string &str = *(std::string *)field;
    deser >> str;
  }
};

// 类模板 TypeSupport
template <typename MembersType>
class TypeSupport {
 public:
  // 反序列化 ROS 消息
  bool deserializeROSmessage(cycdeser &deser,
                             void *ros_message,
                             std::function<void(cycdeser &)> prefix = nullptr);

  // 打印 ROS 消息
  bool printROSmessage(cycprint &deser, std::function<void(cycprint &)> prefix = nullptr);

  // 获取名称
  std::string getName();

  // 判断类型是否自包含
  bool is_type_self_contained();

  // 虚析构函数
  virtual ~TypeSupport() = default;

 protected:
  // 默认构造函数
  TypeSupport();

  // 设置名称
  void setName(const std::string &name);

  // 成员指针
  const MembersType *members_;
  std::string name;

 private:
  // 反序列化 ROS 消息（私有方法）
  bool deserializeROSmessage(cycdeser &deser, const MembersType *members, void *ros_message);

  // 打印 ROS 消息（私有方法）
  bool printROSmessage(cycprint &deser, const MembersType *members);

  // 判断类型是否自包含（私有方法）
  bool is_type_self_contained(const MembersType *members);
};

/**
 * @brief 获取消息的大小
 *
 * @param type_supports 指向rosidl_message_type_support_t结构体的指针，用于提供消息类型支持
 * @return size_t 返回消息的大小（以字节为单位）
 */
size_t get_message_size(const rosidl_message_type_support_t *type_supports);

/**
 * @brief 初始化消息
 *
 * @param type_supports 指向rosidl_message_type_support_t结构体的指针，用于提供消息类型支持
 * @param message 指向需要初始化的消息的指针
 */
void init_message(const rosidl_message_type_support_t *type_supports, void *message);

/**
 * @brief 清理消息
 *
 * @param type_supports 指向rosidl_message_type_support_t结构体的指针，用于提供消息类型支持
 * @param message 指向需要清理的消息的指针
 */
void fini_message(const rosidl_message_type_support_t *type_supports, void *message);

}  // namespace rmw_cyclonedds_cpp

#include "TypeSupport_impl.hpp"

#endif  // TYPESUPPORT_HPP_
