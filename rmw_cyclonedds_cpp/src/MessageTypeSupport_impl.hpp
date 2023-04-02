// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef MESSAGETYPESUPPORT_IMPL_HPP_
#define MESSAGETYPESUPPORT_IMPL_HPP_

#include <cassert>
#include <memory>
#include <regex>
#include <sstream>
#include <string>

#include "MessageTypeSupport.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rmw_cyclonedds_cpp {

/**
 * @brief 构造函数，初始化MessageTypeSupport对象
 *
 * @tparam MembersType 成员类型
 * @param[in] members 消息成员指针
 */
template <typename MembersType>
MessageTypeSupport<MembersType>::MessageTypeSupport(const MembersType* members) {
  // 断言members不为空
  assert(members);
  // 设置成员变量
  this->members_ = members;

  // 创建一个字符串流
  std::ostringstream ss;
  // 获取消息的命名空间
  std::string message_namespace(this->members_->message_namespace_);
  // 获取消息的名称
  std::string message_name(this->members_->message_name_);
  // 如果消息命名空间不为空
  if (!message_namespace.empty()) {
    // 查找并替换C命名空间分隔符为C++命名空间分隔符，以防使用C类型支持
    message_namespace = std::regex_replace(message_namespace, std::regex("__"), "::");
    // 将命名空间添加到字符串流中
    ss << message_namespace << "::";
  }
  // 添加dds命名空间和消息名称到字符串流中
  ss << "dds_::" << message_name << "_";
  // 设置MessageTypeSupport的名称
  this->setName(ss.str().c_str());
}

}  // namespace rmw_cyclonedds_cpp

#endif  // MESSAGETYPESUPPORT_IMPL_HPP_
