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

#ifndef MESSAGETYPESUPPORT_HPP_
#define MESSAGETYPESUPPORT_HPP_

#include <cassert>
#include <memory>

#include "TypeSupport.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rmw_cyclonedds_cpp {
/**
 * @brief 模板类 MessageTypeSupport，继承自 TypeSupport 类
 *
 * @tparam MembersType 成员类型参数
 */
template <typename MembersType>
class MessageTypeSupport : public TypeSupport<MembersType> {
public:
  /**
   * @brief 构造函数，初始化 MessageTypeSupport 对象
   *
   * @param members 指向 MembersType 类型的指针，用于初始化 TypeSupport 基类
   */
  explicit MessageTypeSupport(const MembersType* members);
};

}  // namespace rmw_cyclonedds_cpp
namespace rmw_cyclonedds_cpp {
/**
 * @brief 模板类 MessageTypeSupport，继承自 TypeSupport 类
 *
 * @tparam MembersType 成员类型参数
 */
template <typename MembersType>
class MessageTypeSupport : public TypeSupport<MembersType> {
public:
  /**
   * @brief 构造函数，初始化 MessageTypeSupport 对象
   *
   * @param members 指向 MembersType 类型的指针，用于初始化 TypeSupport 基类
   */
  explicit MessageTypeSupport(const MembersType* members);
};

}  // namespace rmw_cyclonedds_cpp

#include "MessageTypeSupport_impl.hpp"

#endif  // MESSAGETYPESUPPORT_HPP_
