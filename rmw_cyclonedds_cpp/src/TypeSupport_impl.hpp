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

#ifndef TYPESUPPORT_IMPL_HPP_
#define TYPESUPPORT_IMPL_HPP_

#include <cassert>
#include <functional>
#include <string>
#include <vector>

#include "TypeSupport.hpp"
#include "macros.hpp"
#include "rmw/error_handling.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "serdes.hpp"
#include "u16string.hpp"

namespace rmw_cyclonedds_cpp {

template <typename T>
struct GenericCSequence;

// multiple definitions of ambiguous primitive types
SPECIALIZE_GENERIC_C_SEQUENCE(bool, bool)
SPECIALIZE_GENERIC_C_SEQUENCE(byte, uint8_t)
SPECIALIZE_GENERIC_C_SEQUENCE(char, char)
SPECIALIZE_GENERIC_C_SEQUENCE(float32, float)
SPECIALIZE_GENERIC_C_SEQUENCE(float64, double)
SPECIALIZE_GENERIC_C_SEQUENCE(int8, int8_t)
SPECIALIZE_GENERIC_C_SEQUENCE(int16, int16_t)
SPECIALIZE_GENERIC_C_SEQUENCE(uint16, uint16_t)
SPECIALIZE_GENERIC_C_SEQUENCE(int32, int32_t)
SPECIALIZE_GENERIC_C_SEQUENCE(uint32, uint32_t)
SPECIALIZE_GENERIC_C_SEQUENCE(int64, int64_t)
SPECIALIZE_GENERIC_C_SEQUENCE(uint64, uint64_t)

/**
 * @brief 构造函数，初始化TypeSupport对象
 *
 * @tparam MembersType 成员类型
 */
template <typename MembersType>
TypeSupport<MembersType>::TypeSupport() {
  name = "";  // 初始化名称为空字符串
}

/**
 * @brief 设置TypeSupport对象的名称
 *
 * @tparam MembersType 成员类型
 * @param[in] name 要设置的名称
 */
template <typename MembersType>
void TypeSupport<MembersType>::setName(const std::string &name) {
  this->name = std::string(name);  // 将传入的名称赋值给当前对象的名称
}

/**
 * @brief 对齐整数值
 *
 * @tparam T 整数类型
 * @param[in] __align 对齐值
 * @param[in] __int 要对齐的整数
 * @return 返回对齐后的整数
 */
template <typename T>
static inline T align_int_(size_t __align, T __int) noexcept {
  return (__int - 1u + __align) & ~(__align - 1);  // 计算并返回对齐后的整数值
}

/**
 * @brief 调整消息成员字段的大小（C++版）
 *
 * @param[in] member 消息成员指针
 * @param[in,out] field 要调整大小的字段指针
 * @param[in] size 要调整到的大小
 */
inline void resize_field(const rosidl_typesupport_introspection_cpp::MessageMember *member,
                         void *field,
                         size_t size) {
  if (!member->resize_function) {
    throw std::runtime_error(
        "unexpected error: resize function is null");  // 如果调整大小函数为空，抛出异常
  }

  member->resize_function(field, size);  // 调用调整大小函数，调整字段的大小
}

/**
 * @brief 调整消息成员字段的大小（C版）
 *
 * @param[in] member 消息成员指针
 * @param[in,out] field 要调整大小的字段指针
 * @param[in] size 要调整到的大小
 */
inline void resize_field(const rosidl_typesupport_introspection_c__MessageMember *member,
                         void *field,
                         size_t size) {
  if (!member->resize_function) {
    throw std::runtime_error(
        "unexpected error: resize function is null");  // 如果调整大小函数为空，抛出异常
  }

  if (!member->resize_function(field, size)) {
    throw std::runtime_error("unable to resize field");  // 如果调整大小失败，抛出异常
  }
}

/**
 * @brief 反序列化字段
 *
 * @tparam T 字段类型
 * @param[in] member 消息成员指针，包含字段的元数据信息
 * @param[out] field 要反序列化的字段指针
 * @param[in,out] deser 反序列化器对象
 */
template <typename T>
void deserialize_field(const rosidl_typesupport_introspection_cpp::MessageMember *member,
                       void *field,
                       cycdeser &deser) {
  // 如果字段不是数组
  if (!member->is_array_) {
    // 直接反序列化字段值
    deser >> *static_cast<T *>(field);
  } else if (member->array_size_ && !member->is_upper_bound_) {  // 如果字段是固定大小的数组
    // 反序列化固定大小的数组
    deser.deserializeA(static_cast<T *>(field), member->array_size_);
  } else {  // 如果字段是可变大小的数组
    // 反序列化为 std::vector 对象
    auto &vector = *reinterpret_cast<std::vector<T> *>(field);
    deser >> vector;
  }
}

/**
 * @brief 反序列化 std::string 类型的字段
 *
 * @param[in] member 消息成员指针，包含字段的元数据信息
 * @param[out] field 要反序列化的字段指针
 * @param[in,out] deser 反序列化器对象
 */
template <>
inline void deserialize_field<std::string>(
    const rosidl_typesupport_introspection_cpp::MessageMember *member,
    void *field,
    cycdeser &deser) {
  // 如果字段不是数组
  if (!member->is_array_) {
    // 直接反序列化字段值
    deser >> *static_cast<std::string *>(field);
  } else if (member->array_size_ && !member->is_upper_bound_) {  // 如果字段是固定大小的数组
    // 反序列化固定大小的数组
    std::string *array = static_cast<std::string *>(field);
    deser.deserializeA(array, member->array_size_);
  } else {  // 如果字段是可变大小的数组
    // 反序列化为 std::vector 对象
    auto &vector = *reinterpret_cast<std::vector<std::string> *>(field);
    deser >> vector;
  }
}

/**
 * @brief 反序列化 std::wstring 类型的字段
 *
 * @param[in] member 消息成员指针，包含字段的元数据信息
 * @param[out] field 要反序列化的字段指针
 * @param[in,out] deser 反序列化器对象
 */
template <>
inline void deserialize_field<std::wstring>(
    const rosidl_typesupport_introspection_cpp::MessageMember *member,
    void *field,
    cycdeser &deser) {
  std::wstring wstr;  // 定义宽字符串变量
  // 如果字段不是数组
  if (!member->is_array_) {
    // 反序列化宽字符串
    deser >> wstr;
    // 将宽字符串转换为 UTF-16 编码的字符串
    wstring_to_u16string(wstr, *static_cast<std::u16string *>(field));
  } else {
    uint32_t size;  // 数组大小
    // 如果字段是固定大小的数组
    if (member->array_size_ && !member->is_upper_bound_) {
      size = static_cast<uint32_t>(member->array_size_);
    } else {  // 如果字段是可变大小的数组
      // 反序列化数组大小
      deser >> size;
      // 调整字段数组大小
      resize_field(member, field, size);
    }
    // 遍历数组元素
    for (size_t i = 0; i < size; ++i) {
      // 获取数组元素指针
      void *element = member->get_function(field, i);
      auto u16str = static_cast<std::u16string *>(element);
      // 反序列化宽字符串
      deser >> wstr;
      // 将宽字符串转换为 UTF-16 编码的字符串
      wstring_to_u16string(wstr, *u16str);
    }
  }
}

/**
 * @brief 反序列化字段
 *
 * @tparam T 字段类型
 * @param[in] member 消息成员指针
 * @param[out] field 字段数据指针
 * @param[in,out] deser 反序列化对象
 */
template <typename T>
void deserialize_field(const rosidl_typesupport_introspection_c__MessageMember *member,
                       void *field,
                       cycdeser &deser) {
  // 如果不是数组类型
  if (!member->is_array_) {
    // 反序列化单个值
    deser >> *static_cast<T *>(field);
  } else if (member->array_size_ && !member->is_upper_bound_) {  // 如果是固定大小的数组
    // 反序列化固定大小的数组
    deser.deserializeA(static_cast<T *>(field), member->array_size_);
  } else {  // 如果是可变大小的数组
    // 引用泛型C序列
    auto &data = *reinterpret_cast<typename GenericCSequence<T>::type *>(field);
    int32_t dsize = 0;
    // 反序列化数组大小
    deser >> dsize;
    // 初始化泛型序列
    if (!GenericCSequence<T>::init(&data, dsize)) {
      throw std::runtime_error("unable initialize generic sequence");
    }
    // 反序列化可变大小的数组
    deser.deserializeA(reinterpret_cast<T *>(data.data), dsize);
  }
}

/**
 * @brief 反序列化字符串字段的特化版本
 *
 * @param[in] member 消息成员指针
 * @param[out] field 字段数据指针
 * @param[in,out] deser 反序列化对象
 */
template <>
inline void deserialize_field<std::string>(
    const rosidl_typesupport_introspection_c__MessageMember *member, void *field, cycdeser &deser) {
  // 如果不是数组类型
  if (!member->is_array_) {
    using CStringHelper = StringHelper<rosidl_typesupport_introspection_c__MessageMembers>;
    // 分配字符串
    CStringHelper::assign(deser, field);
  } else {
    // 如果是固定大小的数组
    if (member->array_size_ && !member->is_upper_bound_) {
      auto deser_field = static_cast<rosidl_runtime_c__String *>(field);
      // 定义临时字符串，避免在循环中每次都分配内存
      std::string tmpstring;
      for (size_t i = 0; i < member->array_size_; ++i) {
        // 反序列化字符串
        deser.deserialize(tmpstring);
        // 分配字符串
        if (!rosidl_runtime_c__String__assign(&deser_field[i], tmpstring.c_str())) {
          throw std::runtime_error("unable to assign rosidl_runtime_c__String");
        }
      }
    } else {  // 如果是可变大小的数组
      std::vector<std::string> cpp_string_vector;
      // 反序列化字符串向量
      deser >> cpp_string_vector;

      auto &string_array_field = *reinterpret_cast<rosidl_runtime_c__String__Sequence *>(field);
      // 初始化字符串数组
      if (!rosidl_runtime_c__String__Sequence__init(&string_array_field,
                                                    cpp_string_vector.size())) {
        throw std::runtime_error("unable to initialize rosidl_runtime_c__String array");
      }

      for (size_t i = 0; i < cpp_string_vector.size(); ++i) {
        // 分配字符串
        if (!rosidl_runtime_c__String__assign(&string_array_field.data[i],
                                              cpp_string_vector[i].c_str())) {
          throw std::runtime_error("unable to assign rosidl_runtime_c__String");
        }
      }
    }
  }
}

/**
 * @brief 反序列化 std::wstring 类型的字段
 *
 * @param[in] member 消息成员的指针，包含字段的元数据信息
 * @param[out] field 存储反序列化结果的指针
 * @param[in,out] deser 反序列化器对象，用于从二进制数据中提取信息
 */
template <>
inline void deserialize_field<std::wstring>(
    const rosidl_typesupport_introspection_c__MessageMember *member,  // 输入参数：消息成员指针
    void *field,      // 输出参数：存储反序列化结果的指针
    cycdeser &deser)  // 输入输出参数：反序列化器对象
{
  std::wstring wstr;  // 定义一个宽字符串变量

  // 如果字段不是数组类型
  if (!member->is_array_) {
    deser >> wstr;  // 从反序列化器中读取宽字符串
    // 将宽字符串转换为 U16 字符串并存储到 field 中
    wstring_to_u16string(wstr, *static_cast<rosidl_runtime_c__U16String *>(field));
  }
  // 如果字段是固定大小的数组类型
  else if (member->array_size_ && !member->is_upper_bound_) {
    auto array = static_cast<rosidl_runtime_c__U16String *>(field);  // 获取数组指针
    // 遍历数组元素
    for (size_t i = 0; i < member->array_size_; ++i) {
      deser >> wstr;  // 从反序列化器中读取宽字符串
      // 将宽字符串转换为 U16 字符串并存储到数组中
      wstring_to_u16string(wstr, array[i]);
    }
  }
  // 如果字段是可变大小的数组类型（序列）
  else {
    uint32_t size;  // 定义一个变量用于存储序列大小
    deser >> size;  // 从反序列化器中读取序列大小

    // 获取序列指针
    auto sequence = static_cast<rosidl_runtime_c__U16String__Sequence *>(field);

    // 初始化序列，如果初始化失败则抛出异常
    if (!rosidl_runtime_c__U16String__Sequence__init(sequence, size)) {
      throw std::runtime_error("unable to initialize rosidl_runtime_c__U16String sequence");
    }

    // 遍历序列元素
    for (size_t i = 0; i < sequence->size; ++i) {
      deser >> wstr;  // 从反序列化器中读取宽字符串
      // 将宽字符串转换为 U16 字符串并存储到序列中
      wstring_to_u16string(wstr, sequence->data[i]);
    }
  }
}

/**
 * @brief 反序列化ROS消息
 *
 * @tparam MembersType 成员类型
 * @param deser 反序列化对象
 * @param members 成员指针
 * @param ros_message ROS消息指针
 * @return bool 反序列化是否成功
 *
 * 该函数用于将二进制数据反序列化为ROS消息。
 */
template <typename MembersType>
bool TypeSupport<MembersType>::deserializeROSmessage(cycdeser &deser,
                                                     const MembersType *members,
                                                     void *ros_message) {
  // 断言：成员和ROS消息不能为空
  assert(members);
  assert(ros_message);

  // 遍历所有成员
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    // 获取当前成员
    const auto *member = members->members_ + i;
    // 计算字段在ROS消息中的偏移量
    void *field = static_cast<char *>(ros_message) + member->offset_;
    // 根据成员类型进行反序列化
    switch (member->type_id_) {
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        deserialize_field<bool>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        deserialize_field<uint8_t>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        deserialize_field<char>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
        deserialize_field<float>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        deserialize_field<double>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        deserialize_field<int16_t>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        deserialize_field<uint16_t>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        deserialize_field<int32_t>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        deserialize_field<uint32_t>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        deserialize_field<int64_t>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        deserialize_field<uint64_t>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        deserialize_field<std::string>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        deserialize_field<std::wstring>(member, field, deser);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        // 获取子成员
        auto sub_members = (const MembersType *)member->members_->data;
        if (!member->is_array_) {
          // 非数组类型，直接反序列化
          deserializeROSmessage(deser, sub_members, field);
        } else {
          // 数组类型，需要处理数组大小
          size_t array_size = 0;

          if (member->array_size_ && !member->is_upper_bound_) {
            array_size = member->array_size_;
          } else {
            array_size = deser.deserialize_len(1);
            resize_field(member, field, array_size);
          }

          // 检查get_function是否为空
          if (array_size != 0 && !member->get_function) {
            RMW_SET_ERROR_MSG("unexpected error: get_function function is null");
            return false;
          }
          // 遍历数组并反序列化每个元素
          for (size_t index = 0; index < array_size; ++index) {
            deserializeROSmessage(deser, sub_members, member->get_function(field, index));
          }
        }
      } break;
      default:
        // 未知类型，抛出异常
        throw std::runtime_error("unknown type");
    }
  }

  // 反序列化成功
  return true;
}

/**
 * @brief 打印字段值的函数模板
 *
 * @tparam M 成员指针类型
 * @tparam T 数据类型
 * @param[in] member 指向成员的指针
 * @param[in,out] deser cycprint对象，用于序列化和打印
 * @param[out] dummy 临时变量，用于存储读取到的数据
 *
 * 该函数根据传入的成员指针和数据类型，从cycprint对象中读取数据并打印。
 * 如果成员不是数组，则直接读取并打印；
 * 如果成员是数组，则先打印"{"，然后根据数组大小逐个打印元素，最后打印"}"。
 */
template <typename M, typename T>
void print_field(const M *member, cycprint &deser, T &dummy) {
  // 判断成员是否为数组
  if (!member->is_array_) {
    // 不是数组，直接读取并打印
    deser >> dummy;
  } else {
    // 是数组，先打印"{"
    deser.print_constant("{");

    // 判断数组大小是否已知且非上界
    if (member->array_size_ && !member->is_upper_bound_) {
      // 已知数组大小，按大小打印数组元素
      deser.printA(&dummy, member->array_size_);
    } else {
      // 未知数组大小，先获取数组大小
      int32_t dsize = deser.get_len(1);

      // 按获取到的大小打印数组元素
      deser.printA(&dummy, dsize);
    }

    // 打印"}"
    deser.print_constant("}");
  }
}

/**
 * @brief 打印 ROS 消息的函数
 *
 * @tparam MembersType 成员类型模板参数
 * @param deser 一个 cycprint 类型的引用，用于序列化和打印消息
 * @param members 指向 MembersType 类型的指针，表示要打印的消息成员
 * @return bool 返回 true 表示成功打印消息
 */
template <typename MembersType>
bool TypeSupport<MembersType>::printROSmessage(cycprint &deser, const MembersType *members) {
  assert(members);                                         // 断言 members 不为空

  deser.print_constant("{");                               // 打印左大括号开始
  for (uint32_t i = 0; i < members->member_count_; ++i) {  // 遍历所有成员
    if (i != 0) {
      deser.print_constant(",");                           // 在成员之间添加逗号分隔符
    }
    const auto *member = members->members_ + i;            // 获取当前成员的指针
    switch (member->type_id_) {                            // 根据成员的类型 ID 进行处理
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
        bool dummy;
        print_field(member, deser, dummy);  // 处理布尔类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
        uint8_t dummy;
        print_field(member, deser, dummy);  // 处理字节和无符号 8 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
        char dummy;
        print_field(member, deser, dummy);  // 处理字符和 8 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32: {
        float dummy;
        print_field(member, deser, dummy);  // 处理单精度浮点数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64: {
        double dummy;
        print_field(member, deser, dummy);  // 处理双精度浮点数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
        int16_t dummy;
        print_field(member, deser, dummy);  // 处理 16 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
        uint16_t dummy;
        print_field(member, deser, dummy);  // 处理无符号 16 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
        int32_t dummy;
        print_field(member, deser, dummy);  // 处理 32 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
        uint32_t dummy;
        print_field(member, deser, dummy);  // 处理无符号 32 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
        int64_t dummy;
        print_field(member, deser, dummy);  // 处理 64 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
        uint64_t dummy;
        print_field(member, deser, dummy);  // 处理无符号 64 位整数类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
        std::string dummy;
        print_field(member, deser, dummy);  // 处理字符串类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING: {
        std::wstring dummy;
        print_field(member, deser, dummy);  // 处理宽字符串类型
      } break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        auto sub_members = (const MembersType *)member->members_->data;  // 获取子消息成员
        if (!member->is_array_) {                                        // 如果不是数组类型
          printROSmessage(deser, sub_members);                           // 递归打印子消息
        } else {                                                         // 如果是数组类型
          size_t array_size = 0;
          if (member->array_size_ && !member->is_upper_bound_) {
            array_size = member->array_size_;  // 获取固定大小的数组长度
          } else {
            array_size = deser.get_len(1);     // 获取动态大小的数组长度
          }
          deser.print_constant("{");           // 打印左大括号开始
          for (size_t index = 0; index < array_size; ++index) {  // 遍历数组元素
            printROSmessage(deser, sub_members);                 // 递归打印子消息
          }
          deser.print_constant("}");                             // 打印右大括号结束
        }
      } break;
      default:
        throw std::runtime_error("unknown type");  // 抛出未知类型异常
    }
  }
  deser.print_constant("}");  // 打印右大括号结束

  return true;                // 返回成功
}

/**
 * @brief 反序列化ROS消息
 *
 * @tparam MembersType 成员类型
 * @param deser 反序列化对象
 * @param ros_message ROS消息指针
 * @param prefix 前缀函数，可选
 * @return bool 反序列化是否成功
 */
template <typename MembersType>
bool TypeSupport<MembersType>::deserializeROSmessage(cycdeser &deser,
                                                     void *ros_message,
                                                     std::function<void(cycdeser &)> prefix) {
  assert(ros_message);  // 断言ros_message不为空

  if (prefix) {         // 如果有前缀函数
    prefix(deser);      // 调用前缀函数
  }

  if (members_->member_count_ != 0) {                                  // 如果成员数量不为0
    TypeSupport::deserializeROSmessage(deser, members_, ros_message);  // 反序列化ROS消息
  } else {                                                             // 否则
    uint8_t dump = 0;                                                  // 创建一个临时变量
    deser >> dump;  // 读取数据到临时变量
    (void)dump;     // 忽略临时变量
  }

  return true;  // 返回反序列化成功
}

/**
 * @brief 打印ROS消息
 *
 * @tparam MembersType 成员类型
 * @param prt 打印对象
 * @param prefix 前缀函数，可选
 * @return bool 打印是否成功
 */
template <typename MembersType>
bool TypeSupport<MembersType>::printROSmessage(cycprint &prt,
                                               std::function<void(cycprint &)> prefix) {
  if (prefix) {               // 如果有前缀函数
    prt.print_constant("{");  // 打印左大括号
    prefix(prt);              // 调用前缀函数
    prt.print_constant(",");  // 打印逗号
  }

  if (members_->member_count_ != 0) {             // 如果成员数量不为0
    TypeSupport::printROSmessage(prt, members_);  // 打印ROS消息
  } else {                                        // 否则
    uint8_t dump = 0;                             // 创建一个临时变量
    prt >> dump;                                  // 读取数据到临时变量
    (void)dump;                                   // 忽略临时变量
  }

  if (prefix) {               // 如果有前缀函数
    prt.print_constant("}");  // 打印右大括号
  }

  return true;  // 返回打印成功
}

/**
 * @brief 获取类型支持的名称
 *
 * @tparam MembersType 成员类型
 * @return std::string 类型支持的名称
 */
template <typename MembersType>
std::string TypeSupport<MembersType>::getName() {
  return name;  // 返回类型支持的名称
}

/**
 * @brief 判断类型是否自包含
 *
 * @tparam MembersType 成员类型
 * @param[in] members 成员指针
 * @return bool 如果类型自包含则返回true，否则返回false
 */
template <typename MembersType>
bool TypeSupport<MembersType>::is_type_self_contained(const MembersType *members) {
  // 遍历成员列表
  for (uint32_t idx = 0; idx < members->member_count_; ++idx) {
    const auto member = members->members_[idx];  // 获取当前成员

    // 如果消息不是自包含的，即字符串、宽字符串或序列
    if ((member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) ||
        (member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING) ||
        // 数组 => is_array = true, array_size > 0, upper_bound = 0
        // 无界序列 => is_array = true, array size = 0, upper_bound = 0
        // 有界序列 => is_array = true, array size > 0, upper_bound = 1
        (member.is_array_ && (!member.array_size_ || member.is_upper_bound_))) {
      return false;  // 类型不是自包含的
    } else if (member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      // 处理嵌套消息
      auto sub_members = (const MembersType *)member.members_->data;
      if (!is_type_self_contained(sub_members)) {
        return false;
      }
    }
  }

  return true;  // 类型是自包含的
}

/**
 * @brief 判断类型是否自包含（无参数版本）
 *
 * @tparam MembersType 成员类型
 * @return bool 如果类型自包含则返回true，否则返回false
 */
template <typename MembersType>
bool TypeSupport<MembersType>::is_type_self_contained() {
  return TypeSupport::is_type_self_contained(members_);  // 调用带参数版本的函数
}
}  // namespace rmw_cyclonedds_cpp

#endif  // TYPESUPPORT_IMPL_HPP_
