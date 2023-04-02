// Copyright 2019 Rover Robotics via Dan Rose
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
#ifndef TYPESUPPORT2_HPP_
#define TYPESUPPORT2_HPP_

#include <cassert>
#include <functional>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "bytewise.hpp"
#include "exception.hpp"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

namespace rmw_cyclonedds_cpp {
struct AnyValueType;

/// 连续存储对象
template <typename T>
class TypedSpan;

// 类模板定义
template <typename T>
class TypedSpan {
  const T *m_data;      // 指向数据的指针
  const size_t m_size;  // 数据大小

 public:
  /**
   * @brief 构造函数，用于初始化 TypedSpan 对象
   *
   * @param data 指向数据的指针
   * @param size 数据大小
   */
  TypedSpan(const T *data, size_t size) : m_data(data), m_size(size) {}

  // 获取数据大小
  size_t size() const { return m_size; }
  // 获取数据字节大小
  size_t size_bytes() const { return size() * sizeof(T); }
  // 获取数据指针
  const T *data() const { return m_data; }

  // 获取数据起始迭代器
  auto begin() { return m_data; }
  // 获取数据结束迭代器
  auto end() { return m_data + size(); }
};

/**
 * @brief 创建一个类型化的 span 对象
 *
 * @tparam NativeType 数据类型
 * @param m_data 指向数据的指针
 * @param size 数据大小
 * @return 创建好的 TypedSpan 对象
 */
template <typename NativeType>
auto make_typed_span(const NativeType *m_data, size_t size) {
  return TypedSpan<NativeType>{m_data, size};
}

// 类型生成器枚举
enum class TypeGenerator {
  ROSIDL_C,    // C 语言类型生成器
  ROSIDL_Cpp,  // C++ 语言类型生成器
};

/**
 * @brief 类模板，用于生成不同类型的 TypeGeneratorInfo 结构体
 * @tparam TypeGenerator 枚举类型，表示要生成的类型
 */
template <TypeGenerator>
struct TypeGeneratorInfo;

/**
 * @brief 针对 TypeGenerator::ROSIDL_C 的特化版本
 */
template <>
struct TypeGeneratorInfo<TypeGenerator::ROSIDL_C> {
  // 定义枚举值为 TypeGenerator::ROSIDL_C
  static constexpr auto enum_value = TypeGenerator::ROSIDL_C;

  /**
   * @brief 获取标识符
   * @return 返回 rosidl_typesupport_introspection_c__identifier
   */
  static const auto &get_identifier() { return rosidl_typesupport_introspection_c__identifier; }

  // 定义元消息类型
  using MetaMessage = rosidl_typesupport_introspection_c__MessageMembers;
  // 定义元成员类型
  using MetaMember = rosidl_typesupport_introspection_c__MessageMember;
  // 定义元服务类型
  using MetaService = rosidl_typesupport_introspection_c__ServiceMembers;
};

/**
 * @brief 针对 TypeGenerator::ROSIDL_Cpp 的特化版本
 */
template <>
struct TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp> {
  // 定义枚举值为 TypeGenerator::ROSIDL_Cpp
  static constexpr auto enum_value = TypeGenerator::ROSIDL_Cpp;

  /**
   * @brief 获取标识符
   * @return 返回 rosidl_typesupport_introspection_cpp::typesupport_identifier
   */
  static const auto &get_identifier() {
    return rosidl_typesupport_introspection_cpp::typesupport_identifier;
  }

  // 定义元消息类型
  using MetaMessage = rosidl_typesupport_introspection_cpp::MessageMembers;
  // 定义元成员类型
  using MetaMember = rosidl_typesupport_introspection_cpp::MessageMember;
  // 定义元服务类型
  using MetaService = rosidl_typesupport_introspection_cpp::ServiceMembers;
};

/**
 * @brief 类模板，用于获取对应 TypeGenerator 的元消息类型
 * @tparam g TypeGenerator 枚举值
 */
template <TypeGenerator g>
using MetaMessage = typename TypeGeneratorInfo<g>::MetaMessage;

/**
 * @brief 类模板，用于获取对应 TypeGenerator 的元成员类型
 * @tparam g TypeGenerator 枚举值
 */
template <TypeGenerator g>
using MetaMember = typename TypeGeneratorInfo<g>::MetaMember;

/**
 * @brief 类模板，用于获取对应 TypeGenerator 的元服务类型
 * @tparam g TypeGenerator 枚举值
 */
template <TypeGenerator g>
using MetaService = typename TypeGeneratorInfo<g>::MetaService;

// 命名空间别名，将rosidl_typesupport_introspection_cpp命名空间简化为tsi_enum
namespace tsi_enum = rosidl_typesupport_introspection_cpp;

/**
 * @brief ROSIDL_TypeKind 枚举类，用于表示不同的数据类型
 * 这些类型在C和C++之间共享
 */
enum class ROSIDL_TypeKind : uint8_t {
  FLOAT = tsi_enum::ROS_TYPE_FLOAT,              ///< 浮点数类型
  DOUBLE = tsi_enum::ROS_TYPE_DOUBLE,            ///< 双精度浮点数类型
  LONG_DOUBLE = tsi_enum::ROS_TYPE_LONG_DOUBLE,  ///< 长双精度浮点数类型
  CHAR = tsi_enum::ROS_TYPE_CHAR,                ///< 字符类型
  WCHAR = tsi_enum::ROS_TYPE_WCHAR,              ///< 宽字符类型
  BOOLEAN = tsi_enum::ROS_TYPE_BOOLEAN,          ///< 布尔类型
  OCTET = tsi_enum::ROS_TYPE_OCTET,              ///< 八位字节类型
  UINT8 = tsi_enum::ROS_TYPE_UINT8,              ///< 无符号8位整数类型
  INT8 = tsi_enum::ROS_TYPE_INT8,                ///< 有符号8位整数类型
  UINT16 = tsi_enum::ROS_TYPE_UINT16,            ///< 无符号16位整数类型
  INT16 = tsi_enum::ROS_TYPE_INT16,              ///< 有符号16位整数类型
  UINT32 = tsi_enum::ROS_TYPE_UINT32,            ///< 无符号32位整数类型
  INT32 = tsi_enum::ROS_TYPE_INT32,              ///< 有符号32位整数类型
  UINT64 = tsi_enum::ROS_TYPE_UINT64,            ///< 无符号64位整数类型
  INT64 = tsi_enum::ROS_TYPE_INT64,              ///< 有符号64位整数类型
  STRING = tsi_enum::ROS_TYPE_STRING,            ///< 字符串类型
  WSTRING = tsi_enum::ROS_TYPE_WSTRING,          ///< 宽字符串类型

  MESSAGE = tsi_enum::ROS_TYPE_MESSAGE,          ///< 消息类型
};

class StructValueType;
// 为给定的 rosidl_message_type_support_t 创建一个 StructValueType 实例
// @param mts 指向 rosidl_message_type_support_t 的指针
// @return 返回一个包含新创建的 StructValueType 实例的 std::unique_ptr
std::unique_ptr<StructValueType> make_message_value_type(const rosidl_message_type_support_t *mts);

// 为给定的 rosidl_service_type_support_t 创建请求和响应的 StructValueType 实例
// @param svc 指向 rosidl_service_type_support_t 的指针
// @return 返回一个 std::pair，其中 first 是请求的 StructValueType 实例，second 是响应的
// StructValueType 实例
std::pair<std::unique_ptr<StructValueType>, std::unique_ptr<StructValueType>>
make_request_response_value_types(const rosidl_service_type_support_t *svc);

// 枚举类，表示值类型
enum class EValueType {
  // 基本值类型
  PrimitiveValueType,
  // UTF-8 字符串值类型
  U8StringValueType,
  // UTF-16 字符串值类型
  U16StringValueType,
  // 结构体值类型
  StructValueType,
  // 数组值类型
  ArrayValueType,
  // 跨度序列值类型
  SpanSequenceValueType,
  // 布尔向量值类型
  BoolVectorValueType,
};

// 任意值类型结构体
struct AnyValueType {
  // 虚析构函数
  virtual ~AnyValueType() = default;

  // 获取此值类型占用的字节数
  // @return 返回值类型占用的字节数
  virtual size_t sizeof_type() const = 0;

  // 获取表示逻辑值类型的 EValueType
  // @return 返回表示逻辑值类型的 EValueType
  virtual EValueType e_value_type() const = 0;

  // 对给定的一元函数应用此值类型，作为 dynamic_cast 的快速替代方法
  template <typename UnaryFunction>
  auto apply(UnaryFunction f) const;

  // 对给定的一元函数应用此值类型，作为 dynamic_cast 的快速替代方法
  template <typename UnaryFunction>
  auto apply(UnaryFunction f);
};

// 成员结构体
struct Member {
  // 成员名称
  const char *name;
  // 指向 AnyValueType 的指针，表示成员的值类型
  const AnyValueType *value_type;
  // 成员在结构体中的偏移量
  size_t member_offset;
};

/**
 * @class StructValueType
 * @brief 结构值类型类，继承自AnyValueType
 */
class StructValueType : public AnyValueType {
 public:
  /**
   * @brief 获取类型种类
   * @return 返回消息类型
   */
  ROSIDL_TypeKind type_kind() const { return ROSIDL_TypeKind::MESSAGE; }

  /**
   * @brief 获取类型大小
   * @return 返回结构体大小
   */
  size_t sizeof_type() const final { return sizeof_struct(); }

  /**
   * @brief 获取结构体大小（纯虚函数）
   * @return 返回结构体大小
   */
  virtual size_t sizeof_struct() const = 0;

  /**
   * @brief 获取成员数量（纯虚函数）
   * @return 返回成员数量
   */
  virtual size_t n_members() const = 0;

  /**
   * @brief 获取指定成员（纯虚函数）
   * @param[in] 成员索引
   * @return 返回指向成员的指针
   */
  virtual const Member *get_member(size_t) const = 0;

  /**
   * @brief 获取枚举值类型
   * @return 返回StructValueType类型
   */
  EValueType e_value_type() const final { return EValueType::StructValueType; }
};

/**
 * @class ArrayValueType
 * @brief 数组值类型类，继承自AnyValueType
 */
class ArrayValueType : public AnyValueType {
 protected:
  const AnyValueType *m_element_value_type;  ///< 元素值类型指针
  size_t m_size;                             ///< 数组大小

 public:
  /**
   * @brief 构造函数
   * @param[in] element_value_type 元素值类型指针
   * @param[in] size 数组大小
   */
  ArrayValueType(const AnyValueType *element_value_type, size_t size)
      : m_element_value_type(element_value_type), m_size(size) {}

  /**
   * @brief 获取元素值类型
   * @return 返回元素值类型指针
   */
  const AnyValueType *element_value_type() const { return m_element_value_type; }

  /**
   * @brief 获取类型大小
   * @return 返回数组类型大小
   */
  size_t sizeof_type() const final { return m_size * m_element_value_type->sizeof_type(); }

  /**
   * @brief 获取数组大小
   * @return 返回数组大小
   */
  size_t array_size() const { return m_size; }

  /**
   * @brief 获取数据指针
   * @param[in] ptr_to_array 指向数组的指针
   * @return 返回数据指针
   */
  const void *get_data(const void *ptr_to_array) const { return ptr_to_array; }

  /**
   * @brief 获取枚举值类型
   * @return 返回ArrayValueType类型
   */
  EValueType e_value_type() const final { return EValueType::ArrayValueType; }
};

/**
 * @class SpanSequenceValueType
 * @brief 跨度序列值类型类，继承自AnyValueType
 */
class SpanSequenceValueType : public AnyValueType {
 public:
  using AnyValueType::sizeof_type;

  /**
   * @brief 获取元素值类型（纯虚函数）
   * @return 返回元素值类型指针
   */
  virtual const AnyValueType *element_value_type() const = 0;

  /**
   * @brief 获取序列大小（纯虚函数）
   * @param[in] ptr_to_sequence 指向序列的指针
   * @return 返回序列大小
   */
  virtual size_t sequence_size(const void *ptr_to_sequence) const = 0;

  /**
   * @brief 获取序列内容（纯虚函数）
   * @param[in] ptr_to_sequence 指向序列的指针
   * @return 返回序列内容指针
   */
  virtual const void *sequence_contents(const void *ptr_to_sequence) const = 0;

  /**
   * @brief 获取枚举值类型
   * @return 返回SpanSequenceValueType类型
   */
  EValueType e_value_type() const final { return EValueType::SpanSequenceValueType; }
};

/**
 * @class CallbackSpanSequenceValueType
 * @brief 回调跨度序列值类型类，继承自 SpanSequenceValueType 类。
 */
class CallbackSpanSequenceValueType : public SpanSequenceValueType {
 protected:
  const AnyValueType *m_element_value_type;             ///< 元素值类型指针
  std::function<size_t(const void *)> m_size_function;  ///< 获取序列大小的函数
  std::function<const void *(const void *, size_t index)>
      m_get_const_function;                             ///< 获取序列中常量元素的函数

 public:
  /**
   * @brief 构造函数
   * @param element_value_type 元素值类型指针
   * @param size_function 获取序列大小的函数
   * @param get_const_function 获取序列中常量元素的函数
   */
  CallbackSpanSequenceValueType(const AnyValueType *element_value_type,
                                decltype(m_size_function) size_function,
                                decltype(m_get_const_function) get_const_function)
      : m_element_value_type(element_value_type),
        m_size_function(size_function),
        m_get_const_function(get_const_function) {
    assert(m_element_value_type);
    assert(size_function);
    assert(get_const_function);
  }

  size_t sizeof_type() const override { throw std::logic_error("not implemented"); }
  const AnyValueType *element_value_type() const override { return m_element_value_type; }

  /**
   * @brief 获取序列大小
   * @param ptr_to_sequence 序列指针
   * @return 序列大小
   */
  size_t sequence_size(const void *ptr_to_sequence) const override {
    return m_size_function(ptr_to_sequence);
  }

  /**
   * @brief 获取序列内容
   * @param ptr_to_sequence 序列指针
   * @return 序列内容的常量指针
   */
  const void *sequence_contents(const void *ptr_to_sequence) const override {
    if (sequence_size(ptr_to_sequence) == 0) {
      return nullptr;
    }
    return m_get_const_function(ptr_to_sequence, 0);
  }
};

/**
 * @class ROSIDLC_SpanSequenceValueType
 * @brief ROS IDL C 跨度序列值类型类，继承自 SpanSequenceValueType 类。
 */
class ROSIDLC_SpanSequenceValueType : public SpanSequenceValueType {
 protected:
  const AnyValueType *m_element_value_type;  ///< 元素值类型指针

  /**
   * @struct ROSIDLC_SequenceObject
   * @brief ROS IDL C 序列对象结构体
   */
  struct ROSIDLC_SequenceObject {
    void *data;       ///< 数据指针
    size_t size;      ///< 数据中有效元素的数量
    size_t capacity;  ///< 数据中分配的元素数量
  };

  /**
   * @brief 获取序列值
   * @param ptr_to_sequence 序列指针
   * @return ROSIDLC_SequenceObject 结构体常量指针
   */
  const ROSIDLC_SequenceObject *get_value(const void *ptr_to_sequence) const {
    return static_cast<const ROSIDLC_SequenceObject *>(ptr_to_sequence);
  }

 public:
  /**
   * @brief 构造函数
   * @param element_value_type 元素值类型指针
   */
  explicit ROSIDLC_SpanSequenceValueType(const AnyValueType *element_value_type)
      : m_element_value_type(element_value_type) {}

  size_t sizeof_type() const override { return sizeof(ROSIDLC_SequenceObject); }
  const AnyValueType *element_value_type() const override { return m_element_value_type; }

  /**
   * @brief 获取序列大小
   * @param ptr_to_sequence 序列指针
   * @return 序列大小
   */
  size_t sequence_size(const void *ptr_to_sequence) const override {
    return get_value(ptr_to_sequence)->size;
  }

  /**
   * @brief 获取序列内容
   * @param ptr_to_sequence 序列指针
   * @return 序列内容的常量指针
   */
  const void *sequence_contents(const void *ptr_to_sequence) const final {
    return get_value(ptr_to_sequence)->data;
  }
};

/**
 * @brief 基本值类型结构体，继承自AnyValueType
 */
struct PrimitiveValueType : public AnyValueType {
  /// 类型种类常量
  const ROSIDL_TypeKind m_type_kind;

  /**
   * @brief 显式构造函数
   * @param type_kind 类型种类
   */
  explicit constexpr PrimitiveValueType(ROSIDL_TypeKind type_kind) : m_type_kind(type_kind) {
    // 断言：类型不是字符串
    assert(type_kind != ROSIDL_TypeKind::STRING);
    // 断言：类型不是宽字符串
    assert(type_kind != ROSIDL_TypeKind::WSTRING);
    // 断言：类型不是消息
    assert(type_kind != ROSIDL_TypeKind::MESSAGE);
  }

  /**
   * @brief 获取类型种类
   * @return 类型种类
   */
  ROSIDL_TypeKind type_kind() const { return m_type_kind; }

  /**
   * @brief 获取类型大小
   * @return 类型大小
   */
  size_t sizeof_type() const final {
    switch (m_type_kind) {
      case ROSIDL_TypeKind::FLOAT:
        return sizeof(float);
      case ROSIDL_TypeKind::DOUBLE:
        return sizeof(double);
      case ROSIDL_TypeKind::LONG_DOUBLE:
        return sizeof(long double);
      case ROSIDL_TypeKind::CHAR:
        return sizeof(char);
      case ROSIDL_TypeKind::WCHAR:
        return sizeof(char16_t);
      case ROSIDL_TypeKind::BOOLEAN:
        return sizeof(bool);
      case ROSIDL_TypeKind::OCTET:
        return sizeof(unsigned char);
      case ROSIDL_TypeKind::UINT8:
        return sizeof(uint_least8_t);
      case ROSIDL_TypeKind::INT8:
        return sizeof(int_least8_t);
      case ROSIDL_TypeKind::UINT16:
        return sizeof(uint_least16_t);
      case ROSIDL_TypeKind::INT16:
        return sizeof(int_least16_t);
      case ROSIDL_TypeKind::UINT32:
        return sizeof(uint_least32_t);
      case ROSIDL_TypeKind::INT32:
        return sizeof(int_least32_t);
      case ROSIDL_TypeKind::UINT64:
        return sizeof(uint_least64_t);
      case ROSIDL_TypeKind::INT64:
        return sizeof(int_least64_t);
      case ROSIDL_TypeKind::STRING:
      case ROSIDL_TypeKind::WSTRING:
      case ROSIDL_TypeKind::MESSAGE:
      default:
        unreachable();
    }
  }

  /**
   * @brief 获取枚举值类型
   * @return 枚举值类型
   */
  EValueType e_value_type() const override { return EValueType::PrimitiveValueType; }
};

/**
 * @class BoolVectorValueType
 * @brief 布尔向量值类型类，继承自 AnyValueType 类
 */
class BoolVectorValueType : public AnyValueType {
 protected:
  /**
   * @brief 获取指向序列的指针
   * @param ptr_to_sequence 指向序列的 void 指针
   * @return 返回一个指向 std::vector<bool> 的常量指针
   */
  const std::vector<bool> *get_value(const void *ptr_to_sequence) const {
    return static_cast<const std::vector<bool> *>(ptr_to_sequence);
  }

  // 静态成员变量，表示原始值类型的唯一指针
  static std::unique_ptr<PrimitiveValueType> s_element_value_type;

 public:
  /**
   * @brief 获取类型大小
   * @return 返回类型的大小，即 sizeof(std::vector<bool>)
   */
  size_t sizeof_type() const override { return sizeof(std::vector<bool>); }

  /**
   * @brief 获取元素值类型
   * @return 返回一个指向 AnyValueType 的常量指针
   */
  static const AnyValueType *element_value_type() {
    if (!s_element_value_type) {
      s_element_value_type = std::make_unique<PrimitiveValueType>(ROSIDL_TypeKind::BOOLEAN);
    }
    return s_element_value_type.get();
  }

  /**
   * @brief 获取序列的起始迭代器
   * @param ptr_to_sequence 指向序列的 void 指针
   * @return 返回一个 std::vector<bool> 的常量迭代器
   */
  std::vector<bool>::const_iterator begin(const void *ptr_to_sequence) const {
    return get_value(ptr_to_sequence)->begin();
  }

  /**
   * @brief 获取序列的结束迭代器
   * @param ptr_to_sequence 指向序列的 void 指针
   * @return 返回一个 std::vector<bool> 的常量迭代器
   */
  std::vector<bool>::const_iterator end(const void *ptr_to_sequence) const {
    return get_value(ptr_to_sequence)->end();
  }

  /**
   * @brief 获取序列的大小
   * @param ptr_to_sequence 指向序列的 void 指针
   * @return 返回序列的大小
   */
  size_t size(const void *ptr_to_sequence) const { return get_value(ptr_to_sequence)->size(); }

  /**
   * @brief 获取值类型枚举
   * @return 返回 EValueType 枚举值：BoolVectorValueType
   */
  EValueType e_value_type() const final { return EValueType::BoolVectorValueType; }
};

/**
 * @class ROSIDLC_StructValueType
 * @brief ROSIDLC 结构值类型类
 */
class ROSIDLC_StructValueType;

/**
 * @class U8StringValueType
 * @brief U8 字符串值类型类，继承自 AnyValueType 类
 */
class U8StringValueType : public AnyValueType {
 public:
  // 定义字符特性类型别名
  using char_traits = std::char_traits<char>;

  /**
   * @brief 获取非常量数据
   * @param[in] 指向 void 类型的指针
   * @return 返回一个 TypedSpan<char_traits::char_type> 类型的对象
   */
  virtual TypedSpan<char_traits::char_type> data(void *) const = 0;

  /**
   * @brief 获取常量数据
   * @param[in] 指向 const void 类型的指针
   * @return 返回一个 TypedSpan<const char_traits::char_type> 类型的对象
   */
  virtual TypedSpan<const char_traits::char_type> data(const void *) const = 0;

  /**
   * @brief 获取值类型枚举
   * @return 返回 EValueType 枚举值：U8StringValueType
   */
  EValueType e_value_type() const final { return EValueType::U8StringValueType; }
};

/**
 * @class U16StringValueType
 * @brief 一个表示16位字符串值类型的基类，继承自AnyValueType
 */
class U16StringValueType : public AnyValueType {
 public:
  using char_traits = std::char_traits<char16_t>;

  /**
   * @brief 获取非const指针指向的数据
   * @param[in] ptr 非const指针
   * @return 返回一个TypedSpan对象，包含字符类型和大小
   */
  virtual TypedSpan<char_traits::char_type> data(void *) const = 0;

  /**
   * @brief 获取const指针指向的数据
   * @param[in] ptr const指针
   * @return 返回一个TypedSpan对象，包含字符类型和大小
   */
  virtual TypedSpan<const char_traits::char_type> data(const void *) const = 0;

  /**
   * @brief 获取值类型枚举
   * @return 返回EValueType::U16StringValueType
   */
  EValueType e_value_type() const final { return EValueType::U16StringValueType; }
};

/**
 * @struct ROSIDLC_StringValueType
 * @brief 一个表示ROS IDL C字符串值类型的结构体，继承自U8StringValueType
 */
struct ROSIDLC_StringValueType : public U8StringValueType {
 public:
  using type = rosidl_runtime_c__String;

  /**
   * @brief 获取const指针指向的数据
   * @param[in] ptr const指针
   * @return 返回一个TypedSpan对象，包含字符类型和大小
   */
  TypedSpan<const char_traits::char_type> data(const void *ptr) const override {
    auto str = static_cast<const type *>(ptr);
    assert(str->capacity == str->size + 1);
    assert(str->data[str->size] == '\0');
    return {str->data, str->size};
  }

  /**
   * @brief 获取非const指针指向的数据
   * @param[in] ptr 非const指针
   * @return 返回一个TypedSpan对象，包含字符类型和大小
   */
  TypedSpan<char_traits::char_type> data(void *ptr) const override {
    auto str = static_cast<type *>(ptr);
    assert(str->capacity == str->size + 1);
    assert(str->data[str->size + 1] == 0);
    return {str->data, str->size};
  }

  /**
   * @brief 获取类型的大小
   * @return 返回类型的大小
   */
  size_t sizeof_type() const override { return sizeof(type); }
};

/**
 * @class ROSIDLC_WStringValueType
 * @brief 一个表示ROS IDL C宽字符串值类型的类，继承自U16StringValueType
 */
class ROSIDLC_WStringValueType : public U16StringValueType {
 public:
  using type = rosidl_runtime_c__U16String;

  /**
   * @brief 获取const指针指向的数据
   * @param[in] ptr const指针
   * @return 返回一个TypedSpan对象，包含字符类型和大小
   */
  TypedSpan<const char_traits::char_type> data(const void *ptr) const override {
    auto str = static_cast<const type *>(ptr);
    return {reinterpret_cast<const char_traits::char_type *>(str->data), str->size};
  }

  /**
   * @brief 获取非const指针指向的数据
   * @param[in] ptr 非const指针
   * @return 返回一个TypedSpan对象，包含字符类型和大小
   */
  TypedSpan<char_traits::char_type> data(void *ptr) const override {
    auto str = static_cast<type *>(ptr);
    return {reinterpret_cast<char_traits::char_type *>(str->data), str->size};
  }

  /**
   * @brief 获取类型的大小
   * @return 返回类型的大小
   */
  size_t sizeof_type() const override { return sizeof(type); }
};

/**
 * @class ROSIDLCPP_StringValueType
 * @brief 继承自 U8StringValueType 的类，用于处理 std::string 类型的值
 */
class ROSIDLCPP_StringValueType : public U8StringValueType {
 public:
  /// 使用 std::string 作为类型别名
  using type = std::string;

  /**
   * @brief 获取只读数据的方法
   * @param ptr 指向数据的指针
   * @return 返回一个包含数据和大小的 TypedSpan 对象
   */
  TypedSpan<const char_traits::char_type> data(const void *ptr) const override {
    auto str = static_cast<const type *>(ptr);
    return {str->data(), str->size()};
  }

  /**
   * @brief 获取可修改数据的方法
   * @param ptr 指向数据的指针
   * @return 返回一个包含数据和大小的 TypedSpan 对象
   */
  TypedSpan<char_traits::char_type> data(void *ptr) const override {
    auto str = static_cast<type *>(ptr);
    return {str->data(), str->size()};
  }

  /**
   * @brief 获取类型的大小
   * @return 返回类型的大小
   */
  size_t sizeof_type() const override { return sizeof(type); }
};

/**
 * @class ROSIDLCPP_U16StringValueType
 * @brief 继承自 U16StringValueType 的类，用于处理 std::u16string 类型的值
 */
class ROSIDLCPP_U16StringValueType : public U16StringValueType {
 public:
  /// 使用 std::u16string 作为类型别名
  using type = std::u16string;

  /**
   * @brief 获取只读数据的方法
   * @param ptr 指向数据的指针
   * @return 返回一个包含数据和大小的 TypedSpan 对象
   */
  TypedSpan<const char_traits::char_type> data(const void *ptr) const override {
    auto str = static_cast<const type *>(ptr);
    return {str->data(), str->size()};
  }

  /**
   * @brief 获取可修改数据的方法
   * @param ptr 指向数据的指针
   * @return 返回一个包含数据和大小的 TypedSpan 对象
   */
  TypedSpan<char_traits::char_type> data(void *ptr) const override {
    auto str = static_cast<type *>(ptr);
    return {str->data(), str->size()};
  }

  /**
   * @brief 获取类型的大小
   * @return 返回类型的大小
   */
  size_t sizeof_type() const override { return sizeof(type); }
};

/**
 * @brief 对任意值类型应用一元函数 (Apply a unary function to any value type)
 *
 * @tparam UnaryFunction 一元函数类型 (Unary function type)
 * @param f 要应用的一元函数 (The unary function to apply)
 * @return 返回应用一元函数后的结果 (Returns the result after applying the unary function)
 */
template <typename UnaryFunction>
auto AnyValueType::apply(UnaryFunction f) const {
  // 根据值类型进行分支处理 (Switch based on value type)
  switch (e_value_type()) {
    case EValueType::PrimitiveValueType:
      // 应用一元函数到原始值类型 (Apply unary function to primitive value type)
      return f(*static_cast<const PrimitiveValueType *>(this));
    case EValueType::U8StringValueType:
      // 应用一元函数到U8字符串值类型 (Apply unary function to U8 string value type)
      return f(*static_cast<const U8StringValueType *>(this));
    case EValueType::U16StringValueType:
      // 应用一元函数到U16字符串值类型 (Apply unary function to U16 string value type)
      return f(*static_cast<const U16StringValueType *>(this));
    case EValueType::StructValueType:
      // 应用一元函数到结构体值类型 (Apply unary function to struct value type)
      return f(*static_cast<const StructValueType *>(this));
    case EValueType::ArrayValueType:
      // 应用一元函数到数组值类型 (Apply unary function to array value type)
      return f(*static_cast<const ArrayValueType *>(this));
    case EValueType::SpanSequenceValueType:
      // 应用一元函数到跨度序列值类型 (Apply unary function to span sequence value type)
      return f(*static_cast<const SpanSequenceValueType *>(this));
    case EValueType::BoolVectorValueType:
      // 应用一元函数到布尔向量值类型 (Apply unary function to bool vector value type)
      return f(*static_cast<const BoolVectorValueType *>(this));
    default:
      // 不可达代码 (Unreachable code)
      unreachable();
  }
}

/**
 * @brief 对任意值类型应用一元函数 (Apply a unary function to any value type)
 *
 * @tparam UnaryFunction 一元函数类型 (Unary function type)
 * @param f 要应用的一元函数 (The unary function to apply)
 * @return 返回应用一元函数后的结果 (Returns the result after applying the unary function)
 */
template <typename UnaryFunction>
auto AnyValueType::apply(UnaryFunction f) {
  // 根据值类型进行分支处理 (Switch based on value type)
  switch (e_value_type()) {
    case EValueType::PrimitiveValueType:
      // 应用一元函数到原始值类型 (Apply unary function to primitive value type)
      return f(*static_cast<PrimitiveValueType *>(this));
    case EValueType::U8StringValueType:
      // 应用一元函数到U8字符串值类型 (Apply unary function to U8 string value type)
      return f(*static_cast<U8StringValueType *>(this));
    case EValueType::U16StringValueType:
      // 应用一元函数到U16字符串值类型 (Apply unary function to U16 string value type)
      return f(*static_cast<U16StringValueType *>(this));
    case EValueType::StructValueType:
      // 应用一元函数到结构体值类型 (Apply unary function to struct value type)
      return f(*static_cast<StructValueType *>(this));
    case EValueType::ArrayValueType:
      // 应用一元函数到数组值类型 (Apply unary function to array value type)
      return f(*static_cast<ArrayValueType *>(this));
    case EValueType::SpanSequenceValueType:
      // 应用一元函数到跨度序列值类型 (Apply unary function to span sequence value type)
      return f(*static_cast<SpanSequenceValueType *>(this));
    case EValueType::BoolVectorValueType:
      // 应用一元函数到布尔向量值类型 (Apply unary function to bool vector value type)
      return f(*static_cast<BoolVectorValueType *>(this));
    default:
      // 不可达代码 (Unreachable code)
      unreachable();
  }
}

}  // namespace rmw_cyclonedds_cpp
#endif  // TYPESUPPORT2_HPP_
