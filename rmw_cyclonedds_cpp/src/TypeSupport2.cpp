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
#include "TypeSupport2.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/error_handling.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

namespace rmw_cyclonedds_cpp {
/**
 * @class ROSIDLC_StructValueType
 * @brief 继承自 StructValueType 的类，用于处理 ROSIDL C 结构体类型的值
 */
class ROSIDLC_StructValueType : public StructValueType {
  // 实现指针，指向 rosidl_typesupport_introspection_c__MessageMembers 结构体
  const rosidl_typesupport_introspection_c__MessageMembers *impl;
  // 存储结构体成员的向量
  std::vector<Member> m_members;
  // 存储内部值类型的向量
  std::vector<std::unique_ptr<const AnyValueType>> m_inner_value_types;

  /**
   * @brief 创建一个新的值类型对象
   * @tparam ConstructedType 要构造的类型
   * @tparam Args 构造函数参数类型
   * @param args 构造函数参数
   * @return 返回创建的值类型对象的指针
   */
  template <typename ConstructedType, typename... Args>
  ConstructedType *make_value_type(Args &&...args) {
    auto unique_ptr = std::make_unique<ConstructedType>(std::forward<Args>(args)...);
    auto ptr = unique_ptr.get();
    m_inner_value_types.push_back(std::move(unique_ptr));
    return ptr;
  }

 public:
  // 类型生成器常量，表示使用 ROSIDL C 生成器
  static constexpr TypeGenerator gen = TypeGenerator::ROSIDL_C;
  // 构造函数
  explicit ROSIDLC_StructValueType(const rosidl_typesupport_introspection_c__MessageMembers *impl);
  // 获取结构体大小
  size_t sizeof_struct() const override { return impl->size_of_; }
  // 获取成员数量
  size_t n_members() const override { return impl->member_count_; }
  // 获取指定索引的成员
  const Member *get_member(size_t index) const override { return &m_members.at(index); }
};

/**
 * @class ROSIDLCPP_StructValueType
 * @brief 继承自 StructValueType 的类，用于处理 ROSIDL C++ 结构体类型的值
 */
class ROSIDLCPP_StructValueType : public StructValueType {
  // 实现指针，指向 rosidl_typesupport_introspection_cpp::MessageMembers 结构体
  const rosidl_typesupport_introspection_cpp::MessageMembers *impl;
  // 存储结构体成员的向量
  std::vector<Member> m_members;
  // 存储内部值类型的向量
  std::vector<std::unique_ptr<const AnyValueType>> m_inner_value_types;

  /**
   * @brief 创建一个新的值类型对象
   * @tparam ConstructedType 要构造的类型
   * @tparam Args 构造函数参数类型
   * @param args 构造函数参数
   * @return 返回创建的值类型对象的指针
   */
  template <typename ConstructedType, typename... Args>
  ConstructedType *make_value_type(Args &&...args) {
    auto unique_ptr = std::make_unique<ConstructedType>(std::forward<Args>(args)...);
    auto ptr = unique_ptr.get();
    m_inner_value_types.push_back(std::move(unique_ptr));
    return ptr;
  }

 public:
  // 类型生成器常量，表示使用 ROSIDL C++ 生成器
  static constexpr TypeGenerator gen = TypeGenerator::ROSIDL_Cpp;
  // 构造函数
  explicit ROSIDLCPP_StructValueType(
      const rosidl_typesupport_introspection_cpp::MessageMembers *impl);
  // 获取结构体大小
  size_t sizeof_struct() const override { return impl->size_of_; }
  // 获取成员数量
  size_t n_members() const override { return impl->member_count_; }
  // 获取指定索引的成员
  const Member *get_member(size_t index) const final { return &m_members.at(index); }
};

/**
 * @brief 创建消息值类型的唯一指针
 *
 * @param mts 指向rosidl_message_type_support_t结构体的指针
 * @return std::unique_ptr<StructValueType> 返回一个指向StructValueType的唯一指针
 */
std::unique_ptr<StructValueType> make_message_value_type(const rosidl_message_type_support_t *mts) {
  // 尝试获取ROSIDL_C类型的消息类型支持句柄
  if (auto ts_c = get_message_typesupport_handle(
          mts, TypeGeneratorInfo<TypeGenerator::ROSIDL_C>::get_identifier())) {
    // 获取元消息数据并转换为MetaMessage指针
    auto members = static_cast<const MetaMessage<TypeGenerator::ROSIDL_C> *>(ts_c->data);
    // 创建一个ROSIDLC_StructValueType的唯一指针并返回
    return std::make_unique<ROSIDLC_StructValueType>(members);
  } else {
    // 保存先前的错误信息并重置错误
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();

    // 尝试获取ROSIDL_Cpp类型的消息类型支持句柄
    if (auto ts_cpp = get_message_typesupport_handle(
            mts, TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp>::get_identifier())) {
      // 获取元消息数据并转换为MetaMessage指针
      auto members = static_cast<const MetaMessage<TypeGenerator::ROSIDL_Cpp> *>(ts_cpp->data);
      // 创建一个ROSIDLCPP_StructValueType的唯一指针并返回
      return std::make_unique<ROSIDLCPP_StructValueType>(members);
    } else {
      // 保存错误信息并重置错误
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();

      // 抛出运行时异常，表示类型支持不是来自此实现
      throw std::runtime_error(std::string("Type support not from this implementation.  Got:\n") +
                               "    " + prev_error_string.str + "\n" + "    " + error_string.str +
                               "\n" + "while fetching it");
    }
  }
}

/**
 * @brief 创建请求和响应值类型的唯一指针对
 *
 * @param svc_ts 指向rosidl_service_type_support_t结构体的指针
 * @return std::pair<std::unique_ptr<StructValueType>, std::unique_ptr<StructValueType>>
 * 返回一个包含请求和响应值类型唯一指针的pair对象
 */
std::pair<std::unique_ptr<StructValueType>, std::unique_ptr<StructValueType>>
make_request_response_value_types(const rosidl_service_type_support_t *svc_ts) {
  // 尝试获取ROSIDL_C类型的服务类型支持句柄
  if (auto tsc = get_service_typesupport_handle(
          svc_ts, TypeGeneratorInfo<TypeGenerator::ROSIDL_C>::get_identifier())) {
    // 获取类型生成器信息并转换为MetaService指针
    auto typed =
        static_cast<const TypeGeneratorInfo<TypeGenerator::ROSIDL_C>::MetaService *>(tsc->data);
    // 创建一个包含请求和响应值类型唯一指针的pair对象并返回
    return {std::make_unique<ROSIDLC_StructValueType>(typed->request_members_),
            std::make_unique<ROSIDLC_StructValueType>(typed->response_members_)};
  } else {
    // 保存先前的错误信息并重置错误
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();

    // 尝试获取ROSIDL_Cpp类型的服务类型支持句柄
    if (auto tscpp = get_service_typesupport_handle(
            svc_ts, TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp>::get_identifier())) {
      // 获取类型生成器信息并转换为MetaService指针
      auto typed = static_cast<const TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp>::MetaService *>(
          tscpp->data);
      // 创建一个包含请求和响应值类型唯一指针的pair对象并返回
      return {std::make_unique<ROSIDLCPP_StructValueType>(typed->request_members_),
              std::make_unique<ROSIDLCPP_StructValueType>(typed->response_members_)};
    } else {
      // 保存错误信息并重置错误
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();

      // 抛出运行时异常，表示服务类型支持不是来自此实现
      throw std::runtime_error(
          std::string("Service type support not from this implementation.  Got:\n") + "    " +
          prev_error_string.str + "\n" + "    " + error_string.str + "\n" + "while fetching it");
    }
  }
}

/**
 * @brief 构造函数，用于初始化 ROSIDLC_StructValueType 对象
 *
 * @param impl 指向 rosidl_typesupport_introspection_c__MessageMembers 结构体的指针
 */
ROSIDLC_StructValueType::ROSIDLC_StructValueType(
    const rosidl_typesupport_introspection_c__MessageMembers *impl)
    : impl{impl}, m_members{}, m_inner_value_types{} {
  // 遍历 impl 中的成员数量
  for (size_t index = 0; index < impl->member_count_; index++) {
    // 获取当前索引下的成员实现
    auto member_impl = impl->members_[index];

    // 定义一个指向 AnyValueType 的指针
    const AnyValueType *element_value_type;
    // 根据成员类型 ID 判断成员类型，并进行相应处理
    switch (ROSIDL_TypeKind(member_impl.type_id_)) {
      case ROSIDL_TypeKind::MESSAGE:
        m_inner_value_types.push_back(make_message_value_type(member_impl.members_));
        element_value_type = m_inner_value_types.back().get();
        break;
      case ROSIDL_TypeKind::STRING:
        element_value_type = make_value_type<ROSIDLC_StringValueType>();
        break;
      case ROSIDL_TypeKind::WSTRING:
        element_value_type = make_value_type<ROSIDLC_WStringValueType>();
        break;
      default:
        element_value_type =
            make_value_type<PrimitiveValueType>(ROSIDL_TypeKind(member_impl.type_id_));
        break;
    }

    // 定义一个指向 AnyValueType 的指针，用于存储成员值类型
    const AnyValueType *member_value_type;
    // 判断成员是否为数组
    if (!member_impl.is_array_) {
      member_value_type = element_value_type;
    } else if (member_impl.array_size_ != 0 && !member_impl.is_upper_bound_) {
      member_value_type =
          make_value_type<ArrayValueType>(element_value_type, member_impl.array_size_);
    } else if (member_impl.size_function) {
      member_value_type = make_value_type<CallbackSpanSequenceValueType>(
          element_value_type, member_impl.size_function, member_impl.get_const_function);
    } else {
      member_value_type = make_value_type<ROSIDLC_SpanSequenceValueType>(element_value_type);
    }
    // 将成员添加到 m_members 中
    m_members.push_back(Member{
        member_impl.name_,
        member_value_type,
        member_impl.offset_,
    });
  }
}

/**
 * @brief 构造函数，用于初始化 ROSIDLCPP_StructValueType 对象
 *
 * @param impl 指向 rosidl_typesupport_introspection_cpp::MessageMembers 的指针
 */
ROSIDLCPP_StructValueType::ROSIDLCPP_StructValueType(
    const rosidl_typesupport_introspection_cpp::MessageMembers *impl)
    : impl(impl)  // 初始化成员变量 impl
{
  // 遍历 impl 中的所有成员
  for (size_t index = 0; index < impl->member_count_; index++) {
    auto member_impl = impl->members_[index];  // 获取当前成员的实现

    const AnyValueType *element_value_type;
    // 根据成员类型创建相应的值类型对象
    switch (ROSIDL_TypeKind(member_impl.type_id_)) {
      case ROSIDL_TypeKind::MESSAGE:
        m_inner_value_types.push_back(make_message_value_type(member_impl.members_));
        element_value_type = m_inner_value_types.back().get();
        break;
      case ROSIDL_TypeKind::STRING:
        element_value_type = make_value_type<ROSIDLCPP_StringValueType>();
        break;
      case ROSIDL_TypeKind::WSTRING:
        element_value_type = make_value_type<ROSIDLCPP_U16StringValueType>();
        break;
      default:
        element_value_type =
            make_value_type<PrimitiveValueType>(ROSIDL_TypeKind(member_impl.type_id_));
        break;
    }

    const AnyValueType *member_value_type;
    // 判断成员是否为数组，并根据情况创建相应的值类型对象
    if (!member_impl.is_array_) {
      member_value_type = element_value_type;
    } else if (member_impl.array_size_ != 0 && !member_impl.is_upper_bound_) {
      member_value_type =
          make_value_type<ArrayValueType>(element_value_type, member_impl.array_size_);
    } else if (ROSIDL_TypeKind(member_impl.type_id_) == ROSIDL_TypeKind::BOOLEAN) {
      member_value_type = make_value_type<BoolVectorValueType>();
    } else {
      member_value_type = make_value_type<CallbackSpanSequenceValueType>(
          element_value_type, member_impl.size_function, member_impl.get_const_function);
    }
    // 将构造好的成员添加到 m_members 中
    m_members.push_back(Member{
        member_impl.name_,
        member_value_type,
        member_impl.offset_,
    });
  }
}
}  // namespace rmw_cyclonedds_cpp
