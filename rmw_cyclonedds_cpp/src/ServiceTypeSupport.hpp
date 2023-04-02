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

#ifndef SERVICETYPESUPPORT_HPP_
#define SERVICETYPESUPPORT_HPP_

#include <cassert>

#include "TypeSupport.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

struct CustomServiceInfo;

//! \file service_type_support.hpp
//!
//! \brief rmw_cyclonedds_cpp 命名空间中的 ServiceTypeSupport 类模板及其派生类。

namespace rmw_cyclonedds_cpp {

//! \class ServiceTypeSupport
//! \tparam MembersType 成员类型
//!
//! \brief 为服务类型提供支持的基类。
template <typename MembersType>
class ServiceTypeSupport : public TypeSupport<MembersType> {
 protected:
  //! \brief 构造函数
  ServiceTypeSupport();
};

//! \class RequestTypeSupport
//! \tparam ServiceMembersType 服务成员类型
//! \tparam MessageMembersType 消息成员类型
//!
//! \brief 为请求类型提供支持的派生类。
template <typename ServiceMembersType, typename MessageMembersType>
class RequestTypeSupport : public ServiceTypeSupport<MessageMembersType> {
 public:
  //! \brief 显式构造函数
  //!
  //! \param[in] members 服务成员类型指针
  explicit RequestTypeSupport(const ServiceMembersType* members);
};

//! \class ResponseTypeSupport
//! \tparam ServiceMembersType 服务成员类型
//! \tparam MessageMembersType 消息成员类型
//!
//! \brief 为响应类型提供支持的派生类。
template <typename ServiceMembersType, typename MessageMembersType>
class ResponseTypeSupport : public ServiceTypeSupport<MessageMembersType> {
 public:
  //! \brief 显式构造函数
  //!
  //! \param[in] members 服务成员类型指针
  explicit ResponseTypeSupport(const ServiceMembersType* members);
};

}  // namespace rmw_cyclonedds_cpp

#include "ServiceTypeSupport_impl.hpp"

#endif  // SERVICETYPESUPPORT_HPP_
