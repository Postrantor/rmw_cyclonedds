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

#ifndef SERVICETYPESUPPORT_IMPL_HPP_
#define SERVICETYPESUPPORT_IMPL_HPP_

#include <cassert>
#include <regex>
#include <sstream>
#include <string>

#include "ServiceTypeSupport.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rmw_cyclonedds_cpp {

// ServiceTypeSupport构造函数模板
template <typename MembersType>
ServiceTypeSupport<MembersType>::ServiceTypeSupport() {}

// RequestTypeSupport构造函数模板
// @tparam ServiceMembersType 服务成员类型
// @tparam MessageMembersType 消息成员类型
// @param[in] members 服务成员指针
template <typename ServiceMembersType, typename MessageMembersType>
RequestTypeSupport<ServiceMembersType, MessageMembersType>::RequestTypeSupport(
    const ServiceMembersType* members) {
  // 断言members非空
  assert(members);
  // 设置请求成员
  this->members_ = members->request_members_;

  // 创建字符串流
  std::ostringstream ss;
  // 获取服务命名空间和服务名称
  std::string service_namespace(members->service_namespace_);
  std::string service_name(members->service_name_);
  if (!service_namespace.empty()) {
    // 如果使用C类型支持，将C命名空间分隔符替换为C++
    service_namespace = std::regex_replace(service_namespace, std::regex("__"), "::");
    ss << service_namespace << "::";
  }
  // 构建并设置类型名称
  ss << "dds_::" << service_name << "_Request_";
  this->setName(ss.str().c_str());
}

// ResponseTypeSupport构造函数模板
// @tparam ServiceMembersType 服务成员类型
// @tparam MessageMembersType 消息成员类型
// @param[in] members 服务成员指针
template <typename ServiceMembersType, typename MessageMembersType>
ResponseTypeSupport<ServiceMembersType, MessageMembersType>::ResponseTypeSupport(
    const ServiceMembersType* members) {
  // 断言members非空
  assert(members);
  // 设置响应成员
  this->members_ = members->response_members_;

  // 创建字符串流
  std::ostringstream ss;
  // 获取服务命名空间和服务名称
  std::string service_namespace(members->service_namespace_);
  std::string service_name(members->service_name_);
  if (!service_namespace.empty()) {
    // 如果使用C类型支持，将C命名空间分隔符替换为C++
    service_namespace = std::regex_replace(service_namespace, std::regex("__"), "::");
    ss << service_namespace << "::";
  }
  // 构建并设置类型名称
  ss << "dds_::" << service_name << "_Response_";
  this->setName(ss.str().c_str());
}

}  // namespace rmw_cyclonedds_cpp

#endif  // SERVICETYPESUPPORT_IMPL_HPP_
