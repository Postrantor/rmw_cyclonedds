// Copyright 2019 Open Source Robotics Foundation, Inc.
// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef DEMANGLE_HPP_
#define DEMANGLE_HPP_

#include <string>

/// 返回解析后的ROS主题，如果不是ROS主题，则返回原始主题。
std::string _demangle_if_ros_topic(const std::string &topic_name);

/// 返回解析后的ROS类型，如果不是ROS类型，则返回原始类型。
std::string _demangle_if_ros_type(const std::string &dds_type_string);

/**
 * @brief 如果给定主题是某个主题的一部分，则返回该主题名称，否则返回空字符串。
 * @param topic_name 主题名称
 * @return 解析后的ROS主题或空字符串
 */
std::string _demangle_ros_topic_from_topic(const std::string &topic_name);

/**
 * @brief 如果给定主题是服务的一部分，则返回该服务名称，否则返回空字符串。
 * @param topic_name 主题名称
 * @return 解析后的服务名称或空字符串
 */
std::string _demangle_service_from_topic(const std::string &topic_name);

/**
 * @brief 如果给定主题是服务请求的一部分，则返回该服务名称，否则返回空字符串。
 * @param topic_name 主题名称
 * @return 解析后的服务请求名称或空字符串
 */
std::string _demangle_service_request_from_topic(const std::string &topic_name);

/**
 * @brief 如果给定主题是服务回复的一部分，则返回该服务名称，否则返回空字符串。
 * @param topic_name 主题名称
 * @return 解析后的服务回复名称或空字符串
 */
std::string _demangle_service_reply_from_topic(const std::string &topic_name);

/**
 * @brief 如果是ROS srv类型，则返回解析后的服务类型，否则返回空字符串。
 * @param dds_type_name DDS类型名称
 * @return 解析后的服务类型或空字符串
 */
std::string _demangle_service_type_only(const std::string &dds_type_name);

/**
 * @brief 当ROS名称未被混淆时使用
 * @param name 名称
 * @return 原始名称
 */
std::string _identity_demangle(const std::string &name);

// 定义解析函数和混淆函数类型
using DemangleFunction = std::string (*)(const std::string &);
using MangleFunction = DemangleFunction;

#endif  // DEMANGLE_HPP_
