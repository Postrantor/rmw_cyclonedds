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

#include "demangle.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "namespace_prefix.hpp"
#include "rcpputils/find_and_replace.hpp"
#include "rcutils/logging_macros.h"
#include "rcutils/types.h"

/**
 * @brief 本文件包含了一些处理ROS前缀的函数定义
 */

// 使用C语言链接约定，以便在C和C++代码中共享这些常量
extern "C" {
// ROS主题前缀，用于区分不同类型的主题
const char* const ros_topic_prefix = "rt";
// ROS服务请求者前缀，用于区分不同类型的服务请求者
const char* const ros_service_requester_prefix = "rq";
// ROS服务响应前缀，用于区分不同类型的服务响应
const char* const ros_service_response_prefix = "rr";

// 定义一个std::vector<std::string>类型的常量，包含了上述三个ROS前缀
const std::vector<std::string> _ros_prefixes = {ROS_TOPIC_PREFIX, ROS_SERVICE_REQUESTER_PREFIX,
                                                ROS_SERVICE_RESPONSE_PREFIX};
}  // extern "C"

/**
 * @brief 去除名称中的前缀
 * @param name 待处理的字符串
 * @param prefix 要去除的前缀
 * @return 如果`name`以`prefix`开头，则返回去除前缀后的字符串；否则返回空字符串
 */
std::string _resolve_prefix(const std::string& name, const std::string& prefix) {
  // 检查`name`是否以`prefix`开头，且紧跟着'/'
  if (name.rfind(prefix, 0) == 0 && name.at(prefix.length()) == '/') {
    // 返回去除前缀后的字符串
    return name.substr(prefix.length());
  }
  // 如果不满足条件，返回空字符串
  return "";
}

/**
 * @brief 如果存在，从主题名称中去除ROS特定的前缀
 * @param topic_name 主题名称
 * @return 去除ROS前缀后的主题名称
 */
std::string _strip_ros_prefix_if_exists(const std::string& topic_name) {
  // 遍历所有可能的ROS前缀
  for (const auto& prefix : _ros_prefixes) {
    // 检查`topic_name`是否以当前`prefix`开头，且紧跟着'/'
    if (topic_name.rfind(prefix, 0) == 0 && topic_name.at(prefix.length()) == '/') {
      // 返回去除前缀后的字符串
      return topic_name.substr(prefix.length());
    }
  }
  // 如果没有匹配的前缀，返回原始主题名称
  return topic_name;
}

/// 返回解析后的ROS主题名称，如果不是ROS主题，则返回原始名称。
/**
 * \param topic_name 原始主题名称
 * \return 解析后的ROS主题名称或原始主题名称（如果不是ROS主题）
 */
std::string _demangle_if_ros_topic(const std::string& topic_name) {
  // 调用_strip_ros_prefix_if_exists函数处理主题名称
  return _strip_ros_prefix_if_exists(topic_name);
}

/// 返回解析后的ROS类型名称，如果不是ROS类型，则返回原始类型名称。
/**
 * \param dds_type_string 原始DDS类型字符串
 * \return 解析后的ROS类型名称或原始类型名称（如果不是ROS类型）
 */
std::string _demangle_if_ros_type(const std::string& dds_type_string) {
  // 检查最后一个字符是否为'_'
  if (dds_type_string[dds_type_string.size() - 1] != '_') {
    // 不是ROS类型，直接返回原始类型名称
    return dds_type_string;
  }

  // 定义子字符串"dds_::"
  std::string substring = "dds_::";
  // 查找子字符串在原始类型字符串中的位置
  size_t substring_position = dds_type_string.find(substring);
  // 如果没有找到子字符串，则说明不是ROS类型
  if (substring_position == std::string::npos) {
    // 不是ROS类型，直接返回原始类型名称
    return dds_type_string;
  }

  // 提取类型命名空间
  std::string type_namespace = dds_type_string.substr(0, substring_position);
  // 将类型命名空间中的"::"替换为"/"
  type_namespace = rcpputils::find_and_replace(type_namespace, "::", "/");
  // 计算类型名称的起始位置
  size_t start = substring_position + substring.size();
  // 提取类型名称
  std::string type_name = dds_type_string.substr(start, dds_type_string.length() - 1 - start);
  // 返回解析后的ROS类型名称
  return type_namespace + type_name;
}

/// 返回给定主题的主题名称（如果它是其中的一部分），否则返回 ""。
/// \param topic_name 一个表示主题名称的字符串引用
/// \return 如果主题名称是其中的一部分，则返回主题名称，否则返回空字符串
std::string _demangle_ros_topic_from_topic(const std::string& topic_name) {
  // 调用 _resolve_prefix 函数解析主题名称和 ros_topic_prefix
  return _resolve_prefix(topic_name, ros_topic_prefix);
}

/// 返回给定主题的服务名称（如果它是其中的一部分），否则返回 ""。
/// \param prefix 一个表示前缀的字符串引用
/// \param topic_name 一个表示主题名称的字符串引用
/// \param suffix 一个表示后缀的字符串引用
/// \return 如果主题名称是其中的一部分，则返回服务名称，否则返回空字符串
std::string _demangle_service_from_topic(const std::string& prefix,
                                         const std::string& topic_name,
                                         std::string suffix) {
  // 调用 _resolve_prefix 函数解析主题名称和前缀
  std::string service_name = _resolve_prefix(topic_name, prefix);

  // 如果解析后的服务名称为空字符串，则返回空字符串
  if ("" == service_name) {
    return "";
  }

  // 查找后缀在服务名称中的位置
  size_t suffix_position = service_name.rfind(suffix);

  // 如果找到了后缀
  if (suffix_position != std::string::npos) {
    // 检查后缀是否位于服务名称的末尾
    if (service_name.length() - suffix_position - suffix.length() != 0) {
      // 如果后缀不在末尾，记录警告日志并返回空字符串
      RCUTILS_LOG_WARN_NAMED("rmw_cyclonedds_cpp",
                             "service topic has service prefix and a suffix, but not at the end"
                             ", report this: '%s'",
                             topic_name.c_str());
      return "";
    }
  } else {
    // 如果没有找到后缀，记录警告日志并返回空字符串
    RCUTILS_LOG_WARN_NAMED("rmw_cyclonedds_cpp",
                           "service topic has prefix but no suffix"
                           ", report this: '%s'",
                           topic_name.c_str());
    return "";
  }

  // 返回去掉后缀的服务名称
  return service_name.substr(0, suffix_position);
}

/**
 * @brief 从主题名称中解析服务名称
 * @param topic_name 主题名称
 * @return 解析后的服务名称
 */
std::string _demangle_service_from_topic(const std::string& topic_name) {
  // 尝试从主题名称中解析服务回复名称
  const std::string demangled_topic = _demangle_service_reply_from_topic(topic_name);

  // 如果解析成功，返回解析后的服务名称
  if ("" != demangled_topic) {
    return demangled_topic;
  }

  // 否则尝试从主题名称中解析服务请求名称
  return _demangle_service_request_from_topic(topic_name);
}

/**
 * @brief 从主题名称中解析服务请求名称
 * @param topic_name 主题名称
 * @return 解析后的服务请求名称
 */
std::string _demangle_service_request_from_topic(const std::string& topic_name) {
  return _demangle_service_from_topic(ros_service_requester_prefix, topic_name, "Request");
}

/**
 * @brief 从主题名称中解析服务回复名称
 * @param topic_name 主题名称
 * @return 解析后的服务回复名称
 */
std::string _demangle_service_reply_from_topic(const std::string& topic_name) {
  return _demangle_service_from_topic(ros_service_response_prefix, topic_name, "Reply");
}

/**
 * @brief 如果是ROS srv类型，则返回解析后的服务类型，否则返回空字符串
 * @param dds_type_name DDS类型名称
 * @return 解析后的服务类型或空字符串
 */
std::string _demangle_service_type_only(const std::string& dds_type_name) {
  std::string ns_substring = "dds_::";
  size_t ns_substring_position = dds_type_name.find(ns_substring);

  // 不是ROS服务类型
  if (std::string::npos == ns_substring_position) {
    return "";
  }

  auto suffixes = {
      std::string("_Response_"),
      std::string("_Request_"),
  };

  std::string found_suffix = "";
  size_t suffix_position = 0;

  // 查找后缀位置
  for (auto suffix : suffixes) {
    suffix_position = dds_type_name.rfind(suffix);
    if (suffix_position != std::string::npos) {
      if (dds_type_name.length() - suffix_position - suffix.length() != 0) {
        RCUTILS_LOG_WARN_NAMED("rmw_cyclonedds_cpp",
                               "service type contains 'dds_::' and a suffix, but not at the end"
                               ", report this: '%s'",
                               dds_type_name.c_str());
        continue;
      }
      found_suffix = suffix;
      break;
    }
  }

  // 如果没有找到后缀，返回空字符串
  if (std::string::npos == suffix_position) {
    RCUTILS_LOG_WARN_NAMED("rmw_cyclonedds_cpp",
                           "service type contains 'dds_::' but does not have a suffix"
                           ", report this: '%s'",
                           dds_type_name.c_str());
    return "";
  }

  // 从 '[type_namespace::]dds_::<type><suffix>' 格式转换为 '[type_namespace/]<type>'
  std::string type_namespace = dds_type_name.substr(0, ns_substring_position);
  type_namespace = rcpputils::find_and_replace(type_namespace, "::", "/");
  size_t start = ns_substring_position + ns_substring.length();
  std::string type_name = dds_type_name.substr(start, suffix_position - start);

  return type_namespace + type_name;
}

/**
 * @brief 返回原始名称
 * @param name 名称
 * @return 原始名称
 */
std::string _identity_demangle(const std::string& name) { return name; }
