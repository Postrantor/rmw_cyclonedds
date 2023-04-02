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
#ifndef SERIALIZATION_HPP_
#define SERIALIZATION_HPP_

#include <memory>

#include "TypeSupport2.hpp"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "serdata.hpp"

namespace rmw_cyclonedds_cpp {

/**
 * @brief 基础CDR写入器类
 */
class BaseCDRWriter {
 public:
  /**
   * @brief 获取序列化数据的大小
   * @param data 指向数据的指针
   * @return 序列化数据的大小
   */
  virtual size_t get_serialized_size(const void* data) const = 0;

  /**
   * @brief 将数据序列化到目标缓冲区
   * @param dest 指向目标缓冲区的指针
   * @param data 指向数据的指针
   */
  virtual void serialize(void* dest, const void* data) const = 0;

  /**
   * @brief 获取序列化请求的大小
   * @param request 请求包装对象的引用
   * @return 序列化请求的大小
   */
  virtual size_t get_serialized_size(const cdds_request_wrapper_t& request) const = 0;

  /**
   * @brief 将请求序列化到目标缓冲区
   * @param dest 指向目标缓冲区的指针
   * @param request 请求包装对象的引用
   */
  virtual void serialize(void* dest, const cdds_request_wrapper_t& request) const = 0;

  /**
   * @brief 虚析构函数
   */
  virtual ~BaseCDRWriter() = default;
};

/**
 * @brief 创建CDR写入器实例
 * @param value_type 结构值类型的智能指针
 * @return BaseCDRWriter的智能指针
 */
std::unique_ptr<BaseCDRWriter> make_cdr_writer(std::unique_ptr<StructValueType> value_type);
}  // namespace rmw_cyclonedds_cpp

#endif  // SERIALIZATION_HPP_
