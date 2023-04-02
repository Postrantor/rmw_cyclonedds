// Copyright 2018 to 2019 ADLINK Technology
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

#ifndef DESERIALIZATION_EXCEPTION_HPP_
#define DESERIALIZATION_EXCEPTION_HPP_

#include "exception.hpp"

namespace rmw_cyclonedds_cpp {

/**
 * @class DeserializationException
 * @brief 自定义异常类，用于处理反序列化过程中的错误
 */
class DeserializationException : public Exception {
public:
  /**
   * @brief 构造函数
   * @param message 错误信息字符串
   */
  explicit DeserializationException(const char* const& message);

  /**
   * @brief 拷贝构造函数
   * @param ex 另一个DeserializationException对象
   */
  DeserializationException(const DeserializationException& ex);

  /**
   * @brief 赋值运算符重载
   * @param ex 另一个DeserializationException对象
   * @return 当前对象的引用
   */
  DeserializationException& operator=(const DeserializationException& ex);

  /**
   * @brief 析构函数
   */
  virtual ~DeserializationException() throw();

  /**
   * @brief 默认错误信息字符串
   */
  static const char* const DEFAULT_MESSAGE;
};

}  // namespace rmw_cyclonedds_cpp

#endif  // DESERIALIZATION_EXCEPTION_HPP_
