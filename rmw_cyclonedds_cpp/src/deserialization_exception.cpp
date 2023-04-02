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

#include "deserialization_exception.hpp"

// 引入rmw_cyclonedds_cpp命名空间中的DeserializationException类
using rmw_cyclonedds_cpp::DeserializationException;

/**
 * @brief 构造函数，用给定的消息初始化DeserializationException对象
 *
 * @param message 传递给异常的错误消息
 */
DeserializationException::DeserializationException(const char* const& message)
    : Exception(message)  // 调用基类构造函数并传递错误消息
{}

/**
 * @brief 拷贝构造函数，用另一个DeserializationException对象初始化当前对象
 *
 * @param ex 另一个DeserializationException对象
 */
DeserializationException::DeserializationException(const DeserializationException& ex)
    : Exception(ex)  // 调用基类拷贝构造函数并传递异常对象
{}

/**
 * @brief 赋值运算符重载，用于将另一个DeserializationException对象的值赋给当前对象
 *
 * @param ex 另一个DeserializationException对象
 * @return DeserializationException& 返回当前对象的引用
 */
DeserializationException& DeserializationException::operator=(const DeserializationException& ex) {
  if (this != &ex) {           // 检查自赋值情况
    Exception::operator=(ex);  // 调用基类的赋值运算符重载并传递异常对象
  }
  return *this;                // 返回当前对象的引用
}

/**
 * @brief 析构函数，释放DeserializationException对象占用的资源
 */
DeserializationException::~DeserializationException() throw() {}

const char* const DeserializationException::DEFAULT_MESSAGE = "Invalid data";
