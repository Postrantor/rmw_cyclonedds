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

#include "exception.hpp"

using rmw_cyclonedds_cpp::Exception;

/**
 * @brief 构造函数
 * @param message 错误信息字符串
 */
Exception::Exception(const char* const& message) : m_message(message) {}

/**
 * @brief 拷贝构造函数
 * @param ex 另一个Exception对象
 */
Exception::Exception(const Exception& ex) : m_message(ex.m_message) {}

/**
 * @brief 赋值运算符重载
 * @param ex 另一个Exception对象
 * @return 当前对象的引用
 */
Exception& Exception::operator=(const Exception& ex) {
  m_message = ex.m_message;
  return *this;
}

/**
 * @brief 析构函数
 */
Exception::~Exception() throw() {}

/**
 * @brief 获取错误信息字符串
 * @return 错误信息字符串的常量指针
 */
const char* Exception::what() const throw() { return m_message.c_str(); }
