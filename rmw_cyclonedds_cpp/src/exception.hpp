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

#ifndef EXCEPTION_HPP_
#define EXCEPTION_HPP_

#include <stdexcept>
#include <string>

namespace rmw_cyclonedds_cpp {

// 自定义异常类，继承自 std::exception
class Exception : public std::exception {
public:
  // 虚析构函数，允许通过基类指针删除派生类对象
  virtual ~Exception() throw();

  // 返回异常信息的字符串，覆盖 std::exception 的 what() 函数
  virtual const char* what() const throw();

protected:
  // 构造函数，接受一个异常信息字符串作为参数
  explicit Exception(const char* const& message);

  // 拷贝构造函数
  Exception(const Exception& ex);

  // 赋值运算符重载
  Exception& operator=(const Exception& ex);

  // 存储异常信息的字符串
  std::string m_message;
};

/// Stub for code that should never be reachable by design.
/// If it is possible to reach the code due to bad data or other runtime conditions,
/// use a runtime_error instead
// 不可达代码的存根。如果由于错误数据或其他运行时条件可能到达该代码，请使用 runtime_error
[[noreturn]] inline void unreachable() {
// 如果编译器支持 __has_builtin 属性
#if defined(__has_builtin)
// 如果编译器支持 __builtin_unreachable 内建函数
#if __has_builtin(__builtin_unreachable)
  // 使用内建函数表示不可达代码
  __builtin_unreachable();
#endif
// 如果使用的是 GCC 编译器，并且版本大于等于 4.5
#elif (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5))
  // 使用内建函数表示不可达代码
  __builtin_unreachable();
#endif
  // 如果编译器不支持上述内建函数，抛出逻辑错误异常
  throw std::logic_error("This code should be unreachable.");
}
}  // namespace rmw_cyclonedds_cpp

#endif  // EXCEPTION_HPP_
