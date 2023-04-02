// Copyright 2019 ADLINK Technology
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

#ifndef U16STRING_HPP_
#define U16STRING_HPP_

#include <string>

#include "rosidl_runtime_c/u16string_functions.h"

namespace rmw_cyclonedds_cpp {

/**
 * @brief 将rosidl_runtime_c__U16String类型的字符串转换为std::wstring类型的字符串
 * @param u16str 输入的rosidl_runtime_c__U16String类型的字符串
 * @param wstr 输出的std::wstring类型的字符串
 */
void u16string_to_wstring(const rosidl_runtime_c__U16String& u16str, std::wstring& wstr);

/**
 * @brief 将std::wstring类型的字符串转换为rosidl_runtime_c__U16String类型的字符串
 * @param wstr 输入的std::wstring类型的字符串
 * @param u16str 输出的rosidl_runtime_c__U16String类型的字符串
 * @return 转换成功返回true，否则返回false
 */
bool wstring_to_u16string(const std::wstring& wstr, rosidl_runtime_c__U16String& u16str);

/**
 * @brief 将std::u16string类型的字符串转换为std::wstring类型的字符串
 * @param u16str 输入的std::u16string类型的字符串
 * @param wstr 输出的std::wstring类型的字符串
 */
void u16string_to_wstring(const std::u16string& u16str, std::wstring& wstr);

/**
 * @brief 将std::wstring类型的字符串转换为std::u16string类型的字符串
 * @param wstr 输入的std::wstring类型的字符串
 * @param u16str 输出的std::u16string类型的字符串
 * @return 转换成功返回true，否则返回false
 */
bool wstring_to_u16string(const std::wstring& wstr, std::u16string& u16str);

}  // namespace rmw_cyclonedds_cpp

#endif  // U16STRING_HPP_
