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

#include <string>

#include "rosidl_runtime_c/u16string_functions.h"

namespace rmw_cyclonedds_cpp {

/**
 * @brief 将std::u16string转换为std::wstring (Convert std::u16string to std::wstring)
 *
 * @param u16str 输入的std::u16string (Input std::u16string)
 * @param wstr 输出的std::wstring (Output std::wstring)
 */
void u16string_to_wstring(const std::u16string& u16str, std::wstring& wstr) {
  // 调整输出字符串大小 (Resize the output string)
  wstr.resize(u16str.size());
  // 遍历输入字符串并进行类型转换 (Iterate through the input string and perform type conversion)
  for (size_t i = 0; i < u16str.size(); ++i) {
    wstr[i] = static_cast<wchar_t>(u16str[i]);
  }
}

/**
 * @brief 将std::wstring转换为std::u16string (Convert std::wstring to std::u16string)
 *
 * @param wstr 输入的std::wstring (Input std::wstring)
 * @param u16str 输出的std::u16string (Output std::u16string)
 * @return 转换是否成功 (Whether the conversion is successful)
 */
bool wstring_to_u16string(const std::wstring& wstr, std::u16string& u16str) {
  try {
    // 调整输出字符串大小 (Resize the output string)
    u16str.resize(wstr.size());
  } catch (...) {
    // 发生异常时返回false (Return false when an exception occurs)
    return false;
  }
  // 遍历输入字符串并进行类型转换 (Iterate through the input string and perform type conversion)
  for (size_t i = 0; i < wstr.size(); ++i) {
    u16str[i] = static_cast<char16_t>(wstr[i]);
  }
  return true;
}

/**
 * @brief 将rosidl_runtime_c__U16String转换为std::wstring (Convert rosidl_runtime_c__U16String to
 * std::wstring)
 *
 * @param u16str 输入的rosidl_runtime_c__U16String (Input rosidl_runtime_c__U16String)
 * @param wstr 输出的std::wstring (Output std::wstring)
 */
void u16string_to_wstring(const rosidl_runtime_c__U16String& u16str, std::wstring& wstr) {
  // 调整输出字符串大小 (Resize the output string)
  wstr.resize(u16str.size);
  // 遍历输入字符串并进行类型转换 (Iterate through the input string and perform type conversion)
  for (size_t i = 0; i < u16str.size; ++i) {
    wstr[i] = static_cast<wchar_t>(u16str.data[i]);
  }
}

/**
 * @brief 将std::wstring转换为rosidl_runtime_c__U16String (Convert std::wstring to
 * rosidl_runtime_c__U16String)
 *
 * @param wstr 输入的std::wstring (Input std::wstring)
 * @param u16str 输出的rosidl_runtime_c__U16String (Output rosidl_runtime_c__U16String)
 * @return 转换是否成功 (Whether the conversion is successful)
 */
bool wstring_to_u16string(const std::wstring& wstr, rosidl_runtime_c__U16String& u16str) {
  // 调整输出字符串大小并检查是否成功 (Resize the output string and check if it is successful)
  bool succeeded = rosidl_runtime_c__U16String__resize(&u16str, wstr.size());
  if (!succeeded) {
    // 调整大小失败时返回false (Return false when resizing fails)
    return false;
  }
  // 遍历输入字符串并进行类型转换 (Iterate through the input string and perform type conversion)
  for (size_t i = 0; i < wstr.size(); ++i) {
    u16str.data[i] = static_cast<char16_t>(wstr[i]);
  }
  return true;
}

}  // namespace rmw_cyclonedds_cpp
