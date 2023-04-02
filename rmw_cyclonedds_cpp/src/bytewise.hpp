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

#ifndef BYTEWISE_HPP_
#define BYTEWISE_HPP_

#include <cstddef>  // 引入cstddef头文件，包含了ptrdiff_t和size_t类型
#include <cstdint>

#include "dds/ddsrt/endian.h"

using std::ptrdiff_t;  // 使用std命名空间下的ptrdiff_t类型
using std::size_t;     // 使用std命名空间下的size_t类型

// 定义一个枚举类endian，表示字节序
enum class endian {
  little = DDSRT_LITTLE_ENDIAN,  // 小端字节序 - 1
  big = DDSRT_BIG_ENDIAN,        // 大端字节序 - 2
};

// 声明一个constexpr函数native_endian，返回当前系统的字节序
constexpr endian native_endian() { return endian(DDSRT_ENDIAN); }

// 定义一个枚举类byte，表示字节
enum class byte : unsigned char {};

/**
 * @brief 计算指针偏移后的地址
 * @param ptr 指针
 * @param n 偏移量（以字节为单位）
 * @return 返回偏移后的地址
 */
static inline auto byte_offset(void *ptr, ptrdiff_t n) {
  return static_cast<void *>(static_cast<byte *>(ptr) + n);
}

/**
 * @brief 计算const指针偏移后的地址
 * @param ptr const指针
 * @param n 偏移量（以字节为单位）
 * @return 返回偏移后的地址
 */
static inline auto byte_offset(const void *ptr, ptrdiff_t n) {
  return static_cast<const void *>(static_cast<const byte *>(ptr) + n);
}

#endif  // BYTEWISE_HPP_
