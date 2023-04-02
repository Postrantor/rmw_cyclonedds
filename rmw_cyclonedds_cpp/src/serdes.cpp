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
#include "serdes.hpp"

#include <exception>

#include "dds/ddsrt/endian.h"

/**
 * @brief 构造函数，初始化cycdeserbase对象
 * @param data_ 指向数据的指针
 * @param size_ 数据大小
 */
cycdeserbase::cycdeserbase(const char *data_, size_t size_)
    : data(data_), pos(0), lim(size_), swap_bytes(false) {
  // 获取端序字节（跳过data[0]中未使用的第一个字节）
  uint32_t data_endianness = (data[1] == 0x01) ? DDSRT_LITTLE_ENDIAN : DDSRT_BIG_ENDIAN;

  // 如果数据的端序与我们的端序不同：在反序列化时交换字节
  swap_bytes = (DDSRT_ENDIAN != data_endianness);

  // 忽略表示选项（data_[2]和data_[3]）
  data += 4;
  lim -= 4;
}

/**
 * @brief 构造函数，初始化cycdeser对象
 * @param data_ 指向数据的指针
 * @param size_ 数据大小
 */
cycdeser::cycdeser(const void *data_, size_t size_)
    : cycdeserbase(static_cast<const char *>(data_), size_) {}

/**
 * @brief 构造函数，初始化cycprint对象
 * @param buf_ 缓冲区指针
 * @param bufsize_ 缓冲区大小
 * @param data_ 指向数据的指针
 * @param size_ 数据大小
 */
cycprint::cycprint(char *buf_, size_t bufsize_, const void *data_, size_t size_)
    : cycdeserbase(static_cast<const char *>(data_), size_), buf(buf_), bufsize(bufsize_) {}
