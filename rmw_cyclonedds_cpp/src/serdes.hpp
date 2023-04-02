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

#ifndef SERDES_HPP_
#define SERDES_HPP_

#include <inttypes.h>
#include <stdarg.h>
#include <string.h>

#include <array>
#include <cassert>
#include <string>
#include <type_traits>
#include <vector>

#include "deserialization_exception.hpp"

using rmw_cyclonedds_cpp::DeserializationException;

class cycdeserbase {
 public:
  /**
   * @brief 构造函数，用于初始化 cycdeserbase 对象
   *
   * 这个构造函数接收一个指向字符数组的指针和一个大小限制作为参数，
   * 然后将这些参数用于初始化 cycdeserbase 对象。
   *
   * @param data_ 指向字符数组的指针
   * @param lim_ 字符数组的大小限制
   */
  explicit cycdeserbase(const char *data_, size_t lim_);

  /**
   * @brief 删除默认构造函数
   *
   * 通过将默认构造函数标记为 delete，我们禁止了使用默认构造函数创建 cycdeserbase 对象。
   * 这样可以确保只能通过提供有效参数的构造函数来创建对象。
   */
  cycdeserbase() = delete;

 protected:
  /**
   * @brief 交换无符号 16 位整数的字节顺序
   *
   * @param x 一个无符号 16 位整数
   * @return 返回交换字节顺序后的无符号 16 位整数
   */
  inline uint16_t bswap2u(uint16_t x) { return (uint16_t)((x >> 8) | (x << 8)); }

  /**
   * @brief 交换有符号 16 位整数的字节顺序
   *
   * @param x 一个有符号 16 位整数
   * @return 返回交换字节顺序后的有符号 16 位整数
   */
  inline int16_t bswap2(int16_t x) { return (int16_t)bswap2u((uint16_t)x); }

  /**
   * @brief 交换无符号 32 位整数的字节顺序
   *
   * @param x 一个无符号 32 位整数
   * @return 返回交换字节顺序后的无符号 32 位整数
   */
  inline uint32_t bswap4u(uint32_t x) {
    return (x >> 24) | ((x >> 8) & 0xff00) | ((x << 8) & 0xff0000) | (x << 24);
  }

  /**
   * @brief 交换有符号 32 位整数的字节顺序
   *
   * @param x 一个有符号 32 位整数
   * @return 返回交换字节顺序后的有符号 32 位整数
   */
  inline int32_t bswap4(int32_t x) { return (int32_t)bswap4u((uint32_t)x); }

  /**
   * @brief 交换无符号 64 位整数的字节顺序
   *
   * @param x 一个无符号 64 位整数
   * @return 返回交换字节顺序后的无符号 64 位整数
   */
  inline uint64_t bswap8u(uint64_t x) {
    const uint32_t newhi = bswap4u((uint32_t)x);
    const uint32_t newlo = bswap4u((uint32_t)(x >> 32));
    return ((uint64_t)newhi << 32) | (uint64_t)newlo;
  }

  /**
   * @brief 交换有符号 64 位整数的字节顺序
   *
   * @param x 一个有符号 64 位整数
   * @return 返回交换字节顺序后的有符号 64 位整数
   */
  inline int64_t bswap8(int64_t x) { return (int64_t)bswap8u((uint64_t)x); }

  /**
   * @brief 对齐数据位置
   *
   * @param a 对齐的字节数
   */
  inline void align(size_t a) {
    if ((pos % a) != 0) {
      pos += a - (pos % a);
      if (pos > lim) {
        throw DeserializationException("invalid data size");
      }
    }
  }

  /**
   * @brief 验证数据大小是否合法
   *
   * @param count 数据元素的数量
   * @param sz 每个数据元素的大小（字节）
   */
  inline void validate_size(size_t count, size_t sz) {
    assert(sz == 1 || sz == 2 || sz == 4 || sz == 8);
    if (count > (lim - pos) / sz) {
      throw DeserializationException("invalid data size");
    }
  }

  /**
   * @brief 验证字符串数据是否以空字符结尾
   *
   * @param sz 字符串数据的大小（字节）
   */
  inline void validate_str(size_t sz) {
    if (sz > 0 && data[pos + sz - 1] != '\0') {
      throw DeserializationException("string data is not null-terminated");
    }
  }

  const char *data;  ///< 指向字符数组的指针
  size_t pos;        ///< 当前数据位置
  size_t lim;        ///< 数据大小限制
  bool swap_bytes;   ///< 是否交换字节顺序
};

class cycdeser : cycdeserbase {
 public:
  cycdeser(const void *data, size_t size);
  cycdeser() = delete;

  inline cycdeser &operator>>(bool &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(char &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(int8_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(uint8_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(int16_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(uint16_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(int32_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(uint32_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(int64_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(uint64_t &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(float &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(double &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(std::string &x) {
    deserialize(x);
    return *this;
  }
  inline cycdeser &operator>>(std::wstring &x) {
    deserialize(x);
    return *this;
  }
  template <class T>
  inline cycdeser &operator>>(std::vector<T> &x) {
    deserialize(x);
    return *this;
  }
  template <class T, size_t S>
  inline cycdeser &operator>>(std::array<T, S> &x) {
    deserialize(x);
    return *this;
  }

// 定义一个 DESER8 宏，用于生成无需字节交换的 deserialize 函数
#define DESER8(T) DESER(T, )

/**
 * @brief 定义一个通用的 deserialize 函数模板，用于反序列化指定类型的数据。
 *
 * @tparam T 数据类型
 * @tparam fn_swap 字节交换函数（如果需要）
 */
#define DESER(T, fn_swap)                                                              \
  inline void deserialize(T &x) {                                                      \
    align(sizeof(x));                             /* 调整对齐 */                   \
    validate_size(1, sizeof(x));                  /* 验证数据大小 */             \
    x = *reinterpret_cast<const T *>(data + pos); /* 从缓冲区读取数据 */       \
    if (swap_bytes) {                             /* 如果需要交换字节顺序 */ \
      x = fn_swap(x); /* 使用指定的字节交换函数进行转换 */              \
    }                                                                                  \
    pos += sizeof(x); /* 更新当前位置 */                                         \
  }

  // 为以下基本类型生成 deserialize 函数
  DESER8(char);
  DESER8(int8_t);
  DESER8(uint8_t);
  DESER(int16_t, bswap2);
  DESER(uint16_t, bswap2u);
  DESER(int32_t, bswap4);
  DESER(uint32_t, bswap4u);
  DESER(int64_t, bswap8);
  DESER(uint64_t, bswap8u);

// 取消 DESER 宏定义，避免污染全局命名空间
#undef DESER

  /**
   * @brief 反序列化布尔值。
   *
   * @param x 输出的布尔值
   */
  inline void deserialize(bool &x) {
    unsigned char z;
    deserialize(z); /* 反序列化一个无符号字符 */
    x = (z != 0);   /* 将非零值转换为 true，零值转换为 false */
  }

  /**
   * @brief 反序列化浮点数。
   *
   * @param x 输出的浮点数
   */
  inline void deserialize(float &x) { deserialize(*reinterpret_cast<uint32_t *>(&x)); }

  /**
   * @brief 反序列化双精度浮点数。
   *
   * @param x 输出的双精度浮点数
   */
  inline void deserialize(double &x) { deserialize(*reinterpret_cast<uint64_t *>(&x)); }

  /**
   * @brief 反序列化长度信息。
   *
   * @param el_sz 元素大小
   * @return uint32_t 反序列化后的长度
   */
  inline uint32_t deserialize_len(size_t el_sz) {
    uint32_t sz;
    deserialize(sz);          /* 反序列化长度值 */
    validate_size(sz, el_sz); /* 验证数据大小 */
    return sz;
  }

  /**
   * @brief 反序列化字符串。
   *
   * @param x 输出的字符串
   */
  inline void deserialize(std::string &x) {
    const uint32_t sz = deserialize_len(sizeof(char)); /* 反序列化字符串长度 */
    if (sz == 0) {
      x = std::string("");                 /* 如果长度为零，则设置为空字符串 */
    } else {
      validate_str(sz);                    /* 验证字符串大小 */
      x = std::string(data + pos, sz - 1); /* 从缓冲区读取字符串 */
    }
    pos += sz;                             /* 更新当前位置 */
  }

  /**
   * @brief 反序列化宽字符串。
   *
   * @param x 输出的宽字符串
   */
  inline void deserialize(std::wstring &x) {
    const uint32_t sz = deserialize_len(sizeof(wchar_t)); /* 反序列化宽字符串长度 */
    // wstring is not null-terminated in cdr
    x = std::wstring(reinterpret_cast<const wchar_t *>(data + pos), sz); /* 从缓冲区读取宽字符串 */
    pos += sz * sizeof(wchar_t);                                         /* 更新当前位置 */
  }

/**
 * @file
 * @brief 包含一组用于反序列化不同类型数据的宏和内联函数
 */

// 定义一个用于处理无需字节交换的宏
#define DESER8_A(T) DESER_A(T, )

/**
 * @brief 用于反序列化指定类型数据的宏
 *
 * @param T 数据类型
 * @param fn_swap 字节交换函数，如果需要进行字节交换，则传入相应的函数名
 */
#define DESER_A(T, fn_swap)                                                                        \
  /**                                                                                              \
   * @brief 反序列化指定类型的数据                                                      \
   *                                                                                               \
   * @param x 指向存储反序列化结果的指针                                              \
   * @param cnt 要反序列化的元素数量                                                     \
   */                                                                                              \
  inline void deserializeA(T *x, size_t cnt) {                                                     \
    if (cnt > 0) {                                                                                 \
      align(sizeof(T));              /* 对齐操作 */                                            \
      validate_size(cnt, sizeof(T)); /* 验证所需空间大小 */                                \
      if (swap_bytes) {              /* 如果需要进行字节交换 */                          \
        for (size_t i = 0; i < cnt; i++) {                                                         \
          x[i] = fn_swap(*reinterpret_cast<const T *>(data + pos)); /* 使用字节交换函数 */ \
          pos += sizeof(T);                                         /* 更新位置 */             \
        }                                                                                          \
      } else {                   /* 不需要进行字节交换 */                                 \
        memcpy(reinterpret_cast<void *>(x), reinterpret_cast<const void *>(data + pos),            \
               cnt * sizeof(T)); /* 直接拷贝数据 */                                          \
        pos += cnt * sizeof(T);  /* 更新位置 */                                                \
      }                                                                                            \
    }                                                                                              \
  }

  // 使用 DESER8_A 定义无需字节交换的反序列化函数
  DESER8_A(char);
  DESER8_A(int8_t);
  DESER8_A(uint8_t);

  // 使用 DESER_A 定义需要字节交换的反序列化函数
  DESER_A(int16_t, bswap2);
  DESER_A(uint16_t, bswap2u);
  DESER_A(int32_t, bswap4);
  DESER_A(uint32_t, bswap4u);
  DESER_A(int64_t, bswap8);
  DESER_A(uint64_t, bswap8u);

// 取消宏定义
#undef DESER_A

  /**
   * @brief 反序列化浮点数数组
   * @param x 指向浮点数数组的指针
   * @param cnt 数组中元素的数量
   */
  inline void deserializeA(float *x, size_t cnt) {
    // 将 float 类型指针转换为 uint32_t 类型指针，并调用重载的 deserializeA 函数
    deserializeA(reinterpret_cast<uint32_t *>(x), cnt);
  }

  /**
   * @brief 反序列化双精度浮点数数组
   * @param x 指向双精度浮点数数组的指针
   * @param cnt 数组中元素的数量
   */
  inline void deserializeA(double *x, size_t cnt) {
    // 将 double 类型指针转换为 uint64_t 类型指针，并调用重载的 deserializeA 函数
    deserializeA(reinterpret_cast<uint64_t *>(x), cnt);
  }

  /**
   * @brief 反序列化泛型数组
   * @tparam T 数组元素的类型
   * @param x 指向数组的指针
   * @param cnt 数组中元素的数量
   */
  template <class T>
  inline void deserializeA(T *x, size_t cnt) {
    // 遍历数组，逐个反序列化元素
    for (size_t i = 0; i < cnt; i++) {
      deserialize(x[i]);
    }
  }

  /**
   * @brief 反序列化 std::vector 容器
   * @tparam T 容器元素的类型
   * @param x 要反序列化的 std::vector 容器
   */
  template <class T>
  inline void deserialize(std::vector<T> &x) {
    // 获取容器的大小
    const uint32_t sz = deserialize_len(1);
    // 调整容器大小
    x.resize(sz);
    // 反序列化容器中的元素
    deserializeA(x.data(), sz);
  }

  /**
   * @brief 反序列化 std::vector<bool> 容器
   * @param x 要反序列化的 std::vector<bool> 容器
   */
  inline void deserialize(std::vector<bool> &x) {
    // 获取容器的大小
    const uint32_t sz = deserialize_len(sizeof(unsigned char));
    // 调整容器大小
    x.resize(sz);
    for (size_t i = 0; i < sz; i++) {
      x[i] = ((data + pos)[i] != 0);
    }
    pos += sz;
  }
  template <class T, size_t S>
  inline void deserialize(std::array<T, S> &x) {
    deserializeA(x.data(), x.size());
  }
};

class cycprint : cycdeserbase {
 public:
  cycprint(char *buf, size_t bufsize, const void *data, size_t size);
  cycprint() = delete;

  void print_constant(const char *x) { prtf(&buf, &bufsize, "%s", x); }

  inline cycprint &operator>>(bool &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(char &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(int8_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(uint8_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(int16_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(uint16_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(int32_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(uint32_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(int64_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(uint64_t &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(float &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(double &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(std::string &x) {
    print(x);
    return *this;
  }
  inline cycprint &operator>>(std::wstring &x) {
    print(x);
    return *this;
  }
  template <class T>
  inline cycprint &operator>>(std::vector<T> &x) {
    print(x);
    return *this;
  }
  template <class T, size_t S>
  inline cycprint &operator>>(std::array<T, S> &x) {
    print(x);
    return *this;
  }

// 定义宏 PRNT8，用于调用 PRNT 宏并传递空的 fn_swap 参数
#define PRNT8(T, F) PRNT(T, F, )

// 定义宏 PRNT，用于生成内联函数 print，该函数负责打印指定类型的变量
#define PRNT(T, F, fn_swap) inline void print(T &x)                        \  // 定义内联函数 print，接受一个引用参数 x
  {
    align(sizeof(x));
    \    // 调整数据对齐
        validate_size(1, sizeof(x));
    \    // 验证数据大小是否合法
        x = *reinterpret_cast<const T *>(data + pos);
    \    // 从 data 中读取值，并赋给 x
        if (swap_bytes) {
      \  // 如果需要交换字节序
          x = fn_swap(x);
      \  // 使用 fn_swap 函数进行字节序交换
    }
    prtf(&buf, &bufsize, F, x);
    \  // 使用格式化字符串 F 打印 x 的值
        pos += sizeof(x);
    \  // 更新位置
  }

  // 使用 PRNT8 宏为 char 类型生成 print 函数
  PRNT8(char, "'%c'");
  // 使用 PRNT8 宏为 int8_t 类型生成 print 函数
  PRNT8(int8_t, "%" PRId8);
  // 使用 PRNT8 宏为 uint8_t 类型生成 print 函数
  PRNT8(uint8_t, "%" PRIu8);
  // 使用 PRNT 宏为 int16_t 类型生成 print 函数，并传递 bswap2 作为字节序交换函数
  PRNT(int16_t, "%" PRId16, bswap2);
  // 使用 PRNT 宏为 uint16_t 类型生成 print 函数，并传递 bswap2u 作为字节序交换函数
  PRNT(uint16_t, "%" PRIu16, bswap2u);
  // 使用 PRNT 宏为 int32_t 类型生成 print 函数，并传递 bswap4 作为字节序交换函数
  PRNT(int32_t, "%" PRId32, bswap4);
  // 使用 PRNT 宏为 uint32_t 类型生成 print 函数，并传递 bswap4u 作为字节序交换函数
  PRNT(uint32_t, "%" PRIu32, bswap4u);
  // 使用 PRNT 宏为 int64_t 类型生成 print 函数，并传递 bswap8 作为字节序交换函数
  PRNT(int64_t, "%" PRId64, bswap8);
  // 使用 PRNT 宏为 uint64_t 类型生成 print 函数，并传递 bswap8u 作为字节序交换函数
  PRNT(uint64_t, "%" PRIu64, bswap8u);

// 取消定义宏 PRNT
#undef PRNT

  /**
   * @brief 打印布尔值
   *
   * @param x 布尔值引用
   */
  inline void print(bool &x) {
    // 强制类型转换，忽略x的值
    static_cast<void>(x);

    // 定义一个无符号字符变量z
    unsigned char z;

    // 调用print函数打印z
    print(z);
  }

  /**
   * @brief 打印浮点数
   *
   * @param x 浮点数引用
   */
  inline void print(float &x) {
    // 定义一个联合体，包含一个32位无符号整数和一个浮点数
    union {
      uint32_t u;
      float f;
    } tmp;

    // 对齐x的大小
    align(sizeof(x));

    // 验证数据大小是否正确
    validate_size(1, sizeof(x));

    // 从data中读取浮点数并存储到tmp.u中
    tmp.u = *reinterpret_cast<const uint32_t *>(data + pos);

    // 如果需要交换字节序，则进行交换
    if (swap_bytes) {
      tmp.u = bswap4u(tmp.u);
    }

    // 强制类型转换，忽略tmp.u的值
    static_cast<void>(tmp.u);

    // 将浮点数格式化输出到buf中
    prtf(&buf, &bufsize, "%f", tmp.f);

    // 更新位置
    pos += sizeof(x);
  }

  /**
   * @brief 打印双精度浮点数
   *
   * @param x 双精度浮点数引用
   */
  inline void print(double &x) {
    // 定义一个联合体，包含一个64位无符号整数和一个双精度浮点数
    union {
      uint64_t u;
      double f;
    } tmp;

    // 对齐x的大小
    align(sizeof(x));

    // 验证数据大小是否正确
    validate_size(1, sizeof(x));

    // 从data中读取双精度浮点数并存储到tmp.u中
    tmp.u = *reinterpret_cast<const uint64_t *>(data + pos);

    // 如果需要交换字节序，则进行交换
    if (swap_bytes) {
      tmp.u = bswap8u(tmp.u);
    }

    // 强制类型转换，忽略tmp.u的值
    static_cast<void>(tmp.u);

    // 将双精度浮点数格式化输出到buf中
    prtf(&buf, &bufsize, "%f", tmp.f);

    // 更新位置
    pos += sizeof(x);
  }

  /**
   * @brief 获取长度
   *
   * @param el_sz 元素大小
   * @return uint32_t 返回长度
   */
  inline uint32_t get_len(size_t el_sz) {
    // 定义一个32位无符号整数变量sz
    uint32_t sz;

    // 对齐sz的大小
    align(sizeof(sz));

    // 验证数据大小是否正确
    validate_size(1, sizeof(sz));

    // 从data中读取长度并存储到sz中
    sz = *reinterpret_cast<const uint32_t *>(data + pos);

    // 如果需要交换字节序，则进行交换
    if (swap_bytes) {
      sz = bswap4u(sz);
    }

    // 更新位置
    pos += sizeof(sz);

    // 验证数据大小是否正确
    validate_size(sz, el_sz);

    // 返回长度
    return sz;
  }
  /**
   * @brief 打印字符串
   *
   * @param x 要打印的字符串引用
   */
  inline void print(std::string &x) {
    // 获取字符串长度
    const uint32_t sz = get_len(sizeof(char));

    // 验证字符串长度
    validate_str(sz);

    // 计算实际长度，确保不超过 INT32_MAX
    const int len = (sz == 0) ? 0 : (sz > INT32_MAX) ? INT32_MAX : static_cast<int>(sz - 1);

    // 强制类型转换，避免编译器警告
    static_cast<void>(x);

    // 格式化输出字符串
    prtf(&buf, &bufsize, "\"%*.*s\"", len, len, static_cast<const char *>(data + pos));

    // 更新位置
    pos += sz;
  }

  /**
   * @brief 打印宽字符串
   *
   * @param x 要打印的宽字符串引用
   */
  inline void print(std::wstring &x) {
    // 获取宽字符串长度
    const uint32_t sz = get_len(sizeof(wchar_t));

    // 将宽字符串数据转换为 std::wstring 类型
    x = std::wstring(reinterpret_cast<const wchar_t *>(data + pos), sz);

    // 格式化输出宽字符串
    prtf(&buf, &bufsize, "\"%ls\"", x.c_str());

    // 更新位置
    pos += sz * sizeof(wchar_t);
  }

  /**
   * @brief 打印数组
   *
   * @tparam T 数组元素类型
   * @param x 指向数组的指针
   * @param cnt 数组元素个数
   */
  template <class T>
  inline void printA(T *x, size_t cnt) {
    // 输出开始括号
    prtf(&buf, &bufsize, "{");

    // 遍历数组并打印每个元素
    for (size_t i = 0; i < cnt; i++) {
      if (i != 0) {
        prtf(&buf, &bufsize, ",");
      }
      print(*x);
    }

    // 输出结束括号
    prtf(&buf, &bufsize, "}");
  }

  /**
   * @brief 打印 std::vector 容器
   *
   * @tparam T 容器元素类型
   * @param x 要打印的 std::vector 容器引用
   */
  template <class T>
  inline void print(std::vector<T> &x) {
    // 获取容器长度
    const uint32_t sz = get_len(1);

    // 调用 printA 函数打印容器内容
    printA(x.data(), sz);
  }

  /**
   * @brief 打印 std::array 容器
   *
   * @tparam T 容器元素类型
   * @tparam S 容器大小
   * @param x 要打印的 std::array 容器引用
   */
  template <class T, size_t S>
  inline void print(std::array<T, S> &x) {
    // 调用 printA 函数打印容器内容
    printA(x.data(), x.size());
  }

 private:
  /**
   * @brief 以格式化字符串的方式将数据写入缓冲区
   *
   * @param buf 指向缓冲区指针的指针，用于存储格式化后的字符串
   * @param bufsize 指向缓冲区大小的指针，用于记录剩余可用空间
   * @param fmt 格式化字符串，类似于 printf 的格式
   * @param ... 可变参数列表，与格式化字符串对应的数据
   * @return bool 返回值表示是否成功写入缓冲区
   */
  static bool prtf(char *__restrict *buf, size_t *__restrict bufsize, const char *fmt, ...) {
    va_list ap;           // 定义可变参数列表

    if (*bufsize == 0) {  // 如果缓冲区大小为0
      return false;       // 直接返回失败
    }

    va_start(ap, fmt);                           // 初始化可变参数列表
    int n = vsnprintf(*buf, *bufsize, fmt, ap);  // 将格式化字符串写入缓冲区，并获取写入字符数
    va_end(ap);                                  // 结束可变参数列表

    if (n < 0) {                                 // 如果写入失败
      **buf = 0;                                 // 将缓冲区置为空
      return false;                              // 返回失败
    } else if (static_cast<size_t>(n) <= *bufsize) {  // 如果写入字符数小于等于缓冲区大小
      *buf += static_cast<size_t>(n);                 // 更新缓冲区指针
      *bufsize -= static_cast<size_t>(n);             // 更新剩余可用空间
      return *bufsize > 0;                            // 返回是否还有剩余空间
    } else {                                          // 如果写入字符数大于缓冲区大小
      *buf += *bufsize;                               // 更新缓冲区指针至末尾
      *bufsize = 0;                                   // 设置剩余可用空间为0
      return false;                                   // 返回失败
    }
  }

  char *buf;       // 缓冲区指针
  size_t bufsize;  // 缓冲区大小
};

#endif  // SERDES_HPP_
