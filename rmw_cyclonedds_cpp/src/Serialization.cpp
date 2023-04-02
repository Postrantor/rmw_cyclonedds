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

// suppress definition of min/max macros on Windows.
// TODO(dan@digilabs.io): Move this closer to where Windows.h/Windef.h is included
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "Serialization.hpp"

#include <array>
#include <cstring>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "TypeSupport2.hpp"
#include "bytewise.hpp"

namespace rmw_cyclonedds_cpp {

/**
 * @struct CDRCursor
 * @brief CDRCursor 结构体，用于处理字节流中的游标操作。
 */
struct CDRCursor {
  /**
   * @brief 默认构造函数
   */
  CDRCursor() = default;

  /**
   * @brief 默认析构函数
   */
  ~CDRCursor() = default;

  // 不允许意外复制
  /**
   * @brief 删除拷贝构造函数，防止意外复制
   * @param CDRCursor const & 被删除的拷贝构造函数参数
   */
  explicit CDRCursor(CDRCursor const &) = delete;

  /**
   * @brief 删除赋值运算符，防止意外复制
   * @param x CDRCursor const & 被删除的赋值运算符参数
   */
  void operator=(CDRCursor const &x) = delete;

  // 虚函数需要实现
  /**
   * @brief 获取游标当前偏移量
   * @return size_t 当前偏移量
   */
  virtual size_t offset() const = 0;

  /**
   * @brief 推进游标
   * @param n_bytes size_t 需要推进的字节数
   */
  virtual void advance(size_t n_bytes) = 0;

  /**
   * @brief 将字节复制到当前游标位置（如果需要）并推进游标
   * @param data const void * 指向数据的指针
   * @param size size_t 数据大小
   */
  virtual void put_bytes(const void *data, size_t size) = 0;

  /**
   * @brief 判断是否忽略数据
   * @return bool 如果忽略数据返回 true，否则返回 false
   */
  virtual bool ignores_data() const = 0;

  /**
   * @brief 将逻辑原点移动这么多位置
   * @param relative_origin ptrdiff_t 相对原点的偏移量
   */
  virtual void rebase(ptrdiff_t relative_origin) = 0;

  /**
   * @brief 对齐游标
   * @param n_bytes size_t 需要对齐的字节数
   */
  void align(size_t n_bytes) {
    assert(n_bytes > 0);
    size_t start_offset = offset();
    if (n_bytes == 1 || start_offset % n_bytes == 0) {
      return;
    }
    advance(n_bytes - start_offset % n_bytes);
    assert(offset() - start_offset < n_bytes);
    assert(offset() % n_bytes == 0);
  }

  /**
   * @brief 计算两个 CDRCursor 之间的差值
   * @param other const CDRCursor & 另一个 CDRCursor 对象
   * @return ptrdiff_t 两个 CDRCursor 之间的差值
   */
  ptrdiff_t operator-(const CDRCursor &other) const {
    return static_cast<ptrdiff_t>(offset()) - static_cast<ptrdiff_t>(other.offset());
  }
};

/**
 * @brief SizeCursor 结构体，继承自 CDRCursor 类
 */
struct SizeCursor : public CDRCursor {
  /**
   * @brief 默认构造函数，初始化 m_offset 为 0
   */
  SizeCursor() : SizeCursor(0) {}

  /**
   * @brief 构造函数，接收一个初始偏移量
   * @param initial_offset 初始偏移量
   */
  explicit SizeCursor(size_t initial_offset) : m_offset(initial_offset) {}

  /**
   * @brief 构造函数，接收一个 CDRCursor 对象的引用
   * @param c CDRCursor 对象的引用
   */
  explicit SizeCursor(CDRCursor &c) : m_offset(c.offset()) {}

  size_t m_offset;  ///< 偏移量

  /**
   * @brief 获取当前偏移量
   * @return 当前偏移量
   */
  size_t offset() const final { return m_offset; }

  /**
   * @brief 向前推进指定字节数
   * @param n_bytes 要推进的字节数
   */
  void advance(size_t n_bytes) final { m_offset += n_bytes; }

  /**
   * @brief 将字节放入缓冲区，并向前推进指定字节数
   * @param bytes 字节数据
   * @param n_bytes 要推进的字节数
   */
  void put_bytes(const void *, size_t n_bytes) final { advance(n_bytes); }

  /**
   * @brief 检查是否忽略数据
   * @return 忽略数据返回 true，否则返回 false
   */
  bool ignores_data() const final { return true; }

  /**
   * @brief 重新设置基址
   * @param relative_origin 相对原点
   */
  void rebase(ptrdiff_t relative_origin) override {
    // 我们正在移动*原点*，所以这必须在*相反*的方向上改变
    m_offset -= relative_origin;
  }
};

/**
 * @brief DataCursor 结构体，继承自 CDRCursor 类
 */
struct DataCursor : public CDRCursor {
  const void *origin;  ///< 原始指针
  void *position;      ///< 当前位置指针

  /**
   * @brief 构造函数，接收一个指针作为初始位置
   * @param position 初始位置指针
   */
  explicit DataCursor(void *position) : origin(position), position(position) {}

  /**
   * @brief 获取当前偏移量
   * @return 当前偏移量
   */
  size_t offset() const final { return (const byte *)position - (const byte *)origin; }

  /**
   * @brief 向前推进指定字节数，并将推进过程中的数据置零
   * @param n_bytes 要推进的字节数
   */
  void advance(size_t n_bytes) final {
    std::memset(position, '\0', n_bytes);
    position = byte_offset(position, n_bytes);
  }

  /**
   * @brief 将字节放入缓冲区，并向前推进指定字节数
   * @param bytes 字节数据
   * @param n_bytes 要推进的字节数
   */
  void put_bytes(const void *bytes, size_t n_bytes) final {
    if (n_bytes == 0) {
      return;
    }
    std::memcpy(position, bytes, n_bytes);
    position = byte_offset(position, n_bytes);
  }

  /**
   * @brief 检查是否忽略数据
   * @return 忽略数据返回 true，否则返回 false
   */
  bool ignores_data() const final { return false; }

  /**
   * @brief 重新设置基址
   * @param relative_origin 相对原点
   */
  void rebase(ptrdiff_t relative_origin) final { origin = byte_offset(origin, relative_origin); }
};

/**
 * @brief EncodingVersion 枚举类，表示编码版本
 */
enum class EncodingVersion {
  CDR_Legacy,  ///< CDR 旧版编码
  CDR1,        ///< CDR1 编码
};

class CDRWriter : public BaseCDRWriter {
 public:
  /**
   * @brief 缓存键结构体，用于存储对齐大小和值类型的信息。
   */
  struct CacheKey {
    size_t align;                    ///< 对齐大小
    const AnyValueType *value_type;  ///< 值类型指针

    /**
     * @brief 比较两个缓存键是否相等
     * @param other 另一个缓存键对象
     * @return 如果两个缓存键相等，则返回true，否则返回false
     */
    bool operator==(const CacheKey &other) const {
      return align == other.align && value_type == other.value_type;
    }

    /**
     * @brief 哈希结构体，用于计算缓存键的哈希值
     */
    struct Hash {
      /**
       * @brief 计算给定缓存键的哈希值
       * @param k 要计算哈希值的缓存键对象
       * @return 返回计算得到的哈希值
       */
      size_t operator()(const CacheKey &k) const {
        return std::hash<decltype(align)>{}(k.align) ^
               ((std::hash<decltype(value_type)>{}(k.value_type)) << 1U);
      }
    };
  };

  const EncodingVersion eversion;                            ///< 编码版本常量
  const size_t max_align;                                    ///< 最大对齐大小常量
  std::unique_ptr<const StructValueType> m_root_value_type;  ///< 根值类型的智能指针
  std::unordered_map<CacheKey, bool, CacheKey::Hash>
      trivially_serialized_cache;  ///< 存储缓存键和序列化状态的无序映射

 public:
  /**
   * @brief 构造一个 CDRWriter 对象
   *
   * @param root_value_type 一个指向结构值类型的唯一指针
   */
  explicit CDRWriter(std::unique_ptr<const StructValueType> root_value_type)
      : eversion{EncodingVersion::CDR_Legacy},  // 设置编码版本为 CDR_Legacy
        max_align{8},                           // 设置最大对齐值为 8
        m_root_value_type{
            std::move(root_value_type)},  // 移动 root_value_type 到成员变量 m_root_value_type
        trivially_serialized_cache{}      // 初始化空的序列化缓存
  {
    assert(m_root_value_type);            // 断言 m_root_value_type 不为空
    register_serializable_type(m_root_value_type.get());  // 注册可序列化类型
  }

  /**
   * @brief 注册可序列化类型
   *
   * @param t 指向任意值类型的指针
   */
  void register_serializable_type(const AnyValueType *t) {
    for (size_t align = 0; align < max_align; align++) {  // 遍历所有可能的对齐值
      CacheKey key{align, t};                             // 创建缓存键
      if (trivially_serialized_cache.find(key) !=
          trivially_serialized_cache.end()) {  // 如果已经在缓存中找到该键，则跳过
        continue;
      }

      bool &result = trivially_serialized_cache[key];  // 获取缓存结果引用

      switch (t->e_value_type()) {                     // 根据值类型进行处理
        case EValueType::PrimitiveValueType: {
          auto tt = static_cast<const PrimitiveValueType *>(t);
          result = is_trivially_serialized(align, *tt);
        } break;
        case EValueType::ArrayValueType: {
          auto tt = static_cast<const ArrayValueType *>(t);
          result = compute_trivially_serialized(align, *tt);
          register_serializable_type(tt->element_value_type());
        } break;
        case EValueType::StructValueType: {
          auto tt = static_cast<const StructValueType *>(t);
          for (size_t i = 0; i < tt->n_members(); i++) {
            register_serializable_type(tt->get_member(i)->value_type);
          }
          result = is_trivially_serialized(align, *tt);
        } break;
        case EValueType::SpanSequenceValueType: {
          auto tt = static_cast<const SpanSequenceValueType *>(t);
          register_serializable_type(tt->element_value_type());
        }
          result = false;
          break;
        case EValueType::U8StringValueType:
        case EValueType::U16StringValueType:
        case EValueType::BoolVectorValueType:
          result = false;
          break;
        default:
          unreachable();
      }
    }
  }

  /**
   * @brief 获取序列化数据的大小
   *
   * @param data 指向需要序列化的数据的指针
   * @return size_t 序列化数据的大小
   */
  size_t get_serialized_size(const void *data) const override {
    SizeCursor cursor;                   // 创建一个大小游标

    serialize_top_level(&cursor, data);  // 对顶层数据进行序列化
    return cursor.offset();              // 返回序列化后的偏移量
  }

  /**
   * @brief 将数据序列化到目标缓冲区
   *
   * @param dest 目标缓冲区，用于存储序列化后的数据
   * @param data 需要序列化的数据
   */
  void serialize(void *dest, const void *data) const override {
    DataCursor cursor(dest);             // 创建一个数据游标，指向目标缓冲区
    serialize_top_level(&cursor, data);  // 调用顶层序列化函数进行序列化
  }

  /**
   * @brief 获取序列化后的数据大小
   *
   * @param request 请求包装器对象
   * @return size_t 序列化后的数据大小
   */
  size_t get_serialized_size(const cdds_request_wrapper_t &request) const override {
    SizeCursor cursor;                      // 创建一个大小游标
    serialize_top_level(&cursor, request);  // 调用顶层序列化函数进行序列化
    return cursor.offset();                 // 返回序列化后的数据大小
  }

  /**
   * @brief 将请求包装器对象序列化到目标缓冲区
   *
   * @param dest 目标缓冲区，用于存储序列化后的数据
   * @param request 请求包装器对象
   */
  void serialize(void *dest, const cdds_request_wrapper_t &request) const override {
    DataCursor cursor(dest);                // 创建一个数据游标，指向目标缓冲区
    serialize_top_level(&cursor, request);  // 调用顶层序列化函数进行序列化
  }

  /**
   * @brief 顶层序列化函数
   *
   * @param cursor CDRCursor指针，用于操作缓冲区
   * @param data 需要序列化的数据
   */
  void serialize_top_level(CDRCursor *cursor, const void *data) const {
    put_rtps_header(cursor);  // 添加RTPS头部信息

    if (eversion == EncodingVersion::CDR_Legacy) {
      cursor->rebase(+4);  // 如果编码版本为CDR_Legacy，则向前移动游标4个字节
    }

    if (m_root_value_type->n_members() == 0 && eversion == EncodingVersion::CDR_Legacy) {
      char dummy = '\0';                                 // 创建一个空字符
      cursor->put_bytes(&dummy, 1);                      // 将空字符写入缓冲区
    } else {
      serialize(cursor, data, m_root_value_type.get());  // 调用序列化函数进行序列化
    }

    if (eversion == EncodingVersion::CDR_Legacy) {
      cursor->rebase(-4);  // 如果编码版本为CDR_Legacy，则向后移动游标4个字节
    }
  }

  /**
   * @brief 将顶层数据序列化为CDR格式
   *
   * @param cursor 指向CDRCursor对象的指针，用于操作序列化过程中的读写位置
   * @param request 包含待序列化数据的cdds_request_wrapper_t对象引用
   */
  void serialize_top_level(CDRCursor *cursor, const cdds_request_wrapper_t &request) const {
    put_rtps_header(cursor);  // 添加RTPS报文头部

    if (eversion == EncodingVersion::CDR_Legacy) {
      cursor->rebase(+4);  // 如果编码版本为CDR_Legacy，则将游标向前移动4字节
    }

    cursor->put_bytes(&request.header.guid,
                      sizeof(request.header.guid));  // 将请求头部的GUID写入缓冲区
    cursor->put_bytes(&request.header.seq,
                      sizeof(request.header.seq));   // 将请求头部的序列号写入缓冲区

    serialize(cursor, request.data, m_root_value_type.get());  // 序列化请求数据

    if (eversion == EncodingVersion::CDR_Legacy) {
      cursor->rebase(-4);  // 如果编码版本为CDR_Legacy，则将游标向后移动4字节
    }
  }

 protected:
  /**
   * @brief 向CDRCursor中添加RTPS头部信息
   *
   * @param cursor 指向CDRCursor对象的指针，用于存储RTPS头部信息
   */
  void put_rtps_header(CDRCursor *cursor) const {
    // 开始处理消息
    char eversion_byte;
    // 根据编码版本设置eversion_byte
    switch (eversion) {
      case EncodingVersion::CDR_Legacy:
        eversion_byte = '\0';
        break;
      case EncodingVersion::CDR1:
        eversion_byte = '\1';
        break;
      default:
        unreachable();
    }
    // 初始化RTPS头部数组
    std::array<char, 4> rtps_header{{eversion_byte,
                                     // 编码格式为PLAIN_CDR
                                     (native_endian() == endian::little) ? '\1' : '\0',
                                     // 选项字段
                                     '\0', '\0'}};
    // 将RTPS头部数据写入cursor
    cursor->put_bytes(rtps_header.data(), rtps_header.size());
  }

  /**
   * @brief 序列化一个32位无符号整数并将其存储在CDRCursor中
   *
   * @param cursor 指向CDRCursor对象的指针，用于存储序列化后的数据
   * @param value 要序列化的size_t类型值
   */
  void serialize_u32(CDRCursor *cursor, size_t value) const {
    // 确保value不超过uint32_t的最大值
    assert(value <= std::numeric_limits<uint32_t>::max());
    // 将size_t类型的value转换为uint32_t类型
    auto u32_value = static_cast<uint32_t>(value);
    // 对齐CDRCursor中的数据
    cursor->align(4);
    // 将序列化后的u32_value写入cursor
    cursor->put_bytes(&u32_value, 4);
  }

  /**
   * @brief 获取基本类型的CDR序列化大小
   *
   * @param tk ROSIDL_TypeKind枚举值，表示基本类型
   * @return size_t 返回基本类型在CDR序列化时所占用的字节数
   */
  static size_t get_cdr_size_of_primitive(ROSIDL_TypeKind tk) {
    // 根据输入的类型tk，返回相应的序列化字节数
    switch (tk) {
      case ROSIDL_TypeKind::BOOLEAN:
      case ROSIDL_TypeKind::OCTET:
      case ROSIDL_TypeKind::UINT8:
      case ROSIDL_TypeKind::INT8:
      case ROSIDL_TypeKind::CHAR:
        return 1;
      case ROSIDL_TypeKind::UINT16:
      case ROSIDL_TypeKind::INT16:
      case ROSIDL_TypeKind::WCHAR:
        return 2;
      case ROSIDL_TypeKind::UINT32:
      case ROSIDL_TypeKind::INT32:
      case ROSIDL_TypeKind::FLOAT:
        return 4;
      case ROSIDL_TypeKind::UINT64:
      case ROSIDL_TypeKind::INT64:
      case ROSIDL_TypeKind::DOUBLE:
        return 8;
      case ROSIDL_TypeKind::LONG_DOUBLE:
        return 16;
      default:
        return 0;
    }
  }

  /**
   * @brief 判断结构体是否可以进行简单序列化
   *
   * @param align 对齐值
   * @param p 结构体值类型的引用
   * @return bool 如果结构体可以进行简单序列化，则返回true，否则返回false
   */
  bool is_trivially_serialized(size_t align, const StructValueType &p) const {
    align %= max_align;

    size_t offset = align;
    // 遍历结构体的成员
    for (size_t i = 0; i < p.n_members(); i++) {
      auto m = p.get_member(i);
      // 如果成员的偏移量不等于当前计算的偏移量，则返回false
      if (m->member_offset != offset - align) {
        return false;
      }
      // 如果成员的值类型不能进行简单序列化，则返回false
      if (!compute_trivially_serialized(offset % max_align, m->value_type)) {
        return false;
      }
      // 更新偏移量
      offset += m->value_type->sizeof_type();
    }

    // 如果最终偏移量等于初始对齐值加上结构体大小，则返回true，否则返回false
    return offset == align + p.sizeof_struct();
  }

  /**
   * @brief 判断给定的基本值类型是否可以被简单序列化
   * @param align 对齐大小
   * @param v 基本值类型
   * @return 如果可以简单序列化，则返回true，否则返回false
   */
  bool is_trivially_serialized(size_t align, const PrimitiveValueType &v) const {
    align %= max_align;  // 计算对齐余数

    // 值为0表示不是基本类型，这种情况不应该发生，已在其他地方进行检查
    const size_t cdr_alignof = get_cdr_alignof_primitive(v.type_kind());
    assert(0 != cdr_alignof);
    if (align % cdr_alignof != 0) {
      return false;
    }
    return v.sizeof_type() == get_cdr_size_of_primitive(v.type_kind());
  }

  /**
   * @brief 查找多个值类型是否都可以被简单序列化
   * @param align 对齐大小
   * @param evt 任意值类型指针
   * @return 如果所有值类型都可以简单序列化，则返回true，否则返回false
   */
  bool lookup_many_trivially_serialized(size_t align, const AnyValueType *evt) const {
    align %= max_align;  // 计算对齐余数
    // CLEVERNESS ALERT
    // 我们利用了这样一个事实：如果某个元素在偏移量A和偏移量A+N处对齐，
    // 那么它的元素的对齐要求可以整除A+k*N（k为任意整数）
    return lookup_trivially_serialized(align, evt) &&
           lookup_trivially_serialized((align + evt->sizeof_type()) % max_align, evt);
  }

  /**
   * @brief 计算数组值类型是否可以被简单序列化
   * @param align 对齐大小
   * @param v 数组值类型
   * @return 如果可以简单序列化，则返回true，否则返回false
   */
  bool compute_trivially_serialized(size_t align, const ArrayValueType &v) const {
    auto evt = v.element_value_type();  // 获取元素值类型
    align %= max_align;                 // 计算对齐余数
    // CLEVERNESS ALERT
    // 我们利用了这样一个事实：如果某个元素在偏移量A和偏移量A+N处对齐，
    // 那么它的元素的对齐要求可以整除A+k*N（k为任意整数）
    return compute_trivially_serialized(align, evt) &&
           compute_trivially_serialized((align + evt->sizeof_type()) % max_align, evt);
  }

  /**
   * @brief 查找给定的任意值类型是否可以被简单序列化
   * @param align 对齐大小
   * @param p 任意值类型指针
   * @return 如果可以简单序列化，则返回true，否则返回false
   */
  bool lookup_trivially_serialized(size_t align, const AnyValueType *p) const {
    CacheKey key{align % max_align, p};         // 创建缓存键
    return trivially_serialized_cache.at(key);  // 返回缓存中的结果
  }

  /// \brief 计算是否可以通过简单的memcpy序列化此值
  /// \param align 对齐大小
  /// \param p 任意值类型指针
  /// \return 如果可以通过memcpy序列化，则返回true，否则返回false
  bool compute_trivially_serialized(size_t align, const AnyValueType *p) const {
    // 对齐大小取模最大对齐值
    align %= max_align;

    bool result;
    // 根据值类型进行不同处理
    switch (p->e_value_type()) {
      case EValueType::PrimitiveValueType:
        // 原始值类型
        result = is_trivially_serialized(align, *static_cast<const PrimitiveValueType *>(p));
        break;
      case EValueType::StructValueType:
        // 结构体值类型
        result = is_trivially_serialized(align, *static_cast<const StructValueType *>(p));
        break;
      case EValueType::ArrayValueType:
        // 数组值类型
        result = compute_trivially_serialized(align, *static_cast<const ArrayValueType *>(p));
        break;
      case EValueType::U8StringValueType:
      case EValueType::U16StringValueType:
      case EValueType::SpanSequenceValueType:
      case EValueType::BoolVectorValueType:
        // 字符串、跨度序列和布尔向量值类型
        result = false;
        break;
      default:
        // 不可达代码
        unreachable();
    }
    return result;
  }

  /// \brief 获取原始类型的CDR对齐大小
  /// \param tk ROSIDL类型种类
  /// \return 如果值类型不是原始类型，则返回0；否则返回应该对齐的字节数
  size_t get_cdr_alignof_primitive(ROSIDL_TypeKind tk) const {
    // 如果值类型不是原始类型，则返回0；否则返回应该对齐的字节数
    size_t sizeof_ = get_cdr_size_of_primitive(tk);
    return sizeof_ < max_align ? sizeof_ : max_align;
  }

  /**
   * @brief 将数据序列化到CDRCursor中
   *
   * @param cursor CDRCursor指针，用于存储序列化后的数据
   * @param data 需要序列化的原始数据指针
   * @param value_type 原始数据的类型信息
   */
  void serialize(CDRCursor *cursor, const void *data, const PrimitiveValueType &value_type) const {
    // 根据数据类型获取对齐值，并调整cursor的位置
    cursor->align(get_cdr_alignof_primitive(value_type.type_kind()));

    // 获取数据类型所占用的字节数
    size_t n_bytes = get_cdr_size_of_primitive(value_type.type_kind());

    // 根据数据类型进行相应的序列化操作
    switch (value_type.type_kind()) {
      case ROSIDL_TypeKind::FLOAT:
        // 确保浮点数遵循IEEE 754标准
        assert(std::numeric_limits<float>::is_iec559);
        cursor->put_bytes(data, n_bytes);
        return;
      case ROSIDL_TypeKind::DOUBLE:
        // 确保双精度浮点数遵循IEEE 754标准
        assert(std::numeric_limits<double>::is_iec559);
        cursor->put_bytes(data, n_bytes);
        return;
      case ROSIDL_TypeKind::LONG_DOUBLE:
        // 确保长双精度浮点数遵循IEEE 754标准
        assert(std::numeric_limits<long double>::is_iec559);
        cursor->put_bytes(data, n_bytes);
        return;
      case ROSIDL_TypeKind::CHAR:
      case ROSIDL_TypeKind::WCHAR:
      case ROSIDL_TypeKind::BOOLEAN:
      case ROSIDL_TypeKind::OCTET:
      case ROSIDL_TypeKind::UINT8:
      case ROSIDL_TypeKind::INT8:
      case ROSIDL_TypeKind::UINT16:
      case ROSIDL_TypeKind::INT16:
      case ROSIDL_TypeKind::UINT32:
      case ROSIDL_TypeKind::INT32:
      case ROSIDL_TypeKind::UINT64:
      case ROSIDL_TypeKind::INT64:
        // 如果数据类型大小与字节数相等，或者系统为小端序，则直接写入数据
        if (value_type.sizeof_type() == n_bytes || native_endian() == endian::little) {
          cursor->put_bytes(data, n_bytes);
        } else {
          // 否则，计算偏移量并写入数据
          const void *offset_data = byte_offset(data, value_type.sizeof_type() - n_bytes);
          cursor->put_bytes(offset_data, n_bytes);
        }
        return;
      case ROSIDL_TypeKind::STRING:
      case ROSIDL_TypeKind::WSTRING:
      case ROSIDL_TypeKind::MESSAGE:
      default:
        // 不支持的数据类型，抛出异常
        unreachable();
    }
  }

  /**
   * @brief 将给定的数据序列化为U8字符串类型并写入CDRCursor中
   * @param cursor CDRCursor指针，用于存储序列化后的数据
   * @param data 需要序列化的原始数据
   * @param value_type U8StringValueType类型，用于获取字符串数据
   */
  void serialize(CDRCursor *cursor, const void *data, const U8StringValueType &value_type) const {
    auto str = value_type.data(data);       // 获取字符串数据
    serialize_u32(cursor, str.size() + 1);  // 序列化字符串长度并加1（包括空字符）
    cursor->put_bytes(str.data(), str.size());  // 将字符串数据写入cursor
    char terminator = '\0';                     // 定义字符串终止符
    cursor->put_bytes(&terminator, 1);          // 将终止符写入cursor
  }

  /**
   * @brief 将给定的数据序列化为U16字符串类型并写入CDRCursor中
   * @param cursor CDRCursor指针，用于存储序列化后的数据
   * @param data 需要序列化的原始数据
   * @param value_type U16StringValueType类型，用于获取字符串数据
   */
  void serialize(CDRCursor *cursor, const void *data, const U16StringValueType &value_type) const {
    auto str = value_type.data(data);                   // 获取字符串数据
    if (eversion == EncodingVersion::CDR_Legacy) {
      serialize_u32(cursor, str.size());                // 序列化字符串长度
      if (cursor->ignores_data()) {
        cursor->advance(sizeof(wchar_t) * str.size());  // 忽略数据时，直接移动cursor位置
      } else {
        for (wchar_t c : str) {                         // 遍历字符串中的每个字符
          cursor->put_bytes(&c, sizeof(wchar_t));       // 将字符写入cursor
        }
      }
    } else {
      serialize_u32(cursor, str.size_bytes());          // 序列化字符串字节长度
      cursor->put_bytes(str.data(), str.size_bytes());  // 将字符串数据写入cursor
    }
  }

  /**
   * @brief 将给定的数组数据序列化并写入CDRCursor中
   * @param cursor CDRCursor指针，用于存储序列化后的数据
   * @param data 需要序列化的原始数据
   * @param value_type ArrayValueType类型，用于获取数组数据和元素类型
   */
  void serialize(CDRCursor *cursor, const void *data, const ArrayValueType &value_type) const {
    serialize_many(cursor, value_type.get_data(data), value_type.array_size(),
                   value_type.element_value_type());  // 调用serialize_many函数进行数组序列化
  }

  /**
   * @brief 将给定的SpanSequence数据序列化并写入CDRCursor中
   * @param cursor CDRCursor指针，用于存储序列化后的数据
   * @param data 需要序列化的原始数据
   * @param value_type SpanSequenceValueType类型，用于获取序列数据和元素类型
   */
  void serialize(CDRCursor *cursor,
                 const void *data,
                 const SpanSequenceValueType &value_type) const {
    size_t count = value_type.sequence_size(data);  // 获取序列大小
    serialize_u32(cursor, count);                   // 序列化序列大小
    serialize_many(cursor, value_type.sequence_contents(data), count,
                   value_type.element_value_type());  // 调用serialize_many函数进行序列数据序列化
  }

  /**
   * @brief 对数据进行序列化
   * @param cursor CDRCursor指针，用于操作序列化过程中的游标
   * @param data 需要序列化的数据指针
   * @param value_type BoolVectorValueType类型的引用，表示数据的类型信息
   */
  void serialize(CDRCursor *cursor, const void *data, const BoolVectorValueType &value_type) const {
    // 获取数据中布尔值的数量
    size_t count = value_type.size(data);
    // 序列化布尔值的数量
    serialize_u32(cursor, count);
    // 如果游标忽略数据，则直接移动游标
    if (cursor->ignores_data()) {
      cursor->advance(count);
    } else {
      // 遍历布尔值并逐个序列化
      for (auto iter = value_type.begin(data); iter != value_type.end(data); ++iter) {
        bool b = *iter;
        cursor->put_bytes(&b, 1);
      }
    }
  }

  /**
   * @brief 对任意类型的数据进行序列化
   * @param cursor CDRCursor指针，用于操作序列化过程中的游标
   * @param data 需要序列化的数据指针
   * @param value_type AnyValueType指针，表示数据的类型信息
   */
  void serialize(CDRCursor *cursor, const void *data, const AnyValueType *value_type) const {
    // 如果数据类型可以简单地序列化，则直接序列化
    if (lookup_trivially_serialized(cursor->offset(), value_type)) {
      cursor->put_bytes(data, value_type->sizeof_type());
    } else {
      // 根据数据类型的具体子类，调用相应的序列化方法
      if (auto s = dynamic_cast<const PrimitiveValueType *>(value_type)) {
        return serialize(cursor, data, *s);
      }
      if (auto s = dynamic_cast<const U8StringValueType *>(value_type)) {
        return serialize(cursor, data, *s);
      }
      if (auto s = dynamic_cast<const U16StringValueType *>(value_type)) {
        return serialize(cursor, data, *s);
      }
      if (auto s = dynamic_cast<const StructValueType *>(value_type)) {
        return serialize(cursor, data, *s);
      }
      if (auto s = dynamic_cast<const ArrayValueType *>(value_type)) {
        return serialize(cursor, data, *s);
      }
      if (auto s = dynamic_cast<const SpanSequenceValueType *>(value_type)) {
        return serialize(cursor, data, *s);
      }
      if (auto s = dynamic_cast<const BoolVectorValueType *>(value_type)) {
        return serialize(cursor, data, *s);
      }
      // 如果没有匹配到任何已知类型，则抛出异常
      /*
        `unreachable()` 是一个表示代码不应该到达这个点的函数或宏。在这个例子中，`unreachable()`
        表示在执行到这一行时，程序已经尝试了所有已知的数据类型，并且没有找到匹配的序列化方法。因此，如果代码执行到这里，说明发生了错误或者遗漏了某种数据类型的处理。
        通常情况下，`unreachable()`
        会抛出异常、终止程序或触发断言失败，以便开发人员能够注意到这个问题并进行调查。在实际项目中，`unreachable()`
        可能是自定义的函数或宏，具体实现可能因项目而异。
      */
      unreachable();
    }
  }

  /**
   * @brief 将多个数据序列化到CDRCursor中
   *
   * @param cursor CDRCursor指针，用于存储序列化后的数据
   * @param data 需要序列化的数据的指针
   * @param count 需要序列化的数据的数量
   * @param vt 数据类型的AnyValueType指针
   */
  void serialize_many(CDRCursor *cursor,
                      const void *data,
                      size_t count,
                      const AnyValueType *vt) const {
    // 如果没有数据需要处理，直接返回
    if (count == 0) {
      return;
    }

    // 序列化第一个元素
    serialize(cursor, data, vt);

    // 如果值类型是基本类型，那么现在已经对齐了
    // 可能第一个元素不是简单地序列化，但其余的是；
    // 例如，如果结构体中的任何元素具有比第一个元素更严格的CDR对齐方式

    data = byte_offset(data, vt->sizeof_type());
    --count;
    if (count == 0) {
      return;
    }

    // 检查是否可以进行简单序列化
    if (lookup_many_trivially_serialized(cursor->offset(), vt)) {
      size_t value_size = vt->sizeof_type();
      cursor->put_bytes(data, count * value_size);
      return;
    } else {
      // 对剩余的每个元素进行序列化
      for (size_t i = 0; i < count; i++) {
        auto element = byte_offset(data, i * vt->sizeof_type());
        serialize(cursor, element, vt);
      }
    }
  }

  /**
   * @brief 将结构体数据序列化到CDRCursor中
   *
   * @param cursor CDRCursor指针，用于存储序列化后的数据
   * @param struct_data 需要序列化的结构体数据的指针
   * @param struct_info 结构体类型的StructValueType引用
   */
  void serialize(CDRCursor *cursor,
                 const void *struct_data,
                 const StructValueType &struct_info) const {
    // 遍历结构体的每个成员并进行序列化
    for (size_t i = 0; i < struct_info.n_members(); i++) {
      auto member_info = struct_info.get_member(i);
      auto value_type = member_info->value_type;
      auto member_data = byte_offset(struct_data, member_info->member_offset);
      serialize(cursor, member_data, value_type);
    }
  }
};

/**
 * @brief 创建一个 CDRWriter 对象并返回其 unique_ptr
 *
 * 这个函数接收一个 `StructValueType` 的 unique_ptr 作为参数，然后创建一个
 * `CDRWriter` 对象，并将这个 `StructValueType` 移动到新创建的对象中。
 * 最后，返回这个新创建的 `CDRWriter` 对象的 unique_ptr。
 *
 * @param value_type 一个指向 `StructValueType` 对象的 unique_ptr
 * @return 返回一个指向 `BaseCDRWriter` 对象的 unique_ptr
 */
std::unique_ptr<BaseCDRWriter> make_cdr_writer(std::unique_ptr<StructValueType> value_type) {
  // 使用 std::make_unique 创建一个 CDRWriter 对象，并将 value_type 移动到其中
  return std::make_unique<CDRWriter>(std::move(value_type));
}

}  // namespace rmw_cyclonedds_cpp
