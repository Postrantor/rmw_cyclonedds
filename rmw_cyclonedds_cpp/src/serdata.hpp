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
#ifndef SERDATA_HPP_
#define SERDATA_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "TypeSupport2.hpp"
#include "bytewise.hpp"
#include "dds/dds.h"
#include "dds/ddsi/ddsi_serdata.h"

#ifdef DDS_HAS_SHM                        // 如果定义了DDS_HAS_SHM宏
extern "C" {                              // 使用C语言链接方式
#include "dds/ddsi/ddsi_shm_transport.h"  // 包含共享内存传输头文件
}
#endif                                    // DDS_HAS_SHM // 结束DDS_HAS_SHM宏判断

#if !DDS_HAS_DDSI_SERTYPE
#define ddsi_sertype ddsi_sertopic
#define ddsi_sertype_ops ddsi_sertopic_ops
#define sertype_rmw sertopic_rmw
#define sertype_rmw_ops sertopic_rmw_ops
#endif

namespace rmw_cyclonedds_cpp {
class BaseCDRWriter;
}

/**
 * @struct CddsTypeSupport
 * @brief Cyclone DDS 类型支持结构体
 */
struct CddsTypeSupport {
  void* type_support_;                  ///< 类型支持指针，用于存储类型支持的实例
  const char* typesupport_identifier_;  ///< 类型支持标识符，用于表示类型支持的唯一名称
};

/**
 * @struct sertype_rmw
 * @brief RMW 序列化类型结构体，继承自 ddsi_sertype
 */
struct sertype_rmw : ddsi_sertype {
  CddsTypeSupport type_support;  ///< Cyclone DDS 类型支持实例
  bool is_request_header;        ///< 是否为请求头部，用于区分请求和响应
  std::unique_ptr<const rmw_cyclonedds_cpp::BaseCDRWriter>
      cdr_writer;                ///< CDR 写入器，用于序列化和反序列化数据
  bool is_fixed;                 ///< 是否为固定大小的数据类型
  std::mutex serialize_lock;     ///< 序列化锁，用于确保线程安全的序列化操作
};

/**
 * @class serdata_rmw
 * @brief 继承自 ddsi_serdata 的类，用于序列化和反序列化数据
 */
class serdata_rmw : public ddsi_serdata {
 protected:
  // 数据大小
  size_t m_size{0};

  /*
   * 数据的前两个字节是 CDR 编码
   * 第二个两个字节是编码选项
   */
  std::unique_ptr<byte[]> m_data{nullptr};

 public:
  /**
   * @brief 构造函数
   * @param type 指向 ddsi_sertype 类型的指针
   * @param kind ddsi_serdata_kind 枚举类型，表示序列化数据的种类
   */
  serdata_rmw(const ddsi_sertype* type, ddsi_serdata_kind kind);

  /**
   * @brief 调整数据大小
   * @param requested_size 请求的新大小
   */
  void resize(size_t requested_size);

  /**
   * @brief 获取数据大小
   * @return 返回数据大小
   */
  size_t size() const { return m_size; }

  /**
   * @brief 获取数据指针
   * @return 返回数据指针
   */
  void* data() const { return m_data.get(); }
};

/**
 * @struct cdds_request_header
 * @brief 请求头结构体
 */
typedef struct cdds_request_header {
  uint64_t guid;  ///< 全局唯一标识符
  int64_t seq;    ///< 序列号
} cdds_request_header_t;

/**
 * @struct cdds_request_wrapper
 * @brief 请求包装结构体
 */
typedef struct cdds_request_wrapper {
  cdds_request_header_t header;  ///< 请求头
  void* data;                    ///< 数据指针
} cdds_request_wrapper_t;

/**
 * @brief 创建消息类型支持
 *
 * @param[in] untyped_members 未类型化的成员指针
 * @param[in] typesupport_identifier 类型支持标识符
 * @return void* 指向创建的消息类型支持的指针
 */
void* create_message_type_support(const void* untyped_members, const char* typesupport_identifier);

/**
 * @brief 创建请求类型支持
 *
 * @param[in] untyped_members 未类型化的成员指针
 * @param[in] typesupport_identifier 类型支持标识符
 * @return void* 指向创建的请求类型支持的指针
 */
void* create_request_type_support(const void* untyped_members, const char* typesupport_identifier);

/**
 * @brief 创建响应类型支持
 *
 * @param[in] untyped_members 未类型化的成员指针
 * @param[in] typesupport_identifier 类型支持标识符
 * @return void* 指向创建的响应类型支持的指针
 */
void* create_response_type_support(const void* untyped_members, const char* typesupport_identifier);

/**
 * @brief 创建序列化类型
 *
 * @param[in] type_support_identifier 类型支持标识符
 * @param[in] type_support 类型支持指针
 * @param[in] is_request_header 是否为请求头
 * @param[in] message_type_support 消息类型支持的智能指针
 * @param[in] sample_size 样本大小，默认值为0
 * @param[in] is_fixed_type 是否为固定类型，默认值为false
 * @return struct sertype_rmw* 指向创建的序列化类型的指针
 */
struct sertype_rmw* create_sertype(
    const char* type_support_identifier,
    void* type_support,
    bool is_request_header,
    std::unique_ptr<rmw_cyclonedds_cpp::StructValueType> message_type_support,
    const uint32_t sample_size = 0U,
    const bool is_fixed_type = false);

/**
 * @brief 从序列化消息创建序列化数据
 *
 * @param[in] typecmn 序列化类型公共结构体指针
 * @param[in] raw 原始序列化消息指针
 * @param[in] size 序列化消息大小
 * @return struct ddsi_serdata* 指向创建的序列化数据的指针
 */
struct ddsi_serdata* serdata_rmw_from_serialized_message(const struct ddsi_sertype* typecmn,
                                                         const void* raw,
                                                         size_t size);

#endif  // SERDATA_HPP_
