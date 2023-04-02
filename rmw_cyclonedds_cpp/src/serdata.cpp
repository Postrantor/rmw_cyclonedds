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
#include "serdata.hpp"

#include <cstring>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <utility>

#include "MessageTypeSupport.hpp"
#include "Serialization.hpp"
#include "ServiceTypeSupport.hpp"
#include "TypeSupport2.hpp"
#include "bytewise.hpp"
#include "dds/ddsi/q_radmin.h"
#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "serdes.hpp"

/// 使用C语言的rosidl类型支持内省消息成员定义TypeSupport_c别名
using TypeSupport_c =
    rmw_cyclonedds_cpp::TypeSupport<rosidl_typesupport_introspection_c__MessageMembers>;
/// 使用C++语言的rosidl类型支持内省消息成员定义TypeSupport_cpp别名
using TypeSupport_cpp =
    rmw_cyclonedds_cpp::TypeSupport<rosidl_typesupport_introspection_cpp::MessageMembers>;
/// 使用C语言的rosidl类型支持内省消息成员定义MessageTypeSupport_c别名
using MessageTypeSupport_c =
    rmw_cyclonedds_cpp::MessageTypeSupport<rosidl_typesupport_introspection_c__MessageMembers>;
/// 使用C++语言的rosidl类型支持内省消息成员定义MessageTypeSupport_cpp别名
using MessageTypeSupport_cpp =
    rmw_cyclonedds_cpp::MessageTypeSupport<rosidl_typesupport_introspection_cpp::MessageMembers>;
/// 使用C语言的rosidl类型支持内省服务成员和消息成员定义RequestTypeSupport_c别名
using RequestTypeSupport_c =
    rmw_cyclonedds_cpp::RequestTypeSupport<rosidl_typesupport_introspection_c__ServiceMembers,
                                           rosidl_typesupport_introspection_c__MessageMembers>;
/// 使用C++语言的rosidl类型支持内省服务成员和消息成员定义RequestTypeSupport_cpp别名
using RequestTypeSupport_cpp =
    rmw_cyclonedds_cpp::RequestTypeSupport<rosidl_typesupport_introspection_cpp::ServiceMembers,
                                           rosidl_typesupport_introspection_cpp::MessageMembers>;
/// 使用C语言的rosidl类型支持内省服务成员和消息成员定义ResponseTypeSupport_c别名
using ResponseTypeSupport_c =
    rmw_cyclonedds_cpp::ResponseTypeSupport<rosidl_typesupport_introspection_c__ServiceMembers,
                                            rosidl_typesupport_introspection_c__MessageMembers>;
/// 使用C++语言的rosidl类型支持内省服务成员和消息成员定义ResponseTypeSupport_cpp别名
using ResponseTypeSupport_cpp =
    rmw_cyclonedds_cpp::ResponseTypeSupport<rosidl_typesupport_introspection_cpp::ServiceMembers,
                                            rosidl_typesupport_introspection_cpp::MessageMembers>;

/**
 * @brief 检查是否使用 C 语言的自省类型支持
 *
 * @param typesupport_identifier 类型支持标识符
 * @return true 如果使用 C 语言的自省类型支持
 * @return false 如果不使用 C 语言的自省类型支持
 */
static bool using_introspection_c_typesupport(const char *typesupport_identifier) {
  // 比较类型支持标识符与 C 语言自省类型支持的标识符
  return typesupport_identifier == rosidl_typesupport_introspection_c__identifier;
}

/**
 * @brief 检查是否使用 C++ 语言的自省类型支持
 *
 * @param typesupport_identifier 类型支持标识符
 * @return true 如果使用 C++ 语言的自省类型支持
 * @return false 如果不使用 C++ 语言的自省类型支持
 */
static bool using_introspection_cpp_typesupport(const char *typesupport_identifier) {
  // 比较类型支持标识符与 C++ 语言自省类型支持的标识符
  return typesupport_identifier == rosidl_typesupport_introspection_cpp::typesupport_identifier;
}

/**
 * @brief 创建消息类型支持
 *
 * @param untyped_members 未类型化的成员指针
 * @param typesupport_identifier 类型支持标识符
 * @return void* 指向创建的消息类型支持对象的指针
 */
void *create_message_type_support(const void *untyped_members, const char *typesupport_identifier) {
  // 判断是否使用 C 语言的自省类型支持
  if (using_introspection_c_typesupport(typesupport_identifier)) {
    // 类型转换为 C 语言自省类型支持的消息成员指针
    auto members =
        static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(untyped_members);
    // 创建并返回 C 语言消息类型支持对象
    return new MessageTypeSupport_c(members);
  } else if (using_introspection_cpp_typesupport(typesupport_identifier)) {
    // 类型转换为 C++ 语言自省类型支持的消息成员指针
    auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(untyped_members);
    // 创建并返回 C++ 语言消息类型支持对象
    return new MessageTypeSupport_cpp(members);
  }
  // 设置错误信息
  RMW_SET_ERROR_MSG("Unknown typesupport identifier");
  // 返回空指针
  return nullptr;
}

/**
 * @brief 创建请求类型支持
 *
 * @param untyped_members 未类型化的成员指针
 * @param typesupport_identifier 类型支持标识符
 * @return void* 指向创建的请求类型支持对象的指针
 */
void *create_request_type_support(const void *untyped_members, const char *typesupport_identifier) {
  // 判断是否使用 C 语言的自省类型支持
  if (using_introspection_c_typesupport(typesupport_identifier)) {
    // 类型转换为 C 语言自省类型支持的服务成员指针
    auto members =
        static_cast<const rosidl_typesupport_introspection_c__ServiceMembers *>(untyped_members);
    // 创建并返回 C 语言请求类型支持对象
    return new RequestTypeSupport_c(members);
  } else if (using_introspection_cpp_typesupport(typesupport_identifier)) {
    // 类型转换为 C++ 语言自省类型支持的服务成员指针
    auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(untyped_members);
    // 创建并返回 C++ 语言请求类型支持对象
    return new RequestTypeSupport_cpp(members);
  }
  // 设置错误信息
  RMW_SET_ERROR_MSG("Unknown typesupport identifier");
  // 返回空指针
  return nullptr;
}

/**
 * @brief 创建响应类型支持对象
 *
 * @param[in] untyped_members 未类型化的服务成员指针
 * @param[in] typesupport_identifier 类型支持标识符
 * @return void* 返回创建的响应类型支持对象指针，如果失败则返回nullptr
 */
void *create_response_type_support(const void *untyped_members,
                                   const char *typesupport_identifier) {
  // 判断是否使用C语言的内省类型支持
  if (using_introspection_c_typesupport(typesupport_identifier)) {
    // 将未类型化的服务成员转换为C语言内省类型的服务成员
    auto members =
        static_cast<const rosidl_typesupport_introspection_c__ServiceMembers *>(untyped_members);
    // 创建并返回C语言响应类型支持对象
    return new ResponseTypeSupport_c(members);
  } else if (using_introspection_cpp_typesupport(typesupport_identifier)) {
    // 将未类型化的服务成员转换为C++语言内省类型的服务成员
    auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(untyped_members);
    // 创建并返回C++语言响应类型支持对象
    return new ResponseTypeSupport_cpp(members);
  }
  // 设置错误消息：未知的类型支持标识符
  RMW_SET_ERROR_MSG("Unknown typesupport identifier");
  // 返回空指针
  return nullptr;
}

/**
 * @brief 将样本序列化到serdata_rmw中
 *
 * @param[in,out] d serdata_rmw对象指针
 * @param[in] sample 样本数据指针
 */
static void serialize_into_serdata_rmw(serdata_rmw *d, const void *sample) {
  // 将d->type转换为sertype_rmw类型的常量指针
  const struct sertype_rmw *type = static_cast<const struct sertype_rmw *>(d->type);
  try {
    // 判断数据类型是否为SDK_DATA
    if (d->kind != SDK_DATA) {
      // ROS 2不使用键，因此SDK_KEY是简单的
    } else if (!type->is_request_header) {
      // 获取序列化后的大小
      size_t sz = type->cdr_writer->get_serialized_size(sample);
      // 调整serdata_rmw对象的大小
      d->resize(sz);
      // 将样本序列化到serdata_rmw对象中
      type->cdr_writer->serialize(d->data(), sample);
    } else {
      // 将服务调用头数据注入CDR流
      auto wrap = *static_cast<const cdds_request_wrapper_t *>(sample);
      // 获取序列化后的大小
      size_t sz = type->cdr_writer->get_serialized_size(wrap);
      // 调整serdata_rmw对象的大小
      d->resize(sz);
      // 将样本序列化到serdata_rmw对象中
      type->cdr_writer->serialize(d->data(), wrap);
    }
  } catch (std::exception &e) {
    // 设置错误消息
    RMW_SET_ERROR_MSG(e.what());
  }
}

/**
 * @brief 将数据序列化到 serdata_rmw 结构中，支持按需序列化。
 *
 * @param[in] d 指向要序列化的 serdata_rmw 结构的指针。
 */
static void serialize_into_serdata_rmw_on_demand(serdata_rmw *d) {
#ifdef DDS_HAS_SHM
  // 获取类型并去除 const 属性
  auto type = const_cast<sertype_rmw *>(static_cast<const sertype_rmw *>(d->type));
  {
    // 使用锁保护序列化过程
    std::lock_guard<std::mutex> lock(type->serialize_lock);
    // 如果存在 iox_chunk 且 data() 为空
    if (d->iox_chunk && d->data() == nullptr) {
      // 从 iox_chunk 中获取 iceoryx 头部信息
      auto iox_header = iceoryx_header_from_chunk(d->iox_chunk);
      // 如果 iox chunk 中包含序列化后的数据
      if (iox_header->shm_data_state == IOX_CHUNK_CONTAINS_SERIALIZED_DATA) {
        // 调整 d 的大小并将数据从 iox_chunk 复制到 d->data()
        d->resize(iox_header->data_size);
        memcpy(d->data(), d->iox_chunk, iox_header->data_size);
      } else if (iox_header->shm_data_state == IOX_CHUNK_CONTAINS_RAW_DATA) {
        // 如果 iox chunk 中包含原始数据，则进行序列化
        serialize_into_serdata_rmw(const_cast<serdata_rmw *>(d), d->iox_chunk);
      } else {
        // 如果接收到的 iox chunk 未初始化，则设置错误信息
        RMW_SET_ERROR_MSG("Received iox chunk is uninitialized");
      }
    }
  }
#endif
  (void)d;
}

/**
 * @brief 获取 serdata_rmw 结构的大小。
 *
 * @param[in] dcmn 指向要获取大小的 ddsi_serdata 结构的指针。
 * @return 返回 uint32_t 类型的 serdata_rmw 结构大小。
 */
static uint32_t serdata_rmw_size(const struct ddsi_serdata *dcmn) {
  // 将 dcmn 转换为 serdata_rmw 类型
  auto d = static_cast<const serdata_rmw *>(dcmn);
  // 按需序列化数据
  serialize_into_serdata_rmw_on_demand(const_cast<serdata_rmw *>(d));
  // 获取序列化后的数据大小
  size_t size = d->size();
  // 将 size_t 类型转换为 uint32_t 类型
  uint32_t size_u32 = static_cast<uint32_t>(size);
  // 确保转换后的大小与原始大小相等
  assert(size == size_u32);
  // 返回转换后的大小
  return size_u32;
}

/**
 * @brief 释放serdata_rmw结构体的内存
 *
 * @param[in] dcmn 指向要释放的ddsi_serdata结构体的指针
 */
static void serdata_rmw_free(struct ddsi_serdata *dcmn) {
  // 将 dcmn 转换为 serdata_rmw 类型的指针
  auto *d = static_cast<serdata_rmw *>(dcmn);

#ifdef DDS_HAS_SHM
  // 如果存在共享内存块和订阅者，释放共享内存块
  if (d->iox_chunk && d->iox_subscriber) {
    free_iox_chunk(static_cast<iox_sub_t *>(d->iox_subscriber), &d->iox_chunk);
    d->iox_chunk = nullptr;
  }
#endif
  // 删除 serdata_rmw 结构体
  delete d;
}

/**
 * @brief 从序列化数据创建一个serdata_rmw实例
 *
 * @param[in] type 序列化数据类型
 * @param[in] kind 序列化数据种类
 * @param[in] fragchain 分片链表
 * @param[in] size 数据大小
 * @return 返回一个新创建的serdata_rmw实例，如果出现异常则返回nullptr
 */
static struct ddsi_serdata *serdata_rmw_from_ser(const struct ddsi_sertype *type,
                                                 enum ddsi_serdata_kind kind,
                                                 const struct nn_rdata *fragchain,
                                                 size_t size) {
  try {
    // 创建一个新的 serdata_rmw 实例
    auto d = std::make_unique<serdata_rmw>(type, kind);
    uint32_t off = 0;
    // 检查分片链表的最小值是否为0
    assert(fragchain->min == 0);
    // 检查分片链表的最大值加1是否大于等于偏移量，CDR头必须在第一个分片中
    assert(fragchain->maxp1 >= off);
    // 调整serdata_rmw实例的大小
    d->resize(size);

    // 获取数据指针
    auto cursor = d->data();
    // 遍历分片链表
    while (fragchain) {
      if (fragchain->maxp1 > off) {
        // 只有当此分片添加数据时才进行复制
        const unsigned char *payload =
            NN_RMSG_PAYLOADOFF(fragchain->rmsg, NN_RDATA_PAYLOAD_OFF(fragchain));
        auto src = payload + off - fragchain->min;
        auto n_bytes = fragchain->maxp1 - off;
        // 复制数据
        memcpy(cursor, src, n_bytes);
        cursor = byte_offset(cursor, n_bytes);
        off = fragchain->maxp1;
        // 确保偏移量不超过数据大小
        assert(off <= size);
      }
      // 移动到下一个分片
      fragchain = fragchain->nextfrag;
    }
    // 返回创建的 serdata_rmw 实例
    return d.release();
  } catch (std::exception &e) {
    // 设置错误信息并返回nullptr
    RMW_SET_ERROR_MSG(e.what());
    return nullptr;
  }
}

/**
 * @brief 从序列化的数据向量中创建一个serdata_rmw对象
 *
 * @param type 序列化类型
 * @param kind 序列化数据种类
 * @param niov 数据向量的长度
 * @param iov 数据向量
 * @param size 数据大小
 * @return 返回一个新创建的ddsi_serdata对象，如果出现异常则返回nullptr
 */
static struct ddsi_serdata *serdata_rmw_from_ser_iov(const struct ddsi_sertype *type,
                                                     enum ddsi_serdata_kind kind,
                                                     ddsrt_msg_iovlen_t niov,
                                                     const ddsrt_iovec_t *iov,
                                                     size_t size) {
  try {
    // 创建一个serdata_rmw对象并设置其类型和种类
    auto d = std::make_unique<serdata_rmw>(type, kind);
    // 调整serdata_rmw对象的大小以适应数据
    d->resize(size);

    // 获取serdata_rmw对象的数据指针
    auto cursor = d->data();
    // 遍历数据向量并将其内容复制到serdata_rmw对象中
    for (ddsrt_msg_iovlen_t i = 0; i < niov; i++) {
      memcpy(cursor, iov[i].iov_base, iov[i].iov_len);
      cursor = byte_offset(cursor, iov[i].iov_len);
    }
    // 返回serdata_rmw对象的指针
    return d.release();
  } catch (std::exception &e) {
    RMW_SET_ERROR_MSG(e.what());
    return nullptr;
  }
}

/**
 * @brief 从keyhash创建一个serdata_rmw对象
 *
 * @param type 序列化类型
 * @param keyhash 键值哈希
 * @return 返回一个新创建的ddsi_serdata对象
 */
static struct ddsi_serdata *serdata_rmw_from_keyhash(const struct ddsi_sertype *type,
                                                     const struct ddsi_keyhash *keyhash) {
  static_cast<void>(keyhash);  // unused
  // 没有关键字段，所以从_keyhash创建是简单的
  return new serdata_rmw(type, SDK_KEY);
}

/**
 * @brief 从样本中创建一个serdata_rmw对象
 *
 * @param typecmn 序列化类型
 * @param kind 序列化数据种类
 * @param sample 样本数据
 * @return 返回一个新创建的ddsi_serdata对象，如果出现异常则返回nullptr
 */
static struct ddsi_serdata *serdata_rmw_from_sample(const struct ddsi_sertype *typecmn,
                                                    enum ddsi_serdata_kind kind,
                                                    const void *sample) {
  try {
    // 将通用序列化类型转换为特定的sertype_rmw类型
    const struct sertype_rmw *type = static_cast<const struct sertype_rmw *>(typecmn);
    // 创建一个serdata_rmw对象并设置其类型和种类
    auto d = std::make_unique<serdata_rmw>(type, kind);
    // 将样本数据序列化到serdata_rmw对象中
    serialize_into_serdata_rmw(d.get(), sample);
    // 返回serdata_rmw对象的指针
    return d.release();
  } catch (std::exception &e) {
    RMW_SET_ERROR_MSG(e.what());
    return nullptr;
  }
}

#ifdef DDS_HAS_SHM
/**
 * @brief 从 Iceoryx 缓冲区创建一个序列化数据对象
 *
 * @param[in] typecmn 通用的序列化类型对象
 * @param[in] kind 序列化数据的种类（SDK_KEY 或 SDK_DATA）
 * @param[in] sub 订阅者指针
 * @param[in] iox_buffer Iceoryx 缓冲区指针
 * @return 返回一个新创建的 ddsi_serdata 对象，如果出现异常则返回 nullptr
 */
static struct ddsi_serdata *serdata_rmw_from_iox(const struct ddsi_sertype *typecmn,
                                                 enum ddsi_serdata_kind kind,
                                                 void *sub,
                                                 void *iox_buffer) {
  try {
    // 将通用类型转换为特定类型
    const struct sertype_rmw *type = static_cast<const struct sertype_rmw *>(typecmn);
    // 创建一个新的 serdata_rmw 对象
    auto d = std::make_unique<serdata_rmw>(type, kind);
    // 设置 Iceoryx 缓冲区
    d->iox_chunk = iox_buffer;
    // 设置订阅者
    d->iox_subscriber = sub;
    // 返回创建的对象
    return d.release();
  } catch (std::exception &e) {
    // 设置错误信息并返回 nullptr
    RMW_SET_ERROR_MSG(e.what());
    return nullptr;
  }
}
#endif  // DDS_HAS_SHM

/**
 * @brief 从序列化消息创建一个序列化数据对象
 *
 * @param[in] typecmn 通用的序列化类型对象
 * @param[in] raw 序列化消息的原始数据指针
 * @param[in] size 序列化消息的大小
 * @return 返回一个新创建的 ddsi_serdata 对象，如果出现异常则返回 nullptr
 */
struct ddsi_serdata *serdata_rmw_from_serialized_message(const struct ddsi_sertype *typecmn,
                                                         const void *raw,
                                                         size_t size) {
  // 初始化 iov 结构体
  ddsrt_iovec_t iov;
  iov.iov_len = static_cast<ddsrt_iov_len_t>(size);
  if (iov.iov_len != size) {
    return nullptr;
  }
  iov.iov_base = const_cast<void *>(raw);
  // 从序列化数据创建 ddsi_serdata 对象
  return ddsi_serdata_from_ser_iov(typecmn, SDK_DATA, 1, &iov, size);
}

/**
 * @brief 将序列化数据对象转换为未类型化的序列化数据对象
 *
 * @param[in] dcmn 输入的序列化数据对象
 * @return 返回一个新创建的未类型化的 ddsi_serdata 对象
 */
static struct ddsi_serdata *serdata_rmw_to_untyped(const struct ddsi_serdata *dcmn) {
  // 将通用类型转换为特定类型
  auto d = static_cast<const serdata_rmw *>(dcmn);
#if DDS_HAS_DDSI_SERTYPE
  // 创建一个新的未类型化的 serdata_rmw 对象
  auto d1 = new serdata_rmw(d->type, SDK_KEY);
  d1->type = nullptr;
#else
  // 创建一个新的未类型化的 serdata_rmw 对象
  auto d1 = new serdata_rmw(d->topic, SDK_KEY);
  d1->topic = nullptr;
#endif
  // 返回创建的对象
  return d1;
}

/**
 * @brief 将 ddsi_serdata 数据序列化到缓冲区
 *
 * @param[in] dcmn 输入的 ddsi_serdata 结构体指针
 * @param[in] off 偏移量
 * @param[in] sz 序列化数据的大小
 * @param[out] buf 输出缓冲区，用于存储序列化后的数据
 */
static void serdata_rmw_to_ser(const struct ddsi_serdata *dcmn, size_t off, size_t sz, void *buf) {
  // 将 dcmn 转换为 serdata_rmw 类型的指针
  auto d = static_cast<const serdata_rmw *>(dcmn);

  // 对 d 进行按需序列化
  serialize_into_serdata_rmw_on_demand(const_cast<serdata_rmw *>(d));

  // 将序列化后的数据从偏移量 off 处开始复制到输出缓冲区 buf
  memcpy(buf, byte_offset(d->data(), off), sz);
}

/**
 * @brief 将 ddsi_serdata 数据序列化并返回引用
 *
 * @param[in] dcmn 输入的 ddsi_serdata 结构体指针
 * @param[in] off 偏移量
 * @param[in] sz 序列化数据的大小
 * @param[out] ref 输出的序列化数据引用
 * @return 返回序列化后的 ddsi_serdata 结构体指针
 */
static struct ddsi_serdata *serdata_rmw_to_ser_ref(const struct ddsi_serdata *dcmn,
                                                   size_t off,
                                                   size_t sz,
                                                   ddsrt_iovec_t *ref) {
  // 将 dcmn 转换为 serdata_rmw 类型的指针
  auto d = static_cast<const serdata_rmw *>(dcmn);

  // 对 d 进行按需序列化
  serialize_into_serdata_rmw_on_demand(const_cast<serdata_rmw *>(d));

  // 设置序列化数据引用的基地址和长度
  ref->iov_base = byte_offset(d->data(), off);
  ref->iov_len = (ddsrt_iov_len_t)sz;

  // 返回序列化后的 ddsi_serdata 结构体指针
  return ddsi_serdata_ref(d);
}

/**
 * @brief 取消对序列化数据的引用
 *
 * @param[in] dcmn 输入的 ddsi_serdata 结构体指针
 * @param[in] ref 序列化数据引用（未使用）
 */
static void serdata_rmw_to_ser_unref(struct ddsi_serdata *dcmn, const ddsrt_iovec_t *ref) {
  // 忽略未使用的参数 ref
  static_cast<void>(ref);  // unused

  // 取消对序列化数据的引用
  ddsi_serdata_unref(static_cast<serdata_rmw *>(dcmn));
}

/**
 * @brief 将 serdata_rmw 数据转换为样本数据
 *
 * @param[in] dcmn 输入的通用序列化数据
 * @param[out] sample 输出的样本数据
 * @param[in,out] bufptr 缓冲区指针（未使用）
 * @param[in] buflim 缓冲区限制（未使用）
 * @return 成功时返回 true，失败时返回 false
 */
static bool serdata_rmw_to_sample(const struct ddsi_serdata *dcmn,
                                  void *sample,
                                  void **bufptr,
                                  void *buflim) {
  try {
    static_cast<void>(bufptr);                        // 未使用
    static_cast<void>(buflim);                        // 未使用
    auto d = static_cast<const serdata_rmw *>(dcmn);  // 类型转换

#if DDS_HAS_DDSI_SERTYPE
    const struct sertype_rmw *type = static_cast<const struct sertype_rmw *>(d->type);
#else
    const struct sertopic_rmw *type = static_cast<const struct sertopic_rmw *>(d->topic);
#endif
    assert(bufptr == NULL);
    assert(buflim == NULL);
    if (d->kind != SDK_DATA) {
      /* ROS 2 还没有以有意义的方式实现键 */
    } else if (!type->is_request_header) {
      serialize_into_serdata_rmw_on_demand(const_cast<serdata_rmw *>(d));
      cycdeser sd(d->data(), d->size());
      if (using_introspection_c_typesupport(type->type_support.typesupport_identifier_)) {
        auto typed_typesupport =
            static_cast<MessageTypeSupport_c *>(type->type_support.type_support_);
        return typed_typesupport->deserializeROSmessage(sd, sample);
      } else if (  // NOLINT
          using_introspection_cpp_typesupport(type->type_support.typesupport_identifier_)) {
        auto typed_typesupport =
            static_cast<MessageTypeSupport_cpp *>(type->type_support.type_support_);
        return typed_typesupport->deserializeROSmessage(sd, sample);
      }
    } else {
      /* "prefix" lambda 用于将服务调用头数据注入 CDR 流中
         我还没有检查官方 RMW 实现中是如何完成的，所以可能不兼容。 */
      // 类型转换，将 sample 转换为 cdds_request_wrapper_t 类型的指针
      cdds_request_wrapper_t *const wrap = static_cast<cdds_request_wrapper_t *>(sample);

      // 定义一个 lambda 函数，用于从序列化器中读取 guid 和 seq，并存储到 wrap 的 header 中
      auto prefix = [wrap](cycdeser &ser) {
        ser >> wrap->header.guid;
        ser >> wrap->header.seq;
      };

      // 将 d 对象序列化到 serdata_rmw 中
      serialize_into_serdata_rmw_on_demand(const_cast<serdata_rmw *>(d));

      // 创建 cycdeser 对象，用于反序列化 d 的数据和大小
      cycdeser sd(d->data(), d->size());

      // 判断是否使用 C 语言类型支持
      if (using_introspection_c_typesupport(type->type_support.typesupport_identifier_)) {
        // 类型转换，获取 MessageTypeSupport_c 类型的指针
        auto typed_typesupport =
            static_cast<MessageTypeSupport_c *>(type->type_support.type_support_);

        // 使用 C 语言类型支持进行反序列化，并返回结果
        return typed_typesupport->deserializeROSmessage(sd, wrap->data, prefix);
      } else if (using_introspection_cpp_typesupport(type->type_support.typesupport_identifier_)) {
        // 类型转换，获取 MessageTypeSupport_cpp 类型的指针
        auto typed_typesupport =
            static_cast<MessageTypeSupport_cpp *>(type->type_support.type_support_);

        // 使用 C++ 语言类型支持进行反序列化，并返回结果
        return typed_typesupport->deserializeROSmessage(sd, wrap->data, prefix);
      }
    }
  } catch (rmw_cyclonedds_cpp::Exception &e) {
    RMW_SET_ERROR_MSG(e.what());
    return false;
  } catch (std::runtime_error &e) {
    RMW_SET_ERROR_MSG(e.what());
    return false;
  }

  return false;
}

/**
 * @brief 将非类型化的序列化数据转换为样本数据
 *
 * @param[in] type    指向ddsi_sertype结构体的指针，表示数据类型
 * @param[in] dcmn    指向ddsi_serdata结构体的指针，表示非类型化的序列化数据
 * @param[out] sample 用于存储转换后的样本数据的指针
 * @param[out] bufptr 用于存储缓冲区指针的指针
 * @param[in] buflim  缓冲区限制的指针
 * @return bool       返回true，因为ROS 2尚未以有意义的方式处理键
 */
static bool serdata_rmw_untyped_to_sample(const struct ddsi_sertype *type,
                                          const struct ddsi_serdata *dcmn,
                                          void *sample,
                                          void **bufptr,
                                          void *buflim) {
  static_cast<void>(type);    // 类型强制转换，忽略type参数
  static_cast<void>(dcmn);    // 类型强制转换，忽略dcmn参数
  static_cast<void>(sample);  // 类型强制转换，忽略sample参数
  static_cast<void>(bufptr);  // 类型强制转换，忽略bufptr参数
  static_cast<void>(buflim);  // 类型强制转换，忽略buflim参数
  /* ROS 2 doesn't do keys in a meaningful way yet */
  return true;
}

/**
 * @brief 比较两个非类型化序列化数据的键是否相等
 *
 * @param[in] a 指向第一个ddsi_serdata结构体的指针
 * @param[in] b 指向第二个ddsi_serdata结构体的指针
 * @return bool 返回true，因为ROS 2尚未以有意义的方式处理键
 */
static bool serdata_rmw_eqkey(const struct ddsi_serdata *a, const struct ddsi_serdata *b) {
  static_cast<void>(a);  // 类型强制转换，忽略a参数
  static_cast<void>(b);  // 类型强制转换，忽略b参数
  /* ROS 2 doesn't do keys in a meaningful way yet */
  return true;
}

/**
 * @brief 以 doxygen 的形式为函数添加参数列表的说明，并逐行添加详细的中文注释
 *
 * @param tpcmn 指向 ddsi_sertype 结构体的指针，表示序列化类型的公共部分
 * @param dcmn 指向 ddsi_serdata 结构体的指针，表示序列化数据的公共部分
 * @param buf 字符缓冲区，用于存储打印结果
 * @param bufsize 缓冲区大小
 * @return size_t 返回打印到缓冲区的字符数
 */
static size_t serdata_rmw_print(const struct ddsi_sertype *tpcmn,
                                const struct ddsi_serdata *dcmn,
                                char *buf,
                                size_t bufsize) {
  try {
    // 将 dcmn 转换为 serdata_rmw 类型的指针
    auto d = static_cast<const serdata_rmw *>(dcmn);
    // 将 tpcmn 转换为 sertype_rmw 类型的指针
    const struct sertype_rmw *type = static_cast<const struct sertype_rmw *>(tpcmn);

    // 如果数据类型不是 SDK_DATA
    if (d->kind != SDK_DATA) {
      /* ROS 2 doesn't do keys in a meaningful way yet */
      // 返回格式化字符串 ":k:{}" 的长度
      return static_cast<size_t>(snprintf(buf, bufsize, ":k:{}"));
    } else if (!type->is_request_header) {
      // 序列化数据到 serdata_rmw 对象
      serialize_into_serdata_rmw_on_demand(const_cast<serdata_rmw *>(d));
      // 创建 cycprint 对象，用于序列化数据的打印
      cycprint sd(buf, bufsize, d->data(), d->size());

      // 如果使用 C 语言的内省类型支持
      if (using_introspection_c_typesupport(type->type_support.typesupport_identifier_)) {
        auto typed_typesupport =
            static_cast<MessageTypeSupport_c *>(type->type_support.type_support_);
        // 返回打印 ROS 消息的字符数
        return typed_typesupport->printROSmessage(sd);
      } else if (using_introspection_cpp_typesupport(type->type_support.typesupport_identifier_)) {
        auto typed_typesupport =
            static_cast<MessageTypeSupport_cpp *>(type->type_support.type_support_);
        // 返回打印 ROS 消息的字符数
        return typed_typesupport->printROSmessage(sd);
      }
    } else {
      /* The "prefix" lambda is there to inject the service invocation header data into the CDR
        stream -- I haven't checked how it is done in the official RMW implementations, so it is
        probably incompatible. */
      cdds_request_wrapper_t wrap;
      // 定义一个 lambda 函数，用于将服务调用头数据注入到 CDR 流中
      auto prefix = [&wrap](cycprint &ser) {
        ser >> wrap.header.guid;
        ser.print_constant(",");
        ser >> wrap.header.seq;
      };
      // 创建 cycprint 对象，用于序列化数据的打印
      cycprint sd(buf, bufsize, d->data(), d->size());

      // 如果使用 C 语言的内省类型支持
      if (using_introspection_c_typesupport(type->type_support.typesupport_identifier_)) {
        auto typed_typesupport =
            static_cast<MessageTypeSupport_c *>(type->type_support.type_support_);
        // 返回打印 ROS 消息的字符数
        return typed_typesupport->printROSmessage(sd, prefix);
      } else if (using_introspection_cpp_typesupport(type->type_support.typesupport_identifier_)) {
        auto typed_typesupport =
            static_cast<MessageTypeSupport_cpp *>(type->type_support.type_support_);
        // 返回打印 ROS 消息的字符数
        return typed_typesupport->printROSmessage(sd, prefix);
      }
    }
  } catch (rmw_cyclonedds_cpp::Exception &e) {
    // 设置错误信息并返回 false
    RMW_SET_ERROR_MSG(e.what());
    return false;
  } catch (std::runtime_error &e) {
    // 设置错误信息并返回 false
    RMW_SET_ERROR_MSG(e.what());
    return false;
  }

  // 如果执行到这里，返回 false
  return false;
}

/**
 * @brief 获取序列化数据的键哈希值
 *
 * @param[in] d 指向ddsi_serdata结构体的指针，表示序列化数据
 * @param[out] buf 指向ddsi_keyhash结构体的指针，用于存储计算出的键哈希值
 * @param[in] force_md5 布尔值，表示是否强制使用MD5算法计算键哈希值
 */
static void serdata_rmw_get_keyhash(const struct ddsi_serdata *d,
                                    struct ddsi_keyhash *buf,
                                    bool force_md5) {
  // ROS 2尚未以有意义的方式处理键，因此对没有键字段的类型不会调用此函数
  static_cast<void>(d);
  static_cast<void>(force_md5);
  // 将buf指向的内存区域清零，大小为ddsi_keyhash结构体的大小
  memset(buf, 0, sizeof(*buf));
}

// 定义一个ddsi_serdata_ops结构体实例serdata_rmw_ops，包含各种操作函数的指针
static const struct ddsi_serdata_ops serdata_rmw_ops = {
    serdata_rmw_eqkey,
    serdata_rmw_size,
    serdata_rmw_from_ser,
    serdata_rmw_from_ser_iov,
    serdata_rmw_from_keyhash,
    serdata_rmw_from_sample,
    serdata_rmw_to_ser,
    serdata_rmw_to_ser_ref,
    serdata_rmw_to_ser_unref,
    serdata_rmw_to_sample,
    serdata_rmw_to_untyped,
    serdata_rmw_untyped_to_sample,
    serdata_rmw_free,
    serdata_rmw_print,
    serdata_rmw_get_keyhash
#ifdef DDS_HAS_SHM
    ,  // 如果定义了DDS_HAS_SHM，则添加以下操作函数指针
    ddsi_serdata_iox_size,
    serdata_rmw_from_iox
#endif  // DDS_HAS_SHM
};

/**
 * @brief 释放sertype_rmw结构的内存
 * @param tpcmn 指向ddsi_sertype结构的指针
 */
static void sertype_rmw_free(struct ddsi_sertype *tpcmn) {
  // 将tpcmn转换为sertype_rmw结构的指针
  struct sertype_rmw *tp = static_cast<struct sertype_rmw *>(tpcmn);

#if DDS_HAS_DDSI_SERTYPE
  // 如果定义了DDSI_SERTYPE，则调用ddsi_sertype_fini函数
  ddsi_sertype_fini(tpcmn);
#else
  // 否则，调用ddsi_sertopic_fini函数
  ddsi_sertopic_fini(tpcmn);
#endif

  // 如果type_support_字段不为空
  if (tp->type_support.type_support_) {
    // 如果使用introspection_c_typesupport
    if (using_introspection_c_typesupport(tp->type_support.typesupport_identifier_)) {
      // 删除TypeSupport_c实例
      delete static_cast<TypeSupport_c *>(tp->type_support.type_support_);
    } else if (using_introspection_cpp_typesupport(tp->type_support.typesupport_identifier_)) {
      // 如果使用introspection_cpp_typesupport，则删除TypeSupport_cpp实例
      delete static_cast<TypeSupport_cpp *>(tp->type_support.type_support_);
    }
    // 将type_support_字段设置为NULL
    tp->type_support.type_support_ = NULL;
  }

  // 删除sertype_rmw实例
  delete tp;
}

/**
 * @brief 将samples数组中的元素置零
 * @param d 指向ddsi_sertype结构的指针
 * @param samples 指向样本数组的指针
 * @param count 样本数量
 */
static void sertype_rmw_zero_samples(const struct ddsi_sertype *d, void *samples, size_t count) {
  // 忽略未使用的参数
  static_cast<void>(d);
  static_cast<void>(samples);
  static_cast<void>(count);

  // 不使用依赖于将样本置零的代码路径
}

/**
 * @brief 重新分配样本内存空间
 *
 * @param[in,out] ptrs 指向要重新分配的内存指针的指针
 * @param[in] d 指向ddsi_sertype结构体的指针
 * @param[in] old 指向旧内存区域的指针
 * @param[in] oldcount 旧内存区域中的样本数量
 * @param[in] count 新内存区域中的样本数量
 */
static void sertype_rmw_realloc_samples(
    void **ptrs, const struct ddsi_sertype *d, void *old, size_t oldcount, size_t count) {
  static_cast<void>(ptrs);
  static_cast<void>(d);
  static_cast<void>(old);
  static_cast<void>(oldcount);
  static_cast<void>(count);
  // 不使用依赖此功能的代码路径（贷款，处置，使用实例句柄注销，内容过滤器）
  abort();
}

/**
 * @brief 释放样本内存空间
 *
 * @param[in] d 指向ddsi_sertype结构体的指针
 * @param[in,out] ptrs 指向要释放的内存指针的指针
 * @param[in] count 要释放的样本数量
 * @param[in] op 释放操作类型
 */
static void sertype_rmw_free_samples(const struct ddsi_sertype *d,
                                     void **ptrs,
                                     size_t count,
                                     dds_free_op_t op) {
  static_cast<void>(d);      // 未使用
  static_cast<void>(ptrs);   // 未使用
  static_cast<void>(count);  // 未使用
  // 不使用依赖此功能的代码路径（处置，使用实例句柄注销，内容过滤器）
  assert(!(op & DDS_FREE_ALL_BIT));
  (void)op;
}

/**
 * @brief 比较两个ddsi_sertype是否相等
 *
 * @param[in] acmn 指向第一个ddsi_sertype结构体的指针
 * @param[in] bcmn 指向第二个ddsi_sertype结构体的指针
 * @return 如果相等返回true，否则返回false
 */
bool sertype_rmw_equal(const struct ddsi_sertype *acmn, const struct ddsi_sertype *bcmn) {
  // 猜测：具有相同名称和类型名称的类型在具有相同类型支持标识符时才是相同的
  const struct sertype_rmw *a = static_cast<const struct sertype_rmw *>(acmn);
  const struct sertype_rmw *b = static_cast<const struct sertype_rmw *>(bcmn);
  if (a->is_request_header != b->is_request_header) {
    return false;
  }
  if (strcmp(a->type_support.typesupport_identifier_, b->type_support.typesupport_identifier_) !=
      0) {
    return false;
  }
  return true;
}

/**
 * @brief 计算ddsi_sertype的哈希值
 *
 * @param[in] tpcmn 指向ddsi_sertype结构体的指针
 * @return 返回计算得到的哈希值
 */
uint32_t sertype_rmw_hash(const struct ddsi_sertype *tpcmn) {
  const struct sertype_rmw *tp = static_cast<const struct sertype_rmw *>(tpcmn);
  uint32_t h2 = static_cast<uint32_t>(std::hash<bool>{}(tp->is_request_header));
  uint32_t h1 = static_cast<uint32_t>(
      std::hash<std::string>{}(std::string(tp->type_support.typesupport_identifier_)));
  return h1 ^ h2;
}

/**
 * @brief 获取序列化后的大小
 *
 * @param[in] d 指向ddsi_sertype结构体的指针
 * @param[in] sample 需要序列化的样本数据
 * @return 返回序列化后的大小
 */
size_t sertype_get_serialized_size(const struct ddsi_sertype *d, const void *sample) {
  // 将d强制转换为sertype_rmw结构体类型的指针
  const struct sertype_rmw *type = static_cast<const struct sertype_rmw *>(d);
  // 初始化序列化后的大小为0
  size_t serialized_size = 0;
  try {
    // ROS 2暂不支持键，因此仅处理数据
    if (!type->is_request_header) {
      // 获取序列化后的大小
      serialized_size = type->cdr_writer->get_serialized_size(sample);
    } else {
      // 将服务调用头数据注入到CDR流中
      auto wrap = *static_cast<const cdds_request_wrapper_t *>(sample);
      serialized_size = type->cdr_writer->get_serialized_size(wrap);
    }
  } catch (std::exception &e) {
    // 设置错误信息
    RMW_SET_ERROR_MSG(e.what());
  }

  // 返回序列化后的大小
  return serialized_size;
}

/**
 * @brief 将样本序列化到目标缓冲区
 *
 * @param[in] d 指向ddsi_sertype结构体的指针
 * @param[in] sample 需要序列化的样本数据
 * @param[out] dst_buffer 目标缓冲区
 * @param[in] dst_size 目标缓冲区的大小
 * @return 序列化成功返回true，否则返回false
 */
bool sertype_serialize_into(const struct ddsi_sertype *d,
                            const void *sample,
                            void *dst_buffer,
                            size_t dst_size) {
  // 将d强制转换为sertype_rmw结构体类型的指针
  const struct sertype_rmw *type = static_cast<const struct sertype_rmw *>(d);
  try {
    // 忽略目标大小（假设在正确调整大小之前已调整了目标缓冲区）
    static_cast<void>(dst_size);
    // ROS 2不支持键，因此所有数据都是数据(?)
    if (!type->is_request_header) {
      // 将样本序列化到目标缓冲区
      type->cdr_writer->serialize(dst_buffer, sample);
    } else {
      // 将服务调用头数据注入到CDR流中
      auto wrap = *static_cast<const cdds_request_wrapper_t *>(sample);
      type->cdr_writer->serialize(dst_buffer, wrap);
    }
  } catch (std::exception &e) {
    // 设置错误信息
    RMW_SET_ERROR_MSG(e.what());
  }
  // 返回序列化成功
  return true;
}

/**
 * @file sertype_rmw_ops.cpp
 */

// 定义一个结构体，包含了一系列的操作函数指针
static const struct ddsi_sertype_ops sertype_rmw_ops = {
#if DDS_HAS_DDSI_SERTYPE
    ddsi_sertype_v0,              ///< DDSI 序列化类型版本
    nullptr,                      ///< 保留空指针
#endif
    sertype_rmw_free,             ///< 释放序列化类型内存
    sertype_rmw_zero_samples,     ///< 将样本置零
    sertype_rmw_realloc_samples,  ///< 重新分配样本内存
    sertype_rmw_free_samples,     ///< 释放样本内存
    sertype_rmw_equal,            ///< 检查两个序列化类型是否相等
    sertype_rmw_hash              ///< 计算序列化类型的哈希值
#if DDS_HAS_DDSI_SERTYPE
    /* not yet providing type discovery, assignability checking */
    ,
    nullptr,                      ///< 类型发现功能尚未提供，保留空指针
    nullptr,                      ///< 可分配性检查功能尚未提供，保留空指针
    nullptr,                      ///< 保留空指针
    nullptr,                      ///< 保留空指针
    sertype_get_serialized_size,  ///< 获取序列化大小
    sertype_serialize_into        ///< 将数据序列化到缓冲区
#endif
};

/**
 * @brief 创建类型名称字符串
 *
 * @tparam MembersType 成员类型
 * @param[in] untyped_members 未类型化的成员指针
 * @return std::string 类型名称字符串
 */
template <typename MembersType>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_LOCAL inline std::string create_type_name(
    const void *untyped_members) {
  // 将未类型化的成员指针转换为指定类型的成员指针
  auto members = static_cast<const MembersType *>(untyped_members);
  if (!members) {
    RMW_SET_ERROR_MSG("members handle is null");  // 如果成员指针为空，设置错误消息
    return "";
  }

  std::ostringstream ss;                                       // 创建一个字符串流对象
  std::string message_namespace(members->message_namespace_);  // 获取消息命名空间
  // 查找并替换 C 命名空间分隔符为 C++，以防使用 C 类型支持
  message_namespace = std::regex_replace(message_namespace, std::regex("__"), "::");
  std::string message_name(members->message_name_);  // 获取消息名称
  if (!message_namespace.empty()) {
    ss << message_namespace << "::";                 // 将命名空间添加到字符串流中
  }
  ss << "dds_::" << message_name << "_";             // 将消息名称添加到字符串流中
  return ss.str();                                   // 返回生成的类型名称字符串
}

/**
 * @brief 获取类型名称
 *
 * @param[in] type_support_identifier 类型支持标识符
 * @param[in] type_support 类型支持指针
 * @return std::string 返回类型名称
 */
static std::string get_type_name(const char *type_support_identifier, void *type_support) {
  // 使用内省C类型支持
  if (using_introspection_c_typesupport(type_support_identifier)) {
    auto typed_typesupport = static_cast<MessageTypeSupport_c *>(type_support);  // 类型转换
    return typed_typesupport->getName();                                      // 获取类型名称
  } else if (using_introspection_cpp_typesupport(type_support_identifier)) {  // 使用内省C++类型支持
    auto typed_typesupport = static_cast<MessageTypeSupport_cpp *>(type_support);  // 类型转换
    return typed_typesupport->getName();  // 获取类型名称
  } else {
    return "absent";                      // 类型不存在
  }
}

/**
 * @brief 创建sertype_rmw结构体实例
 *
 * @param[in] type_support_identifier 类型支持标识符
 * @param[in] type_support 类型支持指针
 * @param[in] is_request_header 是否为请求头
 * @param[in] message_type 消息类型
 * @param[in] sample_size 样本大小
 * @param[in] is_fixed_type 是否为固定类型
 * @return struct sertype_rmw* 返回创建的sertype_rmw结构体指针
 */
struct sertype_rmw *create_sertype(
    const char *type_support_identifier,
    void *type_support,
    bool is_request_header,
    std::unique_ptr<rmw_cyclonedds_cpp::StructValueType> message_type,
    const uint32_t sample_size,
    const bool is_fixed_type) {
  struct sertype_rmw *st = new struct sertype_rmw;  // 创建sertype_rmw实例
  std::string type_name = get_type_name(type_support_identifier, type_support);  // 获取类型名称
  uint32_t flags = DDSI_SERTYPE_FLAG_TOPICKIND_NO_KEY;  // 初始化标志位

  if (is_fixed_type) {
    flags |= DDSI_SERTYPE_FLAG_FIXED_SIZE;  // 设置固定大小标志
  }

  // 初始化ddsi_sertype结构体
  ddsi_sertype_init_flags(static_cast<struct ddsi_sertype *>(st), type_name.c_str(),
                          &sertype_rmw_ops, &serdata_rmw_ops, flags);

  st->allowed_data_representation = DDS_DATA_REPRESENTATION_FLAG_XCDR1;  // 设置允许的数据表示形式

#ifdef DDS_HAS_SHM
  // TODO(Sumanth) 需要在cyclone中设置一些API
  st->iox_size = sample_size;
#else
  static_cast<void>(sample_size);
#endif                                                                 // DDS_HAS_SHM

  st->type_support.typesupport_identifier_ = type_support_identifier;  // 设置类型支持标识符
  st->type_support.type_support_ = type_support;                       // 设置类型支持指针
  st->is_request_header = is_request_header;                           // 设置是否为请求头
  st->cdr_writer =
      rmw_cyclonedds_cpp::make_cdr_writer(std::move(message_type));    // 创建cdr_writer实例

  return st;  // 返回创建的sertype_rmw实例
}

/**
 * @brief 调整 serdata_rmw 对象的大小
 * @param requested_size 请求调整的大小（字节）
 *
 * 如果请求的大小为0，则将对象大小设置为0并释放内存。
 * 否则，根据请求的大小重新分配内存，并处理 CDR 填充问题。
 */
void serdata_rmw::resize(size_t requested_size) {
  // 如果请求的大小为0
  if (!requested_size) {
    m_size = 0;      // 将对象大小设置为0
    m_data.reset();  // 释放内存
    return;
  }

  // 处理 CDR 填充问题，避免在将数据复制到网络时读取越界
  size_t n_pad_bytes = (0 - requested_size) % 4;
  // 根据请求的大小和填充字节数重新分配内存
  m_data.reset(new byte[requested_size + n_pad_bytes]);
  // 更新对象的大小
  m_size = requested_size + n_pad_bytes;

  // 将未使用的填充字节设置为0，因为调用者不一定会覆盖它
  std::memset(byte_offset(m_data.get(), requested_size), '\0', n_pad_bytes);
}

/**
 * @brief serdata_rmw 构造函数
 * @param type ddsi_sertype 类型指针
 * @param kind ddsi_serdata_kind 枚举值
 *
 * 初始化 ddsi_serdata 结构体。
 */
serdata_rmw::serdata_rmw(const ddsi_sertype *type, ddsi_serdata_kind kind) : ddsi_serdata{} {
  // 初始化 ddsi_serdata 结构体
  ddsi_serdata_init(this, type, kind);
}
