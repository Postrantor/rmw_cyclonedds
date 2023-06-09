// Copyright 2019 ADLINK Technology Limited.
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

#include <algorithm>  // 包含用于算法的函数集合，例如排序和搜索。
#include <atomic>     // 为无锁并发编程提供原子操作。
#include <cassert>    // 为调试目的提供名为 `assert` 的宏。
#include <chrono>     // 包含用于测量持续时间和操作时间点的时间实用程序。
#include <cstring>  // 提供用于处理 C 风格字符串（以空字符结尾的字符数组）的函数。
#include <functional>  // 包含函数对象、绑定器和适配器，用于创建灵活且可重用的组件。
#include <iomanip>  // 提供用于格式化输入和输出流的操纵器。
#include <limits>   // 包含基本类型的数值限制。
#include <map>      // 提供 `std::map` 和 `std::multimap` 关联容器。
#include <memory>   // 包含用于管理内存的实用程序，例如智能指针和分配器。
#include <mutex>    // 提供同步原语，例如互斥锁和锁。
#include <regex>    // 包含用于模式匹配和操作的正则表达式库。
#include <set>      // 提供 `std::set` 和 `std::multiset` 关联容器。
#include <string>   // 包含用于处理字符串的 `std::string` 类。
#include <tuple>    // 提供用于处理固定大小的异构值集合的 `std::tuple` 类模板。
#include <unordered_map>  // 提供 `std::unordered_map` 和 `std::unordered_multimap` 关联容器。
#include <unordered_set>  // 提供 `std::unordered_set` 和 `std::unordered_multiset` 关联容器。
#include <utility>        // 包含实用组件，例如 `std::pair` 和 `std::swap`。
#include <vector>         // 包含用于处理动态数组的 `std::vector` 容器。

// 这些头文件在 ROS 2 的 rmw 层中使用，以实现与底层通信系统（如 DDS）的交互
#include "dds/dds.h"  // 包含 DDS（Data Distribution Service）API 的主要头文件，用于实现 ROS 2 的底层通信。
#include "dds/ddsc/dds_data_allocator.h"  // 包含 DDS 数据分配器 API，用于管理 DDS 数据对象的内存。
#include "dds/ddsc/dds_loan_api.h"  // 包含 DDS 借款 API，用于处理数据样本的借款和归还。
//
#include "MessageTypeSupport.hpp"  // 包含与消息类型支持相关的类和函数。
#include "Serialization.hpp"  // 包含序列化和反序列化功能，用于在 ROS 2 中传输消息。
#include "ServiceTypeSupport.hpp"  // 包含与服务类型支持相关的类和函数。
#include "TypeSupport2.hpp"  // 包含与类型支持相关的类和函数，用于处理不同的数据类型。
#include "demangle.hpp"  // 包含用于解析 C++ 符号名称的函数。
#include "fallthrough_macro.hpp"  // 定义了一个宏，用于标记 switch 语句中的故意穿越行为。
#include "namespace_prefix.hpp"  // 包含用于处理 ROS 2 命名空间前缀的函数。
#include "rcpputils/scope_exit.hpp"  // 包含用于在作用域结束时执行清理操作的实用程序。
#include "rcutils/env.h"             // 包含用于处理环境变量的实用程序。
#include "rcutils/filesystem.h"      // 包含用于处理文件系统操作的实用程序。
#include "rcutils/format_string.h"   // 包含用于格式化字符串的实用程序。
#include "rcutils/logging_macros.h"  // 包含用于记录日志的宏。
#include "rcutils/strdup.h"          // 包含用于复制字符串的实用程序。
//
#include "rmw/allocators.h"  // 包含用于管理内存分配器的实用程序。
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"  // 包含用于将 rcutils 返回值转换为 rmw 返回值的函数。
#include "rmw/error_handling.h"       // 包含用于处理错误的实用程序。
#include "rmw/event.h"                // 包含用于处理 ROS 2 事件的实用程序。
#include "rmw/event_callback_type.h"  // 定义了用于事件回调的类型。
#include "rmw/features.h"  // 包含用于检查 rmw 实现支持的功能的实用程序。
#include "rmw/get_node_info_and_types.h"  // 包含用于获取节点信息和类型的实用程序。
#include "rmw/get_service_names_and_types.h"  // 包含用于获取服务名称和类型的实用程序。
#include "rmw/get_topic_endpoint_info.h"  // 包含用于获取主题端点信息的实用程序。
#include "rmw/get_topic_names_and_types.h"  // 包含用于获取主题名称和类型的实用程序。
#include "rmw/impl/cpp/key_value.hpp"       // 包含用于处理键值对的实用程序。
#include "rmw/impl/cpp/macros.hpp"          // 包含用于定义 rmw 实现相关宏的头文件。
#include "rmw/incompatible_qos_events_statuses.h"  // 包含用于处理不兼容的 QoS 事件状态的实用程序。
#include "rmw/names_and_types.h"  // 包含用于处理名称和类型的实用程序。
#include "rmw/rmw.h"  // 包含 rmw API 的主要头文件，用于实现 ROS 2 的中间件抽象层。
#include "rmw/sanity_checks.h"  // 包含用于执行 rmw 实现的基本检查的实用程序。
#include "rmw/topic_endpoint_info_array.h"  // 包含用于处理主题端点信息数组的实用程序。
#include "rmw/validate_namespace.h"         // 包含用于验证命名空间的实用程序。
#include "rmw/validate_node_name.h"         // 包含用于验证节点名称的实用程序。
#include "rmw_dds_common/context.hpp"      // 包含用于管理 DDS 上下文的类和函数。
#include "rmw_dds_common/graph_cache.hpp"  // 包含用于管理图形缓存的类和函数。
#include "rmw_dds_common/msg/participant_entities_info.hpp"  // 包含参与者实体信息消息的定义。
#include "rmw_dds_common/qos.hpp"  // 包含用于处理 QoS（Quality of Service）设置的实用程序。
#include "rmw_dds_common/security.hpp"  // 包含用于处理 DDS 安全性的实用程序。
#include "rmw_version_test.hpp"         // 包含用于测试 rmw 版本的实用程序。
//
#include "rosidl_runtime_c/type_hash.h"  // 包含用于计算类型哈希值的实用程序。
#include "rosidl_typesupport_cpp/message_type_support.hpp"  // 包含用于处理 C++ 消息类型支持的实用程序。
#include "serdata.hpp"              // 包含与序列化数据相关的类和函数。
#include "serdes.hpp"               // 包含与序列化和反序列化相关的类和函数。
#include "tracetools/tracetools.h"  // 包含用于跟踪 ROS 2 系统性能的实用程序。

// 引入C++标准库中的chrono_literals命名空间，以便使用时间字面量（如1s、2ms等）
using namespace std::literals::chrono_literals;

/* Security 必须在编译时启用，并且需要 cyclone 支持 QOS 属性列表 */
#if DDS_HAS_SECURITY && DDS_HAS_PROPERTY_LIST_QOS
// 如果支持安全性和 QOS 属性列表，则设置 RMW_SUPPORT_SECURITY 为 1
#define RMW_SUPPORT_SECURITY 1
#else
// 否则，设置 RMW_SUPPORT_SECURITY 为 0
#define RMW_SUPPORT_SECURITY 0
#endif

// 如果不支持 DDS_HAS_DDSI_SERTYPE
#if !DDS_HAS_DDSI_SERTYPE
// 定义 ddsi_sertype_unref(x) 为 ddsi_sertopic_unref(x)
#define ddsi_sertype_unref(x) ddsi_sertopic_unref(x)
#endif

/* 设置为 > 0，对于每条在写入后超过此时间（毫秒）的消息，将警告打印到 stderr */
#define REPORT_LATE_MESSAGES 0
/* 设置为 != 0，以定期打印已阻塞超过 1 秒的请求 */
#define REPORT_BLOCKED_REQUESTS 0

/**
 * @brief 宏定义：当错误发生时，设置错误消息并执行指定的代码。
 * @param msg 错误消息
 * @param code 要执行的代码
 */
#define RET_ERR_X(msg, code) \
  do {                       \
    RMW_SET_ERROR_MSG(msg);  \
    code;                    \
  } while (0)

/**
 * @brief 宏定义：检查变量是否为null，如果是，则设置错误消息并执行指定的代码。
 * @param var 要检查的变量
 * @param code 要执行的代码
 */
#define RET_NULL_X(var, code)           \
  do {                                  \
    if (!var) {                         \
      RET_ERR_X(#var " is null", code); \
    }                                   \
  } while (0)

/**
 * @brief 宏定义：检查变量是否分配成功，如果没有，则设置错误消息并执行指定的代码。
 * @param var 要检查的变量
 * @param code 要执行的代码
 */
#define RET_ALLOC_X(var, code)                     \
  do {                                             \
    if (!var) {                                    \
      RET_ERR_X("failed to allocate " #var, code); \
    }                                              \
  } while (0)

/**
 * @brief 宏定义：检查实现标识符是否与期望的一致，如果不一致，则设置错误消息并执行指定的代码。
 * @param var 包含实现标识符的变量
 * @param code 要执行的代码
 */
#define RET_WRONG_IMPLID_X(var, code)                                        \
  do {                                                                       \
    if ((var)->implementation_identifier != eclipse_cyclonedds_identifier) { \
      RET_ERR_X(#var " not from this implementation", code);                 \
    }                                                                        \
  } while (0)

/**
 * @brief 宏定义：检查变量是否为null或空字符串，如果是，则设置错误消息并执行指定的代码。
 * @param var 要检查的变量
 * @param code 要执行的代码
 */
#define RET_NULL_OR_EMPTYSTR_X(var, code)               \
  do {                                                  \
    if (!var || strlen(var) == 0) {                     \
      RET_ERR_X(#var " is null or empty string", code); \
    }                                                   \
  } while (0)

/**
 * @brief 宏定义：当错误发生时，设置错误消息并返回 RMW_RET_ERROR。
 * @param msg 错误消息
 */
#define RET_ERR(msg) RET_ERR_X(msg, return RMW_RET_ERROR)

/**
 * @brief 宏定义：检查变量是否为null，如果是，则返回 RMW_RET_ERROR。
 * @param var 要检查的变量
 */
#define RET_NULL(var) RET_NULL_X(var, return RMW_RET_ERROR)

/**
 * @brief 宏定义：检查变量是否分配成功，如果没有，则返回 RMW_RET_ERROR。
 * @param var 要检查的变量
 */
#define RET_ALLOC(var) RET_ALLOC_X(var, return RMW_RET_ERROR)

/**
 * @brief 宏定义：检查实现标识符是否与期望的一致，如果不一致，则返回
 * RMW_RET_INCORRECT_RMW_IMPLEMENTATION。
 * @param var 包含实现标识符的变量
 */
#define RET_WRONG_IMPLID(var) RET_WRONG_IMPLID_X(var, return RMW_RET_INCORRECT_RMW_IMPLEMENTATION)

/**
 * @brief 宏定义：检查变量是否为null或空字符串，如果是，则返回 RMW_RET_ERROR。
 * @param var 要检查的变量
 */
#define RET_NULL_OR_EMPTYSTR(var) RET_NULL_OR_EMPTYSTR_X(var, return RMW_RET_ERROR)

/**
 * @brief 宏定义：检查函数返回值是否与期望的返回值相等，如果不相等，则设置错误消息并执行指定的代码。
 * @param func 要检查的函数
 * @param expected_ret 期望的返回值
 * @param error_msg 错误消息
 * @param code 要执行的代码
 */
#define RET_EXPECTED(func, expected_ret, error_msg, code) \
  do {                                                    \
    if ((expected_ret) != (func)) {                       \
      RET_ERR_X(error_msg, code);                         \
    }                                                     \
  } while (0)

using rmw_dds_common::msg::ParticipantEntitiesInfo;

/// \brief CycloneDDS的标识符，用于在RMW层中区分不同的DDS实现
const char *const eclipse_cyclonedds_identifier = "rmw_cyclonedds_cpp";
/// \brief CycloneDDS使用的序列化格式，这里是Common Data Representation (CDR)格式
const char *const eclipse_cyclonedds_serialization_format = "cdr";

/**
 * @brief 一个用于生成DDS实例句柄哈希值的结构体
 *
 * 实例句柄是无符号64位整数，精心构造为尽可能接近均匀分布，
 * 原因是使它们成为近乎完美的哈希键，因此我们可以改进默认的哈希函数。
 */
struct dds_instance_handle_hash {
public:
  /**
   * @brief 重载括号运算符，用于计算实例句柄的哈希值
   *
   * @param x 一个常量引用，表示要计算哈希值的实例句柄
   * @return 返回计算得到的哈希值，类型为std::size_t
   */
  std::size_t operator()(dds_instance_handle_t const &x) const noexcept {
    return static_cast<std::size_t>(x);
  }
};

/**
 * @brief 比较两个dds_builtintopic_guid_t对象的大小
 *
 * @param a 一个常量引用，表示要比较的第一个dds_builtintopic_guid_t对象
 * @param b 一个常量引用，表示要比较的第二个dds_builtintopic_guid_t对象
 * @return 如果a小于b，则返回true；否则返回false
 */
bool operator<(dds_builtintopic_guid_t const &a, dds_builtintopic_guid_t const &b) {
  return memcmp(&a, &b, sizeof(dds_builtintopic_guid_t)) < 0;
}

/**
 * @brief 停止发现线程
 *
 * @param[in] context ROS2 RMW层的上下文对象
 * @return rmw_ret_t 返回操作结果，成功或失败
 */
static rmw_ret_t discovery_thread_stop(rmw_dds_common::Context &context);

/**
 * @brief 将DDS QoS转换为RMW QoS
 *
 * @param[in] dds_qos DDS QoS策略指针
 * @param[out] qos_policies RMW QoS策略指针
 * @return bool 转换是否成功
 */
static bool dds_qos_to_rmw_qos(const dds_qos_t *dds_qos, rmw_qos_profile_t *qos_policies);

/**
 * @brief 创建发布者
 *
 * @param[in] dds_ppant DDS参与者实体
 * @param[in] dds_pub DDS发布者实体
 * @param[in] type_supports 消息类型支持
 * @param[in] topic_name 主题名称
 * @param[in] qos_policies QoS策略
 * @param[in] publisher_options 发布者选项
 * @return rmw_publisher_t* 创建的发布者指针
 */
static rmw_publisher_t *create_publisher(
    dds_entity_t dds_ppant,
    dds_entity_t dds_pub,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies,
    const rmw_publisher_options_t *publisher_options);

/**
 * @brief 销毁发布者
 *
 * @param[in] publisher 要销毁的发布者指针
 * @return rmw_ret_t 返回操作结果，成功或失败
 */
static rmw_ret_t destroy_publisher(rmw_publisher_t *publisher);

/**
 * @brief 创建订阅者
 *
 * @param[in] dds_ppant DDS参与者实体
 * @param[in] dds_pub DDS发布者实体
 * @param[in] type_supports 消息类型支持
 * @param[in] topic_name 主题名称
 * @param[in] qos_policies QoS策略
 * @param[in] subscription_options 订阅选项
 * @return rmw_subscription_t* 创建的订阅者指针
 */
static rmw_subscription_t *create_subscription(
    dds_entity_t dds_ppant,
    dds_entity_t dds_pub,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies,
    const rmw_subscription_options_t *subscription_options);

/**
 * @brief 销毁订阅者
 *
 * @param[in] subscription 要销毁的订阅者指针
 * @return rmw_ret_t 返回操作结果，成功或失败
 */
static rmw_ret_t destroy_subscription(rmw_subscription_t *subscription);

/**
 * @brief 创建守护条件
 *
 * @return rmw_guard_condition_t* 创建的守护条件指针
 */
static rmw_guard_condition_t *create_guard_condition();

/**
 * @brief 销毁守护条件
 *
 * @param[in] gc 要销毁的守护条件指针
 * @return rmw_ret_t 返回操作结果，成功或失败
 */
static rmw_ret_t destroy_guard_condition(rmw_guard_condition_t *gc);

// CddsDomain结构定义
struct CddsDomain;

// CddsWaitset结构定义
struct CddsWaitset;

/**
 * @brief Cdds结构定义
 */
struct Cdds {
  std::mutex lock;  // 互斥锁

  // 域ID到每个域状态的映射，用于创建/销毁节点
  std::mutex domains_lock;
  std::map<dds_domainid_t, CddsDomain> domains;

  // 特殊的守护条件，附加到每个等待集，但永远不会触发：
  // 这样，我们可以避免Cyclone在没有实体附加到等待集时总是立即返回的行为
  dds_entity_t gc_for_empty_waitset;

  // 受锁保护的等待集合，用于在删除实体时使所有等待集缓存失效
  std::unordered_set<CddsWaitset *> waitsets;

  // 构造函数
  Cdds() : gc_for_empty_waitset(0) {}
};

/**
 * @file cdds_global_state.cpp
 *
 * @brief 本文件包含了ROS2的RMW层相关的C++代码，主要用于管理全局状态。
 */

/* 使用construct-on-first-use方法来处理全局状态，而不是使用普通的全局变量，
   避免在其他系统组件最后一次使用之前运行其析构函数。
   例如，某些rclcpp测试（在此提交时）在全局析构函数中删除一个守卫条件，
   但是（至少）在Windows上，Cyclone RMW全局析构函数在该测试的全局析构函数之前运行，
   导致rmw_destroy_guard_condition()尝试使用已经销毁的 "Cdds::waitsets"。

   这导致的内存泄漏很小（一个空的域映射和一个空的等待集），并且定义只有一次。
   完全消除或将其存在与init/shutdown绑定的替代方案是有问题的，因为这个状态跨域和上下文使用。

   我看到的唯一实际可行的替代方案是扩展Cyclone的运行时状态（这在这些情况下得到了正确管理），
   但是解决C++全局析构函数限制不是Cyclone的责任。 */
static Cdds &gcdds() {
  // 使用静态指针x来存储Cdds对象的实例，并在第一次使用时创建
  static Cdds *x = new Cdds();
  // 返回Cdds对象的引用
  return *x;
}

/**
 * @brief CddsEntity结构体，包含一个dds_entity_t类型的成员enth。
 */
struct CddsEntity {
  dds_entity_t enth;  ///< dds实体
};

/**
 * @struct CddsDomain
 * @brief 用于跟踪存在的域及其节点数量的结构体。
 *
 * 当前的RMW实现通过显式创建具有以下配置的域来实现仅限本地主机：
 * 1. 将 "localhost" 作为网络接口地址的硬编码选择；
 * 2. 紧接着是 CYCLONEDDS_URI 环境变量的内容。
 *
 * 本地主机名称应解析为 IPv4 的 127.0.0.1（或等效值）和 IPv6 的 ::1，
 * 因此我们不必担心使用 IPv4 还是 IPv6（如使用数字 IP 地址的情况），
 * 也不必担心环回接口的名称。如果计算机的配置没有正确解析 "localhost"，
 * 您仍然可以通过 $CYCLONEDDS_URI 覆盖。
 *
 * CddsDomain 类型用于跟踪哪些域存在以及该域中有多少节点。
 * 由于域是在该域中创建的第一个节点实例化的，因此其他节点必须具有相同的仅限本地主机设置。
 * （如果没有，则会出错。）当删除域中的最后一个节点时，一切都会自动重置。
 *
 * （Cyclone 提供 "loopback" 或类似的通用别名可能更好，用于环回接口...）
 *
 * Cyclone 中当前对创建域的支持存在一些问题，修复这些问题可能会改变或放宽上述内容。
 */
struct CddsDomain {
  bool localhost_only;         ///< 是否仅限本地主机
  uint32_t refcount;           ///< 节点引用计数

  dds_entity_t domain_handle;  ///< 域实体的句柄

  /**
   * @brief 默认构造函数，以便可以安全地使用 operator[] 查找一个域。
   */
  CddsDomain() : localhost_only(false), refcount(0), domain_handle(0) {}

  /**
   * @brief 析构函数
   */
  ~CddsDomain() {}
};

/**
 * @brief 定义结构体 rmw_context_impl_s，该结构体在 rmw/init.h 中声明
 */
struct rmw_context_impl_s {
  rmw_dds_common::Context common;  ///< 公共上下文
  dds_domainid_t domain_id;        ///< DDS 域 ID
  dds_entity_t ppant;              ///< 参与者实体
  rmw_gid_t ppant_gid;             ///< 参与者全局唯一标识符 (GID)

  /* 内置主题读取器的句柄 */
  dds_entity_t rd_participant;   ///< 参与者读取器
  dds_entity_t rd_subscription;  ///< 订阅读取器
  dds_entity_t rd_publication;   ///< 发布读取器

  /* 用于 ROS 2 发布者和订阅者的 DDS 发布者、订阅者 */
  dds_entity_t dds_pub;  ///< DDS 发布者
  dds_entity_t dds_sub;  ///< DDS 订阅者

  /* 参与者引用计数 */
  size_t node_count{0};             ///< 节点计数
  std::mutex initialization_mutex;  ///< 初始化互斥锁

  /* 关闭标志 */
  bool is_shutdown{false};  ///< 是否已关闭

  /* 构造唯一客户端/服务 ID 的 GUID 后缀（由 initialization_mutex 保护） */
  uint32_t client_service_id;  ///< 客户端/服务 ID

  /**
   * @brief 默认构造函数
   */
  rmw_context_impl_s() : common(), domain_id(UINT32_MAX), ppant(0), client_service_id(0) {
    /* 析构函数依赖于这些属性被正确初始化 */
    common.thread_is_running.store(false);
    common.graph_guard_condition = nullptr;
    common.pub = nullptr;
    common.sub = nullptr;
  }

  /**
   * @brief 初始化参与者（如果尚未完成）
   * @param options 指向 rmw_init_options_t 的指针
   * @param domain_id 域 ID
   * @return 返回 rmw_ret_t 类型的结果
   */
  rmw_ret_t init(rmw_init_options_t *options, size_t domain_id);

  /**
   * @brief 当 node_count 达到 0 时，销毁参与者
   * @return 返回 rmw_ret_t 类型的结果
   */
  rmw_ret_t fini();

  /**
   * @brief 析构函数
   */
  ~rmw_context_impl_s() {
    if (0u != this->node_count) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
          "Not all nodes were finished before finishing the context\n."
          "Ensure `rcl_node_fini` is called for all nodes before `rcl_context_fini`,"
          "to avoid leaking.\n");
    }
  }

private:
  /**
   * @brief 清理函数
   */
  void clean_up();
};

/**
 * @struct CddsNode
 * @brief 一个表示ROS2节点的结构体。
 */
struct CddsNode {};

/**
 * @struct user_callback_data_t
 * @brief 一个包含用户回调数据的结构体。
 */
struct user_callback_data_t {
  std::mutex mutex;                        ///< 互斥锁，用于保护回调数据的访问。
  rmw_event_callback_t callback{nullptr};  ///< 用户定义的回调函数。
  const void *user_data{nullptr};          ///< 用户定义的回调数据。
  size_t unread_count{0};                  ///< 未读消息计数。
  rmw_event_callback_t event_callback[DDS_STATUS_ID_MAX + 1]{nullptr};  ///< 事件回调函数数组。
  const void *event_data[DDS_STATUS_ID_MAX + 1]{nullptr};               ///< 事件数据数组。
  size_t event_unread_count[DDS_STATUS_ID_MAX + 1]{0};  ///< 每个事件的未读消息计数数组。
};

/**
 * @struct CddsPublisher
 * @brief 一个表示ROS2发布者的结构体，继承自CddsEntity。
 */
struct CddsPublisher : CddsEntity {
  dds_instance_handle_t pubiid;                 ///< 发布者实例ID。
  rmw_gid_t gid;                                ///< 发布者全局唯一ID。
  struct ddsi_sertype *sertype;                 ///< 序列化类型。
  rosidl_message_type_support_t type_supports;  ///< 消息类型支持。
  dds_data_allocator_t data_allocator;          ///< 数据分配器。
  uint32_t sample_size;                         ///< 样本大小。
  bool is_loaning_available;                    ///< 是否支持loan功能。
  user_callback_data_t user_callback_data;      ///< 用户回调数据结构体实例。
};

/**
 * @struct CddsSubscription
 * @brief 一个表示ROS2订阅者的结构体，继承自CddsEntity。
 */
struct CddsSubscription : CddsEntity {
  rmw_gid_t gid;                                ///< 订阅者全局唯一ID。
  dds_entity_t rdcondh;                         ///< 读取条件句柄。
  rosidl_message_type_support_t type_supports;  ///< 消息类型支持。
  dds_data_allocator_t data_allocator;          ///< 数据分配器。
  bool is_loaning_available;                    ///< 是否支持loan功能。
  user_callback_data_t user_callback_data;      ///< 用户回调数据结构体实例。
};

/**
 * @struct client_service_id_t
 * @brief 一个表示客户端服务ID的结构体。
 */
struct client_service_id_t {
  // 奇怪的是，rmw_request_id_t中的writer_guid比rmw_gid_t中的标识符小
  uint8_t data[sizeof((reinterpret_cast<rmw_request_id_t *>(0))->writer_guid)];  // NOLINT
};

/**
 * @struct CddsCS
 * @brief 一个包含发布者和订阅者的结构体，用于ROS2的RMW层。
 */
struct CddsCS {
  std::unique_ptr<CddsPublisher> pub;     ///< 发布者对象的智能指针
  std::unique_ptr<CddsSubscription> sub;  ///< 订阅者对象的智能指针
  client_service_id_t id;                 ///< 客户端服务ID
};

/**
 * @struct CddsClient
 * @brief 一个包含CddsCS客户端的结构体，用于ROS2的RMW层。
 */
struct CddsClient {
  CddsCS client;  ///< CddsCS客户端对象

#if REPORT_BLOCKED_REQUESTS
  std::mutex lock;                          ///< 互斥锁，用于同步访问
  dds_time_t lastcheck;                     ///< 上次检查的时间
  std::map<int64_t, dds_time_t> reqtime;    ///< 请求时间映射表
#endif
  user_callback_data_t user_callback_data;  ///< 用户回调数据
};

/**
 * @struct CddsService
 * @brief 一个包含CddsCS服务的结构体，用于ROS2的RMW层。
 */
struct CddsService {
  CddsCS service;                           ///< CddsCS服务对象
  user_callback_data_t user_callback_data;  ///< 用户回调数据
};

/**
 * @struct CddsGuardCondition
 * @brief 一个包含守护条件实体的结构体，用于ROS2的RMW层。
 */
struct CddsGuardCondition {
  dds_entity_t gcondh;  ///< 守护条件实体句柄
};

/**
 * @struct CddsEvent
 * @brief 一个继承自CddsEntity的事件结构体，用于ROS2的RMW层。
 */
struct CddsEvent : CddsEntity {
  rmw_event_type_t event_type;  ///< 事件类型
};

/**
 * @struct CddsWaitset
 * @brief 一个用于管理DDS实体的等待集结构。
 */
struct CddsWaitset {
  dds_entity_t waitseth;                  ///< DDS等待集句柄

  std::vector<dds_attach_t> trigs;        ///< 触发器向量
  size_t nelems;                          ///< 元素数量

  std::mutex lock;                        ///< 互斥锁，用于保护访问
  bool inuse;                             ///< 标记等待集是否正在使用
  std::vector<CddsSubscription *> subs;   ///< 订阅者向量
  std::vector<CddsGuardCondition *> gcs;  ///< 守护条件向量
  std::vector<CddsClient *> cls;          ///< 客户端向量
  std::vector<CddsService *> srvs;        ///< 服务向量
  std::vector<CddsEvent> evs;             ///< 事件向量
};

/**
 * @brief 清理等待集缓存。
 */
static void clean_waitset_caches();

#if REPORT_BLOCKED_REQUESTS
/**
 * @brief 检查被阻塞的请求。
 * @param client CddsClient对象的引用
 */
static void check_for_blocked_requests(CddsClient &client);
#endif

#ifndef WIN32
/* TODO(allenh1): check for Clang */
#pragma GCC visibility push(default)
#endif

/**
 * @brief 获取RMW实现标识符。
 * @return 返回eclipse_cyclonedds_identifier字符串
 */
extern "C" const char *rmw_get_implementation_identifier() { return eclipse_cyclonedds_identifier; }

/**
 * @brief 获取序列化格式。
 * @return 返回eclipse_cyclonedds_serialization_format字符串
 */
extern "C" const char *rmw_get_serialization_format() {
  return eclipse_cyclonedds_serialization_format;
}

/**
 * @brief 设置ROS2日志的严重级别 (Set the log severity level for ROS2)
 *
 * @param[in] severity 日志严重级别枚举值 (Enumeration value of log severity level)
 * @return rmw_ret_t 返回操作结果状态 (Return the operation result status)
 */
extern "C" rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity) {
  uint32_t mask = 0;  // 初始化掩码变量 (Initialize the mask variable)

  // 根据传入的严重级别设置相应的掩码 (Set the corresponding mask according to the input severity
  // level)
  switch (severity) {
    default:
      // 如果传入的严重级别无效，设置错误信息并返回无效参数错误 (If the input severity level is
      // invalid, set the error message and return an invalid argument error)
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("%s: Invalid log severity '%d'", __func__, severity);
      return RMW_RET_INVALID_ARGUMENT;
    case RMW_LOG_SEVERITY_DEBUG:
      mask |= DDS_LC_DISCOVERY | DDS_LC_THROTTLE | DDS_LC_CONFIG;
      FALLTHROUGH;
    case RMW_LOG_SEVERITY_INFO:
      mask |= DDS_LC_INFO;
      FALLTHROUGH;
    case RMW_LOG_SEVERITY_WARN:
      mask |= DDS_LC_WARNING;
      FALLTHROUGH;
    case RMW_LOG_SEVERITY_ERROR:
      mask |= DDS_LC_ERROR;
      FALLTHROUGH;
    case RMW_LOG_SEVERITY_FATAL:
      mask |= DDS_LC_FATAL;
  }

  dds_set_log_mask(mask);  // 设置DDS日志掩码 (Set the DDS log mask)
  return RMW_RET_OK;       // 返回操作成功状态 (Return the operation success status)
}

/**
 * @brief DDS监听器回调函数
 *
 * @param[in] entity DDS实体
 * @param[in] arg 用户回调数据指针
 */
static void dds_listener_callback(dds_entity_t entity, void *arg) {
  // 当前未使用
  (void)entity;

  // 将arg转换为user_callback_data_t类型的指针
  auto data = static_cast<user_callback_data_t *>(arg);

  // 使用互斥锁保护数据
  std::lock_guard<std::mutex> guard(data->mutex);

  // 如果有回调函数，则执行回调
  if (data->callback) {
    data->callback(data->user_data, 1);
  } else {
    // 否则，未读计数加1
    data->unread_count++;
  }
}

/**
 * @brief 创建DDS事件回调函数宏
 *
 * @param event_type 事件类型
 * @param EVENT_TYPE 大写事件类型
 */
#define MAKE_DDS_EVENT_CALLBACK_FN(event_type, EVENT_TYPE)                        \
  static void on_##event_type##_fn(                                               \
      dds_entity_t entity, const dds_##event_type##_status_t status, void *arg) { \
    (void)status;                                                                 \
    (void)entity;                                                                 \
    // 将arg转换为user_callback_data_t类型的指针                              \
    auto data = static_cast<user_callback_data_t *>(arg);                      \
    // 使用互斥锁保护数据                                                        \
    std::lock_guard<std::mutex> guard(data->mutex);                            \
    // 获取对应事件的回调函数                                                  \
    auto cb = data->event_callback[DDS_##EVENT_TYPE##_STATUS_ID];              \
    if (cb) {                                                                  \
      // 如果有回调函数，则执行回调                                            \
      cb(data->event_data[DDS_##EVENT_TYPE##_STATUS_ID], 1);                   \
    } else {                                                                   \
      // 否则，未读计数加1                                                      \
      data->event_unread_count[DDS_##EVENT_TYPE##_STATUS_ID]++;                \
    }                                                                          \
  }

// 定义事件回调函数
MAKE_DDS_EVENT_CALLBACK_FN(requested_deadline_missed, REQUESTED_DEADLINE_MISSED)
MAKE_DDS_EVENT_CALLBACK_FN(liveliness_lost, LIVELINESS_LOST)
MAKE_DDS_EVENT_CALLBACK_FN(offered_deadline_missed, OFFERED_DEADLINE_MISSED)
MAKE_DDS_EVENT_CALLBACK_FN(requested_incompatible_qos, REQUESTED_INCOMPATIBLE_QOS)
MAKE_DDS_EVENT_CALLBACK_FN(sample_lost, SAMPLE_LOST)
MAKE_DDS_EVENT_CALLBACK_FN(offered_incompatible_qos, OFFERED_INCOMPATIBLE_QOS)
MAKE_DDS_EVENT_CALLBACK_FN(liveliness_changed, LIVELINESS_CHANGED)
MAKE_DDS_EVENT_CALLBACK_FN(inconsistent_topic, INCONSISTENT_TOPIC)
MAKE_DDS_EVENT_CALLBACK_FN(subscription_matched, SUBSCRIPTION_MATCHED)
MAKE_DDS_EVENT_CALLBACK_FN(publication_matched, PUBLICATION_MATCHED)

/**
 * @brief 设置监听器的事件回调函数
 *
 * @param[in] l 指向dds_listener_t类型的指针，用于设置事件回调函数
 * @param[in] arg 传递给回调函数的参数
 */
static void listener_set_event_callbacks(dds_listener_t *l, void *arg) {
  // 设置请求截止时间未满足的回调函数
  dds_lset_requested_deadline_missed_arg(l, on_requested_deadline_missed_fn, arg, false);
  // 设置请求的QoS不兼容的回调函数
  dds_lset_requested_incompatible_qos_arg(l, on_requested_incompatible_qos_fn, arg, false);
  // 设置样本丢失的回调函数
  dds_lset_sample_lost_arg(l, on_sample_lost_fn, arg, false);
  // 设置生命周期丢失的回调函数
  dds_lset_liveliness_lost_arg(l, on_liveliness_lost_fn, arg, false);
  // 设置提供的截止时间未满足的回调函数
  dds_lset_offered_deadline_missed_arg(l, on_offered_deadline_missed_fn, arg, false);
  // 设置提供的QoS不兼容的回调函数
  dds_lset_offered_incompatible_qos_arg(l, on_offered_incompatible_qos_fn, arg, false);
  // 设置生命周期改变的回调函数
  dds_lset_liveliness_changed_arg(l, on_liveliness_changed_fn, arg, false);
  // 设置主题不一致的回调函数
  dds_lset_inconsistent_topic_arg(l, on_inconsistent_topic_fn, arg, false);
  // 设置订阅匹配的回调函数
  dds_lset_subscription_matched_arg(l, on_subscription_matched_fn, arg, false);
  // 设置发布匹配的回调函数
  dds_lset_publication_matched_arg(l, on_publication_matched_fn, arg, false);
}

/**
 * @brief 获取读写QoS配置
 *
 * @param[in] handle DDS实体句柄
 * @param[out] rmw_qos_policies 指向rmw_qos_profile_t类型的指针，用于存储获取到的QoS配置
 * @return 成功返回true，失败返回false
 */
static bool get_readwrite_qos(dds_entity_t handle, rmw_qos_profile_t *rmw_qos_policies) {
  dds_qos_t *qos = dds_create_qos();
  dds_return_t ret = false;
  if (dds_get_qos(handle, qos) < 0) {
    RMW_SET_ERROR_MSG("get_readwrite_qos: invalid handle");
  } else {
    ret = dds_qos_to_rmw_qos(qos, rmw_qos_policies);
  }
  dds_delete_qos(qos);
  return ret;
}

/**
 * @brief 设置订阅器的新消息回调函数
 *
 * @param[in] rmw_subscription 指向rmw_subscription_t类型的指针，表示订阅器
 * @param[in] callback 新消息回调函数
 * @param[in] user_data 用户数据，传递给回调函数的参数
 * @return 成功返回RMW_RET_OK，失败返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_subscription_set_on_new_message_callback(
    rmw_subscription_t *rmw_subscription, rmw_event_callback_t callback, const void *user_data) {
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
  auto sub = static_cast<CddsSubscription *>(rmw_subscription->data);

  user_callback_data_t *data = &(sub->user_callback_data);

  std::lock_guard<std::mutex> guard(data->mutex);

  // 设置用户回调数据
  data->callback = callback;
  data->user_data = user_data;

  if (callback && data->unread_count) {
    // 在分配回调之前发生的事件，将它们限制在QoS深度内。
    rmw_qos_profile_t sub_qos;

    if (!get_readwrite_qos(sub->enth, &sub_qos)) {
      return RMW_RET_ERROR;
    }

    size_t events = std::min(data->unread_count, sub_qos.depth);

    callback(user_data, events);
    data->unread_count = 0;
  }

  return RMW_RET_OK;
}

/**
 * @brief 设置服务端新请求回调函数
 *
 * @param[in] rmw_service 服务端对象指针
 * @param[in] callback 新请求回调函数
 * @param[in] user_data 用户数据指针
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_service_set_on_new_request_callback(
    rmw_service_t *rmw_service, rmw_event_callback_t callback, const void *user_data) {
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_service, RMW_RET_INVALID_ARGUMENT);

  // 类型转换，获取服务端对象
  auto srv = static_cast<CddsService *>(rmw_service->data);

  // 获取用户回调数据结构体指针
  user_callback_data_t *data = &(srv->user_callback_data);

  // 使用互斥锁保护数据
  std::lock_guard<std::mutex> guard(data->mutex);

  // 设置用户回调数据
  data->callback = callback;
  data->user_data = user_data;

  if (callback && data->unread_count) {
    // 在分配回调之前发生的事件推送
    callback(user_data, data->unread_count);
    data->unread_count = 0;
  }

  return RMW_RET_OK;
}

/**
 * @brief 设置客户端新响应回调函数
 *
 * @param[in] rmw_client 客户端对象指针
 * @param[in] callback 新响应回调函数
 * @param[in] user_data 用户数据指针
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_client_set_on_new_response_callback(
    rmw_client_t *rmw_client, rmw_event_callback_t callback, const void *user_data) {
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);

  // 类型转换，获取客户端对象
  auto cli = static_cast<CddsClient *>(rmw_client->data);

  // 获取用户回调数据结构体指针
  user_callback_data_t *data = &(cli->user_callback_data);

  // 使用互斥锁保护数据
  std::lock_guard<std::mutex> guard(data->mutex);

  // 设置用户回调数据
  data->callback = callback;
  data->user_data = user_data;

  if (callback && data->unread_count) {
    // 在分配回调之前发生的事件推送
    callback(user_data, data->unread_count);
    data->unread_count = 0;
  }

  return RMW_RET_OK;
}

/**
 * @brief 设置事件回调函数
 *
 * 为给定的事件设置回调函数和用户数据。当事件发生时，将调用回调函数。
 * 如果在分配回调之前已经发生了事件，将立即调用回调函数。
 *
 * @tparam T 事件类型
 * @param event 要设置回调函数的事件
 * @param status_id 事件状态ID
 * @param callback 要设置的回调函数
 * @param user_data 要与回调函数关联的用户数据
 */
template <typename T>
static void event_set_callback(
    T event, dds_status_id_t status_id, rmw_event_callback_t callback, const void *user_data) {
  // 获取用户回调数据结构
  user_callback_data_t *data = &(event->user_callback_data);

  // 使用互斥锁保护数据
  std::lock_guard<std::mutex> guard(data->mutex);

  // 设置用户回调数据
  data->event_callback[status_id] = callback;
  data->event_data[status_id] = user_data;

  // 如果已经设置了回调函数并且有未读事件，则立即调用回调函数
  if (callback && data->event_unread_count[status_id]) {
    // 处理在分配回调之前发生的事件
    callback(user_data, data->event_unread_count[status_id]);
    // 将未读事件计数重置为0
    data->event_unread_count[status_id] = 0;
  }
}

/**
 * @brief 设置事件回调函数
 *
 * @param[in] rmw_event 指向rmw_event_t类型的指针，表示要设置回调函数的事件对象
 * @param[in] callback rmw_event_callback_t类型的回调函数
 * @param[in] user_data 用户数据，将作为回调函数的参数传递
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回相应错误代码
 */
extern "C" rmw_ret_t rmw_event_set_callback(
    rmw_event_t *rmw_event, rmw_event_callback_t callback, const void *user_data) {
  // 检查rmw_event是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);

  // 根据事件类型进行处理
  switch (rmw_event->event_type) {
    case RMW_EVENT_LIVELINESS_CHANGED: {
      // 将事件数据转换为CddsSubscription类型
      auto sub_event = static_cast<CddsSubscription *>(rmw_event->data);
      // 设置回调函数及其参数
      event_set_callback(sub_event, DDS_LIVELINESS_CHANGED_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_REQUESTED_DEADLINE_MISSED: {
      auto sub_event = static_cast<CddsSubscription *>(rmw_event->data);
      event_set_callback(sub_event, DDS_REQUESTED_DEADLINE_MISSED_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE: {
      auto sub_event = static_cast<CddsSubscription *>(rmw_event->data);
      event_set_callback(sub_event, DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_MESSAGE_LOST: {
      auto sub_event = static_cast<CddsSubscription *>(rmw_event->data);
      event_set_callback(sub_event, DDS_SAMPLE_LOST_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_SUBSCRIPTION_MATCHED: {
      auto sub_event = static_cast<CddsSubscription *>(rmw_event->data);
      event_set_callback(sub_event, DDS_SUBSCRIPTION_MATCHED_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_LIVELINESS_LOST: {
      auto pub_event = static_cast<CddsPublisher *>(rmw_event->data);
      event_set_callback(pub_event, DDS_LIVELINESS_LOST_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_OFFERED_DEADLINE_MISSED: {
      auto pub_event = static_cast<CddsPublisher *>(rmw_event->data);
      event_set_callback(pub_event, DDS_OFFERED_DEADLINE_MISSED_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE: {
      auto pub_event = static_cast<CddsPublisher *>(rmw_event->data);
      event_set_callback(pub_event, DDS_OFFERED_INCOMPATIBLE_QOS_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE: {
      auto pub_event = static_cast<CddsPublisher *>(rmw_event->data);
      event_set_callback(pub_event, DDS_INCONSISTENT_TOPIC_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE: {
      auto sub_event = static_cast<CddsSubscription *>(rmw_event->data);
      event_set_callback(sub_event, DDS_INCONSISTENT_TOPIC_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_PUBLICATION_MATCHED: {
      auto pub_event = static_cast<CddsPublisher *>(rmw_event->data);
      event_set_callback(pub_event, DDS_PUBLICATION_MATCHED_STATUS_ID, callback, user_data);
      break;
    }

    case RMW_EVENT_INVALID: {
      // 无效事件类型，返回无效参数错误
      return RMW_RET_INVALID_ARGUMENT;
    }
  }
  // 设置回调函数成功，返回RMW_RET_OK
  return RMW_RET_OK;
}

/**
 * @brief 初始化 rmw_init_options_t 结构体
 *
 * @param[in,out] init_options 一个指向待初始化的 rmw_init_options_t 结构体的指针
 * @param[in] allocator 分配器，用于分配内存
 * @return RMW_RET_OK 如果成功，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_init_options_init(
    rmw_init_options_t *init_options, rcutils_allocator_t allocator) {
  // 检查 init_options 是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  // 检查分配器是否有效，如果无效则返回无效参数错误
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  // 检查实现标识符是否已经初始化，如果已经初始化则设置错误消息并返回无效参数错误
  if (NULL != init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 设置实例 ID 为 0
  init_options->instance_id = 0;
  // 设置实现标识符为 eclipse_cyclonedds_identifier
  init_options->implementation_identifier = eclipse_cyclonedds_identifier;
  // 设置分配器
  init_options->allocator = allocator;
  // 设置 impl 为 nullptr
  init_options->impl = nullptr;
  // 设置 localhost_only 为默认值
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;
  // 设置 domain_id 为默认值
  init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
  // 设置 enclave 为 NULL
  init_options->enclave = NULL;
  // 设置 security_options 为零初始化的安全选项
  init_options->security_options = rmw_get_zero_initialized_security_options();
  // 返回成功
  return RMW_RET_OK;
}

/**
 * @brief 复制 rmw_init_options_t 结构体
 *
 * @param[in] src 指向源 rmw_init_options_t 结构体的指针
 * @param[out] dst 指向目标 rmw_init_options_t 结构体的指针
 * @return RMW_RET_OK 如果成功，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_init_options_copy(const rmw_init_options_t *src, rmw_init_options_t *dst) {
  // 检查 src 是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  // 检查 dst 是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  // 检查源实现标识符是否已经初始化，如果未初始化则设置错误消息并返回无效参数错误
  if (NULL == src->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected initialized src");
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 检查源和目标的实现标识符是否匹配，如果不匹配则返回不正确的 RMW 实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      src, src->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查目标实现标识符是否已经初始化，如果已经初始化则设置错误消息并返回无效参数错误
  if (NULL != dst->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 获取分配器
  const rcutils_allocator_t *allocator = &src->allocator;

  // 创建临时 rmw_init_options_t 结构体，并将源结构体的内容复制到临时结构体中
  rmw_init_options_t tmp = *src;
  // 复制 enclave 字符串
  tmp.enclave = rcutils_strdup(tmp.enclave, *allocator);
  // 如果源 enclave 不为空，但复制后的临时 enclave 为空，则返回内存分配错误
  if (NULL != src->enclave && NULL == tmp.enclave) {
    return RMW_RET_BAD_ALLOC;
  }
  // 将安全选项设置为零初始化
  tmp.security_options = rmw_get_zero_initialized_security_options();
  // 复制安全选项
  rmw_ret_t ret =
      rmw_security_options_copy(&src->security_options, allocator, &tmp.security_options);
  // 如果复制安全选项失败，则释放临时 enclave 内存并返回错误代码
  if (RMW_RET_OK != ret) {
    allocator->deallocate(tmp.enclave, allocator->state);
    return ret;
  }
  *dst = tmp;
  return RMW_RET_OK;
}

/**
 * @brief 释放rmw_init_options_t结构体的内存并将其重置为零初始化状态。
 *
 * @param[in,out] init_options 指向要释放和重置的rmw_init_options_t结构体的指针。
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败。
 */
extern "C" rmw_ret_t rmw_init_options_fini(rmw_init_options_t *init_options) {
  // 检查init_options是否为空，如果为空则返回RMW_RET_INVALID_ARGUMENT错误
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);

  // 如果实现标识符为空，则设置错误消息并返回RMW_RET_INVALID_ARGUMENT错误
  if (NULL == init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 检查类型标识符是否匹配，如果不匹配则返回RMW_RET_INCORRECT_RMW_IMPLEMENTATION错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      init_options, init_options->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 获取分配器
  rcutils_allocator_t *allocator = &init_options->allocator;

  // 检查分配器是否有效，如果无效则返回RMW_RET_INVALID_ARGUMENT错误
  RCUTILS_CHECK_ALLOCATOR(allocator, return RMW_RET_INVALID_ARGUMENT);

  // 使用分配器释放enclave内存
  allocator->deallocate(init_options->enclave, allocator->state);

  // 结束安全选项并获取返回值
  rmw_ret_t ret = rmw_security_options_fini(&init_options->security_options, allocator);

  // 将init_options重置为零初始化状态
  *init_options = rmw_get_zero_initialized_init_options();

  // 返回操作结果
  return ret;
}

/**
 * @brief 将dds_guid_t转换为rmw_gid_t。
 *
 * @param[in] guid 输入的dds_guid_t类型GUID。
 * @param[out] gid 输出的rmw_gid_t类型GID。
 */
static void convert_guid_to_gid(const dds_guid_t &guid, rmw_gid_t &gid) {
  // 静态断言，确保rmw_gid_t足够大以容纳Cyclone DDS GUID
  static_assert(
      RMW_GID_STORAGE_SIZE >= sizeof(guid), "rmw_gid_t type too small for a Cyclone DDS GUID");

  // 将gid清零
  memset(&gid, 0, sizeof(gid));

  // 设置实现标识符
  gid.implementation_identifier = eclipse_cyclonedds_identifier;

  // 复制GUID数据到gid.data
  memcpy(gid.data, guid.v, sizeof(guid));
}

/**
 * @brief 获取实体的GID（全局唯一标识符）。
 *
 * @param[in] h 输入的dds_entity_t类型实体句柄。
 * @param[out] gid 输出的rmw_gid_t类型GID。
 */
static void get_entity_gid(dds_entity_t h, rmw_gid_t &gid) {
  // 定义dds_guid_t类型变量guid
  dds_guid_t guid;

  // 获取实体的GUID
  dds_get_guid(h, &guid);

  // 转换GUID并将其存储在gid中
  convert_guid_to_gid(guid, gid);
}

/**
 * @brief 解析用户数据，将其转换为键值对的形式
 *
 * @param[in] qos 指向dds_qos_t结构体的指针，包含了DDS实体的QoS策略信息
 * @return 返回一个std::map，其中key是字符串类型，value是uint8_t类型的vector
 */
static std::map<std::string, std::vector<uint8_t>> parse_user_data(const dds_qos_t *qos) {
  // 创建一个空的map用于存储解析后的用户数据
  std::map<std::string, std::vector<uint8_t>> map;
  void *ud;
  size_t udsz;

  // 如果成功获取用户数据，则进行处理
  if (dds_qget_userdata(qos, &ud, &udsz)) {
    // 将用户数据转换为uint8_t类型的vector
    std::vector<uint8_t> udvec(static_cast<uint8_t *>(ud), static_cast<uint8_t *>(ud) + udsz);
    // 释放用户数据内存
    dds_free(ud);
    // 使用rmw的parse_key_value函数解析用户数据，并将结果存储在map中
    map = rmw::impl::cpp::parse_key_value(udvec);
  }

  // 返回解析后的用户数据map
  return map;
}

/**
 * @brief 获取用户数据中指定key的值
 *
 * @param[in] qos 指向dds_qos_t结构体的指针，包含了DDS实体的QoS策略信息
 * @param[in] key 要查找的key字符串
 * @param[out] value 查找到的key对应的值，以字符串形式返回
 * @return 如果找到指定key，则返回true，否则返回false
 */
static bool get_user_data_key(const dds_qos_t *qos, const std::string key, std::string &value) {
  // 检查qos是否为非空指针
  if (qos != nullptr) {
    // 解析用户数据并获取map
    auto map = parse_user_data(qos);
    // 在map中查找指定的key
    auto name_found = map.find(key);

    // 如果找到了指定的key
    if (name_found != map.end()) {
      // 将找到的值转换为字符串，并赋值给value参数
      value = std::string(name_found->second.begin(), name_found->second.end());
      // 返回true表示找到了指定key
      return true;
    }
  }

  // 如果没有找到指定key，则返回false
  return false;
}

/** @brief 处理参与者实体信息的回调函数
 *
 * @param[in] reader DDS实体的reader对象
 * @param[in] arg 指向rmw_context_impl_t结构体的指针，包含了ROS2上下文的实现细节
 */
static void handle_ParticipantEntitiesInfo(dds_entity_t reader, void *arg) {
  // 忽略reader参数，避免编译器警告
  static_cast<void>(reader);
  // 将arg参数转换为rmw_context_impl_t类型的指针
  rmw_context_impl_t *impl = static_cast<rmw_context_impl_t *>(arg);
  // 创建一个ParticipantEntitiesInfo类型的消息对象
  ParticipantEntitiesInfo msg;
  bool taken;

  // 循环读取消息，直到没有新消息为止
  while (rmw_take(impl->common.sub, &msg, &taken, nullptr) == RMW_RET_OK && taken) {
    // 由于订阅QoS策略的原因，本地发布的数据会被过滤掉
    // 使用graph_cache的update_participant_entities方法更新参与者实体信息
    impl->common.graph_cache.update_participant_entities(msg);
  }
}

/** @brief 处理DCPSParticipant的回调函数
 *
 * @param[in] reader 读取到的DDS实体
 * @param[in] arg 传递给回调函数的参数，这里是rmw_context_impl_t类型
 */
static void handle_DCPSParticipant(dds_entity_t reader, void *arg) {
  // 将arg转换为rmw_context_impl_t类型
  rmw_context_impl_t *impl = static_cast<rmw_context_impl_t *>(arg);
  dds_sample_info_t si;
  void *raw = NULL;

  // 循环处理接收到的数据
  while (dds_take(reader, &raw, &si, 1, 1) == 1) {
    auto s = static_cast<const dds_builtintopic_participant_t *>(raw);
    rmw_gid_t gid;

    // 将GUID转换为GID
    convert_guid_to_gid(s->key, gid);

    // 如果GID与本地参与者的GID相同，则忽略
    if (memcmp(&gid, &impl->common.gid, sizeof(gid)) == 0) {
      // ignore the local participant
    } else if (si.instance_state != DDS_ALIVE_INSTANCE_STATE) {
      // 如果实例状态不是活动状态，则从图缓存中移除参与者
      impl->common.graph_cache.remove_participant(gid);
    } else if (si.valid_data) {
      std::string enclave;

      // 获取用户数据中的enclave键值
      if (get_user_data_key(s->qos, "enclave", enclave)) {
        // 将参与者添加到图缓存中
        impl->common.graph_cache.add_participant(gid, enclave);
      }
    }

    // 归还已处理的数据
    dds_return_loan(reader, &raw, 1);
  }
}

/** @brief 处理内置主题端点的回调函数
 *
 * @param[in] reader 读取到的DDS实体
 * @param[in] impl rmw_context_impl_t类型的指针
 * @param[in] is_reader 是否为读者端点，true表示是读者端点，false表示是写者端点
 */
static void handle_builtintopic_endpoint(
    dds_entity_t reader,       //
    rmw_context_impl_t *impl,  //
    bool is_reader) {
  dds_sample_info_t si;
  void *raw = NULL;

  // 循环处理接收到的数据
  while (dds_take(reader, &raw, &si, 1, 1) == 1) {
    auto s = static_cast<const dds_builtintopic_endpoint_t *>(raw);
    rmw_gid_t gid;

    // 将GUID转换为GID
    convert_guid_to_gid(s->key, gid);

    // 如果实例状态不是活动状态，则从图缓存中移除实体
    if (si.instance_state != DDS_ALIVE_INSTANCE_STATE) {
      impl->common.graph_cache.remove_entity(gid, is_reader);
    } else if (si.valid_data && strncmp(s->topic_name, "DCPS", 4) != 0) {
      rmw_qos_profile_t qos_profile = rmw_qos_profile_unknown;
      rmw_gid_t ppgid;

      // 将DDS QoS转换为RMW QoS
      dds_qos_to_rmw_qos(s->qos, &qos_profile);

      // 将参与者GUID转换为GID
      convert_guid_to_gid(s->participant_key, ppgid);

      rosidl_type_hash_t type_hash = rosidl_get_zero_initialized_type_hash();
      void *userdata;
      size_t userdata_size;

      // 获取用户数据
      if (dds_qget_userdata(s->qos, &userdata, &userdata_size)) {
        RCPPUTILS_SCOPE_EXIT(dds_free(userdata));

        // 解析类型哈希值
        if (RMW_RET_OK !=
            rmw_dds_common::parse_type_hash_from_user_data(
                reinterpret_cast<const uint8_t *>(userdata), userdata_size, type_hash)) {
          RCUTILS_LOG_WARN_NAMED(
              "rmw_cyclonedds_cpp",
              "Failed to parse type hash for topic '%s' with type '%s' from USER_DATA '%*s'.",
              s->topic_name, s->type_name, static_cast<int>(userdata_size),
              reinterpret_cast<char *>(userdata));
          type_hash = rosidl_get_zero_initialized_type_hash();
        }
      }

      // 将实体添加到图缓存中
      impl->common.graph_cache.add_entity(
          gid, std::string(s->topic_name), std::string(s->type_name), type_hash, ppgid, qos_profile,
          is_reader);
    }

    // 归还已处理的数据
    dds_return_loan(reader, &raw, 1);
  }
}

/** @brief 处理DCPSSubscription的函数
 *
 * @param[in] reader 一个dds_entity_t类型的reader实体，用于读取订阅者信息
 * @param[in] arg 一个指向rmw_context_impl_t类型的指针，用于存储ROS2上下文实现的信息
 */
static void handle_DCPSSubscription(dds_entity_t reader, void *arg) {
  // 将arg转换为rmw_context_impl_t类型的指针
  rmw_context_impl_t *impl = static_cast<rmw_context_impl_t *>(arg);
  // 调用handle_builtintopic_endpoint函数处理订阅者端点信息
  handle_builtintopic_endpoint(reader, impl, true);
}

/** @brief 处理DCPSPublication的函数
 *
 * @param[in] reader 一个dds_entity_t类型的reader实体，用于读取发布者信息
 * @param[in] arg 一个指向rmw_context_impl_t类型的指针，用于存储ROS2上下文实现的信息
 */
static void handle_DCPSPublication(dds_entity_t reader, void *arg) {
  // 将arg转换为rmw_context_impl_t类型的指针
  rmw_context_impl_t *impl = static_cast<rmw_context_impl_t *>(arg);
  // 调用handle_builtintopic_endpoint函数处理发布者端点信息
  handle_builtintopic_endpoint(reader, impl, false);
}

/**
 * @brief 用于发现线程的函数，处理参与者、订阅者和发布者实体的信息。
 *
 * @param[in] impl 指向 rmw_context_impl_t 结构体的指针，包含了 ROS2 的上下文实现。
 */
static void discovery_thread(rmw_context_impl_t *impl) {
  // 将 impl->common.sub->data 转换为 CddsSubscription 类型的指针
  const CddsSubscription *sub = static_cast<const CddsSubscription *>(impl->common.sub->data);
  // 将 impl->common.listener_thread_gc->data 转换为 CddsGuardCondition 类型的指针
  const CddsGuardCondition *gc =
      static_cast<const CddsGuardCondition *>(impl->common.listener_thread_gc->data);
  dds_entity_t ws;
  // 删除 ppant 会同时删除 waitset，所以在出错时无需在此处删除 waitset，但这样更卫生
  if ((ws = dds_create_waitset(DDS_CYCLONEDDS_HANDLE)) < 0) {
    RCUTILS_SAFE_FWRITE_TO_STDERR(
        "ros discovery info listener thread: failed to create waitset, will shutdown ...\n");
    return;
  }
  // 我猜我可以通过某种方式附加 lambda 函数，这肯定会更优雅，但这避免了处理 C++
  // 中涉及的怪异性，并且也能正常工作。
  std::vector<std::pair<dds_entity_t, std::function<void(dds_entity_t, rmw_context_impl_t *)>>>
      entries = {
          {gc->gcondh, nullptr},
          {sub->enth, handle_ParticipantEntitiesInfo},
          {impl->rd_participant, handle_DCPSParticipant},
          {impl->rd_subscription, handle_DCPSSubscription},
          {impl->rd_publication, handle_DCPSPublication},
      };
  // 遍历 entries
  for (size_t i = 0; i < entries.size(); i++) {
    if (entries[i].second != nullptr &&
        dds_set_status_mask(entries[i].first, DDS_DATA_AVAILABLE_STATUS) < 0) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
          "ros discovery info listener thread: failed to set reader status masks, "
          "will shutdown ...\n");
      return;
    }
    // 将实体附加到 waitset
    if (dds_waitset_attach(ws, entries[i].first, static_cast<dds_attach_t>(i)) < 0) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
          "ros discovery info listener thread: failed to attach entities to waitset, "
          "will shutdown ...\n");
      dds_delete(ws);
      return;
    }
  }
  std::vector<dds_attach_t> xs(5);
  // 当线程正在运行时，执行以下操作
  while (impl->common.thread_is_running.load()) {
    dds_return_t n;
    // 等待 waitset 中的事件
    if ((n = dds_waitset_wait(ws, xs.data(), xs.size(), DDS_INFINITY)) < 0) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
          "ros discovery info listener thread: wait failed, will shutdown ...\n");
      return;
    }
    // 遍历返回的事件并调用相应的处理函数
    for (int32_t i = 0; i < n; i++) {
      if (entries[xs[i]].second) {
        entries[xs[i]].second(entries[xs[i]].first, impl);
      }
    }
  }
  // 删除 waitset
  dds_delete(ws);
}

/**
 * @brief 启动发现线程
 *
 * @param[in] impl 指向rmw_context_impl_t结构体的指针
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
static rmw_ret_t discovery_thread_start(rmw_context_impl_t *impl) {
  auto common_context = &impl->common;                            // 获取通用上下文
  common_context->thread_is_running.store(true);                  // 设置线程运行状态为true
  common_context->listener_thread_gc = create_guard_condition();  // 创建守护条件
  if (common_context->listener_thread_gc) {
    try {
      common_context->listener_thread = std::thread(discovery_thread, impl);  // 创建发现线程
      return RMW_RET_OK;
    } catch (const std::exception &exc) {
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("Failed to create std::thread: %s", exc.what());
    } catch (...) {
      RMW_SET_ERROR_MSG("Failed to create std::thread");
    }
  } else {
    RMW_SET_ERROR_MSG("Failed to create guard condition");
  }
  common_context->thread_is_running.store(false);  // 设置线程运行状态为false
  if (common_context->listener_thread_gc) {
    if (RMW_RET_OK != destroy_guard_condition(common_context->listener_thread_gc)) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(
          __function__) ":" RCUTILS_STRINGIFY(__LINE__) ": Failed to destroy guard condition");
    }
  }
  return RMW_RET_ERROR;
}

/**
 * @brief 停止发现线程
 *
 * @param[in] common_context 引用rmw_dds_common::Context类型的通用上下文
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
static rmw_ret_t discovery_thread_stop(rmw_dds_common::Context &common_context) {
  if (common_context.thread_is_running.exchange(
          false)) {  // 将线程运行状态设置为false并检查之前的状态
    rmw_ret_t rmw_ret =
        rmw_trigger_guard_condition(common_context.listener_thread_gc);  // 触发守护条件
    if (RMW_RET_OK != rmw_ret) {
      return rmw_ret;
    }
    try {
      common_context.listener_thread.join();  // 等待发现线程结束
    } catch (const std::exception &exc) {
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("Failed to join std::thread: %s", exc.what());
      return RMW_RET_ERROR;
    } catch (...) {
      RMW_SET_ERROR_MSG("Failed to join std::thread");
      return RMW_RET_ERROR;
    }
    rmw_ret = destroy_guard_condition(common_context.listener_thread_gc);  // 销毁守护条件
    if (RMW_RET_OK != rmw_ret) {
      return rmw_ret;
    }
  }
  return RMW_RET_OK;
}

/**
 * @brief 检查并创建指定的DDS域
 *
 * @param[in] did DDS域ID
 * @param[in] localhost_only_option 本地主机选项，用于确定是否仅在本地主机上创建节点
 * @return true 成功创建或增加引用计数
 * @return false 创建失败或本地主机设置不匹配
 */
static bool check_create_domain(dds_domainid_t did, rmw_localhost_only_t localhost_only_option) {
  // 判断是否仅在本地主机上创建节点
  const bool localhost_only = (localhost_only_option == RMW_LOCALHOST_ONLY_ENABLED);

  // 对全局Cdds实例的domains_lock进行加锁，确保线程安全
  std::lock_guard<std::mutex> lock(gcdds().domains_lock);

  // 获取或创建指定的DDS域
  CddsDomain &dom = gcdds().domains[did];

  // 如果引用计数不为0，说明该域已存在
  if (dom.refcount != 0) {
    // 检查本地主机设置是否匹配
    if (localhost_only == dom.localhost_only) {
      // 增加引用计数并返回true
      dom.refcount++;
      return true;
    } else {
      // 本地主机设置不匹配，记录错误并返回false
      RCUTILS_LOG_ERROR_NAMED(
          "rmw_cyclonedds_cpp",
          "rmw_create_node: attempt at creating localhost-only and non-localhost-only nodes "
          "in the same domain");
      return false;
    }
  } else {
    // 设置引用计数为1
    dom.refcount = 1;
    // 设置本地主机选项
    dom.localhost_only = localhost_only;

    // 根据本地主机选项设置网络接口地址
    std::string config =
        localhost_only
            ? "<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"127.0.0.1\"/>"
              "</Interfaces></General></Domain></CycloneDDS>,"
            : "";

    // 获取环境变量CYCLONEDDS_URI的值
    const char *get_env_error;
    const char *config_from_env;
    if ((get_env_error = rcutils_get_env("CYCLONEDDS_URI", &config_from_env)) == nullptr) {
      config += std::string(config_from_env);
    } else {
      // 获取环境变量失败，记录错误并返回false
      RCUTILS_LOG_ERROR_NAMED(
          "rmw_cyclonedds_cpp",
          "rmw_create_node: failed to retrieve CYCLONEDDS_URI environment variable, error %s",
          get_env_error);
      gcdds().domains.erase(did);
      return false;
    }

    // 创建指定的DDS域
    if ((dom.domain_handle = dds_create_domain(did, config.c_str())) < 0) {
      // 创建失败，记录错误并返回false
      RCUTILS_LOG_ERROR_NAMED(
          "rmw_cyclonedds_cpp", "rmw_create_node: failed to create domain, error %s",
          dds_strretcode(dom.domain_handle));
      gcdds().domains.erase(did);
      return false;
    } else {
      // 创建成功，返回true
      return true;
    }
  }
}

/**
 * @brief 检查并销毁指定的域 (Check and destroy the specified domain)
 *
 * @param[in] domain_id 要检查和销毁的域ID (The domain ID to check and destroy)
 */
static void check_destroy_domain(dds_domainid_t domain_id) {
  // 如果域ID不是无效值 (If the domain ID is not invalid)
  if (domain_id != UINT32_MAX) {
    // 对全局CDDS对象的域锁进行加锁，确保线程安全 (Lock the domains_lock of the global CDDS object
    // to ensure thread safety)
    std::lock_guard<std::mutex> lock(gcdds().domains_lock);

    // 获取对应域ID的CddsDomain引用 (Get the reference to the CddsDomain with the corresponding
    // domain ID)
    CddsDomain &dom = gcdds().domains[domain_id];

    // 断言：域的引用计数大于0 (Assertion: The reference count of the domain is greater than 0)
    assert(dom.refcount > 0);

    // 减少域的引用计数 (Decrease the reference count of the domain)
    if (--dom.refcount == 0) {
      // 如果引用计数为0，则删除域 (If the reference count is 0, delete the domain)
      static_cast<void>(dds_delete(dom.domain_handle));

      // 从全局CDDS对象的域映射中移除该域ID (Remove the domain ID from the domains map of the global
      // CDDS object)
      gcdds().domains.erase(domain_id);
    }
  }
}

/**
 * @brief 尝试设置所有需要启用DDS安全性的QoS属性
 *
 * @param[out] qos 一个指向dds_qos_t类型的指针，用于存储配置后的QoS属性
 * @param[in] security_options 一个指向rmw_security_options_t类型的指针，包含安全选项信息
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
static rmw_ret_t configure_qos_for_security(
    dds_qos_t *qos, const rmw_security_options_t *security_options) {
#if RMW_SUPPORT_SECURITY
  // 创建一个存储安全文件路径的无序映射
  std::unordered_map<std::string, std::string> security_files;

  // 如果安全根路径为空，则返回不支持
  if (security_options->security_root_path == nullptr) {
    return RMW_RET_UNSUPPORTED;
  }

  // 获取安全文件路径，如果找不到所有安全文件，则记录日志并返回不支持
  if (!rmw_dds_common::get_security_files(
          "file:", security_options->security_root_path, security_files)) {
    RCUTILS_LOG_INFO_NAMED("rmw_cyclonedds_cpp", "could not find all security files");
    return RMW_RET_UNSUPPORTED;
  }

  // 设置QoS属性以启用DDS安全性
  dds_qset_prop(qos, "dds.sec.auth.identity_ca", security_files["IDENTITY_CA"].c_str());
  dds_qset_prop(qos, "dds.sec.auth.identity_certificate", security_files["CERTIFICATE"].c_str());
  dds_qset_prop(qos, "dds.sec.auth.private_key", security_files["PRIVATE_KEY"].c_str());
  dds_qset_prop(qos, "dds.sec.access.permissions_ca", security_files["PERMISSIONS_CA"].c_str());
  dds_qset_prop(qos, "dds.sec.access.governance", security_files["GOVERNANCE"].c_str());
  dds_qset_prop(qos, "dds.sec.access.permissions", security_files["PERMISSIONS"].c_str());

  // 设置安全认证库相关属性
  dds_qset_prop(qos, "dds.sec.auth.library.path", "dds_security_auth");
  dds_qset_prop(qos, "dds.sec.auth.library.init", "init_authentication");
  dds_qset_prop(qos, "dds.sec.auth.library.finalize", "finalize_authentication");

  // 设置加密库相关属性
  dds_qset_prop(qos, "dds.sec.crypto.library.path", "dds_security_crypto");
  dds_qset_prop(qos, "dds.sec.crypto.library.init", "init_crypto");
  dds_qset_prop(qos, "dds.sec.crypto.library.finalize", "finalize_crypto");

  // 设置访问控制库相关属性
  dds_qset_prop(qos, "dds.sec.access.library.path", "dds_security_ac");
  dds_qset_prop(qos, "dds.sec.access.library.init", "init_access_control");
  dds_qset_prop(qos, "dds.sec.access.library.finalize", "finalize_access_control");

  // 如果存在CRL文件，则设置相应的QoS属性
  if (security_files.count("CRL") > 0) {
    dds_qset_prop(qos, "org.eclipse.cyclonedds.sec.auth.crl", security_files["CRL"].c_str());
  }

  // 返回成功
  return RMW_RET_OK;
#else
  // 如果不支持安全性，忽略QoS参数并检查是否强制执行安全性
  (void)qos;
  if (security_options->enforce_security == RMW_SECURITY_ENFORCEMENT_ENFORCE) {
    // 设置错误消息，提示Cyclone DDS未启用安全支持
    RMW_SET_ERROR_MSG(
        "Security was requested but the Cyclone DDS being used does not have security "
        "support enabled. Recompile Cyclone DDS with the '-DENABLE_SECURITY=ON' "
        "CMake option");
  }
  // 返回不支持
  return RMW_RET_UNSUPPORTED;
#endif
}

/**
 * @brief 初始化 ROS2 RMW 层的上下文实现
 *
 * @param[in] options 指向 rmw_init_options_t 结构体的指针，包含初始化选项
 * @param[in] domain_id 用于 DDS 通信的域 ID
 * @return rmw_ret_t 返回操作结果状态码
 */
rmw_ret_t rmw_context_impl_s::init(rmw_init_options_t *options, size_t domain_id) {
  // 使用互斥锁保护初始化过程
  std::lock_guard<std::mutex> guard(initialization_mutex);
  if (0u != this->node_count) {
    // 如果已经完成了初始化，则增加节点计数并返回成功
    this->node_count++;
    return RMW_RET_OK;
  }

  /* 在创建参与者成功或失败之后持有 domains_lock：
     否则，在 Cyclone 实现 dds_create_domain 的原始版本中，
     rmw_destroy_node 删除最后一个参与者并拆除域时会发生竞争。 */
  this->domain_id = static_cast<dds_domainid_t>(domain_id);

  // 检查是否可以创建域
  if (!check_create_domain(this->domain_id, options->localhost_only)) {
    return RMW_RET_ERROR;
  }

  // 创建 QoS 配置并在出错时进行清理
  std::unique_ptr<dds_qos_t, std::function<void(dds_qos_t *)>> ppant_qos(
      dds_create_qos(), &dds_delete_qos);
  if (ppant_qos == nullptr) {
    this->clean_up();
    return RMW_RET_BAD_ALLOC;
  }
  // 设置用户数据
  std::string user_data =
      std::string("enclave=") + std::string(options->enclave) + std::string(";");
  dds_qset_userdata(ppant_qos.get(), user_data.c_str(), user_data.size());
  // 配置安全相关的 QoS 设置
  if (configure_qos_for_security(ppant_qos.get(), &options->security_options) != RMW_RET_OK) {
    if (RMW_SECURITY_ENFORCEMENT_ENFORCE == options->security_options.enforce_security) {
      this->clean_up();
      return RMW_RET_ERROR;
    }
  }

  // 创建 DDS 参与者
  this->ppant = dds_create_participant(this->domain_id, ppant_qos.get(), nullptr);
  if (this->ppant < 0) {
    this->clean_up();
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp", "rmw_create_node: failed to create DDS participant");
    return RMW_RET_ERROR;
  }
  get_entity_gid(this->ppant, this->ppant_gid);

  /* 创建用于监控发现的 DDS 内置主题的读取器 */
  if ((this->rd_participant = dds_create_reader(
           this->ppant, DDS_BUILTIN_TOPIC_DCPSPARTICIPANT, nullptr, nullptr)) < 0) {
    this->clean_up();
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp", "rmw_create_node: failed to create DCPSParticipant reader");
    return RMW_RET_ERROR;
  }
  if ((this->rd_subscription = dds_create_reader(
           this->ppant, DDS_BUILTIN_TOPIC_DCPSSUBSCRIPTION, nullptr, nullptr)) < 0) {
    this->clean_up();
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp", "rmw_create_node: failed to create DCPSSubscription reader");
    return RMW_RET_ERROR;
  }
  if ((this->rd_publication = dds_create_reader(
           this->ppant, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr)) < 0) {
    this->clean_up();
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp", "rmw_create_node: failed to create DCPSPublication reader");
    return RMW_RET_ERROR;
  }
  /* 创建将用于所有 DDS 写入器/读取器的 DDS 发布者/订阅者对象
     以及用于 RMW 发布者/订阅者。 */
  if ((this->dds_pub = dds_create_publisher(this->ppant, nullptr, nullptr)) < 0) {
    this->clean_up();
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp", "rmw_create_node: failed to create DDS publisher");
    return RMW_RET_ERROR;
  }
  if ((this->dds_sub = dds_create_subscriber(this->ppant, nullptr, nullptr)) < 0) {
    this->clean_up();
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp", "rmw_create_node: failed to create DDS subscriber");
    return RMW_RET_ERROR;
  }

  // 设置 QoS 配置文件
  rmw_qos_profile_t pubsub_qos = rmw_qos_profile_default;
  pubsub_qos.avoid_ros_namespace_conventions = true;
  pubsub_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  pubsub_qos.depth = 1;
  pubsub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  pubsub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  /* 创建用于 rmw_dds_common 发现的
     RMW 发布者/订阅者/保护条件 */
  rmw_publisher_options_t publisher_options = rmw_get_default_publisher_options();
  this->common.pub = create_publisher(
      this->ppant, this->dds_pub,
      rosidl_typesupport_cpp::get_message_type_support_handle<ParticipantEntitiesInfo>(),
      "ros_discovery_info", &pubsub_qos, &publisher_options);
  if (this->common.pub == nullptr) {
    this->clean_up();
    return RMW_RET_ERROR;
  }

  rmw_subscription_options_t subscription_options = rmw_get_default_subscription_options();
  subscription_options.ignore_local_publications = true;
  // FIXME: keyed topics => KEEP_LAST and depth 1.
  pubsub_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  this->common.sub = create_subscription(
      this->ppant, this->dds_sub,
      rosidl_typesupport_cpp::get_message_type_support_handle<ParticipantEntitiesInfo>(),
      "ros_discovery_info", &pubsub_qos, &subscription_options);
  if (this->common.sub == nullptr) {
    this->clean_up();
    return RMW_RET_ERROR;
  }

  // 创建保护条件
  this->common.graph_guard_condition = create_guard_condition();
  if (this->common.graph_guard_condition == nullptr) {
    this->clean_up();
    return RMW_RET_BAD_ALLOC;
  }

  // 设置图缓存的回调函数
  this->common.graph_cache.set_on_change_callback(
      [guard_condition = this->common.graph_guard_condition]() {
        rmw_ret_t ret = rmw_trigger_guard_condition(guard_condition);
        if (ret != RMW_RET_OK) {
          RMW_SET_ERROR_MSG("graph cache on_change_callback failed to trigger guard condition");
        }
      });

  // 将参与者添加到图缓存中
  get_entity_gid(this->ppant, this->common.gid);
  this->common.graph_cache.add_participant(this->common.gid, options->enclave);

  /**
   * @brief 该函数用于启动发现线程并进行错误处理。
   * 详细说明：
   * - 也可以使用一组监听器而不是线程来维护图缓存。
   * -
   * 本地发布的样本不应该进入读取器，因此不应该有由于图缓存的互斥锁已经被（例如）rmw_create_node锁定而导致的死锁。
   * -
   * 无论图缓存实现如何，它都不应该涉及比本地状态更新和触发保护条件更多的操作，所以这应该是安全的。
   * - 然而，图缓存更新可能会很耗时，因此在从网络接收数据的线程上执行这些操作可能不明智。
   */
  rmw_ret_t ret;
  if ((ret = discovery_thread_start(this)) != RMW_RET_OK) {
    this->clean_up();
    return ret;
  }
  // 增加节点计数
  ++this->node_count;
  return RMW_RET_OK;
}

/**
 * @brief 清理rmw_context_impl_t对象的资源。
 */
void rmw_context_impl_t::clean_up() {
  // 停止发现线程
  discovery_thread_stop(common);
  // 清除图缓存的变更回调
  common.graph_cache.clear_on_change_callback();
  // 销毁图保护条件（如果存在）
  if (common.graph_guard_condition) {
    destroy_guard_condition(common.graph_guard_condition);
    common.graph_guard_condition = nullptr;
  }
  // 销毁发布者（如果存在）
  if (common.pub) {
    destroy_publisher(common.pub);
    common.pub = nullptr;
  }
  // 销毁订阅者（如果存在）
  if (common.sub) {
    destroy_subscription(common.sub);
    common.sub = nullptr;
  }
  // 销毁域（如果存在）
  if (ppant > 0 && dds_delete(ppant) < 0) {
    RCUTILS_SAFE_FWRITE_TO_STDERR("Failed to destroy domain in destructor\n");
  }
  ppant = 0;

  // 检查并销毁域
  check_destroy_domain(domain_id);
}

/**
 * @brief 结束rmw_context_impl_s对象的生命周期。
 *
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败。
 */
rmw_ret_t rmw_context_impl_s::fini() {
  // 使用互斥锁保护初始化过程
  std::lock_guard<std::mutex> guard(initialization_mutex);
  // 减少节点计数
  if (0u != --this->node_count) {
    // 如果节点计数不为0，则不应该进行销毁操作
    return RMW_RET_OK;
  }
  // 清理资源
  this->clean_up();
  // 返回成功代码
  return RMW_RET_OK;
}

/**
 * @brief 初始化并分配样本内存
 *
 * @tparam entityT 实体类型
 * @param[in] entity 实体引用
 * @param[in] sample_size 样本大小
 * @param[in] alloc_on_heap 是否在堆上分配内存，默认为 false
 * @return 返回分配的内存指针，如果失败则返回 nullptr
 */
template <typename entityT>
static void *init_and_alloc_sample(
    entityT &entity, const uint32_t sample_size, const bool alloc_on_heap = false) {
  // 初始化数据分配器
  if (alloc_on_heap) {
    RET_EXPECTED(
        dds_data_allocator_init_heap(&entity->data_allocator), DDS_RETCODE_OK,
        "Reader data allocator initialization failed for heap", return nullptr);
  } else {
    RET_EXPECTED(
        dds_data_allocator_init(entity->enth, &entity->data_allocator), DDS_RETCODE_OK,
        "Writer allocator initialisation failed", return nullptr);
  }
  // 为消息+头部分配内存
  // 头部将被初始化，chunk 指针将被返回
  auto chunk_ptr = dds_data_allocator_alloc(&entity->data_allocator, sample_size);
  RMW_CHECK_FOR_NULL_WITH_MSG(chunk_ptr, "Failed to get loan", return nullptr);
  // 不要初始化消息内存，因为这里分配的内存无论如何都会被用户填充，
  // 在此处初始化内存只会对零拷贝路径产生不必要的性能损失
  return chunk_ptr;
}

/**
 * @brief 结束并释放样本内存
 *
 * @tparam entityT 实体类型
 * @param[in] entity 实体引用
 * @param[in] loaned_message 被借用的消息指针
 * @return 返回 rmw_ret_t 类型的结果，成功返回 RMW_RET_OK，失败返回 RMW_RET_ERROR
 */
template <typename entityT>
static rmw_ret_t fini_and_free_sample(entityT &entity, void *loaned_message) {
  // 结束消息
  rmw_cyclonedds_cpp::fini_message(&entity->type_supports, loaned_message);
  // 释放消息内存
  RET_EXPECTED(
      dds_data_allocator_free(&entity->data_allocator, loaned_message), DDS_RETCODE_OK,
      "Failed to free the loaned message", return RMW_RET_ERROR);
  // 结束分配器
  RET_EXPECTED(
      dds_data_allocator_fini(&entity->data_allocator), DDS_RETCODE_OK,
      "Failed to fini data allocator", return RMW_RET_ERROR);
  return RMW_RET_OK;
}

/**
 * @brief 初始化 ROS2 的 RMW 层
 *
 * @param[in] options 指向初始化选项的指针，不能为空
 * @param[out] context 指向要初始化的上下文的指针，不能为空
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_init(const rmw_init_options_t *options, rmw_context_t *context) {
  // 定义返回值变量
  rmw_ret_t ret;

  // 检查输入参数是否为空
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  // 检查实现标识符是否已初始化
  RMW_CHECK_FOR_NULL_WITH_MSG(
      options->implementation_identifier, "expected initialized init options",
      return RMW_RET_INVALID_ARGUMENT);

  // 检查类型标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      options, options->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查 enclave 是否非空
  RMW_CHECK_FOR_NULL_WITH_MSG(
      options->enclave, "expected non-null enclave", return RMW_RET_INVALID_ARGUMENT);

  // 检查上下文的实现标识符是否为空
  if (NULL != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 检查域 ID 是否在有效范围内
  if (options->domain_id >= UINT32_MAX && options->domain_id != RMW_DEFAULT_DOMAIN_ID) {
    RCUTILS_LOG_ERROR_NAMED("rmw_cyclonedds_cpp", "rmw_create_node: domain id out of range");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 在函数退出时恢复上下文
  auto restore_context =
      rcpputils::make_scope_exit([context]() { *context = rmw_get_zero_initialized_context(); });

  // 设置上下文属性
  context->instance_id = options->instance_id;
  context->implementation_identifier = eclipse_cyclonedds_identifier;

  // 使用合理的域 ID，不对 RMW_DEFAULT_DOMAIN_ID 进行特殊处理
  context->actual_domain_id = RMW_DEFAULT_DOMAIN_ID != options->domain_id ? options->domain_id : 0u;

  // 为上下文实现分配内存
  context->impl = new (std::nothrow) rmw_context_impl_t();
  if (nullptr == context->impl) {
    RMW_SET_ERROR_MSG("failed to allocate context impl");
    return RMW_RET_BAD_ALLOC;
  }

  // 在函数退出时清理实现
  auto cleanup_impl = rcpputils::make_scope_exit([context]() { delete context->impl; });

  // 复制初始化选项到上下文
  if ((ret = rmw_init_options_copy(options, &context->options)) != RMW_RET_OK) {
    return ret;
  }

  // 取消清理操作
  cleanup_impl.cancel();
  restore_context.cancel();

  // 返回成功状态
  return RMW_RET_OK;
}

/**
 * @brief 关闭 ROS2 RMW 层的上下文
 *
 * @param[in,out] context 指向要关闭的 rmw_context_t 结构体的指针
 * @return rmw_ret_t 返回操作结果，成功返回 RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_shutdown(rmw_context_t *context) {
  // 检查 context 是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  // 检查 context->impl 是否为空，如果为空则返回预期已初始化上下文的错误信息
  RMW_CHECK_FOR_NULL_WITH_MSG(
      context->impl, "expected initialized context", return RMW_RET_INVALID_ARGUMENT);

  // 检查 context 的实现标识符是否与 CycloneDDS 标识符匹配，如果不匹配则返回错误的 RMW 实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      context, context->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 设置上下文为关闭状态
  context->impl->is_shutdown = true;

  // 返回操作成功
  return RMW_RET_OK;
}

/**
 * @brief 销毁 ROS2 RMW 层的上下文
 *
 * @param[in,out] context 指向要销毁的 rmw_context_t 结构体的指针
 * @return rmw_ret_t 返回操作结果，成功返回 RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_context_fini(rmw_context_t *context) {
  // 检查 context 是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  // 检查 context->impl 是否为空，如果为空则返回预期已初始化上下文的错误信息
  RMW_CHECK_FOR_NULL_WITH_MSG(
      context->impl, "expected initialized context", return RMW_RET_INVALID_ARGUMENT);

  // 检查 context 的实现标识符是否与 CycloneDDS 标识符匹配，如果不匹配则返回错误的 RMW 实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      context, context->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查上下文是否已关闭，如果没有关闭则设置错误消息并返回无效参数错误
  if (!context->impl->is_shutdown) {
    RMW_SET_ERROR_MSG("context has not been shutdown");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 销毁上下文的初始化选项
  rmw_ret_t ret = rmw_init_options_fini(&context->options);

  // 删除上下文的实现对象
  delete context->impl;

  // 将上下文重置为零初始化状态
  *context = rmw_get_zero_initialized_context();

  // 返回操作结果
  return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    NODES                                                          ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 创建一个 ROS2 节点
 *
 * @param[in] context 指向 rmw_context_t 结构体的指针，包含了 ROS2 上下文信息
 * @param[in] name 节点名称，必须是有效的 ROS2 节点名
 * @param[in] namespace_ 节点所在的命名空间，必须是有效的 ROS2 命名空间
 * @return rmw_node_t* 成功时返回指向新创建节点的指针，失败时返回 nullptr
 */
extern "C" rmw_node_t *rmw_create_node(
    rmw_context_t *context, const char *name, const char *namespace_) {
  // 检查 context 是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  // 检查 context 的实现标识符是否与 CycloneDDS 匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      context, context->implementation_identifier, eclipse_cyclonedds_identifier, return nullptr);
  // 检查 context 的实现是否为空
  RMW_CHECK_FOR_NULL_WITH_MSG(context->impl, "expected initialized context", return nullptr);
  // 检查 context 是否已经关闭
  if (context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has been shutdown");
    return nullptr;
  }

  // 验证节点名称的有效性
  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char *reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node name: %s", reason);
    return nullptr;
  }
  // 验证命名空间的有效性
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char *reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node namespace: %s", reason);
    return nullptr;
  }

  // 初始化 context
  ret = context->impl->init(&context->options, context->actual_domain_id);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  // 当退出作用域时，如果没有取消，则调用 context 的 fini 方法
  auto finalize_context = rcpputils::make_scope_exit([context]() { context->impl->fini(); });

  // 创建 CddsNode 实例
  std::unique_ptr<CddsNode> node_impl(new (std::nothrow) CddsNode());
  RET_ALLOC_X(node_impl, return nullptr);

  // 分配 rmw_node_t 结构体内存
  rmw_node_t *node = rmw_node_allocate();
  RET_ALLOC_X(node, return nullptr);
  // 当退出作用域时，如果没有取消，则释放节点相关资源
  auto cleanup_node = rcpputils::make_scope_exit([node]() {
    rmw_free(const_cast<char *>(node->name));
    rmw_free(const_cast<char *>(node->namespace_));
    rmw_node_free(node);
  });

  // 分配并设置节点名称
  node->name = static_cast<const char *>(rmw_allocate(sizeof(char) * strlen(name) + 1));
  RET_ALLOC_X(node->name, return nullptr);
  memcpy(const_cast<char *>(node->name), name, strlen(name) + 1);

  // 分配并设置节点命名空间
  node->namespace_ = static_cast<const char *>(rmw_allocate(sizeof(char) * strlen(namespace_) + 1));
  RET_ALLOC_X(node->namespace_, return nullptr);
  memcpy(const_cast<char *>(node->namespace_), namespace_, strlen(namespace_) + 1);

  {
    // 更新 graph_cache 和发布消息需要保持原子性，以避免竞争条件
    auto common = &context->impl->common;
    std::lock_guard<std::mutex> guard(common->node_update_mutex);
    rmw_dds_common::msg::ParticipantEntitiesInfo participant_msg =
        common->graph_cache.add_node(common->gid, name, namespace_);
    if (RMW_RET_OK != rmw_publish(common->pub, static_cast<void *>(&participant_msg), nullptr)) {
      // 如果发布消息失败，则不需要在从 graph_cache 中移除节点后再发布更新
      static_cast<void>(common->graph_cache.remove_node(common->gid, name, namespace_));
      return nullptr;
    }
  }

  // 取消清理节点的操作
  cleanup_node.cancel();
  // 设置节点实现标识符和数据指针
  node->implementation_identifier = eclipse_cyclonedds_identifier;
  node->data = node_impl.release();
  // 设置节点上下文
  node->context = context;
  // 取消调用 context 的 fini 方法
  finalize_context.cancel();
  // 返回创建的节点指针
  return node;
}

/**
 * @brief 销毁一个ROS2节点
 *
 * @param[in] node 要销毁的rmw_node_t指针
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
extern "C" rmw_ret_t rmw_destroy_node(rmw_node_t *node) {
  // 初始化结果为RMW_RET_OK
  rmw_ret_t result_ret = RMW_RET_OK;
  // 检查输入参数node是否为空，为空则返回RMW_RET_INVALID_ARGUMENT
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查节点类型是否匹配，不匹配则返回RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 将节点数据转换为CddsNode类型
  auto node_impl = static_cast<CddsNode *>(node->data);

  {
    // 虽然graph_cache方法是线程安全的，但缓存更新和发布也必须是原子操作。
    // 否则，可能会出现以下竞态条件：
    // node1-update-get-message / node2-update-get-message / node2-publish / node1-publish
    // 在这种情况下，最后发布的消息是不准确的。
    auto common = &node->context->impl->common;
    std::lock_guard<std::mutex> guard(common->node_update_mutex);
    rmw_dds_common::msg::ParticipantEntitiesInfo participant_msg =
        common->graph_cache.remove_node(common->gid, node->name, node->namespace_);
    result_ret = rmw_publish(common->pub, static_cast<void *>(&participant_msg), nullptr);
  }

  // 获取节点上下文
  rmw_context_t *context = node->context;
  // 释放节点名称内存
  rmw_free(const_cast<char *>(node->name));
  // 释放节点命名空间内存
  rmw_free(const_cast<char *>(node->namespace_));
  // 释放节点内存
  rmw_node_free(const_cast<rmw_node_t *>(node));
  // 删除节点实现
  delete node_impl;
  // 结束节点上下文实现
  context->impl->fini();
  // 返回结果
  return result_ret;
}

/**
 * @brief 获取ROS2节点的图形保护条件
 *
 * @param[in] node 要查询的rmw_node_t指针
 * @return const rmw_guard_condition_t* 返回rmw_guard_condition_t指针，如果失败则返回nullptr
 */
extern "C" const rmw_guard_condition_t *rmw_node_get_graph_guard_condition(const rmw_node_t *node) {
  // 检查输入参数node是否为空，为空则返回nullptr
  RET_NULL_X(node, return nullptr);
  // 检查节点类型是否匹配，不匹配则返回nullptr
  RET_WRONG_IMPLID_X(node, return nullptr);
  // 将节点数据转换为CddsNode类型
  auto node_impl = static_cast<CddsNode *>(node->data);
  // 检查节点实现是否为空，为空则返回nullptr
  RET_NULL_X(node_impl, return nullptr);
  // 返回节点上下文中的图形保护条件
  return node->context->impl->common.graph_guard_condition;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    (DE)SERIALIZATION                                              ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

// 使用C类型支持的消息类型支持别名
using MessageTypeSupport_c =
    rmw_cyclonedds_cpp::MessageTypeSupport<rosidl_typesupport_introspection_c__MessageMembers>;
// 使用C++类型支持的消息类型支持别名
using MessageTypeSupport_cpp =
    rmw_cyclonedds_cpp::MessageTypeSupport<rosidl_typesupport_introspection_cpp::MessageMembers>;

/**
 * @brief 获取序列化消息的大小（未实现）
 *
 * @param[in] type_support 消息类型支持
 * @param[in] message_bounds 消息边界
 * @param[out] size 序列化消息的大小
 * @return rmw_ret_t 返回状态，当前为不支持
 */
extern "C" rmw_ret_t rmw_get_serialized_message_size(
    const rosidl_message_type_support_t *type_support,
    const rosidl_runtime_c__Sequence__bound *message_bounds,
    size_t *size) {
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(size);

  RMW_SET_ERROR_MSG("rmw_get_serialized_message_size: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

/**
 * @brief 将ROS消息序列化
 *
 * @param[in] ros_message ROS消息
 * @param[in] type_support 消息类型支持
 * @param[out] serialized_message 序列化后的消息
 * @return rmw_ret_t 返回状态
 */
extern "C" rmw_ret_t rmw_serialize(
    const void *ros_message,
    const rosidl_message_type_support_t *type_support,
    rmw_serialized_message_t *serialized_message) {
  try {
    // 创建CDR编写器
    auto writer = rmw_cyclonedds_cpp::make_cdr_writer(
        rmw_cyclonedds_cpp::make_message_value_type(type_support));

    // 获取序列化大小
    auto size = writer->get_serialized_size(ros_message);
    // 调整序列化消息的大小
    rmw_ret_t ret = rmw_serialized_message_resize(serialized_message, size);
    if (RMW_RET_OK != ret) {
      RMW_SET_ERROR_MSG("rmw_serialize: failed to allocate space for message");
      return ret;
    }
    // 序列化消息
    writer->serialize(serialized_message->buffer, ros_message);
    // 设置序列化消息的缓冲区长度
    serialized_message->buffer_length = size;
    return RMW_RET_OK;
  } catch (std::exception &e) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("rmw_serialize: failed to serialize: %s", e.what());
    return RMW_RET_ERROR;
  }
}

/**
 * @brief 反序列化ROS2消息
 *
 * 将序列化的消息转换回原始的ROS2消息格式。
 *
 * @param[in] serialized_message 序列化后的消息
 * @param[in] type_support 消息类型支持结构体
 * @param[out] ros_message 反序列化后的ROS2消息
 * @return rmw_ret_t 反序列化操作结果
 */
extern "C" rmw_ret_t rmw_deserialize(
    const rmw_serialized_message_t *serialized_message,
    const rosidl_message_type_support_t *type_support,
    void *ros_message) {
  bool ok;
  try {
    // 创建反序列化对象
    cycdeser sd(serialized_message->buffer, serialized_message->buffer_length);

    // 声明消息类型支持变量
    const rosidl_message_type_support_t *ts;

    // 判断是否为C语言类型支持
    if ((ts = get_message_typesupport_handle(
             type_support, rosidl_typesupport_introspection_c__identifier)) != nullptr) {
      // 获取C语言类型支持成员
      auto members =
          static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts->data);

      // 创建C语言类型支持对象
      MessageTypeSupport_c msgts(members);

      // 反序列化ROS消息
      ok = msgts.deserializeROSmessage(sd, ros_message, nullptr);
    } else {
      // 判断是否为C++语言类型支持
      if ((ts = get_message_typesupport_handle(
               type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier)) !=
          nullptr) {
        // 获取C++语言类型支持成员
        auto members =
            static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);

        // 创建C++语言类型支持对象
        MessageTypeSupport_cpp msgts(members);

        // 反序列化ROS消息
        ok = msgts.deserializeROSmessage(sd, ros_message, nullptr);
      } else {
        // 设置错误信息
        RMW_SET_ERROR_MSG("rmw_serialize: type support trouble");
        return RMW_RET_ERROR;
      }
    }
  } catch (rmw_cyclonedds_cpp::Exception &e) {
    // 设置异常错误信息
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("rmw_serialize: %s", e.what());
    ok = false;
  } catch (std::runtime_error &e) {
    // 设置运行时错误信息
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("rmw_serialize: %s", e.what());
    ok = false;
  }

  // 返回反序列化结果
  return ok ? RMW_RET_OK : RMW_RET_ERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    TOPIC CREATION                                                 ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 函数参数说明
 * @param[in] sertype DDSI使用的主题序列化消息类型
 * @param[in] dds_create_topic_arbitrary 创建主题的函数指针
 * @param[in] create_same_topic 前一个调用创建相同主题的函数指针
 *
 * Publications需要在发布序列化消息时使用DDSI为主题使用的sertype。
 * 在Cyclone的旧（“任意”）接口中，无法知道实际使用的sertype，
 * 因为它可能是在dds_create_topic_arbitrary()调用中提供的那个，
 * 也可能是由前一个调用创建相同主题引入的。
 *
 * 没有发现哪种情况的方法，也没有获取正确sertype的方法。
 * 最好的办法是继续使用创建主题时提供的一个 —— 幸运的是，
 * 使用错误的sertype会产生很少的副作用，但它仍然是错误的。
 *
 * 因为调用者保留所有权，所以这很容易，但在清理时确实需要删除引用。
 *
 * 新的（“通用”）接口在成功时接管引用的所有权，并返回对实际使用的sertype的非计数引用。
 * 引用的生命周期至少与DDS主题存在的生命周期一样长；
 * 主题的生命周期至少与使用它的读/写器的生命周期一样长。因此，可以安全地使用此引用。
 */

/**
 * @brief 创建一个DDS主题
 *
 * 在ROS2的RMW层中，创建一个DDS主题。此函数根据DDS实现是否支持ddsi_sertype来选择不同的接口。
 *
 * @param[in] pp 参与者实体（Participant entity）
 * @param[in] name 主题名称
 * @param[in] sertype DDSI序列化类型
 * @param[out] stact 实际使用的DDSI序列化类型指针（可选）
 * @return 成功时返回创建的主题实体，失败时返回负值
 */
static dds_entity_t create_topic(
    dds_entity_t pp, const char *name, struct ddsi_sertype *sertype, struct ddsi_sertype **stact) {
  // 定义主题实体变量
  dds_entity_t tp;

  // 如果DDS实现支持ddsi_sertype
#if DDS_HAS_DDSI_SERTYPE
  // 使用dds_create_topic_sertype接口创建主题
  tp = dds_create_topic_sertype(pp, name, &sertype, nullptr, nullptr, nullptr);
#else
  // 否则使用dds_create_topic_generic接口创建主题
  static_cast<void>(name);
  tp = dds_create_topic_generic(pp, &sertype, nullptr, nullptr, nullptr);
#endif

  // 如果创建主题失败
  if (tp < 0) {
    // 减少sertype引用计数
    ddsi_sertype_unref(sertype);
  } else {
    // 如果stact非空，则将实际使用的sertype赋值给stact
    if (stact) {
      *stact = sertype;
    }
  }

  // 返回主题实体
  return tp;
}

/**
 * @brief 创建一个主题 (Create a topic)
 *
 * @param[in] pp       参与者实体 (Participant entity)
 * @param[in] name     主题名称 (Topic name)
 * @param[in] sertype  序列化类型 (Serialization type)
 * @return dds_entity_t 创建的主题实体 (Created topic entity)
 */
static dds_entity_t create_topic(dds_entity_t pp, const char *name, struct ddsi_sertype *sertype) {
  // 调用 create_topic 函数创建主题实体，并返回结果
  dds_entity_t tp = create_topic(pp, name, sertype, nullptr);
  return tp;
}

/**
 * @brief 根据 create_topic 的错误代码设置错误消息 (Set error message based on the error code from
 * create_topic)
 *
 * @param[in] topic      主题实体 (Topic entity)
 * @param[in] topic_name 主题名称 (Topic name)
 */
void set_error_message_from_create_topic(dds_entity_t topic, const std::string &topic_name) {
  // 检查主题实体是否小于0，表示出现错误
  assert(topic < 0);

  // 根据不同的错误代码设置相应的错误消息
  if (DDS_RETCODE_BAD_PARAMETER == topic) {
    const std::string error_msg = "failed to create topic [" + topic_name +
                                  "] because the function was given invalid parameters";
    RMW_SET_ERROR_MSG(error_msg.c_str());
  } else if (DDS_RETCODE_INCONSISTENT_POLICY == topic) {
    const std::string error_msg =
        "failed to create topic [" + topic_name +
        "] because it's already in use in this context with incompatible QoS settings";
    RMW_SET_ERROR_MSG(error_msg.c_str());
  } else if (DDS_RETCODE_PRECONDITION_NOT_MET == topic) {
    const std::string error_msg =
        "failed to create topic [" + topic_name +
        "] because it's already in use in this context with a different message type";
    RMW_SET_ERROR_MSG(error_msg.c_str());
  } else {
    const std::string error_msg = "failed to create topic [" + topic_name + "] for unknown reasons";
    RMW_SET_ERROR_MSG(error_msg.c_str());
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    PUBLICATIONS                                                   ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 发布ROS2消息的函数
 *
 * @param[in] publisher 指向要发布消息的rmw_publisher_t结构体的指针
 * @param[in] ros_message 要发布的ROS2消息的指针
 * @param[in] allocation 用于预分配内存的rmw_publisher_allocation_t结构体的指针（当前未使用）
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_publish(
    const rmw_publisher_t *publisher,
    const void *ros_message,
    rmw_publisher_allocation_t *allocation) {
  static_cast<void>(allocation);  // 未使用的参数
  // 检查是否为空，为空则返回无效参数错误
  RMW_CHECK_FOR_NULL_WITH_MSG(
      publisher, "publisher handle is null", return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
      ros_message, "ros message handle is null", return RMW_RET_INVALID_ARGUMENT);

  // 将publisher的data成员转换为CddsPublisher类型的指针
  auto pub = static_cast<CddsPublisher *>(publisher->data);
  assert(pub);
  // 添加追踪点
  TRACEPOINT(rmw_publish, ros_message);
  // 将消息写入DDS
  if (dds_write(pub->enth, ros_message) >= 0) {
    return RMW_RET_OK;                            // 写入成功，返回RMW_RET_OK
  } else {
    RMW_SET_ERROR_MSG("failed to publish data");  // 写入失败，设置错误信息并返回RMW_RET_ERROR
    return RMW_RET_ERROR;
  }
}

/**
 * @brief 发布序列化消息到指定的发布者
 *
 * @param[in] publisher 指向要发布消息的rmw_publisher_t结构体的指针
 * @param[in] serialized_message 指向要发布的序列化消息的rmw_serialized_message_t结构体的指针
 * @param[in] allocation 用于预分配内存的rmw_publisher_allocation_t结构体的指针（当前未使用）
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_publish_serialized_message(
    const rmw_publisher_t *publisher,
    const rmw_serialized_message_t *serialized_message,
    rmw_publisher_allocation_t *allocation) {
  static_cast<void>(allocation);  // 未使用的参数

  // 检查publisher是否为空，如果为空则返回无效参数错误
  RMW_CHECK_FOR_NULL_WITH_MSG(
      publisher, "publisher handle is null", return RMW_RET_INVALID_ARGUMENT);

  // 检查publisher的实现标识符是否与期望的一致，如果不一致则返回错误的RMW实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查serialized_message是否为空，如果为空则返回无效参数错误
  RMW_CHECK_FOR_NULL_WITH_MSG(
      serialized_message, "serialized message handle is null", return RMW_RET_INVALID_ARGUMENT);

  // 将publisher的data成员转换为CddsPublisher类型的指针
  auto pub = static_cast<CddsPublisher *>(publisher->data);

  // 将序列化消息转换为ddsi_serdata结构体
  struct ddsi_serdata *d = serdata_rmw_from_serialized_message(
      pub->sertype, serialized_message->buffer, serialized_message->buffer_length);

#ifdef DDS_HAS_SHM
  // 如果共享内存可用，则发布序列化消息
  if (dds_is_shared_memory_available(pub->enth)) {
    // 初始化并分配样本内存
    auto sample_ptr = init_and_alloc_sample(pub, serialized_message->buffer_length);
    // 检查分配的内存是否为空，如果为空则返回错误
    RET_NULL_X(sample_ptr, return RMW_RET_ERROR);
    // 将序列化消息的缓冲区复制到分配的内存中
    memcpy(sample_ptr, serialized_message->buffer, serialized_message->buffer_length);
    // 设置共享内存数据状态
    shm_set_data_state(sample_ptr, IOX_CHUNK_CONTAINS_SERIALIZED_DATA);
    // 将iox_chunk指向分配的内存
    d->iox_chunk = sample_ptr;
  }
#endif

  // 将序列化消息写入DDS，并检查操作是否成功
  const bool ok = (dds_writecdr(pub->enth, d) >= 0);
  // 根据操作结果返回相应的状态码
  return ok ? RMW_RET_OK : RMW_RET_ERROR;
}

/**
 * @brief 发布一个借用的整数消息 (Publish a loaned integer message)
 *
 * @param[in] publisher 指向要发布消息的rmw_publisher_t结构体的指针 (Pointer to the rmw_publisher_t
 * structure for publishing the message)
 * @param[in] ros_message 要发布的ROS消息的指针 (Pointer to the ROS message to be published)
 * @return RMW_RET_OK 成功发布消息 (Message published successfully)
 * @return 其他错误代码 (Other error codes)
 */
static rmw_ret_t publish_loaned_int(const rmw_publisher_t *publisher, void *ros_message) {
#ifdef DDS_HAS_SHM
  // 检查publisher是否为空，如果为空则返回RMW_RET_INVALID_ARGUMENT错误
  RMW_CHECK_FOR_NULL_WITH_MSG(
      publisher, "publisher handle is null", return RMW_RET_INVALID_ARGUMENT);
  // 如果publisher不支持消息借用，则返回RMW_RET_UNSUPPORTED错误
  if (!publisher->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }
  // 检查publisher的实现标识符是否与eclipse_cyclonedds_identifier匹配，如果不匹配则返回RMW_RET_INCORRECT_RMW_IMPLEMENTATION错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查ros_message是否为空，如果为空则返回RMW_RET_INVALID_ARGUMENT错误
  RMW_CHECK_FOR_NULL_WITH_MSG(
      ros_message, "ROS message handle is null", return RMW_RET_INVALID_ARGUMENT);

  // 将publisher的data转换为CddsPublisher类型
  auto cdds_publisher = static_cast<CddsPublisher *>(publisher->data);
  // 如果cdds_publisher为空，则返回RMW_RET_ERROR错误
  if (!cdds_publisher) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  // 如果publisher允许借用
  if (cdds_publisher->is_loaning_available) {
    // 创建一个serdata_rmw对象
    auto d = new serdata_rmw(cdds_publisher->sertype, ddsi_serdata_kind::SDK_DATA);
    d->iox_chunk = ros_message;
    // 设置数据状态为原始数据
    shm_set_data_state(d->iox_chunk, IOX_CHUNK_CONTAINS_RAW_DATA);
    // 尝试写入数据，如果成功则返回RMW_RET_OK
    if (dds_writecdr(cdds_publisher->enth, d) >= 0) {
      return RMW_RET_OK;
    } else {
      // 发布数据失败，返回RMW_RET_ERROR错误
      RMW_SET_ERROR_MSG("Failed to publish data");
      fini_and_free_sample(cdds_publisher, ros_message);
      ddsi_serdata_unref(d);
      return RMW_RET_ERROR;
    }
  } else {
    // 不允许发布非固定类型的借用消息，返回RMW_RET_ERROR错误
    RMW_SET_ERROR_MSG("Publishing a loaned message of non fixed type is not allowed");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
#else
  // 如果不支持DDS_HAS_SHM，则返回RMW_RET_UNSUPPORTED错误
  static_cast<void>(publisher);
  static_cast<void>(ros_message);
  RMW_SET_ERROR_MSG("rmw_publish_loaned_message not implemented for rmw_cyclonedds_cpp");
  return RMW_RET_UNSUPPORTED;
#endif
}

/**
 * @brief 发布借用的消息 (Publish a loaned message)
 *
 * @param[in] publisher 指向要发布消息的rmw_publisher_t结构体的指针 (Pointer to the rmw_publisher_t
 * structure for publishing the message)
 * @param[in] ros_message 指向要发布的ROS消息的指针 (Pointer to the ROS message to be published)
 * @param[in] allocation 指向rmw_publisher_allocation_t结构体的指针，用于分配内存 (Pointer to the
 * rmw_publisher_allocation_t structure for memory allocation)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
extern "C" rmw_ret_t rmw_publish_loaned_message(
    const rmw_publisher_t *publisher, void *ros_message, rmw_publisher_allocation_t *allocation) {
  // 忽略allocation参数 (Ignore the allocation parameter)
  static_cast<void>(allocation);

  // 调用publish_loaned_int函数并返回结果 (Call the publish_loaned_int function and return the
  // result)
  return publish_loaned_int(publisher, ros_message);
}

/**
 * @brief 获取类型支持信息 (Get type support information)
 *
 * @param[in] type_supports 指向rosidl_message_type_support_t结构体的指针，包含类型支持信息 (Pointer
 * to the rosidl_message_type_support_t structure containing type support information)
 * @return const rosidl_message_type_support_t* 返回类型支持信息的指针，如果未找到则返回nullptr
 * (Return a pointer to the type support information, or nullptr if not found)
 */
static const rosidl_message_type_support_t *get_typesupport(
    const rosidl_message_type_support_t *type_supports) {
  const rosidl_message_type_support_t *ts;

  // 尝试获取C语言类型支持信息 (Try to get the C language type support information)
  if ((ts = get_message_typesupport_handle(
           type_supports, rosidl_typesupport_introspection_c__identifier)) != nullptr) {
    return ts;
  } else {
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();

    // 尝试获取C++语言类型支持信息 (Try to get the C++ language type support information)
    if ((ts = get_message_typesupport_handle(
             type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier)) !=
        nullptr) {
      return ts;
    } else {
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();

      // 设置错误消息，表示未找到类型支持信息 (Set an error message indicating that the type support
      // information was not found)
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "Type support not from this implementation. Got:\n"
          "    %s\n"
          "    %s\n"
          "while fetching it",
          prev_error_string.str, error_string.str);
      return nullptr;
    }
  }
}

/**
 * @brief 生成完全限定主题名称 (Fully Qualified Topic Name)
 *
 * @param[in] prefix 主题前缀
 * @param[in] topic_name 主题名称
 * @param[in] suffix 主题后缀
 * @param[in] avoid_ros_namespace_conventions 是否避免 ROS 命名空间约定
 * @return std::string 完全限定主题名称
 */
static std::string make_fqtopic(
    const char *prefix,
    const char *topic_name,
    const char *suffix,
    bool avoid_ros_namespace_conventions) {
  // 如果避免 ROS 命名空间约定，则直接拼接主题名称和后缀
  if (avoid_ros_namespace_conventions) {
    return std::string(topic_name) + std::string(suffix);
  } else {
    // 否则，拼接前缀、主题名称和后缀
    return std::string(prefix) + std::string(topic_name) + std::string(suffix);
  }
}

/**
 * @brief 生成完全限定主题名称 (Fully Qualified Topic Name)，使用 QoS 策略
 *
 * @param[in] prefix 主题前缀
 * @param[in] topic_name 主题名称
 * @param[in] suffix 主题后缀
 * @param[in] qos_policies QoS 策略
 * @return std::string 完全限定主题名称
 */
static std::string make_fqtopic(
    const char *prefix,
    const char *topic_name,
    const char *suffix,
    const rmw_qos_profile_t *qos_policies) {
  // 使用 QoS 策略中的 avoid_ros_namespace_conventions 参数调用 make_fqtopic
  return make_fqtopic(prefix, topic_name, suffix, qos_policies->avoid_ros_namespace_conventions);
}

/**
 * @brief 检查 rmw 时间是否未指定
 *
 * @param[in] duration rmw 时间
 * @return bool 如果未指定，则返回 true，否则返回 false
 */
static bool is_rmw_duration_unspecified(rmw_time_t duration) {
  // 比较 duration 是否等于 RMW_DURATION_UNSPECIFIED
  return rmw_time_equal(duration, RMW_DURATION_UNSPECIFIED);
}

/**
 * @brief 将 rmw 时间转换为 dds 时间
 *
 * @param[in] duration rmw 时间
 * @return dds_duration_t dds 时间
 */
static dds_duration_t rmw_duration_to_dds(rmw_time_t duration) {
  // 如果 duration 等于 RMW_DURATION_INFINITE，则返回 DDS_INFINITY
  if (rmw_time_equal(duration, RMW_DURATION_INFINITE)) {
    return DDS_INFINITY;
  } else {
    // 否则，将 rmw 时间转换为纳秒并返回
    return rmw_time_total_nsec(duration);
  }
}

/**
 * @brief 将 dds 时间转换为 rmw 时间
 *
 * @param[in] duration dds 时间
 * @return rmw_time_t rmw 时间
 */
static rmw_time_t dds_duration_to_rmw(dds_duration_t duration) {
  // 如果 duration 等于 DDS_INFINITY，则返回 RMW_DURATION_INFINITE
  if (duration == DDS_INFINITY) {
    return RMW_DURATION_INFINITE;
  } else {
    // 否则，将 dds 时间从纳秒转换为 rmw 时间并返回
    return rmw_time_from_nsec(duration);
  }
}

/**
 * @brief 创建用于读写操作的DDS QoS对象
 *
 * @param[in] qos_policies 指向rmw_qos_profile_t结构体的指针，包含QoS策略信息
 * @param[in] type_hash 一个rosidl_type_hash_t类型的引用，表示消息类型的哈希值
 * @param[in] ignore_local_publications 布尔值，表示是否忽略本地发布的消息
 * @param[in] extra_user_data 一个字符串，表示额外的用户数据
 * @return 返回一个dds_qos_t指针，指向创建的QoS对象；如果出现错误，则返回nullptr
 */
static dds_qos_t *create_readwrite_qos(
    const rmw_qos_profile_t *qos_policies,
    const rosidl_type_hash_t &type_hash,
    bool ignore_local_publications,
    const std::string &extra_user_data) {
  // 定义一个DDS持续时间变量
  dds_duration_t ldur;
  // 创建一个新的DDS QoS对象
  dds_qos_t *qos = dds_create_qos();
  // 禁用自动处理（autodispose）
  dds_qset_writer_data_lifecycle(qos, false);

  // 根据历史记录策略设置QoS
  switch (qos_policies->history) {
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      if (qos_policies->depth == RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT) {
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
      } else {
        if (qos_policies->depth < 1 || qos_policies->depth > INT32_MAX) {
          RMW_SET_ERROR_MSG("unsupported history depth");
          dds_delete_qos(qos);
          return nullptr;
        }
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, static_cast<int32_t>(qos_policies->depth));
      }
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      dds_qset_history(qos, DDS_HISTORY_KEEP_ALL, DDS_LENGTH_UNLIMITED);
      break;
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
      return nullptr;
    default:
      rmw_cyclonedds_cpp::unreachable();
  }

  // 根据可靠性策略设置QoS
  switch (qos_policies->reliability) {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_INFINITY);
      break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
      break;
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
      return nullptr;
    default:
      rmw_cyclonedds_cpp::unreachable();
  }

  // 根据持久性策略设置QoS
  switch (qos_policies->durability) {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
      break;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL: {
      dds_history_kind_t hk;
      int32_t hd;
      dds_qget_history(qos, &hk, &hd);
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_durability_service(
          qos, DDS_SECS(0), hk, hd, DDS_LENGTH_UNLIMITED, DDS_LENGTH_UNLIMITED,
          DDS_LENGTH_UNLIMITED);
      break;
    }
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
      return nullptr;
    default:
      rmw_cyclonedds_cpp::unreachable();
  }

  // 设置生命周期策略
  if (!is_rmw_duration_unspecified(qos_policies->lifespan)) {
    dds_qset_lifespan(qos, rmw_duration_to_dds(qos_policies->lifespan));
  }
  // 设置截止时间策略
  if (!is_rmw_duration_unspecified(qos_policies->deadline)) {
    dds_qset_deadline(qos, rmw_duration_to_dds(qos_policies->deadline));
  }

  // 设置活跃度租约持续时间
  if (is_rmw_duration_unspecified(qos_policies->liveliness_lease_duration)) {
    ldur = DDS_INFINITY;
  } else {
    ldur = rmw_duration_to_dds(qos_policies->liveliness_lease_duration);
  }
  // 根据活跃度策略设置QoS
  switch (qos_policies->liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      dds_qset_liveliness(qos, DDS_LIVELINESS_AUTOMATIC, ldur);
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      dds_qset_liveliness(qos, DDS_LIVELINESS_MANUAL_BY_TOPIC, ldur);
      break;
    case RMW_QOS_POLICY_LIVELINESS_UNKNOWN:
      return nullptr;
    default:
      rmw_cyclonedds_cpp::unreachable();
  }
  // 设置是否忽略本地发布的消息
  if (ignore_local_publications) {
    dds_qset_ignorelocal(qos, DDS_IGNORELOCAL_PARTICIPANT);
  }

  // 编码类型哈希值并设置用户数据
  std::string typehash_str;
  if (RMW_RET_OK != rmw_dds_common::encode_type_hash_for_user_data_qos(type_hash, typehash_str)) {
    RCUTILS_LOG_WARN_NAMED(
        "rmw_cyclonedds_cpp",
        "Failed to encode type hash for topic, will not distribute it in USER_DATA.");
    typehash_str.clear();
  }
  std::string user_data = extra_user_data + typehash_str;
  dds_qset_userdata(qos, user_data.data(), user_data.size());

  return qos;
}

/**
 * @brief 将DDS QoS策略转换为RMW QoS策略
 *
 * 该函数根据输入的DDS QoS策略ID，返回对应的RMW QoS策略类型。
 *
 * @param policy_id DDS QoS策略ID
 * @return 对应的RMW QoS策略类型
 */
static rmw_qos_policy_kind_t dds_qos_policy_to_rmw_qos_policy(dds_qos_policy_id_t policy_id) {
  // 使用switch语句根据不同的DDS QoS策略ID进行处理
  switch (policy_id) {
    case DDS_DURABILITY_QOS_POLICY_ID:
      return RMW_QOS_POLICY_DURABILITY;
    case DDS_DEADLINE_QOS_POLICY_ID:
      return RMW_QOS_POLICY_DEADLINE;
    case DDS_LIVELINESS_QOS_POLICY_ID:
      return RMW_QOS_POLICY_LIVELINESS;
    case DDS_RELIABILITY_QOS_POLICY_ID:
      return RMW_QOS_POLICY_RELIABILITY;
    case DDS_HISTORY_QOS_POLICY_ID:
      return RMW_QOS_POLICY_HISTORY;
    case DDS_LIFESPAN_QOS_POLICY_ID:
      return RMW_QOS_POLICY_LIFESPAN;
    default:
      return RMW_QOS_POLICY_INVALID;
  }
}

/**
 * @brief 将DDS QoS设置转换为RMW QoS设置
 *
 * @param[in] dds_qos DDS QoS设置指针
 * @param[out] qos_policies RMW QoS设置指针
 * @return 转换成功返回true，否则返回false
 */
static bool dds_qos_to_rmw_qos(const dds_qos_t *dds_qos, rmw_qos_profile_t *qos_policies) {
  // 检查输入参数是否有效
  assert(dds_qos);
  assert(qos_policies);

  // 处理历史记录策略
  {
    dds_history_kind_t kind;
    int32_t depth;
    // 获取DDS QoS中的历史记录策略
    if (!dds_qget_history(dds_qos, &kind, &depth)) {
      RMW_SET_ERROR_MSG("get_readwrite_qos: history not set");
      return false;
    }
    // 根据历史记录策略类型进行处理
    switch (kind) {
      case DDS_HISTORY_KEEP_LAST:
        qos_policies->history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_policies->depth = (uint32_t)depth;
        break;
      case DDS_HISTORY_KEEP_ALL:
        qos_policies->history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        // 当使用KEEP_ALL策略时，深度无意义
        qos_policies->depth = 0;
        break;
      default:
        rmw_cyclonedds_cpp::unreachable();
    }
  }

  // 处理可靠性策略
  {
    dds_reliability_kind_t kind;
    dds_duration_t max_blocking_time;
    // 获取DDS QoS中的可靠性策略
    if (!dds_qget_reliability(dds_qos, &kind, &max_blocking_time)) {
      RMW_SET_ERROR_MSG("get_readwrite_qos: history not set");
      return false;
    }
    // 根据可靠性策略类型进行处理
    switch (kind) {
      case DDS_RELIABILITY_BEST_EFFORT:
        qos_policies->reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        break;
      case DDS_RELIABILITY_RELIABLE:
        qos_policies->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        break;
      default:
        rmw_cyclonedds_cpp::unreachable();
    }
  }

  // 处理持久性策略
  {
    dds_durability_kind_t kind;
    // 获取DDS QoS中的持久性策略
    if (!dds_qget_durability(dds_qos, &kind)) {
      RMW_SET_ERROR_MSG("get_readwrite_qos: durability not set");
      return false;
    }
    // 根据持久性策略类型进行处理
    switch (kind) {
      case DDS_DURABILITY_VOLATILE:
        qos_policies->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        break;
      case DDS_DURABILITY_TRANSIENT_LOCAL:
        qos_policies->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        break;
      case DDS_DURABILITY_TRANSIENT:
      case DDS_DURABILITY_PERSISTENT:
        qos_policies->durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
        break;
      default:
        rmw_cyclonedds_cpp::unreachable();
    }
  }

  // 处理截止时间策略
  {
    dds_duration_t deadline;
    // 获取DDS QoS中的截止时间策略
    if (!dds_qget_deadline(dds_qos, &deadline)) {
      RMW_SET_ERROR_MSG("get_readwrite_qos: deadline not set");
      return false;
    }
    qos_policies->deadline = dds_duration_to_rmw(deadline);
  }

  // 处理生命周期策略
  {
    dds_duration_t lifespan;
    // 获取DDS QoS中的生命周期策略
    if (!dds_qget_lifespan(dds_qos, &lifespan)) {
      lifespan = DDS_INFINITY;
    }
    qos_policies->lifespan = dds_duration_to_rmw(lifespan);
  }

  // 处理活跃度策略
  {
    dds_liveliness_kind_t kind;
    dds_duration_t lease_duration;
    // 获取DDS QoS中的活跃度策略
    if (!dds_qget_liveliness(dds_qos, &kind, &lease_duration)) {
      RMW_SET_ERROR_MSG("get_readwrite_qos: liveliness not set");
      return false;
    }
    // 根据活跃度策略类型进行处理
    switch (kind) {
      case DDS_LIVELINESS_AUTOMATIC:
        qos_policies->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        break;
      case DDS_LIVELINESS_MANUAL_BY_PARTICIPANT:
        qos_policies->liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
        break;
      case DDS_LIVELINESS_MANUAL_BY_TOPIC:
        qos_policies->liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
        break;
      default:
        rmw_cyclonedds_cpp::unreachable();
    }
    qos_policies->liveliness_lease_duration = dds_duration_to_rmw(lease_duration);
  }

  return true;
}

/**
 * @brief 判断类型是否为自包含类型
 *
 * @param type_supports 指向rosidl_message_type_support_t结构体的指针
 * @return 如果类型是自包含类型，则返回true，否则返回false
 */
static bool is_type_self_contained(const rosidl_message_type_support_t *type_supports) {
  // 获取C++类型支持句柄
  auto ts = get_message_typesupport_handle(
      type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (ts != nullptr) {  // CPP 类型支持
    // 将ts->data转换为rosidl_typesupport_introspection_cpp::MessageMembers指针
    auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
    MessageTypeSupport_cpp mts(members);  // 创建C++消息类型支持对象
    return mts.is_type_self_contained();  // 返回类型是否为自包含类型
  } else {
    // 获取C类型支持句柄
    ts = get_message_typesupport_handle(
        type_supports, rosidl_typesupport_introspection_c__identifier);
    if (ts != nullptr) {  // C 类型支持
      // 将ts->data转换为rosidl_typesupport_introspection_c__MessageMembers指针
      auto members =
          static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts->data);
      MessageTypeSupport_c mts(members);                  // 创建C消息类型支持对象
      return mts.is_type_self_contained();                // 返回类型是否为自包含类型
    } else {
      RMW_SET_ERROR_MSG("Non supported type-supported");  // 设置错误消息
      return false;  // 返回false，表示类型不是自包含类型
    }
  }
}

/**
 * @brief 创建一个CddsPublisher对象
 *
 * @param[in] dds_ppant DDS参与者实体
 * @param[in] dds_pub DDS发布者实体
 * @param[in] type_supports 消息类型支持结构体指针
 * @param[in] topic_name 话题名称
 * @param[in] qos_policies QoS策略指针
 * @return 成功时返回一个新的CddsPublisher指针，失败时返回nullptr
 */
static CddsPublisher *create_cdds_publisher(
    dds_entity_t dds_ppant,
    dds_entity_t dds_pub,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies) {
  // 检查topic_name是否为空或空字符串，如果是则返回nullptr
  RET_NULL_OR_EMPTYSTR_X(topic_name, return nullptr);
  // 检查qos_policies是否为空，如果是则返回nullptr
  RET_NULL_X(qos_policies, return nullptr);
  // 获取类型支持信息
  const rosidl_message_type_support_t *type_support = get_typesupport(type_supports);
  // 检查type_support是否为空，如果是则返回nullptr
  RET_NULL_X(type_support, return nullptr);
  // 创建一个新的CddsPublisher对象
  CddsPublisher *pub = new CddsPublisher();
  dds_entity_t topic;
  dds_qos_t *qos;

  // 构造完全限定的话题名称
  std::string fqtopic_name = make_fqtopic(ROS_TOPIC_PREFIX, topic_name, "", qos_policies);
  // 判断类型是否为固定类型
  bool is_fixed_type = is_type_self_contained(type_support);
  // 获取消息的大小
  uint32_t sample_size = static_cast<uint32_t>(rmw_cyclonedds_cpp::get_message_size(type_support));
  // 创建序列化类型对象
  auto sertype = create_sertype(
      type_support->typesupport_identifier,
      create_message_type_support(type_support->data, type_support->typesupport_identifier), false,
      rmw_cyclonedds_cpp::make_message_value_type(type_supports), sample_size, is_fixed_type);
  struct ddsi_sertype *stact = nullptr;
  // 创建话题
  topic = create_topic(dds_ppant, fqtopic_name.c_str(), sertype, &stact);

  // 创建监听器
  dds_listener_t *listener = dds_create_listener(&pub->user_callback_data);
  // 设置相应的回调函数以便监听事件
  listener_set_event_callbacks(listener, &pub->user_callback_data);

  // 如果创建话题失败，设置错误信息并跳转到fail_topic标签
  if (topic < 0) {
    set_error_message_from_create_topic(topic, fqtopic_name);
    goto fail_topic;
  }
  // 创建读写QoS策略，如果失败则跳转到fail_qos标签
  if ((qos = create_readwrite_qos(qos_policies, *type_support->type_hash, false, "")) == nullptr) {
    goto fail_qos;
  }
  // 创建DDS写入者，如果失败则设置错误信息并跳转到fail_writer标签
  if ((pub->enth = dds_create_writer(dds_pub, topic, qos, listener)) < 0) {
    RMW_SET_ERROR_MSG("failed to create writer");
    goto fail_writer;
  }
  // 获取实例句柄，如果失败则设置错误信息并跳转到fail_instance_handle标签
  if (dds_get_instance_handle(pub->enth, &pub->pubiid) < 0) {
    RMW_SET_ERROR_MSG("failed to get instance handle for writer");
    goto fail_instance_handle;
  }
  // 获取实体的全局唯一ID
  get_entity_gid(pub->enth, pub->gid);
  // 设置序列化类型对象
  pub->sertype = stact;
  // 删除监听器
  dds_delete_listener(listener);
  // 设置类型支持信息
  pub->type_supports = *type_supports;
  // 设置是否支持loan功能
  pub->is_loaning_available = is_fixed_type && dds_is_loan_available(pub->enth);
  // 设置样本大小
  pub->sample_size = sample_size;
  // 删除QoS策略和话题
  dds_delete_qos(qos);
  dds_delete(topic);
  // 返回创建的CddsPublisher对象指针
  return pub;

// 错误处理部分
fail_instance_handle:
  if (dds_delete(pub->enth) < 0) {
    RCUTILS_LOG_ERROR_NAMED("rmw_cyclonedds_cpp", "failed to destroy writer during error handling");
  }
fail_writer:
  dds_delete_qos(qos);
fail_qos:
  dds_delete(topic);
fail_topic:
  delete pub;
  return nullptr;
}

/**
 * @brief 初始化发布者分配
 *
 * @param[in] type_support 消息类型支持
 * @param[in] message_bounds 消息边界
 * @param[out] allocation 发布者分配
 * @return rmw_ret_t 返回操作状态
 */
extern "C" rmw_ret_t rmw_init_publisher_allocation(
    const rosidl_message_type_support_t *type_support,
    const rosidl_runtime_c__Sequence__bound *message_bounds,
    rmw_publisher_allocation_t *allocation) {
  // 强制类型转换，忽略参数
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(allocation);

  // 设置错误消息
  RMW_SET_ERROR_MSG("rmw_init_publisher_allocation: unimplemented");

  // 返回不支持的状态
  return RMW_RET_UNSUPPORTED;
}

/**
 * @brief 终止发布者分配
 *
 * @param[in,out] allocation 发布者分配
 * @return rmw_ret_t 返回操作状态
 */
extern "C" rmw_ret_t rmw_fini_publisher_allocation(rmw_publisher_allocation_t *allocation) {
  // 强制类型转换，忽略参数
  static_cast<void>(allocation);

  // 设置错误消息
  RMW_SET_ERROR_MSG("rmw_fini_publisher_allocation: unimplemented");

  // 返回不支持的状态
  return RMW_RET_UNSUPPORTED;
}

/**
 * @brief 创建发布者
 *
 * @param[in] dds_ppant DDS参与者实体
 * @param[in] dds_pub DDS发布者实体
 * @param[in] type_supports 消息类型支持
 * @param[in] topic_name 主题名称
 * @param[in] qos_policies QoS策略
 * @param[in] publisher_options 发布者选项
 * @return rmw_publisher_t* 返回创建的发布者指针，如果失败则返回nullptr
 */
static rmw_publisher_t *create_publisher(
    dds_entity_t dds_ppant,
    dds_entity_t dds_pub,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies,
    const rmw_publisher_options_t *publisher_options) {
  CddsPublisher *pub;

  // 创建CDDS发布者
  if ((pub = create_cdds_publisher(dds_ppant, dds_pub, type_supports, topic_name, qos_policies)) ==
      nullptr) {
    return nullptr;
  }

  // 清理CDDS发布者
  auto cleanup_cdds_publisher = rcpputils::make_scope_exit([pub]() {
    if (dds_delete(pub->enth) < 0) {
      RCUTILS_LOG_ERROR_NAMED(
          "rmw_cyclonedds_cpp", "failed to delete writer during error handling");
    }
    delete pub;
  });

  // 分配RMW发布者内存
  rmw_publisher_t *rmw_publisher = rmw_publisher_allocate();

  // 检查分配结果
  RET_ALLOC_X(rmw_publisher, return nullptr);

  // 清理RMW发布者
  auto cleanup_rmw_publisher = rcpputils::make_scope_exit([rmw_publisher]() {
    rmw_free(const_cast<char *>(rmw_publisher->topic_name));
    rmw_publisher_free(rmw_publisher);
  });

  // 设置RMW发布者属性
  rmw_publisher->implementation_identifier = eclipse_cyclonedds_identifier;
  rmw_publisher->data = pub;
  rmw_publisher->topic_name = reinterpret_cast<char *>(rmw_allocate(strlen(topic_name) + 1));

  // 检查分配结果
  RET_ALLOC_X(rmw_publisher->topic_name, return nullptr);

  // 复制主题名称
  memcpy(const_cast<char *>(rmw_publisher->topic_name), topic_name, strlen(topic_name) + 1);

  // 设置发布者选项和消息借用功能
  rmw_publisher->options = *publisher_options;
  rmw_publisher->can_loan_messages = pub->is_loaning_available;

  // 取消清理操作
  cleanup_rmw_publisher.cancel();
  cleanup_cdds_publisher.cancel();

  // 返回创建的RMW发布者
  return rmw_publisher;
}

/** @brief 创建一个ROS2发布者 (Create a ROS2 publisher)
 *
 * @param[in] node 指向要创建发布者的节点的指针
 * @param[in] type_supports 消息类型支持结构体的指针 (Pointer to the message type support structure)
 * @param[in] topic_name 要发布的主题名称 (The name of the topic to publish)
 * @param[in] qos_policies 指向QoS策略的指针 (Pointer to the QoS policies)
 * @param[in] publisher_options 指向发布者选项的指针 (Pointer to the publisher options)
 * @return 成功时返回一个指向新创建的发布者的指针，失败时返回nullptr
 */
extern "C" rmw_publisher_t *rmw_create_publisher(
    const rmw_node_t *node,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies,
    const rmw_publisher_options_t *publisher_options) {
  // 检查输入参数是否为空 (Check if input arguments are null)
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier, return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  // 检查主题名称是否为空字符串 (Check if topic_name is an empty string)
  if (0 == strlen(topic_name)) {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  // 验证主题名称是否有效 (Validate the topic name)
  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char *reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return nullptr;
    }
  }
  // 适应任何“最佳可用”QoS选项 (Adapt any 'best available' QoS options)
  rmw_qos_profile_t adapted_qos_policies = *qos_policies;
  rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_topic_publisher(
      node, topic_name, &adapted_qos_policies, rmw_get_subscriptions_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);
  // 检查网络流端点的唯一性要求 (Check for unique network flow endpoints requirement)
  if (publisher_options->require_unique_network_flow_endpoints ==
      RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED) {
    RMW_SET_ERROR_MSG(
        "Strict requirement on unique network flow endpoints for publishers not supported");
    return nullptr;
  }

  // 创建发布者 (Create the publisher)
  rmw_publisher_t *pub = create_publisher(
      node->context->impl->ppant,    //
      node->context->impl->dds_pub,  //
      type_supports,                 //
      topic_name,                    //
      &adapted_qos_policies,         //
      publisher_options);
  if (pub == nullptr) {
    return nullptr;
  }
  // 清理发布者 (Clean up the publisher)
  auto cleanup_publisher = rcpputils::make_scope_exit([pub]() {
    rmw_error_state_t error_state = *rmw_get_error_state();
    rmw_reset_error();
    if (RMW_RET_OK != destroy_publisher(pub)) {
      RMW_SAFE_FWRITE_TO_STDERR(rmw_get_error_string().str);
      RMW_SAFE_FWRITE_TO_STDERR(" during '" RCUTILS_STRINGIFY(__function__) "' cleanup\n");
      rmw_reset_error();
    }
    rmw_set_error_state(error_state.message, error_state.file, error_state.line_number);
  });

  // 更新图 (Update the graph)
  auto common = &node->context->impl->common;
  const auto cddspub = static_cast<const CddsPublisher *>(pub->data);
  {
    std::lock_guard<std::mutex> guard(common->node_update_mutex);
    rmw_dds_common::msg::ParticipantEntitiesInfo msg = common->graph_cache.associate_writer(
        cddspub->gid, common->gid, node->name, node->namespace_);
    if (RMW_RET_OK != rmw_publish(common->pub, static_cast<void *>(&msg), nullptr)) {
      static_cast<void>(common->graph_cache.dissociate_writer(
          cddspub->gid, common->gid, node->name, node->namespace_));
      return nullptr;
    }
  }

  // 取消清理操作 (Cancel the cleanup operation)
  cleanup_publisher.cancel();
  TRACEPOINT(rmw_publisher_init, static_cast<const void *>(pub), cddspub->gid.data);
  return pub;
}

/**
 * @brief 获取发布者的全局唯一标识符 (GID)
 *
 * @param[in] publisher 指向要获取 GID 的发布者对象的指针
 * @param[out] gid 用于存储发布者 GID 的指针
 * @return rmw_ret_t 返回操作结果，成功返回 RMW_RET_OK，否则返回相应错误代码
 */
extern "C" rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t *publisher, rmw_gid_t *gid) {
  // 检查 publisher 参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);

  // 检查 publisher 的实现标识符是否与预期匹配，不匹配则返回错误的 RMW 实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查 gid 参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  // 将 publisher->data 转换为 CddsPublisher 类型的指针
  auto pub = static_cast<const CddsPublisher *>(publisher->data);

  // 设置 gid 的实现标识符
  gid->implementation_identifier = eclipse_cyclonedds_identifier;

  // 初始化 gid->data 为 0
  memset(gid->data, 0, sizeof(gid->data));

  // 静态断言，确保 pub->pubiid 的大小不超过 gid->data 的最大大小
  static_assert(
      sizeof(pub->pubiid) <= sizeof(gid->data), "publisher id is larger than max rmw gid size");

  // 将 pub->pubiid 复制到 gid->data 中
  memcpy(gid->data, &pub->pubiid, sizeof(pub->pubiid));

  // 返回操作成功
  return RMW_RET_OK;
}

/**
 * @brief 获取客户端的全局唯一标识符 (GID)
 *
 * @param[in] client 指向要获取 GID 的客户端对象的指针
 * @param[out] gid 用于存储客户端 GID 的指针
 * @return rmw_ret_t 返回操作结果，成功返回 RMW_RET_OK，否则返回相应错误代码
 */
extern "C" rmw_ret_t rmw_get_gid_for_client(const rmw_client_t *client, rmw_gid_t *gid) {
  // 检查 client 参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);

  // 检查 client 的实现标识符是否与预期匹配，不匹配则返回错误的 RMW 实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      client, client->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查 gid 参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  // 将 client->data 转换为 CddsClient 类型的指针
  const CddsClient *cli = static_cast<const CddsClient *>(client->data);

  // 设置 gid 的实现标识符
  gid->implementation_identifier = eclipse_cyclonedds_identifier;

  // 初始化 gid->data 为 0
  memset(gid->data, 0, sizeof(gid->data));

  // 静态断言，确保 cli->client.id.data 的大小不超过 gid->data 的最大大小
  static_assert(
      sizeof(cli->client.id.data) <= sizeof(gid->data),
      "client id is larger than max rmw gid size");

  // 将 cli->client.id.data 复制到 gid->data 中
  memcpy(gid->data, cli->client.id.data, sizeof(cli->client.id.data));

  // 返回操作成功
  return RMW_RET_OK;
}

/**
 * @brief 比较两个全局唯一标识符（GID）是否相等
 *
 * @param[in] gid1 第一个全局唯一标识符
 * @param[in] gid2 第二个全局唯一标识符
 * @param[out] result 存储比较结果的指针，如果两个 GID 相等，则为 true，否则为 false
 * @return rmw_ret_t 返回 RMW_RET_OK 表示成功，其他值表示失败
 */
extern "C" rmw_ret_t rmw_compare_gids_equal(
    const rmw_gid_t *gid1, const rmw_gid_t *gid2, bool *result) {
  // 检查 gid1 是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
  // 检查 gid1 的实现标识符是否与预期的实现标识符匹配，不匹配则返回错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      gid1, gid1->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查 gid2 是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
  // 检查 gid2 的实现标识符是否与预期的实现标识符匹配，不匹配则返回错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      gid2, gid2->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查 result 是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);
  // 使用 memcmp 进行字节比较，而不是简单的整数比较，因为可能存在对齐问题
  *result = memcmp(gid1->data, gid2->data, sizeof(gid1->data)) == 0;
  return RMW_RET_OK;
}

/**
 * @brief 计算与给定发布者匹配的订阅者数量
 *
 * @param[in] publisher 发布者指针
 * @param[out] subscription_count 存储匹配订阅者数量的指针
 * @return rmw_ret_t 返回 RMW_RET_OK 表示成功，其他值表示失败
 */
extern "C" rmw_ret_t rmw_publisher_count_matched_subscriptions(
    const rmw_publisher_t *publisher, size_t *subscription_count) {
  // 检查 publisher 是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  // 检查 publisher 的实现标识符是否与预期的实现标识符匹配，不匹配则返回错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查 subscription_count 是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);

  // 将 publisher 的 data 转换为 CddsPublisher 类型
  auto pub = static_cast<CddsPublisher *>(publisher->data);
  // 定义匹配状态变量
  dds_publication_matched_status_t status;
  // 获取与发布者匹配的订阅者状态
  if (dds_get_publication_matched_status(pub->enth, &status) < 0) {
    return RMW_RET_ERROR;
  }

  // 将当前匹配的订阅者数量赋值给 subscription_count
  *subscription_count = status.current_count;
  return RMW_RET_OK;
}

/**
 * @brief 确保发布者的活跃性
 *
 * @param[in] publisher 指向要检查活跃性的rmw_publisher_t结构体的指针
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t *publisher) {
  // 检查publisher是否为空，如果为空则返回错误
  RET_NULL(publisher);
  // 检查publisher的实现是否正确，如果不正确则返回错误
  RET_WRONG_IMPLID(publisher);
  // 将publisher的data成员转换为CddsPublisher类型
  auto pub = static_cast<CddsPublisher *>(publisher->data);
  // 调用dds_assert_liveliness函数确保发布者的活跃性，如果返回值小于0，则返回错误
  if (dds_assert_liveliness(pub->enth) < 0) {
    return RMW_RET_ERROR;
  }
  // 如果一切正常，返回RMW_RET_OK
  return RMW_RET_OK;
}

/**
 * @brief 等待所有发布者确认
 *
 * @param[in] publisher 指向要等待确认的rmw_publisher_t结构体的指针
 * @param[in] wait_timeout 等待超时时间
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
rmw_ret_t rmw_publisher_wait_for_all_acked(
    const rmw_publisher_t *publisher, rmw_time_t wait_timeout) {
  // 检查publisher是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  // 检查publisher的实现是否正确，如果不正确则返回错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 将publisher的data成员转换为CddsPublisher类型
  auto pub = static_cast<CddsPublisher *>(publisher->data);
  // 检查pub是否为空，如果为空则设置错误消息并返回无效参数错误
  if (pub == nullptr) {
    RMW_SET_ERROR_MSG("The publisher is not a valid publisher.");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 将rmw_time_t类型的wait_timeout转换为dds_duration_t类型
  dds_duration_t timeout = rmw_duration_to_dds(wait_timeout);
  // 调用dds_wait_for_acks函数等待所有发布者确认
  switch (dds_wait_for_acks(pub->enth, timeout)) {
    case DDS_RETCODE_OK:
      return RMW_RET_OK;
    case DDS_RETCODE_BAD_PARAMETER:
      RMW_SET_ERROR_MSG("The publisher is not a valid publisher.");
      return RMW_RET_INVALID_ARGUMENT;
    case DDS_RETCODE_TIMEOUT:
      return RMW_RET_TIMEOUT;
    case DDS_RETCODE_UNSUPPORTED:
      return RMW_RET_UNSUPPORTED;
    default:
      return RMW_RET_ERROR;
  }
}

/**
 * @brief 获取实际的QoS配置信息
 *
 * 该函数用于获取给定发布者的实际QoS配置信息。
 *
 * @param[in] publisher 指向要查询的rmw_publisher_t结构体的指针
 * @param[out] qos 用于存储查询到的QoS配置信息的rmw_qos_profile_t结构体的指针
 * @return 如果成功，返回RMW_RET_OK；如果传入的参数无效，返回RMW_RET_INVALID_ARGUMENT；
 *         如果RMW实现不匹配，返回RMW_RET_INCORRECT_RMW_IMPLEMENTATION；其他错误情况返回RMW_RET_ERROR。
 */
rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t *publisher, rmw_qos_profile_t *qos) {
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  // 将publisher->data转换为CddsPublisher类型的指针
  auto pub = static_cast<CddsPublisher *>(publisher->data);

  // 获取QoS配置信息，如果成功则返回RMW_RET_OK
  if (get_readwrite_qos(pub->enth, qos)) {
    return RMW_RET_OK;
  }

  return RMW_RET_ERROR;
}

/**
 * @brief 借用已分配的消息内存
 *
 * @param[in] publisher 发布者指针，不能为空
 * @param[in] type_support 消息类型支持结构体指针，不能为空
 * @param[out] ros_message 返回借用的消息指针，不能为空
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t borrow_loaned_message_int(
    const rmw_publisher_t *publisher,
    const rosidl_message_type_support_t *type_support,
    void **ros_message) {
#ifdef DDS_HAS_SHM
  // 检查发布者参数是否为空
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  // 检查发布者是否支持消息借用
  if (!publisher->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }
  // 检查消息类型支持参数是否为空
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
  // 检查发布者的实现标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查ros_message参数是否为空
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  // 检查ros_message是否已经被初始化
  if (*ros_message) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 获取CddsPublisher对象
  auto cdds_publisher = static_cast<CddsPublisher *>(publisher->data);
  // 检查CddsPublisher对象是否为空
  if (!cdds_publisher) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  // 如果发布者支持消息借用
  if (cdds_publisher->is_loaning_available) {
    // 初始化并分配样本内存
    auto sample_ptr = init_and_alloc_sample(cdds_publisher, cdds_publisher->sample_size);
    // 检查分配的样本内存是否为空
    RET_NULL_X(sample_ptr, return RMW_RET_ERROR);
    // 设置返回的ros_message指针
    *ros_message = sample_ptr;
    // 返回成功状态
    return RMW_RET_OK;
  } else {
    // 不允许为非固定类型借用loan
    RMW_SET_ERROR_MSG("Borrowing loan for a non fixed type is not allowed");
    return RMW_RET_ERROR;
  }
#else
  // 当不支持DDS_HAS_SHM时，设置错误信息并返回不支持状态
  (void)publisher;
  (void)type_support;
  (void)ros_message;
  RMW_SET_ERROR_MSG("rmw_borrow_loaned_message not implemented for rmw_cyclonedds_cpp");
  return RMW_RET_UNSUPPORTED;
#endif
}

/**
 * @brief 借用已发布的消息
 *
 * @param[in] publisher 指向已发布消息的指针
 * @param[in] type_support 消息类型支持的指针
 * @param[out] ros_message 存储借用消息的指针
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_borrow_loaned_message(
    const rmw_publisher_t *publisher,
    const rosidl_message_type_support_t *type_support,
    void **ros_message) {
  // 调用内部函数实现借用已发布的消息
  return borrow_loaned_message_int(publisher, type_support, ros_message);
}

/**
 * @brief 从发布者返回借用的消息
 *
 * @param[in] publisher 指向已发布消息的指针
 * @param[in] loaned_message 借用的消息指针
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t return_loaned_message_from_publisher_int(
    const rmw_publisher_t *publisher, void *loaned_message) {
#ifdef DDS_HAS_SHM
  // 检查发布者参数是否为空
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  // 检查发布者是否支持消息借用
  if (!publisher->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }
  // 检查借用的消息参数是否为空
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
  // 检查发布者的实现标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 将发布者数据转换为CddsPublisher类型
  auto cdds_publisher = static_cast<CddsPublisher *>(publisher->data);
  // 检查转换后的发布者数据是否为空
  if (!cdds_publisher) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  // 如果发布者支持借用
  if (cdds_publisher->is_loaning_available) {
    // 结束并释放样本
    return fini_and_free_sample(cdds_publisher, loaned_message);
  } else {
    // 不允许返回非固定类型的loan
    RMW_SET_ERROR_MSG("returning loan for a non fixed type is not allowed");
    return RMW_RET_ERROR;
  }
#else
  // 当不支持DDS_HAS_SHM时，设置错误消息并返回不支持的结果
  (void)publisher;
  (void)loaned_message;
  RMW_SET_ERROR_MSG(
      "rmw_return_loaned_message_from_publisher not implemented for rmw_cyclonedds_cpp");
  return RMW_RET_UNSUPPORTED;
#endif
}

/**
 * @brief 从发布者返回借用的消息
 *
 * 此函数将借用的消息返回给发布者，以便在RMW层进行处理。
 *
 * @param[in] publisher 指向rmw_publisher_t结构体的指针，表示要处理的发布者
 * @param[in] loaned_message 指向借用的消息的指针
 * @return rmw_ret_t 返回操作结果，成功时为RMW_RET_OK，失败时为其他错误代码
 */
extern "C" rmw_ret_t rmw_return_loaned_message_from_publisher(
    const rmw_publisher_t *publisher, void *loaned_message) {
  // 调用内部实现函数并返回结果
  return return_loaned_message_from_publisher_int(publisher, loaned_message);
}

/**
 * @brief 销毁发布者
 *
 * 此函数负责销毁给定的发布者，并释放其相关资源。
 *
 * @param[in] publisher 指向rmw_publisher_t结构体的指针，表示要销毁的发布者
 * @return rmw_ret_t 返回操作结果，成功时为RMW_RET_OK，失败时为其他错误代码
 */
static rmw_ret_t destroy_publisher(rmw_publisher_t *publisher) {
  // 初始化返回值为成功
  rmw_ret_t ret = RMW_RET_OK;

  // 将publisher->data转换为CddsPublisher类型的指针
  auto pub = static_cast<CddsPublisher *>(publisher->data);

  // 如果pub不为空，则执行以下操作
  if (pub != nullptr) {
    // 删除DDS写入器，如果删除失败，则设置错误信息并更新返回值
    if (dds_delete(pub->enth) < 0) {
      RMW_SET_ERROR_MSG("failed to delete writer");
      ret = RMW_RET_ERROR;
    }
    // 删除pub指向的对象
    delete pub;
  }

  // 释放publisher中的topic_name内存
  rmw_free(const_cast<char *>(publisher->topic_name));

  // 释放publisher内存
  rmw_publisher_free(publisher);

  // 返回操作结果
  return ret;
}

/**
 * @brief 销毁一个给定的发布者 (Destroy a given publisher)
 *
 * @param[in] node 指向要销毁发布者的节点的指针 (Pointer to the node where the publisher is to be
 * destroyed)
 * @param[in] publisher 要销毁的发布者的指针 (Pointer to the publisher to be destroyed)
 * @return rmw_ret_t 返回操作结果 (Return the result of the operation)
 */
extern "C" rmw_ret_t rmw_destroy_publisher(rmw_node_t *node, rmw_publisher_t *publisher) {
  // 检查节点是否为空，如果为空则返回无效参数错误 (Check if the node is null, return invalid
  // argument error if it is)
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查发布者是否为空，如果为空则返回无效参数错误 (Check if the publisher is null, return invalid
  // argument error if it is)
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  // 检查节点的实现标识符是否与期望的一致，如果不一致则返回错误的RMW实现错误 (Check if the node's
  // implementation identifier matches the expected one, return incorrect RMW implementation error
  // if it doesn't)
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查发布者的实现标识符是否与期望的一致，如果不一致则返回错误的RMW实现错误 (Check if the
  // publisher's implementation identifier matches the expected one, return incorrect RMW
  // implementation error if it doesn't)
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 初始化返回值为成功 (Initialize the return value as successful)
  rmw_ret_t ret = RMW_RET_OK;
  // 定义错误状态变量 (Define an error state variable)
  rmw_error_state_t error_state;
  {
    // 获取节点上下文的公共实现部分 (Get the common implementation part of the node context)
    auto common = &node->context->impl->common;
    // 将发布者的数据转换为CddsPublisher类型 (Cast the publisher's data to CddsPublisher type)
    const auto cddspub = static_cast<const CddsPublisher *>(publisher->data);
    // 对节点更新互斥锁进行加锁 (Lock the node update mutex)
    std::lock_guard<std::mutex> guard(common->node_update_mutex);
    // 从图缓存中解除与写入器的关联 (Dissociate the writer from the graph cache)
    rmw_dds_common::msg::ParticipantEntitiesInfo msg = common->graph_cache.dissociate_writer(
        cddspub->gid, common->gid, node->name, node->namespace_);
    // 发布解除关联的消息 (Publish the dissociated message)
    rmw_ret_t publish_ret = rmw_publish(common->pub, static_cast<void *>(&msg), nullptr);
    // 如果发布操作失败，则保存错误状态并重置错误 (If the publish operation fails, save the error
    // state and reset the error)
    if (RMW_RET_OK != publish_ret) {
      error_state = *rmw_get_error_state();
      ret = publish_ret;
      rmw_reset_error();
    }
  }

  // 销毁发布者 (Destroy the publisher)
  rmw_ret_t inner_ret = destroy_publisher(publisher);
  // 如果销毁操作失败，则保存错误状态并重置错误 (If the destroy operation fails, save the error
  // state and reset the error)
  if (RMW_RET_OK != inner_ret) {
    if (RMW_RET_OK != ret) {
      RMW_SAFE_FWRITE_TO_STDERR(rmw_get_error_string().str);
      RMW_SAFE_FWRITE_TO_STDERR(" during '" RCUTILS_STRINGIFY(__function__) "'\n");
    } else {
      error_state = *rmw_get_error_state();
      ret = inner_ret;
    }
    rmw_reset_error();
  }

  // 如果返回值不是成功，则设置错误状态 (If the return value is not successful, set the error state)
  if (RMW_RET_OK != ret) {
    rmw_set_error_state(error_state.message, error_state.file, error_state.line_number);
  }

  // 返回操作结果 (Return the result of the operation)
  return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    SUBSCRIPTIONS                                                  ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 创建一个CddsSubscription对象，用于订阅ROS2主题。
 *
 * @param[in] dds_ppant DDS参与者实体。
 * @param[in] dds_sub DDS订阅者实体。
 * @param[in] type_supports 消息类型支持。
 * @param[in] topic_name 主题名称。
 * @param[in] qos_policies QoS策略。
 * @param[in] ignore_local_publications 是否忽略本地发布的消息。
 * @return 成功时返回一个指向CddsSubscription的指针，失败时返回nullptr。
 */
static CddsSubscription *create_cdds_subscription(
    dds_entity_t dds_ppant,
    dds_entity_t dds_sub,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies,
    bool ignore_local_publications) {
  // 检查是否为空或空字符串，如果是则返回nullptr
  RET_NULL_OR_EMPTYSTR_X(topic_name, return nullptr);
  RET_NULL_X(qos_policies, return nullptr);
  // 获取类型支持
  const rosidl_message_type_support_t *type_support = get_typesupport(type_supports);
  RET_NULL_X(type_support, return nullptr);

  // 创建一个CddsSubscription对象
  CddsSubscription *sub = new CddsSubscription();
  dds_entity_t topic;
  dds_qos_t *qos;

  // 构造完全限定的主题名称
  std::string fqtopic_name = make_fqtopic(ROS_TOPIC_PREFIX, topic_name, "", qos_policies);
  // 检查类型是否为固定类型
  bool is_fixed_type = is_type_self_contained(type_support);
  // 获取消息样本大小
  uint32_t sample_size = static_cast<uint32_t>(rmw_cyclonedds_cpp::get_message_size(type_support));
  // 创建序列化类型
  auto sertype = create_sertype(
      type_support->typesupport_identifier,
      create_message_type_support(type_support->data, type_support->typesupport_identifier), false,
      rmw_cyclonedds_cpp::make_message_value_type(type_supports), sample_size, is_fixed_type);
  // 创建主题
  topic = create_topic(dds_ppant, fqtopic_name.c_str(), sertype);

  // 创建DDS监听器
  dds_listener_t *listener = dds_create_listener(&sub->user_callback_data);
  // 设置回调以侦听新消息
  dds_lset_data_available_arg(listener, dds_listener_callback, &sub->user_callback_data, false);
  // 设置相应的回调以侦听事件
  listener_set_event_callbacks(listener, &sub->user_callback_data);

  // 如果创建主题失败，设置错误信息并跳转到fail_topic
  if (topic < 0) {
    set_error_message_from_create_topic(topic, fqtopic_name);
    goto fail_topic;
  }
  // 创建读写QoS，如果失败则跳转到fail_qos
  if ((qos = create_readwrite_qos(
           qos_policies, *type_support->type_hash, ignore_local_publications, "")) == nullptr) {
    goto fail_qos;
  }
  // 创建DDS阅读器，如果失败则跳转到fail_reader
  if ((sub->enth = dds_create_reader(dds_sub, topic, qos, listener)) < 0) {
    RMW_SET_ERROR_MSG("failed to create reader");
    goto fail_reader;
  }
  // 获取实体的全局ID
  get_entity_gid(sub->enth, sub->gid);
  // 创建读取条件，如果失败则跳转到fail_readcond
  if ((sub->rdcondh = dds_create_readcondition(sub->enth, DDS_ANY_STATE)) < 0) {
    RMW_SET_ERROR_MSG("failed to create readcondition");
    goto fail_readcond;
  }
  // 删除监听器
  dds_delete_listener(listener);
  // 设置类型支持
  sub->type_supports = *type_support;
  // 设置是否支持loan功能
  sub->is_loaning_available = is_fixed_type && dds_is_loan_available(sub->enth);
  // 删除QoS和主题
  dds_delete_qos(qos);
  dds_delete(topic);
  // 返回创建的CddsSubscription对象
  return sub;

// 错误处理部分
fail_readcond:
  if (dds_delete(sub->enth) < 0) {
    RCUTILS_LOG_ERROR_NAMED("rmw_cyclonedds_cpp", "failed to delete reader during error handling");
  }
fail_reader:
  dds_delete_qos(qos);
fail_qos:
  dds_delete(topic);
fail_topic:
  delete sub;
  return nullptr;
}

/**
 * @brief 初始化订阅分配
 *
 * @param[in] type_support 消息类型支持结构体指针
 * @param[in] message_bounds 消息边界指针
 * @param[out] allocation 订阅分配结构体指针
 * @return rmw_ret_t 返回操作状态
 */
extern "C" rmw_ret_t rmw_init_subscription_allocation(
    const rosidl_message_type_support_t *type_support,
    const rosidl_runtime_c__Sequence__bound *message_bounds,
    rmw_subscription_allocation_t *allocation) {
  // 将输入参数转换为void类型，避免编译器警告
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(allocation);

  // 设置错误消息，表示该函数未实现
  RMW_SET_ERROR_MSG("rmw_init_subscription_allocation: unimplemented");

  // 返回不支持的状态
  return RMW_RET_UNSUPPORTED;
}

/**
 * @brief 结束订阅分配
 *
 * @param[in,out] allocation 订阅分配结构体指针
 * @return rmw_ret_t 返回操作状态
 */
extern "C" rmw_ret_t rmw_fini_subscription_allocation(rmw_subscription_allocation_t *allocation) {
  // 将输入参数转换为void类型，避免编译器警告
  static_cast<void>(allocation);

  // 设置错误消息，表示该函数未实现
  RMW_SET_ERROR_MSG("rmw_fini_subscription_allocation: unimplemented");

  // 返回不支持的状态
  return RMW_RET_UNSUPPORTED;
}

/**
 * @brief 创建一个订阅者，并返回一个指向rmw_subscription_t的指针
 *
 * 在ROS2的RMW层创建一个订阅者，用于接收特定主题的消息。
 *
 * @param[in] dds_ppant DDS参与者实体
 * @param[in] dds_sub DDS订阅者实体
 * @param[in] type_supports 消息类型支持结构体指针
 * @param[in] topic_name 要订阅的主题名称
 * @param[in] qos_policies QoS策略设置
 * @param[in] subscription_options 订阅选项
 * @return 成功时返回一个指向rmw_subscription_t的指针，失败时返回nullptr
 */
static rmw_subscription_t *create_subscription(
    dds_entity_t dds_ppant,
    dds_entity_t dds_sub,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies,
    const rmw_subscription_options_t *subscription_options) {
  // 创建CddsSubscription对象
  CddsSubscription *sub;
  rmw_subscription_t *rmw_subscription;

  // 如果创建CddsSubscription失败，则返回nullptr
  if ((sub = create_cdds_subscription(
           dds_ppant,      //
           dds_sub,        //
           type_supports,  //
           topic_name,     //
           qos_policies,   //
           subscription_options->ignore_local_publications)) == nullptr) {
    return nullptr;
  }

  // 设置清理订阅的作用域退出函数
  auto cleanup_subscription = rcpputils::make_scope_exit([sub]() {
    if (dds_delete(sub->rdcondh) < 0) {
      RMW_SAFE_FWRITE_TO_STDERR(
          "failed to delete readcondition during '" RCUTILS_STRINGIFY(__function__) "' cleanup\n");
    }
    if (dds_delete(sub->enth) < 0) {
      RMW_SAFE_FWRITE_TO_STDERR(
          "failed to delete reader during '" RCUTILS_STRINGIFY(__function__) "' cleanup\n");
    }
    delete sub;
  });

  // 分配rmw_subscription内存
  rmw_subscription = rmw_subscription_allocate();
  RET_ALLOC_X(rmw_subscription, return nullptr);

  // 设置清理rmw_subscription的作用域退出函数
  auto cleanup_rmw_subscription = rcpputils::make_scope_exit([rmw_subscription]() {
    rmw_free(const_cast<char *>(rmw_subscription->topic_name));
    rmw_subscription_free(rmw_subscription);
  });

  // 初始化rmw_subscription的属性
  rmw_subscription->implementation_identifier = eclipse_cyclonedds_identifier;
  rmw_subscription->data = sub;
  rmw_subscription->topic_name = static_cast<const char *>(rmw_allocate(strlen(topic_name) + 1));
  RET_ALLOC_X(rmw_subscription->topic_name, return nullptr);
  memcpy(const_cast<char *>(rmw_subscription->topic_name), topic_name, strlen(topic_name) + 1);
  rmw_subscription->options = *subscription_options;
  rmw_subscription->can_loan_messages = sub->is_loaning_available;
  rmw_subscription->is_cft_enabled = false;

  // 取消清理订阅和rmw_subscription的作用域退出函数
  cleanup_subscription.cancel();
  cleanup_rmw_subscription.cancel();

  // 返回创建的rmw_subscription
  return rmw_subscription;
}

/**
 * @brief 创建一个订阅者
 *
 * @param[in] node 指向要创建订阅者的节点的指针
 * @param[in] type_supports 消息类型支持结构体的指针
 * @param[in] topic_name 要订阅的主题名称
 * @param[in] qos_policies 指向QoS策略的指针
 * @param[in] subscription_options 指向订阅选项的指针
 * @return 成功时返回一个指向新创建的订阅者的指针，失败时返回nullptr
 */
extern "C" rmw_subscription_t *rmw_create_subscription(
    const rmw_node_t *node,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies,
    const rmw_subscription_options_t *subscription_options) {
  /// 前面是一系列的参数校验
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier, return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (0 == strlen(topic_name)) {  // 检查topic_name是否为空字符串
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  // 如果不遵循ROS命名空间约定，则验证主题名称
  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char *reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic_name argument: %s", reason);
      return nullptr;
    }
  }
  // 适配最佳可用QoS选项
  rmw_qos_profile_t adapted_qos_policies = *qos_policies;
  rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_topic_subscription(
      node,                   //
      topic_name,             //
      &adapted_qos_policies,  //
      rmw_get_publishers_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  // 检查subscription_options参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);
  // 检查是否严格要求唯一的网络流端点
  if (subscription_options->require_unique_network_flow_endpoints ==
      RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED) {
    RMW_SET_ERROR_MSG(
        "Strict requirement on unique network flow endpoints for subscriptions not supported");
    return nullptr;
  }

  /// 这里开始才是创建实体对象
  // 创建订阅者
  rmw_subscription_t *sub = create_subscription(
      node->context->impl->ppant,    //
      node->context->impl->dds_sub,  //
      type_supports,                 //
      topic_name,                    //
      &adapted_qos_policies,         //
      subscription_options);
  if (sub == nullptr) {
    return nullptr;
  }
  // 创建清理订阅者的作用域退出对象
  auto cleanup_subscription = rcpputils::make_scope_exit([sub]() {
    rmw_error_state_t error_state = *rmw_get_error_state();
    rmw_reset_error();
    if (RMW_RET_OK != destroy_subscription(sub)) {
      RMW_SAFE_FWRITE_TO_STDERR(rmw_get_error_string().str);
      RMW_SAFE_FWRITE_TO_STDERR(" during '" RCUTILS_STRINGIFY(__function__) "' cleanup\n");
      rmw_reset_error();
    }
    rmw_set_error_state(error_state.message, error_state.file, error_state.line_number);
  });

  // 更新图信息
  auto common = &node->context->impl->common;
  const auto cddssub = static_cast<const CddsSubscription *>(sub->data);
  std::lock_guard<std::mutex> guard(common->node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
      common->graph_cache.associate_reader(cddssub->gid, common->gid, node->name, node->namespace_);
  if (RMW_RET_OK != rmw_publish(common->pub, static_cast<void *>(&msg), nullptr)) {
    static_cast<void>(common->graph_cache.dissociate_reader(
        cddssub->gid, common->gid, node->name, node->namespace_));
    return nullptr;
  }

  // 取消清理订阅者的作用域退出操作
  cleanup_subscription.cancel();
  TRACEPOINT(rmw_subscription_init, static_cast<const void *>(sub), cddssub->gid.data);
  return sub;
}

/**
 * @brief 计算与订阅者匹配的发布者数量
 *
 * @param[in] subscription 指向要查询的订阅者对象的指针
 * @param[out] publisher_count 匹配的发布者数量
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_subscription_count_matched_publishers(
    const rmw_subscription_t *subscription, size_t *publisher_count) {
  // 检查传入的subscription参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  // 检查传入的subscription的实现标识符是否与当前使用的RMW实现匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查传入的publisher_count参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_count, RMW_RET_INVALID_ARGUMENT);

  // 将订阅者对象的数据转换为CddsSubscription类型
  auto sub = static_cast<CddsSubscription *>(subscription->data);
  dds_subscription_matched_status_t status;
  // 获取订阅者匹配状态
  if (dds_get_subscription_matched_status(sub->enth, &status) < 0) {
    return RMW_RET_ERROR;
  }

  // 设置匹配的发布者数量
  *publisher_count = status.current_count;
  return RMW_RET_OK;
}

/**
 * @brief 获取订阅者的实际QoS配置
 *
 * @param[in] subscription 指向要查询的订阅者对象的指针
 * @param[out] qos 存储实际QoS配置的结构体指针
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_subscription_get_actual_qos(
    const rmw_subscription_t *subscription, rmw_qos_profile_t *qos) {
  // 检查传入的subscription参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  // 检查传入的subscription的实现标识符是否与当前使用的RMW实现匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查传入的qos参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  // 将订阅者对象的数据转换为CddsSubscription类型
  auto sub = static_cast<CddsSubscription *>(subscription->data);
  // 获取订阅者的实际QoS配置
  if (get_readwrite_qos(sub->enth, qos)) {
    return RMW_RET_OK;
  }
  return RMW_RET_ERROR;
}

/**
 * @brief 设置订阅者的内容过滤器选项（未实现）
 *
 * @param[in] subscription 指向要设置的订阅者对象的指针
 * @param[in] options 内容过滤器选项结构体指针
 * @return rmw_ret_t 返回操作结果，由于未实现，始终返回RMW_RET_UNSUPPORTED
 */
extern "C" rmw_ret_t rmw_subscription_set_content_filter(
    rmw_subscription_t *subscription, const rmw_subscription_content_filter_options_t *options) {
  // 转换传入的参数类型，但不使用它们
  static_cast<void>(subscription);
  static_cast<void>(options);

  // 设置错误消息并返回不支持的错误代码
  RMW_SET_ERROR_MSG("rmw_subscription_set_content_filter: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

/**
 * @brief 获取订阅者的内容过滤器选项 (Get the content filter options for a subscription)
 *
 * @param[in] subscription 订阅者指针 (Pointer to the subscription)
 * @param[in] allocator 分配器指针，用于分配内存 (Pointer to the allocator for memory allocation)
 * @param[out] options 内容过滤器选项指针，用于存储结果 (Pointer to the content filter options to
 * store the result)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
extern "C" rmw_ret_t rmw_subscription_get_content_filter(
    const rmw_subscription_t *subscription,
    rcutils_allocator_t *allocator,
    rmw_subscription_content_filter_options_t *options) {
  // 将输入参数转换为 void 类型，避免编译器警告 (Cast input parameters to void type to avoid
  // compiler warnings)
  static_cast<void>(subscription);
  static_cast<void>(allocator);
  static_cast<void>(options);

  // 设置错误信息并返回不支持的操作 (Set error message and return unsupported operation)
  RMW_SET_ERROR_MSG("rmw_subscription_get_content_filter: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

/**
 * @brief 销毁订阅者 (Destroy a subscription)
 *
 * @param[in] subscription 要销毁的订阅者指针 (Pointer to the subscription to be destroyed)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
static rmw_ret_t destroy_subscription(rmw_subscription_t *subscription) {
  // 初始化返回值为成功 (Initialize return value as success)
  rmw_ret_t ret = RMW_RET_OK;

  // 将订阅者数据转换为 CddsSubscription 类型 (Cast subscription data to CddsSubscription type)
  auto sub = static_cast<CddsSubscription *>(subscription->data);

  // 清理等待集缓存 (Clean waitset caches)
  clean_waitset_caches();

  // 删除读取条件，如果失败则设置错误信息并更新返回值 (Delete read condition, if failed set error
  // message and update return value)
  if (dds_delete(sub->rdcondh) < 0) {
    RMW_SET_ERROR_MSG("failed to delete readcondition");
    ret = RMW_RET_ERROR;
  }

  // 删除读取器，如果失败则设置错误信息并更新返回值 (Delete reader, if failed set error message and
  // update return value)
  if (dds_delete(sub->enth) < 0) {
    if (RMW_RET_OK == ret) {
      RMW_SET_ERROR_MSG("failed to delete reader");
      ret = RMW_RET_ERROR;
    } else {
      RMW_SAFE_FWRITE_TO_STDERR("failed to delete reader\n");
    }
  }

  // 删除 CddsSubscription 对象 (Delete the CddsSubscription object)
  delete sub;

  // 释放订阅者的主题名称内存 (Free the memory of the subscription's topic name)
  rmw_free(const_cast<char *>(subscription->topic_name));

  // 释放订阅者内存 (Free the subscription memory)
  rmw_subscription_free(subscription);

  // 返回操作结果 (Return the operation result)
  return ret;
}

/**
 * @brief 销毁一个订阅者
 *
 * @param[in] node 指向要销毁订阅者的节点的指针
 * @param[in] subscription 要销毁的订阅者对象的指针
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_destroy_subscription(rmw_node_t *node, rmw_subscription_t *subscription) {
  // 检查输入参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);

  // 检查节点和订阅者的实现标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_ret_t ret = RMW_RET_OK;
  rmw_error_state_t error_state;
  rmw_error_string_t error_string;

  {
    auto common = &node->context->impl->common;
    const auto cddssub = static_cast<const CddsSubscription *>(subscription->data);

    // 加锁以保护共享资源
    std::lock_guard<std::mutex> guard(common->node_update_mutex);

    // 从图缓存中分离读取器
    rmw_dds_common::msg::ParticipantEntitiesInfo msg = common->graph_cache.dissociate_reader(
        cddssub->gid, common->gid, node->name, node->namespace_);

    // 发布分离读取器的消息
    ret = rmw_publish(common->pub, static_cast<void *>(&msg), nullptr);

    // 如果发布失败，记录错误状态和错误字符串
    if (RMW_RET_OK != ret) {
      error_state = *rmw_get_error_state();
      error_string = rmw_get_error_string();
      rmw_reset_error();
    }
  }

  // 销毁订阅者对象
  rmw_ret_t local_ret = destroy_subscription(subscription);

  // 检查销毁操作是否成功
  if (RMW_RET_OK != local_ret) {
    if (RMW_RET_OK != ret) {
      RMW_SAFE_FWRITE_TO_STDERR(error_string.str);
      RMW_SAFE_FWRITE_TO_STDERR(" during '" RCUTILS_STRINGIFY(__function__) "'\n");
    }
    ret = local_ret;
  } else if (RMW_RET_OK != ret) {
    rmw_set_error_state(error_state.message, error_state.file, error_state.line_number);
  }

  return ret;
}

/**
 * @brief 从dds_sample_info_t结构体中提取信息并填充到rmw_message_info_t结构体中
 *
 * @param[in] info DDS采样信息，包含了消息的元数据
 * @param[out] message_info ROS2消息信息，用于存储从DDS采样信息中提取的数据
 */
static void message_info_from_sample_info(
    const dds_sample_info_t &info, rmw_message_info_t *message_info) {
  // 设置发布者的实现标识符为eclipse_cyclonedds_identifier
  message_info->publisher_gid.implementation_identifier = eclipse_cyclonedds_identifier;

  // 将message_info中的publisher_gid.data数组初始化为0
  memset(message_info->publisher_gid.data, 0, sizeof(message_info->publisher_gid.data));

  // 确保info.publication_handle的大小不超过message_info->publisher_gid.data的大小
  assert(sizeof(info.publication_handle) <= sizeof(message_info->publisher_gid.data));

  // 将info.publication_handle复制到message_info->publisher_gid.data中
  memcpy(
      message_info->publisher_gid.data, &info.publication_handle, sizeof(info.publication_handle));

  // 设置消息的源时间戳
  message_info->source_timestamp = info.source_timestamp;

  // TODO(iluetkeb) 当Cyclone实现接收时间戳时，添加接收时间戳
  message_info->received_timestamp = 0;

  // 设置消息的发布序列号为不支持状态
  message_info->publication_sequence_number = RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED;

  // 设置消息的接收序列号为不支持状态
  message_info->reception_sequence_number = RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED;
}

/**
 * @brief 从订阅者中获取一条消息
 *
 * @param[in] subscription 订阅者对象指针
 * @param[out] ros_message 存储接收到的消息的指针
 * @param[out] taken 是否成功获取到消息的标志
 * @param[out] message_info 消息相关信息的指针，可以为 nullptr
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t rmw_take_int(
    const rmw_subscription_t *subscription,
    void *ros_message,
    bool *taken,
    rmw_message_info_t *message_info) {
  // 检查 taken 参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  // 检查 ros_message 参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

  // 检查 subscription 参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);

  // 检查订阅者的实现标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription handle, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 将订阅者的数据转换为 CddsSubscription 类型
  CddsSubscription *sub = static_cast<CddsSubscription *>(subscription->data);
  RET_NULL(sub);

  // 定义样本信息变量
  dds_sample_info_t info;

  // 循环尝试获取消息
  while (dds_take(sub->enth, &ros_message, &info, 1, 1) == 1) {
    // 如果获取到的消息有效
    if (info.valid_data) {
      // 设置 taken 标志为 true
      *taken = true;

      // 如果 message_info 不为空，将样本信息转换为消息信息
      if (message_info) {
        message_info_from_sample_info(info, message_info);
      }

      // 如果启用了报告延迟消息功能
#if REPORT_LATE_MESSAGES > 0
      dds_time_t tnow = dds_time();
      dds_time_t dt = tnow - info.source_timestamp;
      if (dt >= DDS_MSECS(REPORT_LATE_MESSAGES)) {
        fprintf(stderr, "** sample in history for %.fms\n", static_cast<double>(dt) / 1e6);
      }
#endif
      // 跳转到操作完成标签
      goto take_done;
    }
  }

  // 如果没有获取到消息，设置 taken 标志为 false
  *taken = false;

// 操作完成标签
take_done:
  TRACEPOINT(
      rmw_take, static_cast<const void *>(subscription), static_cast<const void *>(ros_message),
      (message_info ? message_info->source_timestamp : 0LL), *taken);

  // 返回操作成功
  return RMW_RET_OK;
}

/**
 * @brief 从订阅者中获取一定数量的消息序列
 *
 * @param[in] subscription 订阅者指针
 * @param[in] count 要获取的消息数量
 * @param[out] message_sequence 存储获取到的消息序列的结构体指针
 * @param[out] message_info_sequence 存储获取到的消息信息序列的结构体指针
 * @param[out] taken 实际获取到的消息数量
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回相应错误代码
 */
static rmw_ret_t rmw_take_seq(
    const rmw_subscription_t *subscription,
    size_t count,
    rmw_message_sequence_t *message_sequence,
    rmw_message_info_sequence_t *message_info_sequence,
    size_t *taken) {
  // 检查taken参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  // 检查message_sequence参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);

  // 检查message_info_sequence参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);

  // 检查subscription参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RET_WRONG_IMPLID(subscription);

  // 检查订阅者实现标识符是否匹配，不匹配则返回错误的RMW实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription handle, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 如果count为0，则返回无效参数错误
  if (0u == count) {
    RMW_SET_ERROR_MSG("count cannot be 0");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 如果message_sequence的容量小于count，则返回无效参数错误
  if (count > message_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insuffient capacity in message_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 如果message_info_sequence的容量小于count，则返回无效参数错误
  if (count > message_info_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insuffient capacity in message_info_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 如果count大于uint32_t的最大值，则返回错误
  if (count > (std::numeric_limits<uint32_t>::max)()) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Cannot take %zu samples at once, limit is %" PRIu32, count,
        (std::numeric_limits<uint32_t>::max)());
    return RMW_RET_ERROR;
  }

  // 获取订阅者数据
  CddsSubscription *sub = static_cast<CddsSubscription *>(subscription->data);
  RET_NULL(sub);

  // 初始化消息信息向量
  std::vector<dds_sample_info_t> infos(count);
  auto maxsamples = static_cast<uint32_t>(count);
  // 从订阅者中获取消息序列
  auto ret = dds_take(sub->enth, message_sequence->data, infos.data(), count, maxsamples);

  // 如果返回值小于0，表示没有获取到消息，则返回错误
  if (ret < 0) {
    return RMW_RET_ERROR;
  }

  // 初始化已获取和未获取的消息向量，用于重新排序消息序列
  std::vector<void *> taken_msg;
  std::vector<void *> not_taken_msg;
  *taken = 0u;

  // 遍历获取到的消息信息
  for (int ii = 0; ii < ret; ++ii) {
    const dds_sample_info_t &info = infos[ii];

    void *message = &message_sequence->data[ii];
    rmw_message_info_t *message_info = &message_info_sequence->data[*taken];

    // 如果消息有效，则将其添加到已获取的消息向量中，并更新已获取的消息数量
    if (info.valid_data) {
      taken_msg.push_back(message);
      (*taken)++;
      if (message_info) {
        message_info_from_sample_info(info, message_info);
      }
    } else {
      // 如果消息无效，则将其添加到未获取的消息向量中
      not_taken_msg.push_back(message);
    }
  }

  // 将已获取的消息放入消息序列的前部分
  for (size_t ii = 0; ii < taken_msg.size(); ++ii) {
    message_sequence->data[ii] = taken_msg[ii];
  }

  // 将未获取的消息放入消息序列的后部分
  for (size_t ii = 0; ii < not_taken_msg.size(); ++ii) {
    message_sequence->data[ii + taken_msg.size()] = not_taken_msg[ii];
  }

  // 更新消息序列和消息信息序列的大小
  message_sequence->size = *taken;
  message_info_sequence->size = *taken;

  // 返回操作成功
  return RMW_RET_OK;
}

/**
 * @brief 从订阅者中获取序列化消息
 *
 * @param[in] subscription 订阅者对象指针
 * @param[out] serialized_message 序列化消息的指针
 * @param[out] taken 是否成功获取到序列化消息的标志
 * @param[out] message_info 消息信息的指针，可选参数，如果为 nullptr，则不设置消息信息
 * @return rmw_ret_t 返回操作结果状态码
 */
static rmw_ret_t rmw_take_ser_int(
    const rmw_subscription_t *subscription,
    rmw_serialized_message_t *serialized_message,
    bool *taken,
    rmw_message_info_t *message_info) {
  // 检查输入参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  // 检查订阅者实现是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription handle, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 获取CddsSubscription对象
  CddsSubscription *sub = static_cast<CddsSubscription *>(subscription->data);
  RET_NULL(sub);

  dds_sample_info_t info;
  struct ddsi_serdata *d;

  // 循环尝试获取序列化消息
  while (dds_takecdr(sub->enth, &d, 1, &info, DDS_ANY_STATE) == 1) {
    if (info.valid_data) {
      // 设置消息信息
      if (message_info) {
        message_info_from_sample_info(info, message_info);
      }

#ifdef DDS_HAS_SHM
      /**
       * @brief 从共享内存中获取数据并序列化
       *
       * @param[in] d 指向ddsi_serdata的指针，包含了共享内存块的信息
       * @param[in] sub 订阅者类型支持的指针
       * @param[out] serialized_message 序列化后的消息
       * @param[out] taken 是否成功获取到数据的标志
       * @return rmw_ret_t 返回操作结果
       */
      if (d->iox_chunk != nullptr) {
        // 获取Iceoryx头部信息
        auto iox_header = iceoryx_header_from_chunk(d->iox_chunk);
        // 判断共享内存中的数据是否为序列化数据
        if (iox_header->shm_data_state == IOX_CHUNK_CONTAINS_SERIALIZED_DATA) {
          // 获取数据大小
          const size_t size = iox_header->data_size;
          // 调整序列化消息的大小
          if (rmw_serialized_message_resize(serialized_message, size) != RMW_RET_OK) {
            ddsi_serdata_unref(d);
            *taken = false;
            return RMW_RET_ERROR;
          }
          // 将共享内存中的序列化数据复制到序列化消息缓冲区
          ddsi_serdata_to_ser(d, 0, size, serialized_message->buffer);
          // 设置序列化消息的长度
          serialized_message->buffer_length = size;
          // 取消对共享内存块的引用
          ddsi_serdata_unref(d);
          *taken = true;
          return RMW_RET_OK;
        } else if (iox_header->shm_data_state == IOX_CHUNK_CONTAINS_RAW_DATA) {
          // 将共享内存中的原始数据序列化
          if (rmw_serialize(d->iox_chunk, &sub->type_supports, serialized_message) != RMW_RET_OK) {
            RMW_SET_ERROR_MSG("Failed to serialize sample from loaned memory");
            ddsi_serdata_unref(d);
            *taken = false;
            return RMW_RET_ERROR;
          }
          // 取消对共享内存块的引用
          ddsi_serdata_unref(d);
          *taken = true;
          return RMW_RET_OK;
        } else {
          // 如果共享内存中的数据未初始化，则返回错误
          RMW_SET_ERROR_MSG("The recieved sample over SHM is not initialized");
          ddsi_serdata_unref(d);
          return RMW_RET_ERROR;
        }
        // 释放chunk
        free_iox_chunk(static_cast<iox_sub_t *>(d->iox_subscriber), &d->iox_chunk);
      } else  // NOLINT
#endif
      /**
       * @brief 将ddsi_serdata对象序列化为rmw_serialized_message，并设置taken标志。
       *
       * @param[in] d 输入的ddsi_serdata对象，需要被序列化。
       * @param[out] serialized_message 输出的rmw_serialized_message对象，存储序列化后的数据。
       * @param[out] taken 输出布尔指针，表示是否成功序列化并设置了serialized_message。
       * @return RMW_RET_OK 如果序列化成功，否则返回RMW_RET_ERROR。
       */
      {
        // 获取ddsi_serdata对象的大小
        size_t size = ddsi_serdata_size(d);

        // 调整rmw_serialized_message的大小以适应序列化数据
        if (rmw_serialized_message_resize(serialized_message, size) != RMW_RET_OK) {
          // 如果调整大小失败，则取消引用ddsi_serdata对象并设置taken为false
          ddsi_serdata_unref(d);
          *taken = false;
          return RMW_RET_ERROR;
        }
        // 将ddsi_serdata对象序列化到rmw_serialized_message的缓冲区中
        ddsi_serdata_to_ser(d, 0, size, serialized_message->buffer);
        // 设置rmw_serialized_message的缓冲区长度为序列化数据的大小
        serialized_message->buffer_length = size;
        // 取消引用ddsi_serdata对象
        ddsi_serdata_unref(d);
        // 设置taken为true，表示已成功序列化并设置了serialized_message
        *taken = true;
        // 返回RMW_RET_OK表示序列化成功
        return RMW_RET_OK;
      }
    }
    ddsi_serdata_unref(d);
  }
  *taken = false;
  return RMW_RET_OK;
}

/**
 * @brief 从订阅中获取已借出的消息
 *
 * @param[in] subscription 订阅指针
 * @param[out] loaned_message 已借出的消息指针
 * @param[out] taken 是否成功获取到消息的标志
 * @param[out] message_info 消息信息指针，可选参数
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t rmw_take_loan_int(
    const rmw_subscription_t *subscription,
    void **loaned_message,
    bool *taken,
    rmw_message_info_t *message_info) {
#ifdef DDS_HAS_SHM
  // 检查订阅是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  // 检查订阅是否支持消息借用
  if (!subscription->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }
  // 检查已借出的消息指针是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
  // 检查是否成功获取到消息的标志是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  // 检查订阅句柄类型是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription handle, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 将订阅数据转换为CddsSubscription类型
  auto cdds_subscription = static_cast<CddsSubscription *>(subscription->data);
  // 检查订阅数据是否为空
  if (!cdds_subscription) {
    RMW_SET_ERROR_MSG("Subscription data is null");
    return RMW_RET_ERROR;
  }

  dds_sample_info_t info;
  struct ddsi_serdata *d;
  // 循环获取消息
  while (dds_takecdr(cdds_subscription->enth, &d, 1, &info, DDS_ANY_STATE) == 1) {
    // 如果消息有效
    if (info.valid_data) {
      // 设置消息信息
      if (message_info) {
        message_info_from_sample_info(info, message_info);
      }
      // 如果共享内存中有数据
      if (d->iox_chunk != nullptr) {
        // 根据数据类型返回用户所需的数据
        auto iox_header = iceoryx_header_from_chunk(d->iox_chunk);
        // 如果共享内存中的数据是序列化形式
        if (iox_header->shm_data_state == IOX_CHUNK_CONTAINS_SERIALIZED_DATA) {
          rmw_serialized_message_t ser_msg;
          ser_msg.buffer_length = iox_header->data_size;
          ser_msg.buffer = static_cast<uint8_t *>(d->iox_chunk);
          // 反序列化共享内存缓冲区中的样本
          if (rmw_deserialize(&ser_msg, &cdds_subscription->type_supports, *loaned_message) !=
              RMW_RET_OK) {
            RMW_SET_ERROR_MSG("Failed to deserialize sample from shared memory buffer");
            ddsi_serdata_unref(d);
            *taken = false;
            return RMW_RET_ERROR;
          }
        } else if (iox_header->shm_data_state == IOX_CHUNK_CONTAINS_RAW_DATA) {
          // 如果共享内存中的数据是原始形式
          *loaned_message = d->iox_chunk;
        } else {
          RMW_SET_ERROR_MSG("Received iox chunk is uninitialized");
          ddsi_serdata_unref(d);
          *taken = false;
          return RMW_RET_ERROR;
        }
        *taken = true;
        // 初始化分配器以便在归还借用时释放块
        dds_data_allocator_init(cdds_subscription->enth, &cdds_subscription->data_allocator);
        // 将借出的块设置为null，以便在rmw_serdata_free()中不释放借出的块，而是在`rmw_return_loaned_message_from_subscription()`调用时释放
        d->iox_chunk = nullptr;
        ddsi_serdata_unref(d);
        return RMW_RET_OK;
      } else if (d->type->iox_size > 0U) {
        // 初始化并分配样本
        auto sample_ptr = init_and_alloc_sample(cdds_subscription, d->type->iox_size, true);
        RET_NULL_X(sample_ptr, return RMW_RET_ERROR);
        ddsi_serdata_to_sample(d, sample_ptr, nullptr, nullptr);
        *loaned_message = sample_ptr;
        ddsi_serdata_unref(d);
        *taken = true;
        return RMW_RET_OK;
      } else {
        RMW_SET_ERROR_MSG("Data nor loan is available to take");
        ddsi_serdata_unref(d);
        *taken = false;
        return RMW_RET_ERROR;
      }
    }
    ddsi_serdata_unref(d);
  }
  *taken = false;
  return RMW_RET_OK;
#else
  // 不支持的情况下设置错误消息
  static_cast<void>(subscription);
  static_cast<void>(loaned_message);
  static_cast<void>(taken);
  static_cast<void>(message_info);
  RMW_SET_ERROR_MSG("rmw_take_loaned_message not implemented for rmw_cyclonedds_cpp");
  return RMW_RET_UNSUPPORTED;
#endif
}

/**
 * @brief 从订阅者中获取消息
 *
 * @param[in] subscription 订阅者指针
 * @param[out] ros_message 存储接收到的消息的指针
 * @param[out] taken 是否成功接收到消息的标志
 * @param[in] allocation 分配器指针（未使用）
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take(
    const rmw_subscription_t *subscription,
    void *ros_message,
    bool *taken,
    rmw_subscription_allocation_t *allocation) {
  static_cast<void>(allocation);                                   // 忽略分配器参数
  return rmw_take_int(subscription, ros_message, taken, nullptr);  // 调用内部实现函数
}

/**
 * @brief 从订阅者中获取消息及其相关信息
 *
 * @param[in] subscription 订阅者指针
 * @param[out] ros_message 存储接收到的消息的指针
 * @param[out] taken 是否成功接收到消息的标志
 * @param[out] message_info 存储接收到的消息相关信息的指针
 * @param[in] allocation 分配器指针（未使用）
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_with_info(
    const rmw_subscription_t *subscription,
    void *ros_message,
    bool *taken,
    rmw_message_info_t *message_info,
    rmw_subscription_allocation_t *allocation) {
  static_cast<void>(allocation);  // 忽略分配器参数
  RMW_CHECK_ARGUMENT_FOR_NULL(
      message_info,
      RMW_RET_INVALID_ARGUMENT);  // 检查 message_info 参数是否为空
  return rmw_take_int(subscription, ros_message, taken, message_info);  // 调用内部实现函数
}

/**
 * @brief 从订阅者中获取一系列消息及其相关信息
 *
 * @param[in] subscription 订阅者指针
 * @param[in] count 要获取的消息数量
 * @param[out] message_sequence 存储接收到的消息序列的指针
 * @param[out] message_info_sequence 存储接收到的消息相关信息序列的指针
 * @param[out] taken 实际接收到的消息数量
 * @param[in] allocation 分配器指针（未使用）
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_sequence(
    const rmw_subscription_t *subscription,
    size_t count,
    rmw_message_sequence_t *message_sequence,
    rmw_message_info_sequence_t *message_info_sequence,
    size_t *taken,
    rmw_subscription_allocation_t *allocation) {
  static_cast<void>(allocation);  // 忽略分配器参数
  return rmw_take_seq(
      subscription, count, message_sequence, message_info_sequence,
      taken);  // 调用内部实现函数
}

/**
 * @brief 从订阅者中获取序列化后的消息
 *
 * @param[in] subscription 订阅者指针
 * @param[out] serialized_message 存储接收到的序列化消息的指针
 * @param[out] taken 是否成功接收到序列化消息的标志
 * @param[in] allocation 分配器指针（未使用）
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_serialized_message(
    const rmw_subscription_t *subscription,
    rmw_serialized_message_t *serialized_message,
    bool *taken,
    rmw_subscription_allocation_t *allocation) {
  static_cast<void>(allocation);  // 忽略分配器参数
  return rmw_take_ser_int(subscription, serialized_message, taken, nullptr);  // 调用内部实现函数
}

/**
 * @brief 从订阅者中获取序列化后的消息及其相关信息
 *
 * @param[in] subscription 订阅者指针
 * @param[out] serialized_message 存储接收到的序列化消息的指针
 * @param[out] taken 是否成功接收到序列化消息的标志
 * @param[out] message_info 存储接收到的消息相关信息的指针
 * @param[in] allocation 分配器指针（未使用）
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_serialized_message_with_info(
    const rmw_subscription_t *subscription,
    rmw_serialized_message_t *serialized_message,
    bool *taken,
    rmw_message_info_t *message_info,
    rmw_subscription_allocation_t *allocation) {
  static_cast<void>(allocation);  // 忽略分配器参数

  RMW_CHECK_ARGUMENT_FOR_NULL(
      message_info,
      RMW_RET_INVALID_ARGUMENT);  // 检查 message_info 参数是否为空

  return rmw_take_ser_int(
      subscription, serialized_message, taken,
      message_info);  // 调用内部实现函数
}

/**
 * @brief 从订阅者中获取借用的消息
 *
 * @param[in] subscription 订阅者指针，用于接收消息
 * @param[out] loaned_message 借用的消息指针，用于存储接收到的消息
 * @param[out] taken 指示是否成功获取到消息的布尔指针
 * @param[in] allocation 分配器指针（未使用）
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_loaned_message(
    const rmw_subscription_t *subscription,
    void **loaned_message,
    bool *taken,
    rmw_subscription_allocation_t *allocation) {
  // 忽略分配器参数
  static_cast<void>(allocation);
  // 调用内部函数处理
  return rmw_take_loan_int(subscription, loaned_message, taken, nullptr);
}

/**
 * @brief 从订阅者中获取借用的消息，并附带消息信息
 *
 * @param[in] subscription 订阅者指针，用于接收消息
 * @param[out] loaned_message 借用的消息指针，用于存储接收到的消息
 * @param[out] taken 指示是否成功获取到消息的布尔指针
 * @param[out] message_info 消息信息指针，用于存储接收到的消息的相关信息
 * @param[in] allocation 分配器指针（未使用）
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_loaned_message_with_info(
    const rmw_subscription_t *subscription,
    void **loaned_message,
    bool *taken,
    rmw_message_info_t *message_info,
    rmw_subscription_allocation_t *allocation) {
  // 忽略分配器参数
  static_cast<void>(allocation);
  // 检查消息信息指针是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  // 调用内部函数处理
  return rmw_take_loan_int(subscription, loaned_message, taken, message_info);
}

/**
 * @brief 从订阅者中返回借用的消息
 *
 * @param[in] subscription 订阅者指针，不能为空
 * @param[in,out] loaned_message 借用的消息指针，不能为空
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t return_loaned_message_from_subscription_int(
    const rmw_subscription_t *subscription, void *loaned_message) {
#ifdef DDS_HAS_SHM
  // 检查订阅者参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  // 检查订阅者是否支持消息借用
  if (!subscription->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }
  // 检查借用的消息参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
  // 检查订阅者句柄类型是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      subscription handle, subscription->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 将订阅者数据转换为CddsSubscription类型
  auto cdds_subscription = static_cast<CddsSubscription *>(subscription->data);
  // 检查订阅者数据是否为空
  if (!cdds_subscription) {
    RMW_SET_ERROR_MSG("Subscription data is null");
    return RMW_RET_ERROR;
  }

  // 如果订阅者允许借用
  if (cdds_subscription->is_loaning_available) {
    // 释放并完成样本
    return fini_and_free_sample(cdds_subscription, loaned_message);
  } else {
    // 不允许为非固定类型返回借用
    RMW_SET_ERROR_MSG("returning loan for a non fixed type is not allowed");
    return RMW_RET_ERROR;
  }
  // 返回操作成功
  return RMW_RET_OK;
#else
  // 当不支持DDS_HAS_SHM时，设置错误消息并返回不支持的结果
  (void)subscription;
  (void)loaned_message;
  RMW_SET_ERROR_MSG(
      "rmw_return_loaned_message_from_subscription not implemented for rmw_cyclonedds_cpp");
  return RMW_RET_UNSUPPORTED;
#endif
}

/**
 * @brief 从订阅者返回借用的消息
 *
 * 此函数将借用的消息返回给订阅者。它是ROS2 RMW层的一部分。
 *
 * @param[in] subscription 指向rmw_subscription_t结构体的指针，表示订阅者
 * @param[in,out] loaned_message 指向借用的消息的指针，该消息将被返回给订阅者
 * @return 返回rmw_ret_t类型的结果，表示操作成功或失败
 */
extern "C" rmw_ret_t rmw_return_loaned_message_from_subscription(
    const rmw_subscription_t *subscription, void *loaned_message) {
  // 调用内部实现函数，并传入订阅者和借用的消息
  return return_loaned_message_from_subscription_int(subscription, loaned_message);
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    EVENTS                                                         ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/// @brief 映射 RMW_EVENT 到相应的 DDS 状态
/// mapping of RMW_EVENT to the corresponding DDS status
static const std::unordered_map<rmw_event_type_t, uint32_t> mask_map{
    // Liveliness 变化事件
    {RMW_EVENT_LIVELINESS_CHANGED, DDS_LIVELINESS_CHANGED_STATUS},
    // 请求的 Deadline 被错过事件
    {RMW_EVENT_REQUESTED_DEADLINE_MISSED, DDS_REQUESTED_DEADLINE_MISSED_STATUS},
    // Liveliness 丢失事件
    {RMW_EVENT_LIVELINESS_LOST, DDS_LIVELINESS_LOST_STATUS},
    // 提供的 Deadline 被错过事件
    {RMW_EVENT_OFFERED_DEADLINE_MISSED, DDS_OFFERED_DEADLINE_MISSED_STATUS},
    // 请求的 QoS 不兼容事件
    {RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS},
    // 提供的 QoS 不兼容事件
    {RMW_EVENT_OFFERED_QOS_INCOMPATIBLE, DDS_OFFERED_INCOMPATIBLE_QOS_STATUS},
    // 消息丢失事件
    {RMW_EVENT_MESSAGE_LOST, DDS_SAMPLE_LOST_STATUS},
    // 发布者不兼容类型事件
    {RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE, DDS_INCONSISTENT_TOPIC_STATUS},
    // 订阅者不兼容类型事件
    {RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE, DDS_INCONSISTENT_TOPIC_STATUS},
    // 订阅匹配事件
    {RMW_EVENT_SUBSCRIPTION_MATCHED, DDS_SUBSCRIPTION_MATCHED_STATUS},
    // 发布匹配事件
    {RMW_EVENT_PUBLICATION_MATCHED, DDS_PUBLICATION_MATCHED_STATUS}};

/// @brief 检查事件是否受支持
/// @param[in] event_t 要检查的事件类型
/// @return 如果事件受支持，则返回 true，否则返回 false
static bool is_event_supported(const rmw_event_type_t event_t) {
  return mask_map.count(event_t) == 1;
}

/// @brief 从 RMW 事件类型获取对应的 DDS 状态类型
/// @param[in] event_t RMW 事件类型
/// @return 对应的 DDS 状态类型
static uint32_t get_status_kind_from_rmw(const rmw_event_type_t event_t) {
  return mask_map.at(event_t);
}

/// @brief 初始化 RMW 事件
/// @param[out] rmw_event 要初始化的 RMW 事件指针
/// @param[in] topic_endpoint_impl_identifier 主题端点实现标识符
/// @param[in] data 事件数据指针
/// @param[in] event_type 事件类型
/// @return 成功时返回 RMW_RET_OK，失败时返回相应的错误代码
static rmw_ret_t init_rmw_event(
    rmw_event_t *rmw_event,
    const char *topic_endpoint_impl_identifier,
    void *data,
    rmw_event_type_t event_type) {
  // 检查 rmw_event 参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  // 检查 topic_endpoint_impl_identifier 参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_endpoint_impl_identifier, RMW_RET_INVALID_ARGUMENT);
  // 检查 data 参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(data, RMW_RET_INVALID_ARGUMENT);
  // 检查事件类型是否受支持
  if (!is_event_supported(event_type)) {
    RMW_SET_ERROR_MSG("provided event_type is not supported by rmw_cyclonedds_cpp");
    return RMW_RET_UNSUPPORTED;
  }
  // 设置实现标识符
  rmw_event->implementation_identifier = topic_endpoint_impl_identifier;
  // 设置事件数据
  rmw_event->data = data;
  // 设置事件类型
  rmw_event->event_type = event_type;
  // 返回成功状态
  return RMW_RET_OK;
}

/**
 * @brief 初始化发布者事件
 *
 * @param[in] rmw_event 指向要初始化的rmw_event_t结构体的指针
 * @param[in] publisher 指向已创建的rmw_publisher_t结构体的指针
 * @param[in] event_type 要初始化的事件类型
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_publisher_event_init(
    rmw_event_t *rmw_event, const rmw_publisher_t *publisher, rmw_event_type_t event_type) {
  // 检查publisher是否为空，为空则返回RMW_RET_INVALID_ARGUMENT
  RET_NULL(publisher);
  // 检查publisher的实现ID是否正确，不正确则返回RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  RET_WRONG_IMPLID(publisher);
  // 调用init_rmw_event函数进行事件初始化，并返回结果
  return init_rmw_event(
      rmw_event, publisher->implementation_identifier, publisher->data, event_type);
}

/**
 * @brief 初始化订阅者事件
 *
 * @param[in] rmw_event 指向要初始化的rmw_event_t结构体的指针
 * @param[in] subscription 指向已创建的rmw_subscription_t结构体的指针
 * @param[in] event_type 要初始化的事件类型
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_subscription_event_init(
    rmw_event_t *rmw_event, const rmw_subscription_t *subscription, rmw_event_type_t event_type) {
  // 检查subscription是否为空，为空则返回RMW_RET_INVALID_ARGUMENT
  RET_NULL(subscription);
  // 检查subscription的实现ID是否正确，不正确则返回RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  RET_WRONG_IMPLID(subscription);
  // 调用init_rmw_event函数进行事件初始化，并返回结果
  return init_rmw_event(
      rmw_event, subscription->implementation_identifier, subscription->data, event_type);
}

/**
 * @brief 从事件句柄中获取事件信息。
 *
 * @param[in] event_handle 事件句柄，不能为空。
 * @param[out] event_info 存储事件信息的指针，不能为空。
 * @param[out] taken 用于表示是否成功获取到事件信息的布尔值指针，不能为空。
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回RMW_RET_ERROR。
 */
extern "C" rmw_ret_t rmw_take_event(
    const rmw_event_t *event_handle, void *event_info, bool *taken) {
  // 检查event_handle是否为空
  RET_NULL(event_handle);
  // 检查event_handle的实现ID是否正确
  RET_WRONG_IMPLID(event_handle);
  // 检查taken是否为空
  RET_NULL(taken);
  // 检查event_info是否为空
  RET_NULL(event_info);

  // 根据事件类型处理事件
  switch (event_handle->event_type) {
    case RMW_EVENT_LIVELINESS_CHANGED: {
      // 类型转换
      auto ei = static_cast<rmw_liveliness_changed_status_t *>(event_info);
      auto sub = static_cast<CddsSubscription *>(event_handle->data);
      dds_liveliness_changed_status_t st;

      // 获取活跃状态改变信息
      if (dds_get_liveliness_changed_status(sub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置活跃状态改变信息
        ei->alive_count = static_cast<int32_t>(st.alive_count);
        ei->not_alive_count = static_cast<int32_t>(st.not_alive_count);
        ei->alive_count_change = st.alive_count_change;
        ei->not_alive_count_change = st.not_alive_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_REQUESTED_DEADLINE_MISSED: {
      // 类型转换
      auto ei = static_cast<rmw_requested_deadline_missed_status_t *>(event_info);
      auto sub = static_cast<CddsSubscription *>(event_handle->data);
      dds_requested_deadline_missed_status_t st;

      // 获取请求的截止时间未满足状态信息
      if (dds_get_requested_deadline_missed_status(sub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置请求的截止时间未满足状态信息
        ei->total_count = static_cast<int32_t>(st.total_count);
        ei->total_count_change = st.total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE: {
      // 类型转换
      auto ei = static_cast<rmw_requested_qos_incompatible_event_status_t *>(event_info);
      auto sub = static_cast<CddsSubscription *>(event_handle->data);
      dds_requested_incompatible_qos_status_t st;

      // 获取请求的QoS不兼容状态信息
      if (dds_get_requested_incompatible_qos_status(sub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置请求的QoS不兼容状态信息
        ei->total_count = static_cast<int32_t>(st.total_count);
        ei->total_count_change = st.total_count_change;
        ei->last_policy_kind =
            dds_qos_policy_to_rmw_qos_policy(static_cast<dds_qos_policy_id_t>(st.last_policy_id));
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_MESSAGE_LOST: {
      // 类型转换
      auto ei = static_cast<rmw_message_lost_status_t *>(event_info);
      auto sub = static_cast<CddsSubscription *>(event_handle->data);
      dds_sample_lost_status_t st;

      // 获取消息丢失状态信息
      if (dds_get_sample_lost_status(sub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      }
      // 设置消息丢失状态信息
      ei->total_count = static_cast<size_t>(st.total_count);
      ei->total_count_change = static_cast<size_t>(st.total_count_change);
      *taken = true;
      return RMW_RET_OK;
    }

    case RMW_EVENT_SUBSCRIPTION_MATCHED: {
      // 类型转换
      auto ei = static_cast<rmw_matched_status_t *>(event_info);
      auto sub = static_cast<CddsSubscription *>(event_handle->data);

      dds_subscription_matched_status_t st;
      // 获取订阅匹配状态信息
      if (dds_get_subscription_matched_status(sub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      }
      // 设置订阅匹配状态信息
      ei->total_count = static_cast<size_t>(st.total_count);
      ei->total_count_change = static_cast<size_t>(st.total_count_change);
      ei->current_count = static_cast<size_t>(st.current_count);
      ei->current_count_change = st.current_count_change;
      *taken = true;
      return RMW_RET_OK;
    }

    case RMW_EVENT_LIVELINESS_LOST: {
      // 类型转换
      auto ei = static_cast<rmw_liveliness_lost_status_t *>(event_info);
      auto pub = static_cast<CddsPublisher *>(event_handle->data);
      dds_liveliness_lost_status_t st;

      // 获取活跃状态丢失信息
      if (dds_get_liveliness_lost_status(pub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置活跃状态丢失信息
        ei->total_count = static_cast<int32_t>(st.total_count);
        ei->total_count_change = st.total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_OFFERED_DEADLINE_MISSED: {
      // 类型转换
      auto ei = static_cast<rmw_offered_deadline_missed_status_t *>(event_info);
      auto pub = static_cast<CddsPublisher *>(event_handle->data);
      dds_offered_deadline_missed_status_t st;

      // 获取提供的截止时间未满足状态信息
      if (dds_get_offered_deadline_missed_status(pub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置提供的截止时间未满足状态信息
        ei->total_count = static_cast<int32_t>(st.total_count);
        ei->total_count_change = st.total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE: {
      // 类型转换
      auto ei = static_cast<rmw_offered_qos_incompatible_event_status_t *>(event_info);
      auto pub = static_cast<CddsPublisher *>(event_handle->data);
      dds_offered_incompatible_qos_status_t st;

      // 获取提供的QoS不兼容状态信息
      if (dds_get_offered_incompatible_qos_status(pub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置提供的QoS不兼容状态信息
        ei->total_count = static_cast<int32_t>(st.total_count);
        ei->total_count_change = st.total_count_change;
        ei->last_policy_kind =
            dds_qos_policy_to_rmw_qos_policy(static_cast<dds_qos_policy_id_t>(st.last_policy_id));
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE: {
      // 类型转换
      auto it = static_cast<rmw_incompatible_type_status_t *>(event_info);
      auto pub = static_cast<CddsPublisher *>(event_handle->data);

      const dds_entity_t topic = dds_get_topic(pub->enth);
      dds_inconsistent_topic_status_t st;
      // 获取发布者不兼容类型状态信息
      if (dds_get_inconsistent_topic_status(topic, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置发布者不兼容类型状态信息
        it->total_count = static_cast<int32_t>(st.total_count);
        it->total_count_change = st.total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE: {
      // 类型转换
      auto it = static_cast<rmw_incompatible_type_status_t *>(event_info);
      auto sub = static_cast<CddsSubscription *>(event_handle->data);

      const dds_entity_t topic = dds_get_topic(sub->enth);
      dds_inconsistent_topic_status_t st;
      // 获取订阅者不兼容类型状态信息
      if (dds_get_inconsistent_topic_status(topic, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      } else {
        // 设置订阅者不兼容类型状态信息
        it->total_count = static_cast<int32_t>(st.total_count);
        it->total_count_change = st.total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    }

    case RMW_EVENT_PUBLICATION_MATCHED: {
      // 类型转换
      auto ei = static_cast<rmw_matched_status_t *>(event_info);
      auto pub = static_cast<CddsPublisher *>(event_handle->data);

      dds_publication_matched_status st;
      // 获取发布匹配状态信息
      if (dds_get_publication_matched_status(pub->enth, &st) < 0) {
        *taken = false;
        return RMW_RET_ERROR;
      }
      // 设置发布匹配状态信息
      ei->total_count = static_cast<size_t>(st.total_count);
      ei->total_count_change = static_cast<size_t>(st.total_count_change);
      ei->current_count = static_cast<size_t>(st.current_count);
      ei->current_count_change = st.current_count_change;
      *taken = true;
      return RMW_RET_OK;
    }

    case RMW_EVENT_INVALID: {
      break;
    }

    default:
      rmw_cyclonedds_cpp::unreachable();
  }
  *taken = false;
  return RMW_RET_ERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    GUARDS AND WAITSETS                                            ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 创建一个守护条件对象
 *
 * @return 返回创建的守护条件对象指针，如果创建失败则返回nullptr
 */
static rmw_guard_condition_t *create_guard_condition() {
  // 定义守护条件句柄
  rmw_guard_condition_t *guard_condition_handle;

  // 创建CddsGuardCondition实例
  auto *gcond_impl = new CddsGuardCondition();

  // 使用CycloneDDS API创建守护条件，并检查是否成功
  if ((gcond_impl->gcondh = dds_create_guardcondition(DDS_CYCLONEDDS_HANDLE)) < 0) {
    // 设置错误信息
    RMW_SET_ERROR_MSG("failed to create guardcondition");
    // 跳转到错误处理标签
    goto fail_guardcond;
  }

  // 分配内存并初始化守护条件句柄
  guard_condition_handle = new rmw_guard_condition_t;
  guard_condition_handle->implementation_identifier = eclipse_cyclonedds_identifier;
  guard_condition_handle->data = gcond_impl;

  // 返回守护条件句柄
  return guard_condition_handle;

// 错误处理标签
fail_guardcond:
  // 删除CddsGuardCondition实例
  delete (gcond_impl);
  // 返回空指针
  return nullptr;
}

/**
 * @brief 创建一个守护条件对象（外部C接口）
 *
 * @param[in] context ROS2上下文对象
 * @return 返回创建的守护条件对象指针，如果创建失败则返回nullptr
 */
extern "C" rmw_guard_condition_t *rmw_create_guard_condition(rmw_context_t *context) {
  // 忽略上下文参数
  (void)context;
  // 调用内部创建守护条件函数
  return create_guard_condition();
}

/**
 * @brief 销毁一个守护条件对象
 *
 * @param[in] guard_condition_handle 守护条件句柄
 * @return 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
static rmw_ret_t destroy_guard_condition(rmw_guard_condition_t *guard_condition_handle) {
  // 检查输入参数是否为空
  RET_NULL(guard_condition_handle);

  // 获取CddsGuardCondition实例
  auto *gcond_impl = static_cast<CddsGuardCondition *>(guard_condition_handle->data);

  // 清理等待集缓存
  clean_waitset_caches();

  // 使用CycloneDDS API删除守护条件
  dds_delete(gcond_impl->gcondh);

  // 删除CddsGuardCondition实例和守护条件句柄
  delete gcond_impl;
  delete guard_condition_handle;

  // 返回操作成功
  return RMW_RET_OK;
}

/**
 * @brief 销毁一个守护条件对象（外部C接口）
 *
 * @param[in] guard_condition_handle 守护条件句柄
 * @return 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t *guard_condition_handle) {
  // 调用内部销毁守护条件函数
  return destroy_guard_condition(guard_condition_handle);
}

/**
 * @brief 触发守护条件
 *
 * @param[in] guard_condition_handle 守护条件句柄
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_trigger_guard_condition(
    const rmw_guard_condition_t *guard_condition_handle) {
  // 检查守护条件句柄是否为空
  RET_NULL(guard_condition_handle);
  // 检查实现标识符是否正确
  RET_WRONG_IMPLID(guard_condition_handle);
  // 将守护条件句柄的数据转换为 CddsGuardCondition 类型
  auto *gcond_impl = static_cast<CddsGuardCondition *>(guard_condition_handle->data);
  // 设置守护条件状态为 true
  dds_set_guardcondition(gcond_impl->gcondh, true);
  // 返回成功
  return RMW_RET_OK;
}

/**
 * @brief 创建等待集
 *
 * @param[in] context ROS2 上下文
 * @param[in] max_conditions 最大条件数量
 * @return rmw_wait_set_t* 返回创建的等待集指针
 */
extern "C" rmw_wait_set_t *rmw_create_wait_set(rmw_context_t *context, size_t max_conditions) {
  // 检查上下文是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  // 忽略最大条件数量参数
  (void)max_conditions;
  // 分配等待集内存
  rmw_wait_set_t *wait_set = rmw_wait_set_allocate();
  CddsWaitset *ws = nullptr;
  // 检查分配结果
  RET_ALLOC_X(wait_set, goto fail_alloc_wait_set);
  // 设置实现标识符
  wait_set->implementation_identifier = eclipse_cyclonedds_identifier;
  // 分配等待集数据内存
  wait_set->data = rmw_allocate(sizeof(CddsWaitset));
  // 检查分配结果
  RET_ALLOC_X(wait_set->data, goto fail_alloc_wait_set_data);
  // 默认构造 CddsWaitset 字段
  ws = static_cast<CddsWaitset *>(wait_set->data);
  // 尝试在已分配的内存中构造 CddsWaitset 对象
  RMW_TRY_PLACEMENT_NEW(ws, ws, goto fail_placement_new, CddsWaitset, );
  if (!ws) {
    RMW_SET_ERROR_MSG("failed to construct wait set info struct");
    goto fail_ws;
  }
  // 初始化 inuse 和 nelems 字段
  ws->inuse = false;
  ws->nelems = 0;

  // 创建等待集句柄
  if ((ws->waitseth = dds_create_waitset(DDS_CYCLONEDDS_HANDLE)) < 0) {
    RMW_SET_ERROR_MSG("failed to create waitset");
    goto fail_waitset;
  }

  {
    std::lock_guard<std::mutex> lock(gcdds().lock);
    // 懒加载创建虚拟守护条件
    if (gcdds().waitsets.size() == 0) {
      if ((gcdds().gc_for_empty_waitset = dds_create_guardcondition(DDS_CYCLONEDDS_HANDLE)) < 0) {
        RMW_SET_ERROR_MSG("failed to create guardcondition for handling empty waitsets");
        goto fail_create_dummy;
      }
    }
    // 附加永不触发的守护条件。因为它永远不会被触发，所以它永远不会包含在 dds_waitset_wait 的结果中
    if (dds_waitset_attach(ws->waitseth, gcdds().gc_for_empty_waitset, INTPTR_MAX) < 0) {
      RMW_SET_ERROR_MSG("failed to attach dummy guard condition for blocking on empty waitset");
      goto fail_attach_dummy;
    }
    // 将新创建的等待集插入到全局等待集列表中
    gcdds().waitsets.insert(ws);
  }

  return wait_set;

fail_attach_dummy:
fail_create_dummy:
  // 删除等待集句柄
  dds_delete(ws->waitseth);
fail_waitset:
fail_ws:
  // 调用 CddsWaitset 析构函数
  RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(ws->~CddsWaitset(), ws);
fail_placement_new:
  // 释放等待集数据内存
  rmw_free(wait_set->data);
fail_alloc_wait_set_data:
  // 释放等待集内存
  rmw_wait_set_free(wait_set);
fail_alloc_wait_set:
  return nullptr;
}

/**
 * @brief 销毁等待集 (wait set)
 *
 * @param wait_set 要销毁的等待集指针
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回相应错误代码
 */
extern "C" rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t *wait_set) {
  // 检查输入参数是否为空
  RET_NULL(wait_set);
  // 检查实现ID是否正确
  RET_WRONG_IMPLID(wait_set);
  // 初始化结果为成功
  auto result = RMW_RET_OK;
  // 将wait_set->data转换为CddsWaitset类型
  auto ws = static_cast<CddsWaitset *>(wait_set->data);
  // 检查ws是否为空
  RET_NULL(ws);
  // 删除DDS等待集句柄
  dds_delete(ws->waitseth);
  {
    // 加锁保护全局数据结构
    std::lock_guard<std::mutex> lock(gcdds().lock);
    // 从全局数据结构中删除等待集
    gcdds().waitsets.erase(ws);
    // 如果没有等待集，则删除垃圾回收器
    if (gcdds().waitsets.size() == 0) {
      dds_delete(gcdds().gc_for_empty_waitset);
      gcdds().gc_for_empty_waitset = 0;
    }
  }
  // 尝试析构CddsWaitset对象，如果失败则设置结果为错误
  RMW_TRY_DESTRUCTOR(ws->~CddsWaitset(), ws, result = RMW_RET_ERROR);
  // 释放wait_set->data内存
  rmw_free(wait_set->data);
  // 释放wait_set内存
  rmw_wait_set_free(wait_set);
  // 返回操作结果
  return result;
}

/**
 * @brief 检查是否需要重新附加实体
 *
 * @tparam T 实体类型
 * @param cached 缓存的实体列表
 * @param count 实体数量
 * @param ary 实体数组指针
 * @return bool 如果需要重新附加，则返回true，否则返回false
 */
template <typename T>
static bool require_reattach(const std::vector<T *> &cached, size_t count, void **ary) {
  // 如果ary为空或count为0，检查缓存的实体列表是否为空
  if (ary == nullptr || count == 0) {
    return cached.size() != 0;
  } else if (count != cached.size()) {  // 如果实体数量与缓存的实体列表大小不同，则需要重新附加
    return true;
  } else {
    // 比较缓存的实体列表和实体数组，如果不相等则需要重新附加
    return memcmp(
               static_cast<const void *>(cached.data()), static_cast<void *>(ary),
               count * sizeof(void *)) != 0;
  }
}

/**
 * @brief 判断是否需要重新附加事件
 *
 * @param[in] cached 缓存的CddsEvent向量
 * @param[in] events 指向rmw_events_t结构体的指针
 * @return 如果需要重新附加事件，则返回true，否则返回false
 */
static bool require_reattach(const std::vector<CddsEvent> &cached, rmw_events_t *events) {
  // 如果events为空指针或event_count为0
  if (events == nullptr || events->event_count == 0) {
    // 返回缓存的大小是否不等于0
    return cached.size() != 0;
  } else if (events->event_count != cached.size()) {  // 如果events的event_count与缓存的大小不相等
    return true;                                      // 需要重新附加事件
  } else {
    // 遍历events中的所有事件
    for (size_t i = 0; i < events->event_count; ++i) {
      // 获取当前事件并进行类型转换
      rmw_event_t *current_event = static_cast<rmw_event_t *>(events->events[i]);
      // 获取缓存中对应的CddsEvent对象
      CddsEvent c = cached.at(i);
      // 如果实体句柄或事件类型与缓存中的不匹配
      if (c.enth != static_cast<CddsEntity *>(current_event->data)->enth ||
          c.event_type != current_event->event_type) {
        return true;  // 需要重新附加事件
      }
    }
    return false;  // 不需要重新附加事件
  }
}

/**
 * @brief 从CddsWaitset中分离所有实体
 *
 * @param[in] ws 指向CddsWaitset结构体的指针
 */
static void waitset_detach(CddsWaitset *ws) {
  // 遍历ws中的所有订阅者，并从waitseth中分离它们的rdcondh
  for (auto &&x : ws->subs) {
    dds_waitset_detach(ws->waitseth, x->rdcondh);
  }
  // 遍历ws中的所有gcs，并从waitseth中分离它们的gcondh
  for (auto &&x : ws->gcs) {
    dds_waitset_detach(ws->waitseth, x->gcondh);
  }
  // 遍历ws中的所有服务，并从waitseth中分离它们的service.sub->rdcondh
  for (auto &&x : ws->srvs) {
    dds_waitset_detach(ws->waitseth, x->service.sub->rdcondh);
  }
  // 遍历ws中的所有客户端，并从waitseth中分离它们的client.sub->rdcondh
  for (auto &&x : ws->cls) {
    dds_waitset_detach(ws->waitseth, x->client.sub->rdcondh);
  }
  // 重置各个实体向量的大小为0
  ws->subs.resize(0);
  ws->gcs.resize(0);
  ws->srvs.resize(0);
  ws->cls.resize(0);
  // 将nelems设置为0
  ws->nelems = 0;
}

/**
 * @brief 清理等待集缓存
 *
 * 当删除订阅者、保护条件、服务或客户端时调用（因为这些可能已经在等待集中缓存），
 * 并从所有等待集中删除所有缓存的实体（只是为了简化生活）。
 * 假设不允许在仍在使用实体时删除它...
 */
static void clean_waitset_caches() {
  // 获取全局CDDS数据结构的锁
  std::lock_guard<std::mutex> lock(gcdds().lock);
  // 遍历所有等待集
  for (auto &&ws : gcdds().waitsets) {
    // 获取当前等待集的锁
    std::lock_guard<std::mutex> wslock(ws->lock);
    // 如果等待集未被使用，则将其分离
    if (!ws->inuse) {
      waitset_detach(ws);
    }
  }
}

/**
 * @brief 收集事件实体
 *
 * @param[in] events 指向rmw_events_t结构的指针，包含要收集的事件信息
 * @param[out] entities 一个无序集合，用于存储收集到的事件实体
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
static rmw_ret_t gather_event_entities(
    const rmw_events_t *events, std::unordered_set<dds_entity_t> &entities) {
  // 检查输入参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(events, RMW_RET_INVALID_ARGUMENT);

  // 定义一个映射，用于存储DDS实体和状态掩码
  std::unordered_map<dds_entity_t, uint32_t> status_mask_map;

  // 遍历所有事件
  for (size_t i = 0; i < events->event_count; ++i) {
    // 获取当前事件
    rmw_event_t *current_event = static_cast<rmw_event_t *>(events->events[i]);
    // 获取DDS实体
    dds_entity_t dds_entity = static_cast<CddsEntity *>(current_event->data)->enth;
    // 检查DDS实体是否有效
    if (dds_entity <= 0) {
      RMW_SET_ERROR_MSG("Event entity handle is invalid");
      return RMW_RET_ERROR;
    }

    // 检查事件类型是否受支持
    if (is_event_supported(current_event->event_type)) {
      // 如果映射中不存在该实体，则将其添加到映射中并初始化状态掩码为0
      if (status_mask_map.find(dds_entity) == status_mask_map.end()) {
        status_mask_map[dds_entity] = 0;
      }

      // 获取RMW事件类型对应的状态种类
      uint32_t status_kind = get_status_kind_from_rmw(current_event->event_type);
      // 当Cyclone支持将不一致的主题作为事件报告时，应重新启用此功能
      if (status_kind != DDS_INCONSISTENT_TOPIC_STATUS) {
        status_mask_map[dds_entity] |= get_status_kind_from_rmw(current_event->event_type);
      }
    }
  }
  // 遍历映射中的所有键值对
  for (auto &pair : status_mask_map) {
    // 使用支持的类型设置状态条件的掩码
    dds_return_t ret = dds_set_status_mask(pair.first, pair.second);
    // 检查设置状态掩码是否成功
    if (ret != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("Failed setting the status mask");
      return RMW_RET_ERROR;
    }
    // 将实体添加到输出集合中
    entities.insert(pair.first);
  }

  // 返回成功
  return RMW_RET_OK;
}

/**
 * @brief 处理激活的事件
 *
 * 该函数遍历所有给定的事件，检查它们是否有效并更新事件列表。
 *
 * @param[in,out] events 指向rmw_events_t结构的指针，包含要处理的事件列表
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，返回RMW_RET_ERROR表示错误
 */
static rmw_ret_t handle_active_events(rmw_events_t *events) {
  // 如果事件列表不为空
  if (events) {
    // 遍历事件列表中的每个事件
    for (size_t i = 0; i < events->event_count; ++i) {
      // 获取当前事件，并将其转换为rmw_event_t类型
      rmw_event_t *current_event = static_cast<rmw_event_t *>(events->events[i]);
      // 获取DDS实体，并将其转换为CddsEntity类型
      dds_entity_t dds_entity = static_cast<CddsEntity *>(current_event->data)->enth;
      // 检查DDS实体是否有效
      if (dds_entity <= 0) {
        // 设置错误消息并返回错误代码
        RMW_SET_ERROR_MSG("Event entity handle is invalid");
        return RMW_RET_ERROR;
      }

      // 定义状态掩码变量
      uint32_t status_mask;
      // 获取DDS实体的状态更改
      dds_get_status_changes(dds_entity, &status_mask);
      // 检查事件类型是否受支持以及状态掩码是否与给定事件类型匹配
      if (!is_event_supported(current_event->event_type) ||
          !static_cast<bool>(status_mask & get_status_kind_from_rmw(current_event->event_type))) {
        // 如果不支持或不匹配，则将事件列表中的当前事件设置为nullptr
        events->events[i] = nullptr;
      }
    }
  }
  // 返回成功代码
  return RMW_RET_OK;
}

/**
 * @brief 等待并处理订阅、守护条件、服务、客户端和事件。
 *
 * @param[in] subs 订阅者列表，可以为 nullptr。
 * @param[in] gcs 守护条件列表，可以为 nullptr。
 * @param[in] srvs 服务列表，可以为 nullptr。
 * @param[in] cls 客户端列表，可以为 nullptr。
 * @param[in] evs 事件列表，可以为 nullptr。
 * @param[in,out] wait_set 等待集对象。
 * @param[in] wait_timeout 等待超时时间。
 * @return rmw_ret_t 返回操作结果。
 */
extern "C" rmw_ret_t rmw_wait(
    rmw_subscriptions_t *subs,
    rmw_guard_conditions_t *gcs,
    rmw_services_t *srvs,
    rmw_clients_t *cls,
    rmw_events_t *evs,
    rmw_wait_set_t *wait_set,
    const rmw_time_t *wait_timeout) {
  // 检查 wait_set 是否为空，如果为空则返回 RMW_RET_INVALID_ARGUMENT 错误
  RET_NULL_X(wait_set, return RMW_RET_INVALID_ARGUMENT);
  // 检查 wait_set 的实现 ID 是否正确
  RET_WRONG_IMPLID(wait_set);
  // 将 wait_set 的 data 转换为 CddsWaitset 类型
  CddsWaitset *ws = static_cast<CddsWaitset *>(wait_set->data);
  // 检查 ws 是否为空
  RET_NULL(ws);

  {
    // 对 ws 的 lock 进行加锁
    std::lock_guard<std::mutex> lock(ws->lock);
    // 如果 ws 正在使用中，则不支持在单个 waitset 上同时调用 rmw_wait
    if (ws->inuse) {
      RMW_SET_ERROR_MSG("concurrent calls to rmw_wait on a single waitset is not supported");
      return RMW_RET_ERROR;
    }
    // 设置 ws 为正在使用中
    ws->inuse = true;
  }

  // 检查是否需要重新附加订阅、守护条件、服务、客户端和事件
  if (require_reattach(
          ws->subs, subs ? subs->subscriber_count : 0, subs ? subs->subscribers : nullptr) ||
      require_reattach(
          ws->gcs, gcs ? gcs->guard_condition_count : 0, gcs ? gcs->guard_conditions : nullptr) ||
      require_reattach(ws->srvs, srvs ? srvs->service_count : 0, srvs ? srvs->services : nullptr) ||
      require_reattach(ws->cls, cls ? cls->client_count : 0, cls ? cls->clients : nullptr) ||
      require_reattach(ws->evs, evs)) {
    // 初始化元素数量为 0
    size_t nelems = 0;
    // 从 ws 中分离所有元素
    waitset_detach(ws);
/**
 * @brief 宏定义 ATTACH，用于将不同类型的实体附加到等待集中，并调整相关向量的大小。
 *
 * @param type 实体的类型，例如 CddsSubscription、CddsGuardCondition 等。
 * @param var 与实体类型对应的向量变量，例如 subs、gcs 等。
 * @param name 实体名称的前缀，例如 subscriber、guard_condition 等。
 * @param cond 实体的条件句柄，例如 rdcondh、gcondh 等。
 */
#define ATTACH(type, var, name, cond)                                                              \
  do {                                                                                             \
    ws->var.resize(0);                    /* 将向量 var 的大小重置为 0 */                 \
    if (var) {                            /* 如果 var 存在 */                                  \
      ws->var.reserve(var->name##_count); /* 预留 var 向量的空间，大小为实体数量 */ \
      for (size_t i = 0; i < var->name##_count; i++) {     /* 遍历实体 */                      \
        auto x = static_cast<type *>(var->name##s[i]);     /* 将实体转换为指定类型 */    \
        ws->var.push_back(x);                              /* 将实体添加到向量 var 中 */  \
        dds_waitset_attach(ws->waitseth, x->cond, nelems); /* 将实体附加到等待集 */       \
        nelems++;                                          /* 增加 nelems 计数 */              \
      }                                                                                            \
    }                                                                                              \
  } while (0)

    // 使用 ATTACH 宏定义分别处理不同类型的实体
    ATTACH(CddsSubscription, subs, subscriber, rdcondh);  // 处理 CddsSubscription 类型的实体
    ATTACH(CddsGuardCondition, gcs, guard_condition, gcondh);  // 处理 CddsGuardCondition 类型的实体
    ATTACH(CddsService, srvs, service, service.sub->rdcondh);  // 处理 CddsService 类型的实体
    ATTACH(CddsClient, cls, client, client.sub->rdcondh);      // 处理 CddsClient 类型的实体
#undef ATTACH                                                  // 取消宏定义 ATTACH

    /**
     * @param[in] evs 指向rmw_events_t结构体的指针，包含了事件的数量和事件数组。
     * @param[out] ws 指向CddsWaitset结构体的指针，用于存储处理后的事件信息。
     *
     * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败。
     */
    ws->evs.resize(0);                                  // 将ws中的事件向量大小重置为0

    if (evs) {                                          // 如果evs不为空
      std::unordered_set<dds_entity_t> event_entities;  // 创建一个无序集合，用于存储DDS实体
      rmw_ret_t ret_code = gather_event_entities(evs, event_entities);  // 收集事件实体
      if (ret_code != RMW_RET_OK) {                    // 如果收集过程出现错误
        return ret_code;                               // 返回错误代码
      }
      for (auto e : event_entities) {                  // 遍历事件实体集合
        dds_waitset_attach(ws->waitseth, e, nelems);   // 将事件实体附加到等待集合中
        nelems++;                                      // 增加元素计数
      }
      ws->evs.reserve(evs->event_count);               // 为ws中的事件向量预留空间
      for (size_t i = 0; i < evs->event_count; i++) {  // 遍历事件数组
        auto current_event = static_cast<rmw_event_t *>(evs->events[i]);  // 获取当前事件
        CddsEvent ev;  // 创建CddsEvent结构体实例
        ev.enth = static_cast<CddsEntity *>(current_event->data)->enth;  // 设置实体句柄
        ev.event_type = current_event->event_type;                       // 设置事件类型
        ws->evs.push_back(ev);  // 将事件添加到ws的事件向量中
      }
    }

    ws->nelems = nelems;  // 更新ws中的元素数量
  }

  /**
   * @param[in] wait_timeout 指向rmw_time_t结构体的指针，表示等待超时时间。
   * @param[out] ws 指向CddsWaitset结构体的指针，用于存储触发事件信息。
   */
  ws->trigs.resize(ws->nelems + 1);  // 调整触发事件向量的大小为元素数量加1

  // 计算超时时间，如果wait_timeout为空，则使用DDS_NEVER，否则将其转换为纳秒
  const dds_time_t timeout =
      (wait_timeout == NULL) ? DDS_NEVER : (dds_time_t)rmw_time_total_nsec(*wait_timeout);
  ws->trigs.resize(ws->nelems + 1);  // 再次调整触发事件向量的大小为元素数量加1

  // 等待触发事件，返回触发的事件数量
  const dds_return_t ntrig =
      dds_waitset_wait(ws->waitseth, ws->trigs.data(), ws->trigs.size(), timeout);
  ws->trigs.resize(ntrig);  // 调整触发事件向量的大小为实际触发的事件数量
  std::sort(ws->trigs.begin(), ws->trigs.end());  // 对触发事件向量进行排序
  ws->trigs.push_back((dds_attach_t)-1);  // 在触发事件向量末尾添加一个-1作为结束标志
}

/**
 * @brief 处理DDS触发事件的宏函数。
 *
 * @param type 触发事件的类型，如CddsSubscription、CddsGuardCondition等。
 * @param var 变量名，用于存储触发事件的实例。
 * @param name 事件名称。
 * @param cond 条件句柄。
 * @param on_triggered 当触发事件时执行的操作。
 */
#define DETACH(type, var, name, cond, on_triggered) \
  do {                                              \
    if (var) {                                      \
  // 遍历所有事件实例                                              \
      for (size_t i = 0; i < var->name##_count; i++) {                  \
        // 将事件实例转换为指定类型的指针                              \
        auto x = static_cast<type *>(var->name##s[i]);                  \
        // 检查当前触发索引是否与nelems相等                             \
        if (ws->trigs[trig_idx] == static_cast<dds_attach_t>(nelems)) { \
          // 执行触发事件操作                                          \
          on_triggered;                                                 \
          // 增加触发索引                                              \
          trig_idx++;                                                   \
        } else {                                                        \
          // 将事件实例设置为空指针                                    \
          var->name##s[i] = nullptr;                                    \
        }                                                               \
        // 增加元素计数器                                              \
        nelems++;                                                       \
      }                                                                 \
    }                                                                   \
  } while (0)

{
  dds_attach_t trig_idx = 0;  // 初始化触发索引
  bool dummy;                 // 定义一个虚拟布尔变量
  size_t nelems = 0;          // 初始化元素计数器

  // 处理CddsSubscription类型的事件
  DETACH(CddsSubscription, subs, subscriber, rdcondh, (void)x);
  // 处理CddsGuardCondition类型的事件
  DETACH(
      CddsGuardCondition, gcs, guard_condition, gcondh, dds_take_guardcondition(x->gcondh, &dummy));
  // 处理CddsService类型的事件
  DETACH(CddsService, srvs, service, service.sub->rdcondh, (void)x);
  // 处理CddsClient类型的事件
  DETACH(CddsClient, cls, client, client.sub->rdcondh, (void)x);

  // 取消宏定义
#undef DETACH

  // 处理活动事件
  handle_active_events(evs);
}

#if REPORT_BLOCKED_REQUESTS
// 遍历 ws->cls 中的所有元素
for (auto const &c : ws->cls) {
  // 检查每个元素是否有被阻塞的请求
  check_for_blocked_requests(*c);
}
#endif

{
  // 使用 std::lock_guard 对象对 ws->lock 进行加锁，保证线程安全
  std::lock_guard<std::mutex> lock(ws->lock);
  // 将 ws->inuse 设置为 false，表示当前 ws 不再使用中
  ws->inuse = false;
}

// 如果 ws->trigs 的大小为 1，则返回 RMW_RET_TIMEOUT，否则返回 RMW_RET_OK
return (ws->trigs.size() == 1) ? RMW_RET_TIMEOUT : RMW_RET_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    CLIENTS AND SERVERS                                            ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 获取匹配的端点函数类型定义
 *
 * @param h 实体句柄
 * @param xs 匹配的实例句柄数组
 * @param nxs 数组大小
 * @return dds_return_t 返回DDS操作结果
 */
using get_matched_endpoints_fn_t =
    dds_return_t (*)(dds_entity_t h, dds_instance_handle_t *xs, size_t nxs);

/**
 * @brief 内置主题端点类型定义
 *
 * 使用智能指针和自定义删除器管理内存
 */
using BuiltinTopicEndpoint = std::
    unique_ptr<dds_builtintopic_endpoint_t, std::function<void(dds_builtintopic_endpoint_t *)>>;

/**
 * @brief 获取匹配的端点
 *
 * @param h 实体句柄
 * @param fn 获取匹配端点的函数指针
 * @param res 存储匹配端点的向量
 * @return rmw_ret_t 返回RMW操作结果
 */
static rmw_ret_t get_matched_endpoints(
    dds_entity_t h, get_matched_endpoints_fn_t fn, std::vector<dds_instance_handle_t> &res) {
  dds_return_t ret;
  // 调用获取匹配端点的函数
  if ((ret = fn(h, res.data(), res.size())) < 0) {
    return RMW_RET_ERROR;
  }
  // 如果返回值大于等于向量大小，需要调整向量大小并重新尝试
  while (static_cast<size_t>(ret) >= res.size()) {
    // 128是一个完全任意的边际值，用于减少在并行创建/删除匹配时重试的风险
    res.resize(static_cast<size_t>(ret) + 128);
    if ((ret = fn(h, res.data(), res.size())) < 0) {
      return RMW_RET_ERROR;
    }
  }
  // 调整向量大小为实际返回值
  res.resize(static_cast<size_t>(ret));
  return RMW_RET_OK;
}

/**
 * @brief 释放内置主题端点资源
 *
 * @param e 内置主题端点指针
 */
static void free_builtintopic_endpoint(dds_builtintopic_endpoint_t *e) {
  dds_delete_qos(e->qos);
  dds_free(e->topic_name);
  dds_free(e->type_name);
  dds_free(e);
}

/**
 * @brief 获取匹配的订阅数据
 *
 * @param writer 写者实体句柄
 * @param readerih 读者实例句柄
 * @return BuiltinTopicEndpoint 返回内置主题端点智能指针
 */
static BuiltinTopicEndpoint get_matched_subscription_data(
    dds_entity_t writer, dds_instance_handle_t readerih) {
  // 使用自定义删除器创建智能指针
  BuiltinTopicEndpoint ep(
      dds_get_matched_subscription_data(writer, readerih), free_builtintopic_endpoint);
  return ep;
}

/**
 * @brief 获取匹配的发布端数据
 *
 * @param[in] reader 读取器实体
 * @param[in] writerih 写入器实例句柄
 * @return BuiltinTopicEndpoint 匹配的发布端数据
 */
static BuiltinTopicEndpoint get_matched_publication_data(
    dds_entity_t reader, dds_instance_handle_t writerih) {
  // 使用dds_get_matched_publication_data函数获取匹配的发布端数据，并创建BuiltinTopicEndpoint对象
  BuiltinTopicEndpoint ep(
      dds_get_matched_publication_data(reader, writerih), free_builtintopic_endpoint);

  // 返回BuiltinTopicEndpoint对象
  return ep;
}

/**
 * @brief 将客户端服务ID转换为字符串
 *
 * @param[in] id 客户端服务ID
 * @return std::string 转换后的字符串
 */
static const std::string csid_to_string(const client_service_id_t &id) {
  // 创建一个输出字符串流对象
  std::ostringstream os;
  // 设置输出格式为十六进制
  os << std::hex;
  // 输出第一个字节，宽度为2，不足时用0填充
  os << std::setw(2) << static_cast<int>(id.data[0]);
  // 遍历剩余字节并输出
  for (size_t i = 1; i < sizeof(id.data); i++) {
    os << "." << static_cast<int>(id.data[i]);
  }
  // 返回转换后的字符串
  return os.str();
}

/**
 * @brief 从服务端接收请求并处理
 *
 * @param[in] cs CddsCS对象，包含了DDS实体和相关的信息
 * @param[out] request_header 请求头信息，包括请求ID、源时间戳等
 * @param[out] ros_data 存储接收到的ROS数据
 * @param[out] taken 是否成功接收到有效数据
 * @param[out] source_timestamp 源时间戳（可选）
 * @param[in] srcfilter 过滤器，用于过滤特定的实例句柄（可选）
 * @return rmw_ret_t 返回操作结果状态码
 */
static rmw_ret_t rmw_take_response_request(
    CddsCS *cs,
    rmw_service_info_t *request_header,
    void *ros_data,
    bool *taken,
    dds_time_t *source_timestamp,
    dds_instance_handle_t srcfilter) {
  // 检查taken参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  // 检查ros_data参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_data, RMW_RET_INVALID_ARGUMENT);
  // 检查request_header参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);

  // 定义cdds_request_wrapper_t类型的wrap变量，并将ros_data赋值给wrap.data
  cdds_request_wrapper_t wrap;
  dds_sample_info_t info;
  wrap.data = ros_data;

  // 将wrap转换为void指针类型
  void *wrap_ptr = static_cast<void *>(&wrap);

  // 循环读取数据，直到没有有效数据
  while (dds_take(cs->sub->enth, &wrap_ptr, &info, 1, 1) == 1) {
    // 如果读取到的数据有效
    if (info.valid_data) {
      // 静态断言，确保request_header的大小符合预期
      static_assert(
          sizeof(request_header->request_id.writer_guid) ==
              sizeof(wrap.header.guid) + sizeof(info.publication_handle),
          "request header size assumptions not met");

      // 将wrap.header.guid复制到request_header->request_id.writer_guid
      memcpy(
          static_cast<void *>(request_header->request_id.writer_guid),
          static_cast<const void *>(&wrap.header.guid), sizeof(wrap.header.guid));

      // 将info.publication_handle复制到request_header->request_id.writer_guid的后半部分
      memcpy(
          static_cast<void *>(request_header->request_id.writer_guid + sizeof(wrap.header.guid)),
          static_cast<const void *>(&info.publication_handle), sizeof(info.publication_handle));

      // 设置请求头的序列号和源时间戳
      request_header->request_id.sequence_number = wrap.header.seq;
      request_header->source_timestamp = info.source_timestamp;

      // TODO(iluetkeb) replace with real received timestamp when available in cyclone
      request_header->received_timestamp = 0;

      // 如果提供了source_timestamp参数，则设置其值
      if (source_timestamp) {
        *source_timestamp = info.source_timestamp;
      }

      // 如果没有提供srcfilter或者srcfilter与wrap.header.guid匹配，则设置taken为true并返回RMW_RET_OK
      if (srcfilter == 0 || srcfilter == wrap.header.guid) {
        *taken = true;
        return RMW_RET_OK;
      }
    }
  }

  // 如果没有接收到有效数据，设置taken为false并返回RMW_RET_OK
  *taken = false;
  return RMW_RET_OK;
}

/**
 * @brief 从服务端接收响应
 *
 * @param[in] client 指向客户端实例的指针
 * @param[out] request_header 包含请求头信息的指针
 * @param[out] ros_response 存储接收到的响应数据的指针
 * @param[out] taken 布尔值，表示是否成功接收到响应
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_response(
    const rmw_client_t *client,
    rmw_service_info_t *request_header,
    void *ros_response,
    bool *taken) {
  // 检查client参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);

  // 检查client的实现标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      client, client->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 将client的data成员转换为CddsClient类型
  auto info = static_cast<CddsClient *>(client->data);

  // 定义源时间戳变量
  dds_time_t source_timestamp;

  // 调用rmw_take_response_request函数接收响应
  rmw_ret_t ret = rmw_take_response_request(
      &info->client, request_header, ros_response, taken, &source_timestamp,
      info->client.pub->pubiid);

#if REPORT_BLOCKED_REQUESTS
  // 如果成功接收到响应，则进行以下操作
  if (ret == RMW_RET_OK && *taken) {
    // 使用互斥锁保护共享资源
    std::lock_guard<std::mutex> lock(info->lock);

    // 获取请求头中的序列号
    uint64_t seq = request_header->sequence_number;

    // 计算当前时间、响应时间差和请求时间差
    dds_time_t tnow = dds_time();
    dds_time_t dtresp = tnow - source_timestamp;
    dds_time_t dtreq = tnow - info->reqtime[seq];

    // 如果请求时间差或响应时间差超过阈值，则输出警告信息
    if (dtreq > DDS_MSECS(REPORT_LATE_MESSAGES) || dtresp > DDS_MSECS(REPORT_LATE_MESSAGES)) {
      fprintf(
          stderr, "** response time %.fms; response in history for %.fms\n",
          static_cast<double>(dtreq) / 1e6, static_cast<double>(dtresp) / 1e6);
    }

    // 从reqtime映射中删除对应的序列号
    info->reqtime.erase(seq);
  }
#endif

  // 返回操作结果
  return ret;
}

#if REPORT_BLOCKED_REQUESTS
/**
 * @brief 检查阻塞的请求并报告
 *
 * @param[in] client CddsClient对象，用于检查阻塞的请求
 */
static void check_for_blocked_requests(CddsClient &client) {
  dds_time_t tnow = dds_time();                   // 获取当前时间
  std::lock_guard<std::mutex> lock(client.lock);  // 对client对象加锁
  if (tnow > client.lastcheck + DDS_SECS(1)) {    // 如果距离上次检查已经超过1秒
    client.lastcheck = tnow;                      // 更新上次检查时间
    for (auto const &r : client.reqtime) {        // 遍历请求时间列表
      dds_time_t dt = tnow - r.second;            // 计算请求等待时间
      if (dt > DDS_SECS(1)) {                     // 如果等待时间超过1秒
        fprintf(
            stderr, "** already waiting for %.fms\n",
            static_cast<double>(dt) / 1e6);  // 输出等待时间信息
      }
    }
  }
}
#endif

/**
 * @brief 接收服务请求
 *
 * @param[in] service 服务对象指针
 * @param[out] request_header 请求头信息
 * @param[out] ros_request ROS请求数据
 * @param[out] taken 是否成功接收到请求
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_take_request(
    const rmw_service_t *service,
    rmw_service_info_t *request_header,
    void *ros_request,
    bool *taken) {
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);  // 检查service参数是否为空
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      service, service->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);       // 检查实现标识符是否匹配
  auto info = static_cast<CddsService *>(service->data);  // 获取服务数据
  return rmw_take_response_request(
      &info->service, request_header, ros_request, taken, nullptr,
      false);  // 调用rmw_take_response_request处理请求
}

/**
 * @brief 发送响应请求
 *
 * @param[in] cs CddsCS对象指针，包含发布者和订阅者信息
 * @param[in] header 请求头信息
 * @param[in] ros_data ROS响应数据
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t rmw_send_response_request(
    CddsCS *cs, const cdds_request_header_t &header, const void *ros_data) {
  const cdds_request_wrapper_t wrap = {header, const_cast<void *>(ros_data)};  // 创建请求包装对象
  if (dds_write(cs->pub->enth, static_cast<const void *>(&wrap)) >=
      0) {                                     // 将请求包装对象写入发布者
    return RMW_RET_OK;                         // 返回成功
  } else {
    RMW_SET_ERROR_MSG("cannot publish data");  // 设置错误消息
    return RMW_RET_ERROR;                      // 返回错误
  }
}

/**
 * @brief 客户端存在状态枚举
 */
enum class client_present_t {
  FAILURE,  ///< 检查时发生错误
  MAYBE,    ///< 未匹配到读者，但写者仍存在
  YES,      ///< 匹配到读者
  GONE      ///< 既没有读者也没有写者
};

/**
 * @brief 检查客户端服务端点
 *
 * @param[in] ep 指向dds_builtintopic_endpoint_t的指针
 * @param[in] key 用户数据键
 * @param[in] needle 需要匹配的字符串
 * @return 如果客户端ID等于needle，则返回true，否则返回false
 */
static bool check_client_service_endpoint(
    const dds_builtintopic_endpoint_t *ep, const std::string key, const std::string needle) {
  if (ep != nullptr) {
    std::string clientid;
    get_user_data_key(ep->qos, key, clientid);  // 获取用户数据键值
    return clientid == needle;                  // 比较客户端ID和needle是否相等
  }
  return false;
}

/**
 * @brief 检查响应读者的存在状态
 *
 * @param[in] service CddsCS类型的服务对象
 * @param[in] reqwrih 请求写者实例句柄
 * @return 返回客户端存在状态
 */
static client_present_t check_for_response_reader(
    const CddsCS &service, const dds_instance_handle_t reqwrih) {
  auto reqwr = get_matched_publication_data(service.sub->enth, reqwrih);  // 获取匹配的发布数据
  std::string clientid;
  if (reqwr == nullptr) {
    return client_present_t::GONE;  // 如果请求写者为空，则返回GONE状态
  } else if (!get_user_data_key(reqwr->qos, "clientid", clientid)) {
    // 向后兼容：没有客户端ID的客户端，假设一切正常
    return client_present_t::YES;
  } else {
    // 查找此客户端的读者：如果我们已经匹配到它，则一切正常；
    // 如果没有，继续等待
    std::vector<dds_instance_handle_t> rds;
    if (get_matched_endpoints(service.pub->enth, dds_get_matched_subscriptions, rds) < 0) {
      RMW_SET_ERROR_MSG("rmw_send_response: failed to get reader/writer matches");
      return client_present_t::FAILURE;  // 获取匹配的端点失败，返回FAILURE状态
    }
    // 如果我们已经匹配到此客户端的读者，则一切正常
    for (const auto &rdih : rds) {
      auto rd = get_matched_subscription_data(service.pub->enth, rdih);
      if (check_client_service_endpoint(rd.get(), "clientid", clientid)) {
        return client_present_t::YES;  // 匹配到客户端的读者，返回YES状态
      }
    }
    return client_present_t::MAYBE;  // 没有匹配到客户端的读者，但写者仍存在，返回MAYBE状态
  }
}

/**
 * @brief 发送服务响应
 *
 * @param[in] service 服务对象指针，不能为空
 * @param[in] request_header 请求头指针，包含请求的元数据，不能为空
 * @param[in] ros_response ROS响应消息指针，不能为空
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_send_response(
    const rmw_service_t *service, rmw_request_id_t *request_header, void *ros_response) {
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  // 检查实现标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      service, service->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);

  // 获取服务信息
  CddsService *info = static_cast<CddsService *>(service->data);
  cdds_request_header_t header;
  dds_instance_handle_t reqwrih;

  // 静态断言，检查请求头大小假设是否满足
  static_assert(
      sizeof(request_header->writer_guid) == sizeof(header.guid) + sizeof(reqwrih),
      "request header size assumptions not met");

  // 复制GUID和实例句柄
  memcpy(
      static_cast<void *>(&header.guid), static_cast<const void *>(request_header->writer_guid),
      sizeof(header.guid));
  memcpy(
      static_cast<void *>(&reqwrih),
      static_cast<const void *>(request_header->writer_guid + sizeof(header.guid)),
      sizeof(reqwrih));

  // 设置序列号
  header.seq = request_header->sequence_number;

  // 阻塞，直到响应读取器与响应写入器匹配（这是一个解决方法：rmw_service_server_is_available应该在此之前一直返回false）
  // TODO(eboasson): rmw_service_server_is_available 应该阻止请求而不是这里 (#191)
  client_present_t st;
  std::chrono::system_clock::time_point tnow = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point tend = tnow + 100ms;
  while ((st = check_for_response_reader(info->service, reqwrih)) == client_present_t::MAYBE &&
         tnow < tend) {
    dds_sleepfor(DDS_MSECS(10));
    tnow = std::chrono::system_clock::now();
  }

  // 根据客户端状态处理结果
  switch (st) {
    case client_present_t::FAILURE:
      break;
    case client_present_t::MAYBE:
      return RMW_RET_TIMEOUT;
    case client_present_t::YES:
      return rmw_send_response_request(&info->service, header, ros_response);
    case client_present_t::GONE:
      return RMW_RET_OK;
  }

  // 返回错误
  return RMW_RET_ERROR;
}

/**
 * @brief 发送请求到服务端 (Send a request to the service server)
 *
 * @param[in] client 指向客户端实例的指针 (Pointer to the client instance)
 * @param[in] ros_request 指向ROS请求消息的指针 (Pointer to the ROS request message)
 * @param[out] sequence_id 请求序列号的指针，用于跟踪请求 (Pointer to the request sequence number,
 * used for tracking requests)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
extern "C" rmw_ret_t rmw_send_request(
    const rmw_client_t *client, const void *ros_request, int64_t *sequence_id) {
  // 定义一个静态原子变量，用于生成请求ID (Define a static atomic variable for generating request
  // IDs)
  static std::atomic_uint next_request_id;

  // 检查client参数是否为空 (Check if the client parameter is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);

  // 检查client的实现标识符是否与期望的一致 (Check if the client's implementation identifier matches
  // the expected one)
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      client, client->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查ros_request参数是否为空 (Check if the ros_request parameter is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);

  // 检查sequence_id参数是否为空 (Check if the sequence_id parameter is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);

  // 获取客户端信息 (Get the client information)
  auto info = static_cast<CddsClient *>(client->data);

  // 定义请求头 (Define the request header)
  cdds_request_header_t header;
  header.guid = info->client.pub->pubiid;
  header.seq = *sequence_id = ++next_request_id;

#if REPORT_BLOCKED_REQUESTS
  {
    // 锁定互斥量 (Lock the mutex)
    std::lock_guard<std::mutex> lock(info->lock);
    info->reqtime[header.seq] = dds_time();
  }
#endif

  // 发送请求 (Send the request)
  return rmw_send_response_request(&info->client, header, ros_request);
}

/**
 * @brief 获取服务类型支持 (Get service type support)
 *
 * @param[in] type_supports 指向rosidl_service_type_support_t实例的指针 (Pointer to the
 * rosidl_service_type_support_t instance)
 * @return const rosidl_service_type_support_t* 返回类型支持的指针，如果没有找到则返回nullptr
 * (Return a pointer to the type support, or nullptr if not found)
 */
static const rosidl_service_type_support_t *get_service_typesupport(
    const rosidl_service_type_support_t *type_supports) {
  const rosidl_service_type_support_t *ts;

  // 尝试获取C语言类型支持 (Try to get C language type support)
  if ((ts = get_service_typesupport_handle(
           type_supports, rosidl_typesupport_introspection_c__identifier)) != nullptr) {
    return ts;
  } else {
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();

    // 尝试获取C++语言类型支持 (Try to get C++ language type support)
    if ((ts = get_service_typesupport_handle(
             type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier)) !=
        nullptr) {
      return ts;
    } else {
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();

      // 设置错误消息 (Set the error message)
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "Service type support not from this implementation. Got:\n"
          "    %s\n"
          "    %s\n"
          "while fetching it",
          prev_error_string.str, error_string.str);
      return nullptr;
    }
  }
}

/**
 * @brief 获取唯一的客户端/服务端ID (Get a unique client/service ID)
 *
 * @param[in] node ROS2节点指针 (Pointer to the ROS2 node)
 * @param[out] id 生成的唯一客户端/服务端ID (Generated unique client/service ID)
 */
static void get_unique_csid(const rmw_node_t *node, client_service_id_t &id) {
  // 获取节点上下文实现 (Get the node context implementation)
  auto impl = node->context->impl;

  // 静态断言，确保dds_guid_t大小适用于id.data (Static assertion to ensure dds_guid_t size is
  // suitable for id.data)
  static_assert(
      sizeof(dds_guid_t) <= sizeof(id.data), "client/service id assumed it can hold a DDSI GUID");

  // 静态断言，确保dds_guid_t大小适用于rmw_gid_t.data (Static assertion to ensure dds_guid_t size is
  // suitable for rmw_gid_t.data)
  static_assert(
      sizeof(dds_guid_t) <= sizeof((reinterpret_cast<rmw_gid_t *>(0))->data),
      "client/service id assumes rmw_gid_t can hold a DDSI GUID");

  uint32_t x;

  {
    // 使用互斥锁保护初始化过程 (Protect the initialization process with a mutex lock)
    std::lock_guard<std::mutex> guard(impl->initialization_mutex);
    x = ++impl->client_service_id;
  }

  // 构造ID，首先取实体前缀（即GID的前12个字节，GID本身只是用0填充的GUID）；
  // 然后用大端计数器值覆盖实体ID (Construct the ID by taking the entity prefix (which is just the
  // first 12 bytes of the GID, which itself is just the GUID padded with 0's); then overwrite the
  // entity ID with the big-endian counter value)
  memcpy(id.data, impl->ppant_gid.data, 12);
  for (size_t i = 0, s = 24; i < 4; i++, s -= 8) {
    id.data[12 + i] = static_cast<uint8_t>(x >> s);
  }
}

/**
 * @brief 初始化客户端或服务端的通信对象
 *
 * @param[out] cs 用于存储创建的CddsCS对象
 * @param[in] cb_data 用户回调数据
 * @param[in] node ROS2节点
 * @param[in] type_supports 服务类型支持
 * @param[in] service_name 服务名称
 * @param[in] qos_policies QoS策略
 * @param[in] is_service 是否为服务端（true为服务端，false为客户端）
 * @return rmw_ret_t 返回RMW_RET_OK表示成功，其他值表示失败
 */
static rmw_ret_t rmw_init_cs(
    CddsCS *cs,
    user_callback_data_t *cb_data,
    const rmw_node_t *node,
    const rosidl_service_type_support_t *type_supports,
    const char *service_name,
    const rmw_qos_profile_t *qos_policies,
    bool is_service) {
  // 检查node参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查node的实现标识符是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查type_supports参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, RMW_RET_INVALID_ARGUMENT);
  // 检查service_name参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  // 检查service_name是否为空字符串
  if (0 == strlen(service_name)) {
    RMW_SET_ERROR_MSG("service_name argument is an empty string");
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 检查qos_policies参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, RMW_RET_INVALID_ARGUMENT);
  // 验证服务名称是否有效
  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char *reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("service_name argument is invalid: %s", reason);
      return RMW_RET_INVALID_ARGUMENT;
    }
  }

  // 获取服务类型支持
  const rosidl_service_type_support_t *type_support = get_service_typesupport(type_supports);
  RET_NULL(type_support);

  auto pub = std::make_unique<CddsPublisher>();
  auto sub = std::make_unique<CddsSubscription>();
  std::string subtopic_name, pubtopic_name;
  void *pub_type_support, *sub_type_support;
  dds_qos_t *pub_qos, *sub_qos;
  const rosidl_type_hash_t *pub_type_hash;
  const rosidl_type_hash_t *sub_type_hash;
  std::string user_data;

  std::unique_ptr<rmw_cyclonedds_cpp::StructValueType> pub_msg_ts, sub_msg_ts;

  // 创建DDS监听器
  dds_listener_t *listener = dds_create_listener(cb_data);
  dds_lset_data_available_arg(listener, dds_listener_callback, cb_data, false);

  // 根据is_service判断是服务端还是客户端，并设置相应的类型支持和主题名称
  if (is_service) {
    std::tie(sub_msg_ts, pub_msg_ts) =
        rmw_cyclonedds_cpp::make_request_response_value_types(type_supports);

    sub_type_support =
        create_request_type_support(type_support->data, type_support->typesupport_identifier);
    sub_type_hash = type_supports->request_typesupport->type_hash;
    pub_type_support =
        create_response_type_support(type_support->data, type_support->typesupport_identifier);
    pub_type_hash = type_supports->response_typesupport->type_hash;
    subtopic_name =
        make_fqtopic(ROS_SERVICE_REQUESTER_PREFIX, service_name, "Request", qos_policies);
    pubtopic_name = make_fqtopic(ROS_SERVICE_RESPONSE_PREFIX, service_name, "Reply", qos_policies);
  } else {
    std::tie(pub_msg_ts, sub_msg_ts) =
        rmw_cyclonedds_cpp::make_request_response_value_types(type_supports);

    pub_type_support =
        create_request_type_support(type_support->data, type_support->typesupport_identifier);
    pub_type_hash = type_supports->request_typesupport->type_hash;
    sub_type_support =
        create_response_type_support(type_support->data, type_support->typesupport_identifier);
    sub_type_hash = type_supports->response_typesupport->type_hash;
    pubtopic_name =
        make_fqtopic(ROS_SERVICE_REQUESTER_PREFIX, service_name, "Request", qos_policies);
    subtopic_name = make_fqtopic(ROS_SERVICE_RESPONSE_PREFIX, service_name, "Reply", qos_policies);
  }

  // 打印调试信息
  RCUTILS_LOG_DEBUG_NAMED(
      "rmw_cyclonedds_cpp", "************ %s Details *********", is_service ? "Service" : "Client");
  RCUTILS_LOG_DEBUG_NAMED("rmw_cyclonedds_cpp", "Sub Topic %s", subtopic_name.c_str());
  RCUTILS_LOG_DEBUG_NAMED("rmw_cyclonedds_cpp", "Pub Topic %s", pubtopic_name.c_str());
  RCUTILS_LOG_DEBUG_NAMED("rmw_cyclonedds_cpp", "***********");

  dds_entity_t pubtopic, subtopic;
  struct sertype_rmw *pub_st, *sub_st;

  // 创建发布者和订阅者的类型支持
  pub_st = create_sertype(
      type_support->typesupport_identifier, pub_type_support, true, std::move(pub_msg_ts));
  struct ddsi_sertype *pub_stact;
  pubtopic = create_topic(node->context->impl->ppant, pubtopic_name.c_str(), pub_st, &pub_stact);
  if (pubtopic < 0) {
    set_error_message_from_create_topic(pubtopic, pubtopic_name);
    goto fail_pubtopic;
  }

  sub_st = create_sertype(
      type_support->typesupport_identifier, sub_type_support, true, std::move(sub_msg_ts));
  subtopic = create_topic(node->context->impl->ppant, subtopic_name.c_str(), sub_st);
  if (subtopic < 0) {
    set_error_message_from_create_topic(subtopic, subtopic_name);
    goto fail_subtopic;
  }

  // 为读写器创建唯一标识符
  get_unique_csid(node, cs->id);
  user_data = std::string(is_service ? "serviceid=" : "clientid=") + csid_to_string(cs->id) +
              std::string(";");

  // 创建QoS策略
  if ((pub_qos = create_readwrite_qos(qos_policies, *pub_type_hash, false, user_data)) == nullptr) {
    goto fail_pub_qos;
  }
  if ((sub_qos = create_readwrite_qos(qos_policies, *sub_type_hash, false, user_data)) == nullptr) {
    goto fail_sub_qos;
  }

  // 创建DDS写入器和读取器
  if ((pub->enth = dds_create_writer(node->context->impl->dds_pub, pubtopic, pub_qos, nullptr)) <
      0) {
    RMW_SET_ERROR_MSG("failed to create writer");
    goto fail_writer;
  }
  get_entity_gid(pub->enth, pub->gid);
  pub->sertype = pub_stact;
  if ((sub->enth = dds_create_reader(node->context->impl->dds_sub, subtopic, sub_qos, listener)) <
      0) {
    RMW_SET_ERROR_MSG("failed to create reader");
    goto fail_reader;
  }
  get_entity_gid(sub->enth, sub->gid);
  if ((sub->rdcondh = dds_create_readcondition(sub->enth, DDS_ANY_STATE)) < 0) {
    RMW_SET_ERROR_MSG("failed to create readcondition");
    goto fail_readcond;
  }
  if (dds_get_instance_handle(pub->enth, &pub->pubiid) < 0) {
    RMW_SET_ERROR_MSG("failed to get instance handle for writer");
    goto fail_instance_handle;
  }
  // 删除监听器、QoS策略和主题
  dds_delete_listener(listener);
  dds_delete_qos(pub_qos);
  dds_delete_qos(sub_qos);
  dds_delete(subtopic);
  dds_delete(pubtopic);

  // 将创建的发布者和订阅者对象存储到CddsCS中
  cs->pub = std::move(pub);
  cs->sub = std::move(sub);
  return RMW_RET_OK;

// 错误处理部分
fail_instance_handle:
  dds_delete(sub->rdcondh);
fail_readcond:
  dds_delete(sub->enth);
fail_reader:
  dds_delete(pub->enth);
fail_writer:
  dds_delete_qos(sub_qos);
fail_sub_qos:
  dds_delete_qos(pub_qos);
fail_pub_qos:
  dds_delete(subtopic);
fail_subtopic:
  dds_delete(pubtopic);
fail_pubtopic:
  return RMW_RET_ERROR;
}

/**
 * @brief 释放CddsCS结构体中的资源
 *
 * @param cs 指向CddsCS结构体的指针
 */
static void rmw_fini_cs(CddsCS *cs) {
  dds_delete(cs->sub->rdcondh);  // 删除读取条件句柄
  dds_delete(cs->sub->enth);     // 删除订阅者实体句柄
  dds_delete(cs->pub->enth);     // 删除发布者实体句柄
}

/**
 * @brief 销毁客户端实例
 *
 * @param node 指向rmw_node_t结构体的指针
 * @param client 指向rmw_client_t结构体的指针
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应错误代码
 */
static rmw_ret_t destroy_client(const rmw_node_t *node, rmw_client_t *client) {
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);  // 检查节点参数是否为空
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);  // 检查节点实现标识符是否匹配
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);  // 检查客户端参数是否为空
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      client, client->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);  // 检查客户端实现标识符是否匹配
  auto info = static_cast<CddsClient *>(client->data);  // 获取客户端数据
  clean_waitset_caches();                               // 清理等待集缓存

  {
    // 更新图信息
    auto common = &node->context->impl->common;
    std::lock_guard<std::mutex> guard(common->node_update_mutex);  // 加锁保护
    static_cast<void>(common->graph_cache.dissociate_writer(
        info->client.pub->gid, common->gid, node->name,
        node->namespace_));                                                 // 移除发布者关联
    rmw_dds_common::msg::ParticipantEntitiesInfo msg = common->graph_cache.dissociate_reader(
        info->client.sub->gid, common->gid, node->name, node->namespace_);  // 移除订阅者关联
    if (RMW_RET_OK != rmw_publish(common->pub, static_cast<void *>(&msg), nullptr)) {
      RMW_SET_ERROR_MSG("failed to publish ParticipantEntitiesInfo when destroying service");
    }
  }

  rmw_fini_cs(&info->client);                          // 释放客户端资源
  delete info;                                         // 删除客户端信息
  rmw_free(const_cast<char *>(client->service_name));  // 释放客户端服务名内存
  rmw_client_free(client);                             // 释放客户端实例
  return RMW_RET_OK;                                   // 返回操作成功
}

/**
 * @brief 创建一个ROS2服务客户端 (Create a ROS2 service client)
 *
 * @param[in] node 指向要创建客户端的节点的指针 (Pointer to the node for which the client is to be
 * created)
 * @param[in] type_supports 服务类型支持结构体的指针 (Pointer to the service type support structure)
 * @param[in] service_name 要创建的服务的名称 (Name of the service to be created)
 * @param[in] qos_policies 服务质量配置 (Quality of Service configuration for the service)
 * @return rmw_client_t* 成功时返回指向新创建的客户端的指针，失败时返回nullptr (Pointer to the newly
 * created client on success, nullptr on failure)
 */
extern "C" rmw_client_t *rmw_create_client(
    const rmw_node_t *node,
    const rosidl_service_type_support_t *type_supports,
    const char *service_name,
    const rmw_qos_profile_t *qos_policies) {
  // 检查qos_policies参数是否为空 (Check if qos_policies argument is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);

  // 创建CddsClient实例 (Create a CddsClient instance)
  CddsClient *info = new CddsClient();

#if REPORT_BLOCKED_REQUESTS
  // 初始化最后检查时间 (Initialize the last check time)
  info->lastcheck = 0;
#endif

  // 更新QoS策略以获得最佳可用性 (Update QoS policies for best available services)
  rmw_qos_profile_t adapted_qos_policies =
      rmw_dds_common::qos_profile_update_best_available_for_services(*qos_policies);

  // 初始化客户端 (Initialize the client)
  if (rmw_init_cs(
          &info->client, &info->user_callback_data, node, type_supports, service_name,
          &adapted_qos_policies, false) != RMW_RET_OK) {
    delete (info);
    return nullptr;
  }

  // 分配rmw_client实例 (Allocate an rmw_client instance)
  rmw_client_t *rmw_client = rmw_client_allocate();
  RET_NULL_X(rmw_client, goto fail_client);

  // 设置实现标识符和数据 (Set implementation identifier and data)
  rmw_client->implementation_identifier = eclipse_cyclonedds_identifier;
  rmw_client->data = info;

  // 分配并设置服务名称 (Allocate and set the service name)
  rmw_client->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  RET_NULL_X(rmw_client->service_name, goto fail_service_name);
  memcpy(const_cast<char *>(rmw_client->service_name), service_name, strlen(service_name) + 1);

  {
    // 更新图信息 (Update graph information)
    auto common = &node->context->impl->common;
    std::lock_guard<std::mutex> guard(common->node_update_mutex);
    static_cast<void>(common->graph_cache.associate_writer(
        info->client.pub->gid, common->gid, node->name, node->namespace_));
    rmw_dds_common::msg::ParticipantEntitiesInfo msg = common->graph_cache.associate_reader(
        info->client.sub->gid, common->gid, node->name, node->namespace_);

    // 发布更新后的图信息 (Publish updated graph information)
    if (RMW_RET_OK != rmw_publish(common->pub, static_cast<void *>(&msg), nullptr)) {
      static_cast<void>(destroy_client(node, rmw_client));
      return nullptr;
    }
  }

  // 返回创建的客户端 (Return the created client)
  return rmw_client;

// 错误处理 (Error handling)
fail_service_name:
  rmw_client_free(rmw_client);
fail_client:
  rmw_fini_cs(&info->client);
  delete info;
  return nullptr;
}

/**
 * @brief 销毁一个客户端实例 (Destroy a client instance)
 *
 * @param[in] node 指向要销毁的客户端所属的节点的指针 (Pointer to the node that the client belongs
 * to)
 * @param[in] client 要销毁的客户端实例的指针 (Pointer to the client instance to be destroyed)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
extern "C" rmw_ret_t rmw_destroy_client(rmw_node_t *node, rmw_client_t *client) {
  // 调用 destroy_client 函数并返回结果 (Call the destroy_client function and return the result)
  return destroy_client(node, client);
}

/**
 * @brief 销毁一个服务实例 (Destroy a service instance)
 *
 * @param[in] node 指向要销毁的服务所属的节点的指针 (Pointer to the node that the service belongs
 * to)
 * @param[in] service 要销毁的服务实例的指针 (Pointer to the service instance to be destroyed)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
static rmw_ret_t destroy_service(const rmw_node_t *node, rmw_service_t *service) {
  // 检查 node 参数是否为空，如果为空则返回无效参数错误 (Check if the node argument is null, return
  // an invalid argument error if it is)
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  // 检查 node 的实现标识符与期望的实现标识符是否匹配，如果不匹配则返回错误 (Check if the node's
  // implementation identifier matches the expected one, return an error if it doesn't)
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查 service 参数是否为空，如果为空则返回无效参数错误 (Check if the service argument is null,
  // return an invalid argument error if it is)
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);

  // 检查 service 的实现标识符与期望的实现标识符是否匹配，如果不匹配则返回错误 (Check if the
  // service's implementation identifier matches the expected one, return an error if it doesn't)
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      service, service->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 将 service 的 data 成员转换为 CddsService 类型的指针 (Cast the service's data member to a
  // pointer of type CddsService)
  auto info = static_cast<CddsService *>(service->data);

  // 清理等待集缓存 (Clean up waitset caches)
  clean_waitset_caches();

  {
    // 更新图信息 (Update graph information)
    auto common = &node->context->impl->common;
    std::lock_guard<std::mutex> guard(common->node_update_mutex);
    static_cast<void>(common->graph_cache.dissociate_writer(
        info->service.pub->gid, common->gid, node->name, node->namespace_));
    rmw_dds_common::msg::ParticipantEntitiesInfo msg = common->graph_cache.dissociate_reader(
        info->service.sub->gid, common->gid, node->name, node->namespace_);

    // 发布更新后的图信息 (Publish the updated graph information)
    if (RMW_RET_OK != rmw_publish(common->pub, static_cast<void *>(&msg), nullptr)) {
      RMW_SET_ERROR_MSG("failed to publish ParticipantEntitiesInfo when destroying service");
    }
  }

  // 销毁服务实例的资源 (Destroy resources of the service instance)
  rmw_fini_cs(&info->service);
  delete info;
  rmw_free(const_cast<char *>(service->service_name));
  rmw_service_free(service);

  // 返回操作成功 (Return operation success)
  return RMW_RET_OK;
}

/**
 * @brief 创建一个ROS2服务 (Create a ROS2 service)
 *
 * @param[in] node 指向要创建服务的节点的指针 (Pointer to the node where the service is to be
 * created)
 * @param[in] type_supports 服务类型支持结构体的指针 (Pointer to the service type support structure)
 * @param[in] service_name 要创建的服务的名称 (Name of the service to be created)
 * @param[in] qos_policies 服务的QoS策略 (QoS policies for the service)
 * @return rmw_service_t* 成功时返回指向新创建的服务的指针，失败时返回nullptr (Pointer to the newly
 * created service on success, nullptr on failure)
 */
extern "C" rmw_service_t *rmw_create_service(
    const rmw_node_t *node,
    const rosidl_service_type_support_t *type_supports,
    const char *service_name,
    const rmw_qos_profile_t *qos_policies) {
  // 检查qos_policies参数是否为空 (Check if qos_policies argument is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);

  // 创建一个CddsService实例 (Create a CddsService instance)
  CddsService *info = new CddsService();

  // 更新QoS策略以获得最佳可用性 (Update QoS policies for best available)
  rmw_qos_profile_t adapted_qos_policies =
      rmw_dds_common::qos_profile_update_best_available_for_services(*qos_policies);

  // 初始化客户端-服务端通信 (Initialize client-server communication)
  if (rmw_init_cs(
          &info->service, &info->user_callback_data, node, type_supports, service_name,
          &adapted_qos_policies, true) != RMW_RET_OK) {
    delete (info);
    return nullptr;
  }

  // 分配rmw_service实例 (Allocate rmw_service instance)
  rmw_service_t *rmw_service = rmw_service_allocate();
  RET_NULL_X(rmw_service, goto fail_service);

  // 设置实现标识符和数据 (Set implementation identifier and data)
  rmw_service->implementation_identifier = eclipse_cyclonedds_identifier;
  rmw_service->data = info;

  // 分配并设置服务名称 (Allocate and set service name)
  rmw_service->service_name =
      reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  RET_NULL_X(rmw_service->service_name, goto fail_service_name);
  memcpy(const_cast<char *>(rmw_service->service_name), service_name, strlen(service_name) + 1);

  /**
   * @brief 更新图 (Update graph)
   *
   * @param[in] node 当前节点的指针，用于获取上下文和其他信息
   * @param[in] info 服务信息的指针，包含发布者和订阅者的相关信息
   * @param[in] rmw_service RMW服务的指针，用于在失败时销毁服务
   * @return nullptr 如果发布失败，否则不返回任何值
   */
  {
    // 获取公共上下文对象的引用
    auto common = &node->context->impl->common;

    // 使用互斥锁保护节点更新操作
    std::lock_guard<std::mutex> guard(common->node_update_mutex);

    // 将发布者与当前节点关联
    static_cast<void>(common->graph_cache.associate_writer(
        info->service.pub->gid, common->gid, node->name, node->namespace_));

    // 将订阅者与当前节点关联，并获取参与者实体信息消息
    rmw_dds_common::msg::ParticipantEntitiesInfo msg = common->graph_cache.associate_reader(
        info->service.sub->gid, common->gid, node->name, node->namespace_);

    // 发布参与者实体信息消息
    if (RMW_RET_OK != rmw_publish(common->pub, static_cast<void *>(&msg), nullptr)) {
      // 如果发布失败，则销毁服务并返回nullptr
      static_cast<void>(destroy_service(node, rmw_service));
      return nullptr;
    }
  }

  return rmw_service;

// 错误处理 (Error handling)
fail_service_name:
  rmw_service_free(rmw_service);
fail_service:
  rmw_fini_cs(&info->service);
  delete info;
  return nullptr;
}

/**
 * @brief 销毁一个ROS2服务 (Destroy a ROS2 service)
 *
 * @param[in] node 指向要销毁服务的节点的指针 (Pointer to the node where the service is to be
 * destroyed)
 * @param[in] service 要销毁的服务的指针 (Pointer to the service to be destroyed)
 * @return rmw_ret_t 返回操作结果 (Return the result of the operation)
 */
extern "C" rmw_ret_t rmw_destroy_service(rmw_node_t *node, rmw_service_t *service) {
  return destroy_service(node, service);
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    INTROSPECTION                                                  ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 获取节点名称和命名空间
 *
 * @param[in] node 输入的节点指针
 * @param[out] node_names 输出的节点名称数组
 * @param[out] node_namespaces 输出的节点命名空间数组
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应错误代码
 */
extern "C" rmw_ret_t rmw_get_node_names(
    const rmw_node_t *node,
    rcutils_string_array_t *node_names,
    rcutils_string_array_t *node_namespaces) {
  // 检查输入参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  // 检查节点类型是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查节点名称数组是否为空
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_names)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 检查节点命名空间数组是否为空
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_namespaces)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 获取公共上下文
  auto common_context = &node->context->impl->common;

  // 获取默认分配器
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // 从图缓存中获取节点名称和命名空间
  return common_context->graph_cache.get_node_names(
      node_names, node_namespaces, nullptr, &allocator);
}

/**
 * @brief 获取节点名称、命名空间和enclaves
 *
 * @param[in] node 输入的节点指针
 * @param[out] node_names 输出的节点名称数组
 * @param[out] node_namespaces 输出的节点命名空间数组
 * @param[out] enclaves 输出的enclaves数组
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应错误代码
 */
extern "C" rmw_ret_t rmw_get_node_names_with_enclaves(
    const rmw_node_t *node,
    rcutils_string_array_t *node_names,
    rcutils_string_array_t *node_namespaces,
    rcutils_string_array_t *enclaves) {
  // 检查输入参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  // 检查节点类型是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查节点名称数组是否为空
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_names)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 检查节点命名空间数组是否为空
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_namespaces)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 检查enclaves数组是否为空
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(enclaves)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 获取公共上下文
  auto common_context = &node->context->impl->common;

  // 获取默认分配器
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // 从图缓存中获取节点名称、命名空间和enclaves
  return common_context->graph_cache.get_node_names(
      node_names, node_namespaces, enclaves, &allocator);
}

/**
 * @brief 获取ROS2主题的名称和类型
 *
 * @param[in] node 指向rmw_node_t结构体的指针，表示一个ROS2节点
 * @param[in,out] allocator 指向rcutils_allocator_t结构体的指针，用于分配内存
 * @param[in] no_demangle 布尔值，表示是否需要对主题和类型进行解扰处理
 * @param[out] tptyp 指向rmw_names_and_types_t结构体的指针，用于存储获取到的主题名称和类型
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_get_topic_names_and_types(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    bool no_demangle,
    rmw_names_and_types_t *tptyp) {
  // 检查输入参数node是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查节点实现标识符是否匹配，不匹配则返回错误的RMW实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查分配器参数是否有效，无效则返回无效参数错误
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  // 检查tptyp是否为空，为空则返回无效参数错误
  if (RMW_RET_OK != rmw_names_and_types_check_zero(tptyp)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 根据no_demangle选择解扰函数
  DemangleFunction demangle_topic = _demangle_ros_topic_from_topic;
  DemangleFunction demangle_type = _demangle_if_ros_type;
  if (no_demangle) {
    demangle_topic = _identity_demangle;
    demangle_type = _identity_demangle;
  }
  // 获取节点上下文的公共部分
  auto common_context = &node->context->impl->common;
  // 调用graph_cache的get_names_and_types方法获取主题名称和类型
  return common_context->graph_cache.get_names_and_types(
      demangle_topic, demangle_type, allocator, tptyp);
}

/**
 * @brief 获取ROS2服务的名称和类型
 *
 * @param[in] node 指向rmw_node_t结构体的指针，表示一个ROS2节点
 * @param[in,out] allocator 指向rcutils_allocator_t结构体的指针，用于分配内存
 * @param[out] sntyp 指向rmw_names_and_types_t结构体的指针，用于存储获取到的服务名称和类型
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_get_service_names_and_types(
    const rmw_node_t *node, rcutils_allocator_t *allocator, rmw_names_and_types_t *sntyp) {
  // 检查输入参数node是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查节点实现标识符是否匹配，不匹配则返回错误的RMW实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查分配器参数是否有效，无效则返回无效参数错误
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  // 检查sntyp是否为空，为空则返回无效参数错误
  if (RMW_RET_OK != rmw_names_and_types_check_zero(sntyp)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 获取节点上下文的公共部分
  auto common_context = &node->context->impl->common;
  // 调用graph_cache的get_names_and_types方法获取服务名称和类型
  return common_context->graph_cache.get_names_and_types(
      _demangle_service_from_topic, _demangle_service_type_only, allocator, sntyp);
}

/**
 * @brief 获取话题名称
 *
 * @param[in] endpoint_handle DDS实体句柄
 * @param[out] name 话题名称
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t get_topic_name(dds_entity_t endpoint_handle, std::string &name) {
  // 创建一个临时缓冲区，用于存储话题名称
  std::vector<char> tmp(128);
  // 调用 dds_get_name 函数获取话题名称，并将其存储在 tmp 缓冲区中
  dds_return_t rc = dds_get_name(dds_get_topic(endpoint_handle), tmp.data(), tmp.size());
  if (rc > 0 && static_cast<size_t>(rc) >= tmp.size()) {
    // 如果话题名称太长，无法放入缓冲区，则调整缓冲区大小
    tmp.resize(static_cast<size_t>(rc) + 1);
    rc = dds_get_name(dds_get_topic(endpoint_handle), tmp.data(), tmp.size());
  }
  if (rc < 0) {
    // 如果返回值小于0，表示出现错误
    return RMW_RET_ERROR;
  } else if (static_cast<size_t>(rc) >= tmp.size()) {
    // 如果话题名称长度大于等于缓冲区大小，表示出现错误
    return RMW_RET_ERROR;
  }

  // 将获取到的话题名称赋值给输出参数 name
  name = std::string(tmp.begin(), tmp.begin() + rc);
  return RMW_RET_OK;
}

/**
 * @brief 检查服务端是否存在读写器
 *
 * @param[in] client CddsCS 类型的客户端对象
 * @param[out] is_available 服务端是否可用的标志
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t check_for_service_reader_writer(const CddsCS &client, bool *is_available) {
  // 创建两个向量，分别存储读者和写者实例句柄
  std::vector<dds_instance_handle_t> rds, wrs;
  // 检查输出参数 is_available 是否为空指针，且其值为 false
  assert(is_available != nullptr && !*is_available);
  if (get_matched_endpoints(client.pub->enth, dds_get_matched_subscriptions, rds) < 0 ||
      get_matched_endpoints(client.sub->enth, dds_get_matched_publications, wrs) < 0) {
    RMW_SET_ERROR_MSG("rmw_service_server_is_available: failed to get reader/writer matches");
    return RMW_RET_ERROR;
  }
  // 首先从匹配的读者中提取所有服务ID
  std::set<std::string> needles;
  for (const auto &rdih : rds) {
    auto rd = get_matched_subscription_data(client.pub->enth, rdih);
    std::string serviceid;
    if (rd && get_user_data_key(rd->qos, "serviceid", serviceid)) {
      needles.insert(serviceid);
    }
  }
  if (needles.empty()) {
    // 如果没有匹配到具有服务ID的服务，但存在匹配的请求读者和响应写者，
    // 则使用旧方法，只需检查匹配的实例是否存在。
    *is_available = !rds.empty() && !wrs.empty();
  } else {
    // 扫描写者，查看是否至少有一个响应写者与发现的请求读者匹配
    for (const auto &wrih : wrs) {
      auto wr = get_matched_publication_data(client.sub->enth, wrih);
      std::string serviceid;
      if (wr && get_user_data_key(wr->qos, "serviceid", serviceid) &&
          needles.find(serviceid) != needles.end()) {
        *is_available = true;
        break;
      }
    }
  }
  return RMW_RET_OK;
}

/**
 * @brief 检查服务端是否可用
 *
 * @param[in] node          ROS2节点指针
 * @param[in] client        ROS2客户端指针
 * @param[out] is_available 服务端是否可用的布尔值指针
 * @return rmw_ret_t        返回操作结果状态
 */
extern "C" rmw_ret_t rmw_service_server_is_available(
    const rmw_node_t *node, const rmw_client_t *client, bool *is_available) {
  // 检查节点是否为空
  RET_NULL(node);
  // 检查节点实现ID是否正确
  RET_WRONG_IMPLID(node);
  // 检查客户端是否为空
  RET_NULL(client);
  // 检查客户端实现ID是否正确
  RET_WRONG_IMPLID(client);
  // 检查is_available指针是否为空
  RET_NULL(is_available);
  // 初始化is_available为false
  *is_available = false;

  // 获取客户端数据并转换为CddsClient类型
  auto info = static_cast<CddsClient *>(client->data);
  // 获取节点上下文中的公共上下文
  auto common_context = &node->context->impl->common;

  // 定义发布和订阅主题名称字符串
  std::string sub_topic_name, pub_topic_name;
  // 获取发布和订阅主题名称，如果获取失败则返回错误信息
  if (get_topic_name(info->client.pub->enth, pub_topic_name) < 0 ||
      get_topic_name(info->client.sub->enth, sub_topic_name) < 0) {
    RMW_SET_ERROR_MSG("rmw_service_server_is_available: failed to get topic names");
    return RMW_RET_ERROR;
  }

  // 定义请求订阅者数量
  size_t number_of_request_subscribers = 0;
  // 获取发布主题的读取器数量
  rmw_ret_t ret =
      common_context->graph_cache.get_reader_count(pub_topic_name, &number_of_request_subscribers);
  // 如果获取失败或者读取器数量为0，则返回结果
  if (ret != RMW_RET_OK || 0 == number_of_request_subscribers) {
    return ret;
  }
  // 定义响应发布者数量
  size_t number_of_response_publishers = 0;
  // 获取订阅主题的写入器数量
  ret =
      common_context->graph_cache.get_writer_count(sub_topic_name, &number_of_response_publishers);
  // 如果获取失败或者写入器数量为0，则返回结果
  if (ret != RMW_RET_OK || 0 == number_of_response_publishers) {
    return ret;
  }
  // 检查服务端的读写器是否存在，并更新is_available的值
  return check_for_service_reader_writer(info->client, is_available);
}

/**
 * @brief 计算给定主题上的发布者数量 (Count the number of publishers on a given topic)
 *
 * @param[in] node 指向rmw_node_t结构体的指针 (Pointer to an rmw_node_t structure)
 * @param[in] topic_name 要查询的主题名称 (The topic name to query)
 * @param[out] count 存储发布者数量的变量指针 (Pointer to a variable to store the publisher count)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
extern "C" rmw_ret_t rmw_count_publishers(
    const rmw_node_t *node, const char *topic_name, size_t *count) {
  // 检查node参数是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查节点类型是否匹配，如果不匹配则返回错误的RMW实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查topic_name参数是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  // 验证完整的主题名称
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  // 如果主题名称验证结果无效，则设置错误消息并返回无效参数错误
  if (RMW_TOPIC_VALID != validation_result) {
    const char *reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 检查count参数是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  // 获取节点上下文的公共部分
  auto common_context = &node->context->impl->common;
  // 构造完全限定主题名称
  const std::string mangled_topic_name = make_fqtopic(ROS_TOPIC_PREFIX, topic_name, "", false);
  // 获取发布者数量并返回结果
  return common_context->graph_cache.get_writer_count(mangled_topic_name, count);
}

/**
 * @brief 计算给定主题上的订阅者数量 (Count the number of subscribers on a given topic)
 *
 * @param[in] node 指向rmw_node_t结构体的指针 (Pointer to an rmw_node_t structure)
 * @param[in] topic_name 要查询的主题名称 (The topic name to query)
 * @param[out] count 存储订阅者数量的变量指针 (Pointer to a variable to store the subscriber count)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
extern "C" rmw_ret_t rmw_count_subscribers(
    const rmw_node_t *node, const char *topic_name, size_t *count) {
  // 检查node参数是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查节点类型是否匹配，如果不匹配则返回错误的RMW实现错误
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查topic_name参数是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  // 验证完整的主题名称
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  // 如果主题名称验证结果无效，则设置错误消息并返回无效参数错误
  if (RMW_TOPIC_VALID != validation_result) {
    const char *reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 检查count参数是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  // 获取节点上下文的公共部分
  auto common_context = &node->context->impl->common;
  // 构造完全限定主题名称
  const std::string mangled_topic_name = make_fqtopic(ROS_TOPIC_PREFIX, topic_name, "", false);
  // 获取订阅者数量并返回结果
  return common_context->graph_cache.get_reader_count(mangled_topic_name, count);
}

/**
 * @brief 定义获取节点名称和类型的函数指针类型
 *
 * @param[in] context 一个指向rmw_dds_common::Context的指针
 * @param[in] node_name 节点名称
 * @param[in] node_namespace 节点命名空间
 * @param[in] demangle_topic 解析主题名称的函数
 * @param[in] demangle_type 解析类型名称的函数
 * @param[in] allocator 分配器
 * @param[out] topic_names_and_types 存储获取到的主题名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
using GetNamesAndTypesByNodeFunction = rmw_ret_t (*)(
    rmw_dds_common::Context *,
    const std::string &,
    const std::string &,
    DemangleFunction,
    DemangleFunction,
    rcutils_allocator_t *,
    rmw_names_and_types_t *);

/**
 * @brief 获取节点的主题名称和类型
 *
 * @param[in] node 指向rmw_node_t的指针
 * @param[in] allocator 分配器
 * @param[in] node_name 节点名称
 * @param[in] node_namespace 节点命名空间
 * @param[in] demangle_topic 解析主题名称的函数
 * @param[in] demangle_type 解析类型名称的函数
 * @param[in] no_demangle 是否不进行解析
 * @param[in] get_names_and_types_by_node 获取节点名称和类型的函数指针
 * @param[out] topic_names_and_types 存储获取到的主题名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t get_topic_names_and_types_by_node(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    const char *node_name,
    const char *node_namespace,
    DemangleFunction demangle_topic,
    DemangleFunction demangle_type,
    bool no_demangle,
    GetNamesAndTypesByNodeFunction get_names_and_types_by_node,
    rmw_names_and_types_t *topic_names_and_types) {
  // 检查节点是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  // 检查节点类型是否匹配
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // 检查分配器是否有效
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  // 验证节点名称
  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(node_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char *reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("node_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 验证节点命名空间
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(node_namespace, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char *reason = rmw_namespace_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("node_namespace argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 检查主题名称和类型是否为空
  ret = rmw_names_and_types_check_zero(topic_names_and_types);
  if (RMW_RET_OK != ret) {
    return ret;
  }

  // 获取公共上下文
  auto common_context = &node->context->impl->common;
  // 设置解析函数
  if (no_demangle) {
    demangle_topic = _identity_demangle;
    demangle_type = _identity_demangle;
  }
  // 调用获取节点名称和类型的函数
  return get_names_and_types_by_node(
      common_context, node_name, node_namespace, demangle_topic, demangle_type, allocator,
      topic_names_and_types);
}

/**
 * @brief 获取节点的读者名称和类型
 *
 * @param[in] common_context 指向rmw_dds_common::Context的指针
 * @param[in] node_name 节点名称
 * @param[in] node_namespace 节点命名空间
 * @param[in] demangle_topic 解析主题名称的函数
 * @param[in] demangle_type 解析类型名称的函数
 * @param[in] allocator 分配器
 * @param[out] topic_names_and_types 存储获取到的主题名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t get_reader_names_and_types_by_node(
    rmw_dds_common::Context *common_context,
    const std::string &node_name,
    const std::string &node_namespace,
    DemangleFunction demangle_topic,
    DemangleFunction demangle_type,
    rcutils_allocator_t *allocator,
    rmw_names_and_types_t *topic_names_and_types) {
  // 调用graph_cache的get_reader_names_and_types_by_node方法获取读者名称和类型
  return common_context->graph_cache.get_reader_names_and_types_by_node(
      node_name, node_namespace, demangle_topic, demangle_type, allocator, topic_names_and_types);
}

/**
 * @brief 获取节点的写者名称和类型
 *
 * @param[in] common_context 指向rmw_dds_common::Context的指针
 * @param[in] node_name 节点名称
 * @param[in] node_namespace 节点命名空间
 * @param[in] demangle_topic 解析主题名称的函数
 * @param[in] demangle_type 解析类型名称的函数
 * @param[in] allocator 分配器
 * @param[out] topic_names_and_types 存储获取到的主题名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
static rmw_ret_t get_writer_names_and_types_by_node(
    rmw_dds_common::Context *common_context,
    const std::string &node_name,
    const std::string &node_namespace,
    DemangleFunction demangle_topic,
    DemangleFunction demangle_type,
    rcutils_allocator_t *allocator,
    rmw_names_and_types_t *topic_names_and_types) {
  // 调用graph_cache的get_writer_names_and_types_by_node方法获取写者名称和类型
  return common_context->graph_cache.get_writer_names_and_types_by_node(
      node_name, node_namespace, demangle_topic, demangle_type, allocator, topic_names_and_types);
}

/**
 * @brief 获取指定节点的订阅者名称和类型
 *
 * @param[in] node 指向要查询的节点的指针
 * @param[in] allocator 用于分配内存的分配器
 * @param[in] node_name 要查询的节点的名称
 * @param[in] node_namespace 要查询的节点的命名空间
 * @param[in] no_demangle 是否取消对主题名称和类型的解扰
 * @param[out] tptyp 存储获取到的订阅者名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_get_subscriber_names_and_types_by_node(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    const char *node_name,
    const char *node_namespace,
    bool no_demangle,
    rmw_names_and_types_t *tptyp) {
  // 调用通用函数获取订阅者名称和类型
  return get_topic_names_and_types_by_node(
      node, allocator, node_name, node_namespace, _demangle_ros_topic_from_topic,
      _demangle_if_ros_type, no_demangle, get_reader_names_and_types_by_node, tptyp);
}

/**
 * @brief 获取指定节点的发布者名称和类型
 *
 * @param[in] node 指向要查询的节点的指针
 * @param[in] allocator 用于分配内存的分配器
 * @param[in] node_name 要查询的节点的名称
 * @param[in] node_namespace 要查询的节点的命名空间
 * @param[in] no_demangle 是否取消对主题名称和类型的解扰
 * @param[out] tptyp 存储获取到的发布者名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_get_publisher_names_and_types_by_node(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    const char *node_name,
    const char *node_namespace,
    bool no_demangle,
    rmw_names_and_types_t *tptyp) {
  // 调用通用函数获取发布者名称和类型
  return get_topic_names_and_types_by_node(
      node, allocator, node_name, node_namespace, _demangle_ros_topic_from_topic,
      _demangle_if_ros_type, no_demangle, get_writer_names_and_types_by_node, tptyp);
}

/**
 * @brief 获取指定节点的服务名称和类型
 *
 * @param[in] node 指向要查询的节点的指针
 * @param[in] allocator 用于分配内存的分配器
 * @param[in] node_name 要查询的节点的名称
 * @param[in] node_namespace 要查询的节点的命名空间
 * @param[out] sntyp 存储获取到的服务名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_get_service_names_and_types_by_node(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    const char *node_name,
    const char *node_namespace,
    rmw_names_and_types_t *sntyp) {
  // 调用通用函数获取服务名称和类型
  return get_topic_names_and_types_by_node(
      node, allocator, node_name, node_namespace, _demangle_service_request_from_topic,
      _demangle_service_type_only, false, get_reader_names_and_types_by_node, sntyp);
}

/**
 * @brief 获取指定节点的客户端名称和类型
 *
 * @param[in] node 指向要查询的节点的指针
 * @param[in] allocator 用于分配内存的分配器
 * @param[in] node_name 要查询的节点的名称
 * @param[in] node_namespace 要查询的节点的命名空间
 * @param[out] sntyp 存储获取到的客户端名称和类型的结构体指针
 * @return rmw_ret_t 返回操作结果
 */
extern "C" rmw_ret_t rmw_get_client_names_and_types_by_node(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    const char *node_name,
    const char *node_namespace,
    rmw_names_and_types_t *sntyp) {
  // 调用通用函数获取客户端名称和类型
  return get_topic_names_and_types_by_node(
      node, allocator, node_name, node_namespace, _demangle_service_reply_from_topic,
      _demangle_service_type_only, false, get_reader_names_and_types_by_node, sntyp);
}

/**
 * @brief 获取指定话题上的发布者信息 (Get publishers information on a specific topic)
 *
 * @param[in] node 指向ROS2节点的指针 (Pointer to the ROS2 node)
 * @param[in] allocator 用于分配内存的分配器 (Allocator for memory allocation)
 * @param[in] topic_name 要查询的话题名称 (Topic name to query)
 * @param[in] no_mangle 是否对话题名称进行解扰处理 (Whether to demangle the topic name or not)
 * @param[out] publishers_info 存储发布者信息的数组 (Array to store the publishers information)
 * @return rmw_ret_t 返回操作结果 (Return operation result)
 */
extern "C" rmw_ret_t rmw_get_publishers_info_by_topic(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    const char *topic_name,
    bool no_mangle,
    rmw_topic_endpoint_info_array_t *publishers_info) {
  // 检查节点是否为空 (Check if the node is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  // 检查节点类型是否匹配 (Check if the node type matches)
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // 检查分配器是否有效 (Check if the allocator is valid)
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);

  // 检查话题名称是否为空 (Check if the topic name is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);

  // 检查发布者信息数组是否为空 (Check if the publishers info array is null)
  if (RMW_RET_OK != rmw_topic_endpoint_info_array_check_zero(publishers_info)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 获取节点上下文 (Get the node context)
  auto common_context = &node->context->impl->common;

  // 初始化话题名称和解扰函数 (Initialize topic name and demangle function)
  std::string mangled_topic_name = topic_name;
  DemangleFunction demangle_type = _identity_demangle;

  // 如果不进行解扰处理，则生成完全限定的话题名称 (If not demangling, generate fully qualified topic
  // name)
  if (!no_mangle) {
    mangled_topic_name = make_fqtopic(ROS_TOPIC_PREFIX, topic_name, "", false);
    demangle_type = _demangle_if_ros_type;
  }

  // 通过话题名称获取发布者信息 (Get publishers information by topic name)
  return common_context->graph_cache.get_writers_info_by_topic(
      mangled_topic_name, demangle_type, allocator, publishers_info);
}

/**
 * @brief 获取指定主题的订阅者信息 (Get subscriptions info by topic)
 *
 * @param[in] node 指向 ROS2 节点的指针 (Pointer to the ROS2 node)
 * @param[in] allocator 用于分配内存的分配器 (Allocator for memory allocation)
 * @param[in] topic_name 需要查询的主题名称 (Topic name to query)
 * @param[in] no_mangle 是否对主题名称进行解析 (Whether to demangle the topic name or not)
 * @param[out] subscriptions_info 存储订阅者信息的数组 (Array to store the subscriptions info)
 * @return rmw_ret_t 返回操作结果 (Return operation result)
 */
extern "C" rmw_ret_t rmw_get_subscriptions_info_by_topic(
    const rmw_node_t *node,
    rcutils_allocator_t *allocator,
    const char *topic_name,
    bool no_mangle,
    rmw_topic_endpoint_info_array_t *subscriptions_info) {
  // 检查参数是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_topic_endpoint_info_array_check_zero(subscriptions_info)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 获取节点上下文 (Get node context)
  auto common_context = &node->context->impl->common;
  // 初始化主题名称 (Initialize topic name)
  std::string mangled_topic_name = topic_name;
  // 设置解析类型函数 (Set demangle type function)
  DemangleFunction demangle_type = _identity_demangle;
  // 如果不进行解析 (If not demangling)
  if (!no_mangle) {
    // 创建完全限定主题名称 (Create fully qualified topic name)
    mangled_topic_name = make_fqtopic(ROS_TOPIC_PREFIX, topic_name, "", false);
    // 设置解析 ROS 类型函数 (Set demangle ROS type function)
    demangle_type = _demangle_if_ros_type;
  }
  // 获取指定主题的订阅者信息 (Get subscriptions info by topic)
  return common_context->graph_cache.get_readers_info_by_topic(
      mangled_topic_name, demangle_type, allocator, subscriptions_info);
}

/**
 * @brief 检查 QoS 配置文件是否兼容 (Check if QoS profiles are compatible)
 *
 * @param[in] publisher_profile 发布者 QoS 配置文件 (Publisher QoS profile)
 * @param[in] subscription_profile 订阅者 QoS 配置文件 (Subscription QoS profile)
 * @param[out] compatibility 存储兼容性结果的变量 (Variable to store compatibility result)
 * @param[out] reason 存储原因的字符串 (String to store the reason)
 * @param[in] reason_size 原因字符串的大小 (Size of the reason string)
 * @return rmw_ret_t 返回操作结果 (Return operation result)
 */
extern "C" rmw_ret_t rmw_qos_profile_check_compatible(
    const rmw_qos_profile_t publisher_profile,
    const rmw_qos_profile_t subscription_profile,
    rmw_qos_compatibility_type_t *compatibility,
    char *reason,
    size_t reason_size) {
  // 检查 QoS 配置文件是否兼容 (Check if QoS profiles are compatible)
  return rmw_dds_common::qos_profile_check_compatible(
      publisher_profile, subscription_profile, compatibility, reason, reason_size);
}

/**
 * @brief 获取客户端请求发布者的实际QoS配置
 *
 * @param[in] client 指向rmw_client_t结构体的指针
 * @param[out] qos 指向rmw_qos_profile_t结构体的指针，用于存储获取到的QoS配置
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回相应错误代码
 */
extern "C" rmw_ret_t rmw_client_request_publisher_get_actual_qos(
    const rmw_client_t *client, rmw_qos_profile_t *qos) {
  // 检查client参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  // 检查qos参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  // 将client->data转换为CddsClient类型
  auto cli = static_cast<CddsClient *>(client->data);

  // 获取实际QoS配置并将其存储在qos中，成功则返回RMW_RET_OK
  if (get_readwrite_qos(cli->client.pub->enth, qos)) {
    return RMW_RET_OK;
  }

  // 设置错误信息并返回错误代码
  RMW_SET_ERROR_MSG("failed to get client's request publisher QoS");
  return RMW_RET_ERROR;
}

/**
 * @brief 获取客户端响应订阅者的实际QoS配置
 *
 * @param[in] client 指向rmw_client_t结构体的指针
 * @param[out] qos 指向rmw_qos_profile_t结构体的指针，用于存储获取到的QoS配置
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回相应错误代码
 */
extern "C" rmw_ret_t rmw_client_response_subscription_get_actual_qos(
    const rmw_client_t *client, rmw_qos_profile_t *qos) {
  // 检查client参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  // 检查qos参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  // 将client->data转换为CddsClient类型
  auto cli = static_cast<CddsClient *>(client->data);

  // 获取实际QoS配置并将其存储在qos中，成功则返回RMW_RET_OK
  if (get_readwrite_qos(cli->client.sub->enth, qos)) {
    return RMW_RET_OK;
  }

  // 设置错误信息并返回错误代码
  RMW_SET_ERROR_MSG("failed to get client's response subscription QoS");
  return RMW_RET_ERROR;
}

/**
 * @brief 获取服务端响应发布者的实际QoS配置
 *
 * @param[in] service 指向rmw_service_t结构体的指针
 * @param[out] qos 指向rmw_qos_profile_t结构体的指针，用于存储获取到的QoS配置
 * @return rmw_ret_t 返回操作结果，成功返回RMW_RET_OK，失败返回相应错误代码
 */
extern "C" rmw_ret_t rmw_service_response_publisher_get_actual_qos(
    const rmw_service_t *service, rmw_qos_profile_t *qos) {
  // 检查service参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  // 检查qos参数是否为空，为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  // 将service->data转换为CddsService类型
  auto srv = static_cast<CddsService *>(service->data);

  // 获取实际QoS配置并将其存储在qos中，成功则返回RMW_RET_OK
  if (get_readwrite_qos(srv->service.pub->enth, qos)) {
    return RMW_RET_OK;
  }

  // 设置错误信息并返回错误代码
  RMW_SET_ERROR_MSG("failed to get service's response publisher QoS");
  return RMW_RET_ERROR;
}

/**
 * @brief 获取服务请求订阅的实际QoS配置
 *
 * 该函数用于获取ROS2 RMW层中服务请求订阅的实际QoS配置。
 *
 * @param[in] service 指向rmw_service_t类型的指针，表示要查询的服务
 * @param[out] qos 指向rmw_qos_profile_t类型的指针，用于存储查询到的QoS配置
 * @return 返回rmw_ret_t类型的结果，表示函数执行成功或失败
 */
extern "C" rmw_ret_t rmw_service_request_subscription_get_actual_qos(
    const rmw_service_t *service, rmw_qos_profile_t *qos) {
  // 检查输入参数service是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  // 检查输入参数qos是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  // 将service->data转换为CddsService类型的指针，并赋值给srv变量
  auto srv = static_cast<CddsService *>(service->data);

  // 调用get_readwrite_qos函数获取服务请求订阅的实际QoS配置
  if (get_readwrite_qos(srv->service.sub->enth, qos)) {
    // 如果获取成功，则返回RMW_RET_OK
    return RMW_RET_OK;
  }

  // 如果获取失败，设置错误信息并返回RMW_RET_ERROR
  RMW_SET_ERROR_MSG("failed to get service's request subscription QoS");
  return RMW_RET_ERROR;
}

/**
 * @brief 检查指定的RMW特性是否受支持
 *
 * 该函数用于检查ROS2 RMW层中指定的特性是否受支持。
 *
 * @param[in] feature rmw_feature_t类型的值，表示要查询的特性
 * @return 返回bool类型的结果，表示特性是否受支持
 */
extern "C" bool rmw_feature_supported(rmw_feature_t feature) {
  // 忽略输入参数feature，直接返回false表示不支持该特性
  (void)feature;
  return false;
}
