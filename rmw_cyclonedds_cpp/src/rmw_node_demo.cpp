/*
  粗略提出和pub/sub最相关的函数
*/

// clang-format off
// 这些头文件在 ROS 2 的 rmw 层中使用，以实现与底层通信系统（如 DDS）的交互
#include "MessageTypeSupport.hpp"  // 包含与消息类型支持相关的类和函数。
#include "Serialization.hpp"  // 包含序列化和反序列化功能，用于在 ROS 2 中传输消息。
#include "ServiceTypeSupport.hpp"  // 包含与服务类型支持相关的类和函数。
#include "TypeSupport2.hpp"  // 包含与类型支持相关的类和函数，用于处理不同的数据类型。
#include "demangle.hpp"             // 包含用于解析 C++ 符号名称的函数。
#include "fallthrough_macro.hpp"  // 定义了一个宏，用于标记 switch 语句中的故意穿越行为。
#include "namespace_prefix.hpp"  // 包含用于处理 ROS 2 命名空间前缀的函数。

#include "dds/dds.h"  // 包含 DDS（Data Distribution Service）API 的主要头文件，用于实现 ROS 2 的底层通信。
#include "dds/ddsc/dds_data_allocator.h"  // 包含 DDS 数据分配器 API，用于管理 DDS 数据对象的内存。
#include "dds/ddsc/dds_loan_api.h"  // 包含 DDS 借款 API，用于处理数据样本的借款和归还。

#include "rmw/rmw.h"  // 包含 rmw API 的主要头文件，用于实现 ROS 2 的中间件抽象层。
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"  // 包含用于将 rcutils 返回值转换为 rmw 返回值的函数。
#include "rmw/event.h"                // 包含用于处理 ROS 2 事件的实用程序。
#include "rmw/event_callback_type.h"  // 定义了用于事件回调的类型。
#include "rmw/get_node_info_and_types.h"  // 包含用于获取节点信息和类型的实用程序。
#include "rmw/get_service_names_and_types.h"  // 包含用于获取服务名称和类型的实用程序。
#include "rmw/get_topic_endpoint_info.h"  // 包含用于获取主题端点信息的实用程序。
#include "rmw/get_topic_names_and_types.h"  // 包含用于获取主题名称和类型的实用程序。
#include "rmw/impl/cpp/key_value.hpp"       // 包含用于处理键值对的实用程序。
#include "rmw/impl/cpp/macros.hpp"          // 包含用于定义 rmw 实现相关宏的头文件。
#include "rmw/incompatible_qos_events_statuses.h"  // 包含用于处理不兼容的 QoS 事件状态的实用程序。
#include "rmw/names_and_types.h"  // 包含用于处理名称和类型的实用程序。
#include "rmw/sanity_checks.h"  // 包含用于执行 rmw 实现的基本检查的实用程序。
#include "rmw/topic_endpoint_info_array.h"  // 包含用于处理主题端点信息数组的实用程序。
#include "rmw_dds_common/context.hpp"      // 包含用于管理 DDS 上下文的类和函数。
#include "rmw_dds_common/graph_cache.hpp"  // 包含用于管理图形缓存的类和函数。
#include "rmw_dds_common/msg/participant_entities_info.hpp"  // 包含参与者实体信息消息的定义。
#include "rmw_dds_common/qos.hpp"  // 包含用于处理 QoS（Quality of Service）设置的实用程序。

#include "rosidl_runtime_c/type_hash.h"  // 包含用于计算类型哈希值的实用程序。
#include "rosidl_typesupport_cpp/message_type_support.hpp"  // 包含用于处理 C++ 消息类型支持的实用程序。
#include "serdata.hpp"              // 包含与序列化数据相关的类和函数。
#include "serdes.hpp"               // 包含与序列化和反序列化相关的类和函数。
#include "tracetools/tracetools.h"  // 包含用于跟踪 ROS 2 系统性能的实用程序。
// clang-format on

using rmw_dds_common::msg::ParticipantEntitiesInfo;

/// \brief CycloneDDS的标识符，用于在RMW层中区分不同的DDS实现
const char *const eclipse_cyclonedds_identifier = "rmw_cyclonedds_cpp";
/// \brief CycloneDDS使用的序列化格式，这里是Common Data Representation (CDR)格式
const char *const eclipse_cyclonedds_serialization_format = "cdr";

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
 * 前几个都是 dds 需要的属性
 * 后一个是为了ros 提供的
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
 * 同理
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
 * @brief CddsEntity结构体，包含一个dds_entity_t类型的成员enth。
 */
// typedef int32_t dds_entity_t;
// 就是handles，即hash值表示
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
 */
struct CddsDomain {
  bool localhost_only;         ///< 是否仅限本地主机
  uint32_t refcount;           ///< 节点引用计数
  dds_entity_t domain_handle;  ///< 域实体的句柄

  /**
   * @brief 默认构造函数，以便可以安全地使用 operator[] 查找一个域。
   */
  CddsDomain() : localhost_only(false), refcount(0), domain_handle(0) {}
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

  rmw_ret_t init(rmw_init_options_t *options, size_t domain_id);
  rmw_ret_t fini();

  ~rmw_context_impl_s() {}

private:
  void clean_up();
};

/**
 * @struct CddsNode
 * @brief 一个表示ROS2节点的结构体。
 */
struct CddsNode {};

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
  bool is_loaning_available;                    ///< 是否支持贷款功能。
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
  bool is_loaning_available;                    ///< 是否支持贷款功能。
  user_callback_data_t user_callback_data;      ///< 用户回调数据结构体实例。
};

/**
 * @struct CddsEvent
 * @brief 一个继承自CddsEntity的事件结构体，用于ROS2的RMW层。
 */
struct CddsEvent : CddsEntity {
  rmw_event_type_t event_type;  ///< 事件类型
};

/**
 * @brief 获取RMW实现标识符。
 * @return 返回eclipse_cyclonedds_identifier字符串
 */
extern "C" const char *rmw_get_implementation_identifier() { return eclipse_cyclonedds_identifier; }

/*
===================================================================
===================================================================
===================================================================
*/

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
 * @brief 初始化 rmw_init_options_t 结构体
 *
 * @param[in,out] init_options 一个指向待初始化的 rmw_init_options_t 结构体的指针
 * @param[in] allocator 分配器，用于分配内存
 * @return RMW_RET_OK 如果成功，否则返回相应的错误代码
 */
extern "C" rmw_ret_t rmw_init_options_init(
    rmw_init_options_t *init_options, rcutils_allocator_t allocator) {
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  if (NULL != init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  init_options->instance_id = 0;
  init_options->implementation_identifier = eclipse_cyclonedds_identifier;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;
  init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
  init_options->enclave = NULL;
  init_options->security_options = rmw_get_zero_initialized_security_options();

  return RMW_RET_OK;
}

/**
 * @brief 处理内置主题端点的回调函数
 *
 * @param[in] reader 读取到的DDS实体
 * @param[in] impl rmw_context_impl_t类型的指针
 * @param[in] is_reader 是否为读者端点，true表示是读者端点，false表示是写者端点
 */
static void handle_builtintopic_endpoint(
    dds_entity_t reader, rmw_context_impl_t *impl, bool is_reader) {
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

/**
 * @brief 处理DCPSSubscription的函数
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

/**
 * @brief 处理DCPSPublication的函数
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
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      context, context->implementation_identifier, eclipse_cyclonedds_identifier, return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(context->impl, "expected initialized context", return nullptr);
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

  // 使用dds_create_topic_sertype接口创建主题
  tp = dds_create_topic_sertype(pp, name, &sertype, nullptr, nullptr, nullptr);
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
  RMW_CHECK_FOR_NULL_WITH_MSG(
      publisher, "publisher handle is null", return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
      serialized_message, "serialized message handle is null", return RMW_RET_INVALID_ARGUMENT);

  // 将publisher的data成员转换为CddsPublisher类型的指针
  auto pub = static_cast<CddsPublisher *>(publisher->data);

  // 将序列化消息转换为ddsi_serdata结构体
  struct ddsi_serdata *d = serdata_rmw_from_serialized_message(
      pub->sertype, serialized_message->buffer, serialized_message->buffer_length);

  // 将序列化消息写入DDS，并检查操作是否成功
  const bool ok = (dds_writecdr(pub->enth, d) >= 0);
  // 根据操作结果返回相应的状态码
  return ok ? RMW_RET_OK : RMW_RET_ERROR;
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
 * @brief 创建一个CddsPublisher对象
 *
 * @param[in] dds_ppant DDS参与者实体
 * @param[in] dds_pub DDS发布者实体
 * @param[in] type_supports 消息类型支持结构体指针
 * @param[in] topic_name 话题名称
 * 前几个都是 dds 中需要的
 * 后面几个是 ros 中需要的
 * @param[in] qos_policies QoS策略指针
 * @return 成功时返回一个新的CddsPublisher指针，失败时返回nullptr
 */
static CddsPublisher *create_cdds_publisher(
    dds_entity_t dds_ppant,
    dds_entity_t dds_pub,
    const rosidl_message_type_support_t *type_supports,
    const char *topic_name,
    const rmw_qos_profile_t *qos_policies) {
  //
  RET_NULL_OR_EMPTYSTR_X(topic_name, return nullptr);
  RET_NULL_X(qos_policies, return nullptr);
  const rosidl_message_type_support_t *type_support = get_typesupport(type_supports);
  RET_NULL_X(type_support, return nullptr);

  // 创建一个新的CddsPublisher对象
  CddsPublisher *pub = new CddsPublisher();  // 在前面定义的一个结构体
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
  // 设置是否支持贷款功能
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
 * @brief 创建发布者
 *
 * @param[in] dds_ppant DDS参与者实体
 * @param[in] dds_pub DDS发布者实体
 * @param[in] type_supports 消息类型支持
 * @param[in] topic_name 主题名称
 *
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

/**
 * @brief 创建一个ROS2发布者 (Create a ROS2 publisher)
 *
 * @param[in] node 指向要创建发布者的节点的指针 (Pointer to the node where the publisher will be
 * created)
 * @param[in] type_supports 消息类型支持结构体的指针 (Pointer to the message type support structure)
 * @param[in] topic_name 要发布的主题名称 (The name of the topic to publish)
 *
 * @param[in] qos_policies 指向QoS策略的指针 (Pointer to the QoS policies)
 * @param[in] publisher_options 指向发布者选项的指针 (Pointer to the publisher options)
 * @return 成功时返回一个指向新创建的发布者的指针，失败时返回nullptr (On success, returns a pointer
 * to the newly created publisher, otherwise nullptr)
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
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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
 * @brief 计算与给定发布者匹配的订阅者数量
 *
 * @param[in] publisher 发布者指针
 * @param[out] subscription_count 存储匹配订阅者数量的指针
 * @return rmw_ret_t 返回 RMW_RET_OK 表示成功，其他值表示失败
 */
extern "C" rmw_ret_t rmw_publisher_count_matched_subscriptions(
    const rmw_publisher_t *publisher, size_t *subscription_count) {
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      publisher, publisher->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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
 * @brief 计算给定主题上的发布者数量 (Count the number of publishers on a given topic)
 *
 * @param[in] node 指向rmw_node_t结构体的指针 (Pointer to an rmw_node_t structure)
 * @param[in] topic_name 要查询的主题名称 (The topic name to query)
 * @param[out] count 存储发布者数量的变量指针 (Pointer to a variable to store the publisher count)
 * @return rmw_ret_t 返回操作结果 (Return the operation result)
 */
extern "C" rmw_ret_t rmw_count_publishers(
    const rmw_node_t *node, const char *topic_name, size_t *count) {
  // 检查参数是否为空，如果为空则返回无效参数错误
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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
  // 检查是否为空
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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

/*
===============================================================
===============================================================
===============================================================
*/

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
  // 检查是否为空 (Check if the node is null)
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
      node, node->implementation_identifier, eclipse_cyclonedds_identifier,
      return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
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
  // 检查参数是否为空 (Check if node argument is NULL)
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

/*
================================================================
================================================================
================================================================
*/

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
