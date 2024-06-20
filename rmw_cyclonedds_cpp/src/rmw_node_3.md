---
tag: #domain #discovery
---

## 对比 humble <-> jazzy 中 domain discovery 的区别

这点在 ros2 的变更文档中也有更新：[rmw_cyclonedds] Improve handling of dynamic discovery (#429)

[Improve handling of dynamic discovery by gbiggs](https://github.com/ros2/rmw_cyclonedds/pull/429/files)
[Iron Irwini Changelog](https://docs.ros.org/en/humble/Releases/Iron-Irwini-Complete-Changelog.html)

## **You:**

这是 rmw_cyclonedds 的一段代码，请详细分析其中的逻辑，并添加详细的注释：

```cpp
static bool check_create_domain(dds_domainid_t did, rmw_discovery_options_t * discovery_options)
{
  std::lock_guard<std::mutex> lock(gcdds().domains_lock);
  /* return true: n_nodes incremented, discovery params set correctly, domain exists
     "      false: n_nodes unchanged, domain left intact if it already existed */
  CddsDomain & dom = gcdds().domains[did];
  if (dom.refcount != 0) {
    /* Discovery parameters must match */
    bool options_equal = false;
    const auto rc =
      rmw_discovery_options_equal(discovery_options, &dom.discovery_options, &options_equal);
    if (RMW_RET_OK != rc) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "check_create_domain: unable to check if discovery options are equal: %i",
        rc);
      return false;
    }
    if (options_equal) {
      dom.refcount++;
      return true;
    } else {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "check_create_domain: attempt at creating nodes in the same domain with different "
        "discovery parameters");
      return false;
    }
  } else {
    dom.refcount = 1;
    dom.discovery_options = *discovery_options;

    bool add_localhost_as_static_peer;
    bool add_static_peers;
    bool disable_multicast;

    switch (discovery_options->automatic_discovery_range) {
      case RMW_AUTOMATIC_DISCOVERY_RANGE_NOT_SET:
        RMW_SET_ERROR_MSG("automatic discovery range must be set");
        return false;
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET:
        add_localhost_as_static_peer = false;
        add_static_peers = true;
        disable_multicast = false;
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_SYSTEM_DEFAULT:
        /* Avoid changing DDS discovery options*/
        add_localhost_as_static_peer = false;
        add_static_peers = false;
        disable_multicast = false;
        if (discovery_options->static_peers_count > 0) {
          RCUTILS_LOG_WARN_NAMED(
            "rmw_cyclonedds_cpp",
            "check_create_domain: %lu static peers were specified, but discovery is "
            "set to use the RMW implementation default, so these static peers will be ignored.",
            discovery_options->static_peers_count);
        }
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_LOCALHOST:
        /* Automatic discovery on localhost only */
        add_localhost_as_static_peer = true;
        add_static_peers = true;
        disable_multicast = true;
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_OFF:
        /* Automatic discovery off: disable multicast entirely. */
        add_localhost_as_static_peer = false;
        add_static_peers = false;
        disable_multicast = true;
        if (discovery_options->static_peers_count > 0) {
          RCUTILS_LOG_WARN_NAMED(
            "rmw_cyclonedds_cpp",
            "check_create_domain: %lu static peers were specified, but discovery is "
            "turned off, so these static peers will be ignored.",
            discovery_options->static_peers_count);
        }
        break;
      default:
        RMW_SET_ERROR_MSG("automatic_discovery_range is an unknown value");
        return false;
        break;
    }

    std::string config;
    if (
      add_localhost_as_static_peer ||
      add_static_peers ||
      disable_multicast)
    {
      config = "<CycloneDDS><Domain>";

      if (disable_multicast) {
        config += "<General><AllowMulticast>false</AllowMulticast></General>";
      }

      const bool discovery_off =
        disable_multicast && !add_localhost_as_static_peer && !add_static_peers;
      if (discovery_off) {
        /* This means we have an OFF range, so we should use the domain tag to
          block all attemtps at automatic discovery. Another participant would
          need to use this exact same domain tag, down to the PID, to discover
          the endpoints of this node.

          Setting ParticipantIndex to none eliminates the 119 limit on the number
          of participants on a machine.
          */
        config += "<Discovery><ParticipantIndex>none</ParticipantIndex>";
        config += "<Tag>ros_discovery_off_" + std::to_string(rcutils_get_pid()) + "</Tag>";
      } else {
        config += "<Discovery><ParticipantIndex>auto</ParticipantIndex>";
        // This controls the number of participants that can be discovered on a single host,
        // which is roughly equivalent to the number of ROS 2 processes.
        // If it's too small then we won't connect to all participants.
        // If it's too large then we will send a lot of announcement traffic.
        // The default number here is picked arbitrarily.
        config += "<MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>";
      }

      if (  // NOLINT
        (add_static_peers && discovery_options->static_peers_count > 0) ||
        add_localhost_as_static_peer)
      {
        config += "<Peers>";

        if (add_localhost_as_static_peer) {
          config += "<Peer address=\"localhost\"/>";
        }

        for (size_t ii = 0; ii < discovery_options->static_peers_count; ++ii) {
          config += "<Peer address=\"";
          config += discovery_options->static_peers[ii].peer_address;
          config += "\"/>";
        }
        config += "</Peers>";
      }

      /* NOTE: Empty configuration fragments are ignored, so it is safe to
        unconditionally append a comma. */
      config += "</Discovery></Domain></CycloneDDS>,";
    }

    /* Emulate default behaviour of Cyclone of reading CYCLONEDDS_URI */
    const char * get_env_error;
    const char * config_from_env;
    if ((get_env_error = rcutils_get_env("CYCLONEDDS_URI", &config_from_env)) == nullptr) {
      config += std::string(config_from_env);
    } else {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "rmw_create_node: failed to retrieve CYCLONEDDS_URI environment variable, error %s",
        get_env_error);
      gcdds().domains.erase(did);
      return false;
    }

    RCUTILS_LOG_DEBUG_NAMED("rmw_cyclonedds_cpp", "Config XML is %s", config.c_str());

    if ((dom.domain_handle = dds_create_domain(did, config.c_str())) < 0) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "rmw_create_node: failed to create domain, error %s", dds_strretcode(dom.domain_handle));
      gcdds().domains.erase(did);
      return false;
    } else {
      return true;
    }
  }
}
```

## **ChatGPT:**

以下是对 `check_create_domain` 函数的详细分析和注释：

```cpp
static bool check_create_domain(dds_domainid_t did, rmw_discovery_options_t * discovery_options)
{
  // 使用互斥锁保护全局域结构体的操作
  std::lock_guard<std::mutex> lock(gcdds().domains_lock);

  // 获取指定域ID对应的域结构体
  CddsDomain & dom = gcdds().domains[did];

  // 检查域是否已存在（引用计数不为0）
  if (dom.refcount != 0) {
    // 检查发现参数是否一致
    bool options_equal = false;
    const auto rc = rmw_discovery_options_equal(discovery_options, &dom.discovery_options, &options_equal);

    // 如果比较发现参数时出错，记录错误日志并返回false
    if (RMW_RET_OK != rc) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "check_create_domain: unable to check if discovery options are equal: %i",
        rc);
      return false;
    }

    // 如果发现参数一致，增加引用计数并返回true
    if (options_equal) {
      dom.refcount++;
      return true;
    } else {
      // 如果发现参数不一致，记录错误日志并返回false
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "check_create_domain: attempt at creating nodes in the same domain with different "
        "discovery parameters");
      return false;
    }
  } else {
    // 如果域不存在（引用计数为0），初始化域的引用计数和发现参数
    dom.refcount = 1;
    dom.discovery_options = *discovery_options;

    // 定义局部变量，用于配置发现选项
    bool add_localhost_as_static_peer;
    bool add_static_peers;
    bool disable_multicast;

    // 根据自动发现范围设置发现选项
    switch (discovery_options->automatic_discovery_range) {
      case RMW_AUTOMATIC_DISCOVERY_RANGE_NOT_SET:
        // 如果自动发现范围未设置，记录错误信息并返回false
        RMW_SET_ERROR_MSG("automatic discovery range must be set");
        return false;
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET:
        // 设置发现范围为子网
        add_localhost_as_static_peer = false;
        add_static_peers = true;
        disable_multicast = false;
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_SYSTEM_DEFAULT:
        // 使用系统默认发现范围
        add_localhost_as_static_peer = false;
        add_static_peers = false;
        disable_multicast = false;
        if (discovery_options->static_peers_count > 0) {
          RCUTILS_LOG_WARN_NAMED(
            "rmw_cyclonedds_cpp",
            "check_create_domain: %lu static peers were specified, but discovery is "
            "set to use the RMW implementation default, so these static peers will be ignored.",
            discovery_options->static_peers_count);
        }
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_LOCALHOST:
        // 仅在本地主机上进行自动发现
        add_localhost_as_static_peer = true;
        add_static_peers = true;
        disable_multicast = true;
        break;
      case RMW_AUTOMATIC_DISCOVERY_RANGE_OFF:
        // 禁用自动发现
        add_localhost_as_static_peer = false;
        add_static_peers = false;
        disable_multicast = true;
        if (discovery_options->static_peers_count > 0) {
          RCUTILS_LOG_WARN_NAMED(
            "rmw_cyclonedds_cpp",
            "check_create_domain: %lu static peers were specified, but discovery is "
            "turned off, so these static peers will be ignored.",
            discovery_options->static_peers_count);
        }
        break;
      default:
        // 如果发现范围值未知，记录错误信息并返回false
        RMW_SET_ERROR_MSG("automatic_discovery_range is an unknown value");
        return false;
        break;
    }

    // 构建配置字符串
    std::string config;
    if (
      add_localhost_as_static_peer ||
      add_static_peers ||
      disable_multicast)
    {
      config = "<CycloneDDS><Domain>";

      if (disable_multicast) {
        config += "<General><AllowMulticast>false</AllowMulticast></General>";
      }

      const bool discovery_off =
        disable_multicast && !add_localhost_as_static_peer && !add_static_peers;
      if (discovery_off) {
        // 如果禁用了发现功能，配置阻止所有自动发现尝试
        config += "<Discovery><ParticipantIndex>none</ParticipantIndex>";
        config += "<Tag>ros_discovery_off_" + std::to_string(rcutils_get_pid()) + "</Tag>";
      } else {
        config += "<Discovery><ParticipantIndex>auto</ParticipantIndex>";
        // 设置单个主机上可发现的参与者数量
        config += "<MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>";
      }

      if (
        (add_static_peers && discovery_options->static_peers_count > 0) ||
        add_localhost_as_static_peer)
      {
        config += "<Peers>";

        if (add_localhost_as_static_peer) {
          config += "<Peer address=\"localhost\"/>";
        }

        for (size_t ii = 0; ii < discovery_options->static_peers_count; ++ii) {
          config += "<Peer address=\"";
          config += discovery_options->static_peers[ii].peer_address;
          config += "\"/>";
        }
        config += "</Peers>";
      }

      config += "</Discovery></Domain></CycloneDDS>,";
    }

    // 模拟CycloneDDS读取CYCLONEDDS_URI环境变量的默认行为
    const char * get_env_error;
    const char * config_from_env;
    if ((get_env_error = rcutils_get_env("CYCLONEDDS_URI", &config_from_env)) == nullptr) {
      config += std::string(config_from_env);
    } else {
      // 记录错误日志并返回false
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "rmw_create_node: failed to retrieve CYCLONEDDS_URI environment variable, error %s",
        get_env_error);
      gcdds().domains.erase(did);
      return false;
    }

    RCUTILS_LOG_DEBUG_NAMED("rmw_cyclonedds_cpp", "Config XML is %s", config.c_str());

    // 尝试创建域
    if ((dom.domain_handle = dds_create_domain(did, config.c_str())) < 0) {
      // 如果创建域失败，记录错误日志并返回false
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_cyclonedds_cpp",
        "rmw_create_node: failed to create domain, error %s", dds_strretcode(dom.domain_handle));
      gcdds().domains.erase(did);
      return false;
    } else {
      // 如果创建域成功，返回true
      return true;
    }
  }
}
```

### 逻辑分析

1. **锁保护**：函数首先获取一个互斥锁以保护对全局域结构体的访问，确保线程安全。
2. **检查域是否已存在**：
   - 如果域已存在（引用计数不为 0），检查新提供的发现参数是否与现有参数一致。
   - 如果参数一致，增加引用计数并返回 true。
   - 如果参数不一致，记录错误并返回 false。
3. **创建新域**：
   - 如果域不存在，初始化引用计数和发现参数。
   - 根据自动发现范围配置相关选项。
   - 构建域的配置字符串。
   - 尝试从环境变量获取额外的配置。
   - 尝试创建域，如果失败则记录错误并返回 false，如果成功则返回 true。
