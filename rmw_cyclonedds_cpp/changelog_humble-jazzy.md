---
tag:
---

对比一下 humble <-> jazzy 之间的区别：

[diff](https://github.com/ros2/rmw_cyclonedds/compare/humble...rolling)

有一些有意思的修改：

- [ ] 在扩展 ros2 命令行的时候，考虑使用 best_available 策略？
      https://github.com/ros2/rmw_cyclonedds/commit/4e711c7451912b0237322a50b33ddb0516cef4be
- [ ] rmw_cyclonedds discovery
      这边提交还是很有价值的：https://github.com/ros2/rmw_cyclonedds/pull/429/files
      D:\Document\Skill\ROS2\rmw\rmw_cyclonedds\rmw_cyclonedds_cpp\src\rmw_node_3.md
- [ ] 避免直接使用 dds 公共公用 mutex (`#474 <https://github.com/ros2/rmw_cyclonedds/issues/474>`\_)

## change log

### 3.0.0 (2024-06-17)

- 使 rmw*service_server_is_available 返回 RMW_RET_INVALID_ARGUMENT (`#496 <https://github.com/ros2/rmw_cyclonedds/issues/496>`*)
- 在 rmw*create_node 中使用 rmw_namespace_validation_result_string() (`#497 <https://github.com/ros2/rmw_cyclonedds/issues/497>`*)
- 使 rmw*destroy_wait_set 返回 RMW_RET_INVALID_ARGUMENT (`#498 <https://github.com/ros2/rmw_cyclonedds/issues/498>`*)
- 在 message*info 中将 received_timestamp 设置为 system_clock::now() (`#491 <https://github.com/ros2/rmw_cyclonedds/issues/491>`*)

### 2.3.0 (2024-04-26)

### 2.2.0 (2024-04-09)

- 为发布/订阅序列化消息添加跟踪点 (`#485 <https://github.com/ros2/rmw_cyclonedds/issues/485>`\_)

### 2.1.1 (2024-03-28)

- 删除大量不必要的宏。(`#482 <https://github.com/ros2/rmw_cyclonedds/issues/482>`\_)
- 比较字符串内容和字符串指针地址。(`#481 <https://github.com/ros2/rmw_cyclonedds/issues/481>`\_)

### 2.1.0 (2024-01-24)

- 在 rmw*publish 追踪点中添加时间戳 (`#454 <https://github.com/ros2/rmw_cyclonedds/issues/454>`*)

### 2.0.0 (2023-11-06)

- 避免直接使用 dds 公共公用 mutex (`#474 <https://github.com/ros2/rmw_cyclonedds/issues/474>`\_)

### 1.10.0 (2023-10-04)

- 添加 rmw count clients,services impl (`#427 <https://github.com/ros2/rmw_cyclonedds/issues/427>`\_)
- 对 CMakeLists.txt 稍作修改。(`#468 <https://github.com/ros2/rmw_cyclonedds/issues/468>`\_)

### 1.9.0 (2023-08-21)

- 处理完错误后，清除错误。(`#464 <https://github.com/ros2/rmw_cyclonedds/issues/464>`\_)
- 工具借用信息发布代码路径

### 1.8.0 (2023-06-12)

- 为与跟踪点相关的宏使用 TRACETOOLS\_ 前缀 (`#450 <https://github.com/ros2/rmw_cyclonedds/issues/450>`\_)

### 1.7.0 (2023-04-27)

### 1.6.0 (2023-04-12)

- 动态订阅（BONUS：分配器）：rmw*cyclonedds (`#451 <https://github.com/ros2/rmw_cyclonedds/issues/451>`*)
- 为新的 rmw 接口添加存根 (`#447 <https://github.com/ros2/rmw_cyclonedds/issues/447>`\_)
- [rmw_cyclonedds] 改进对动态发现的处理 (`#429 <https://github.com/ros2/rmw_cyclonedds/issues/429>`\_)
- 调用 get*type_hash_func (`#448 <https://github.com/ros2/rmw_cyclonedds/issues/448>`*)
- 在发现中分配类型哈希(rep2011) (`#437 <https://github.com/ros2/rmw_cyclonedds/issues/437>`\_)
- 禁用不一致的主题事件。(``#444 <https://github.com/ros2/rmw_cyclonedds/issues/444>`\_)
- 实现匹配事件 (`#435 <https://github.com/ros2/rmw_cyclonedds/issues/435>`\_)
- 实现不一致主题。(`#431 <https://github.com/ros2/rmw_cyclonedds/issues/431>`\_)

### 1.5.1 (2023-02-14)

- 确保在 CHECK*TYPE_IDENTIFIER_MATCH 中添加分号。(`#432 <https://github.com/ros2/rmw_cyclonedds/issues/432>`*)
- [滚动] 更新维护者 - 2022-11-07 (`#428 <https://github.com/ros2/rmw_cyclonedds/issues/428>`\_)

### 1.5.0 (2022-11-02)

- 导出 CycloneDDS 依赖 (`#424 <https://github.com/ros2/rmw_cyclonedds/issues/424>`\_)
- 在访问对象前添加 NULL 检查。(`#423 <https://github.com/ros2/rmw_cyclonedds/issues/423>`\_)
- 增加 rmw*get_gid_for_client 植入 (`#402 <https://github.com/ros2/rmw_cyclonedds/issues/402>`*)
- 使 topic_name 成为常数 ref
- 创建主题失败时，在错误信息中添加主题名称

### 1.4.1 (2022-09-13)

- 改进创建主题失败时的错误信息 (`#405 <https://github.com/ros2/rmw_cyclonedds/issues/405>`\_)
- 将错误使用 %d 打印 uint32*t 改为 PRIu32 (`#253 <https://github.com/ros2/rmw_cyclonedds/issues/253>`*)
- 增加 cstring include。(`#393 <https://github.com/ros2/rmw_cyclonedds/issues/393>`\_)

### 1.4.0 (2022-05-03)

- 处理 "best*available "QoS 策略 (`#389 <https://github.com/ros2/rmw_cyclonedds/issues/389>`*)
