// Copyright 2021 Ericsson AB
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

#include "rmw/error_handling.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/rmw.h"
#include "rmw/types.h"

/**
 * @brief 本文件包含两个函数，用于获取发布者和订阅者的网络流端点。
 */

extern "C" {

/**
 * @brief 获取发布者的网络流端点。
 *
 * @param[in] publisher 指向要查询的rmw_publisher_t结构体的指针。
 * @param[in] allocator 用于分配内存的rcutils_allocator_t结构体的指针。
 * @param[out] network_flow_endpoint_array
 * 存储网络流端点信息的rmw_network_flow_endpoint_array_t结构体的指针。
 * @return RMW_RET_UNSUPPORTED 表示该函数尚未实现。
 */
rmw_ret_t rmw_publisher_get_network_flow_endpoints(
    const rmw_publisher_t* publisher,
    rcutils_allocator_t* allocator,
    rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
  (void)publisher;                    // 忽略publisher参数
  (void)allocator;                    // 忽略allocator参数
  (void)network_flow_endpoint_array;  // 忽略network_flow_endpoint_array参数
  RMW_SET_ERROR_MSG("rmw_publisher_get_network_flow_endpoints not implemented");  // 设置错误消息
  return RMW_RET_UNSUPPORTED;  // 返回不支持的状态
}

/**
 * @brief 获取订阅者的网络流端点。
 *
 * @param[in] subscription 指向要查询的rmw_subscription_t结构体的指针。
 * @param[in] allocator 用于分配内存的rcutils_allocator_t结构体的指针。
 * @param[out] network_flow_endpoint_array
 * 存储网络流端点信息的rmw_network_flow_endpoint_array_t结构体的指针。
 * @return RMW_RET_UNSUPPORTED 表示该函数尚未实现。
 */
rmw_ret_t rmw_subscription_get_network_flow_endpoints(
    const rmw_subscription_t* subscription,
    rcutils_allocator_t* allocator,
    rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
  (void)subscription;                 // 忽略subscription参数
  (void)allocator;                    // 忽略allocator参数
  (void)network_flow_endpoint_array;  // 忽略network_flow_endpoint_array参数
  RMW_SET_ERROR_MSG("rmw_subscription_get_network_flow_endpoints not implemented");  // 设置错误消息
  return RMW_RET_UNSUPPORTED;  // 返回不支持的状态
}

}  // extern "C"
