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
#ifndef RMW_VERSION_TEST_HPP_
#define RMW_VERSION_TEST_HPP_

/**
 * @file
 * @brief 本文件包含一个宏，用于检查RMW版本是否大于等于给定的主要、次要和补丁版本。
 */

// 判断RMW版本是否大于等于给定的主要(major)、次要(minor)和补丁(patch)版本
#define RMW_VERSION_GTE(major, minor, patch)                                                                                \
  (major < RMW_VERSION_MAJOR || /* 如果给定的主要版本小于RMW的主要版本，则为真 */                       \
   (major == RMW_VERSION_MAJOR && /* 如果给定的主要版本等于RMW的主要版本，则继续比较次要版本 */   \
    (minor < RMW_VERSION_MINOR || /* 如果给定的次要版本小于RMW的次要版本，则为真 */                     \
     (minor == RMW_VERSION_MINOR && /* 如果给定的次要版本等于RMW的次要版本，则继续比较补丁版本 */ \
      patch <= RMW_VERSION_PATCH)))) /* 如果给定的补丁版本小于等于RMW的补丁版本，则为真 */

#endif                               // RMW_VERSION_TEST_HPP_
