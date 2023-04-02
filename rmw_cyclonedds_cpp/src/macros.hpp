// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef MACROS_HPP_
#define MACROS_HPP_

#include <limits>
#include <string>
// 使用宏定义SPECIALIZE_GENERIC_C_SEQUENCE，用于生成特定类型的C序列结构体和相关函数
#define SPECIALIZE_GENERIC_C_SEQUENCE(C_NAME, C_TYPE) \
  template <>  // 定义一个模板特化结构体GenericCSequence，用于处理特定类型C_TYPE的序列
struct GenericCSequence<C_TYPE> {  // 使用type别名表示rosidl_runtime_c__##C_NAME##__Sequence类型
  using type = rosidl_runtime_c__##C_NAME##__Sequence;

  // 定义一个静态成员函数fini，用于释放type类型的数组内存
  static void fini(type* array) {  // 调用rosidl_runtime_c__##C_NAME##__Sequence__fini函数释放内存
    rosidl_runtime_c__##C_NAME##__Sequence__fini(array);
  }

  // 定义一个静态成员函数init，用于初始化type类型的数组，并指定大小为size
  static bool init(
      type* array,
      size_t size) {  // 调用rosidl_runtime_c__##C_NAME##__Sequence__init函数进行初始化，并返回结果
    return rosidl_runtime_c__##C_NAME##__Sequence__init(array, size);
  }
};

// 结束宏定义保护
#endif  // MACROS_HPP_
