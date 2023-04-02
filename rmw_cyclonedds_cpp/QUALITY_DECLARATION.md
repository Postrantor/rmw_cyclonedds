---
@translate by openai@2023-03-31 20:21:56
---

This document is a declaration of software quality for the `rmw_cyclonedds_cpp` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

> 本文档是关于`rmw_cyclonedds_cpp`软件包的质量声明，基于 [REP-2004](https://www.ros.org/reps/rep-2004.html) 中的指南。

# `rmw_cyclonedds_cpp` Quality Declaration

The package `rmw_cyclonedds_cpp` claims to be in the **Quality Level 2** category.

> `rmw_cyclonedds_cpp`软件包声称属于**质量等级 2**类别。

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 2 in REP-2004](https://www.ros.org/reps/rep-2004.html).

> 以下是关于此声明的基本原理、说明和注意事项，按照[REP-2004 中的质量等级 2 软件包要求](https://www.ros.org/reps/rep-2004.html)中的每个要求进行组织。

## Version Policy [1]

### Version Scheme [1.i]

`rmw_cyclonedds_cpp` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning).

> 根据[ROS 2 开发人员指南]（https://docs.ros.org/ en/rolling/rolling/contributing/developer-guide.-guide.html#versioning），根据 ROS 核心包装的建议使用`rmw_cyclonedds_cpp“使用`semver'。

### Version Stability [1.ii]

`rmw_cyclonedds_cpp` is at a stable version, i.e. `>= 1.0.0`.

The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

> 当前版本可在其[package.xml](package.xml)中找到，更改历史记录可在其[CHANGELOG](CHANGELOG.rst)中找到。

### Public API Declaration [1.iii]

Public API for `rmw_cyclonedds_cpp` is declared through `rmw`.

> 公共 API 用于`rmw_cyclonedds_cpp`是通过`rmw`声明的。

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`rmw_cyclonedds_cpp` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

> `rmw_cyclonedds_cpp`不会在发布的 ROS 分布中破坏公共 API，即一旦发布 ROS 发行，就不会发行重大版本。

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`rmw_cyclonedds_cpp` contains C and C++ code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

> `rmw_cyclonedds_cpp`包含 C 和 C ++代码，因此必须与 ABI 稳定性有关，并且将在 ROS 分布中保持 ABI 稳定性。

## Change Control Process [2]

`rmw_cyclonedds_cpp` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#quality-practices).

> `rmw_cyclonedds_cpp`遵循[ROS 2 开发人员指南]（https://docs.ros.org/en/rolling/rolling/contributing/developer-guide.html#quality-protal-practices）中的ROS Core 软件包的建议指南。

### Change Requests [2.i]

This package requires that all changes occur through a pull request.

> 此程序包要求所有更改都通过拉取请求进行。

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy.

> 此软件包使用 DCO 作为确认贡献者来源策略的依据。

More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

> 更多信息可以在 [CONTRIBUTING](../CONTRIBUTING.md) 中找到。

### Peer Review Policy [2.iii]

Following the recommended guidelines for ROS Core packages, all pull requests must have at least 1 peer review.

> 遵循 ROS Core 软件包的推荐指南，所有拉取请求必须至少有 1 个同行评审。

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers).

> 所有 pull 请求必须通过所有[第一等级平台](https://www.ros.org/reps/rep-2000.html#support-tiers)上的 CI。

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

> 所有拉取请求在合并之前必须解决相关文档更改。

## Documentation [3]

### Feature Documentation [3.i]

`rmw_cyclonedds_cpp`'s features are documented through `rmw`.

It has a [feature list](https://docs.ros2.org/latest/api/rmw/) and each item in the list links to the corresponding feature documentation.

> 它有一个[功能列表](https://docs.ros2.org/latest/api/rmw/)，列表中的每个项目都链接到相应的功能文档。

There is documentation for all of the features, and new features require documentation before being added.

> 所有功能都有文档，新功能在添加之前需要文档。

### Public API Documentation [3.ii]

`rmw_cyclonedds_cpp`'s API is declared and documented by `rmw`.

It has embedded API documentation and it is generated using doxygen and is hosted [alongside the feature documentation](https://docs.ros2.org/latest/api/rmw/).

> 它包含了内嵌的 API 文档，使用 doxygen 生成，并托管在[功能文档旁边](https://docs.ros2.org/latest/api/rmw/)。

There is documentation for all of the public API, and new additions to the public API require documentation before being added.

> 所有公共 API 都有文档，并且在添加新的公共 API 之前需要编写文档。

### License [3.iii]

The license for `rmw_cyclonedds_cpp` is Apache 2.0, and a summary is in each source file, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the [LICENSE](../LICENSE) file.

> `rmw_cyclonedds_cpp`的许可证为 Apache 2.0，每个源文件中都有摘要，类型在[package.xml](package.xml)清单文件中声明，完整的许可证副本位于[ LICENSE](../LICENSE)文件中。

There is an automated test which runs a linter that ensures each file has a license statement.

> 有一个自动化测试，它运行一个 linter 来确保每个文件都有许可声明。

Most recent test results can be found [here](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rmw_cyclonedds_cpp/copyright/)

> 最近的测试结果可以在[这里](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rmw_cyclonedds_cpp/copyright/)找到。

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rmw_cyclonedds_cpp`.

> 版权持有者在`rmw_cyclonedds_cpp`的每个源代码文件中都提供了一个版权声明。

There is an automated test which runs a linter that ensures each file has at least one copyright statement.

> 有一个自动化测试会运行一个 linter，确保每个文件至少有一个版权声明。

The results of the test can be found [here](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rmw_cyclonedds_cpp/copyright/).

> 测试结果可以在[这里](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rmw_cyclonedds_cpp/copyright/)找到。

## Testing [4]

### Feature Testing [4.i]

All of the `rmw_cyclonedds_cpp` public features are ROS middleware features.

> 所有的`rmw_cyclonedds_cpp`公共功能都是 ROS 中间件功能。

Unit, integration, and system tests higher up in the stack, such as those found in [`test_rmw_implementation`](https://github.com/ros2/rmw_implementation/tree/master/test_rmw_implementation), [`test_rclcpp`](https://github.com/ros2/system_tests/tree/master/test_rclcpp), and [`test_communication`](https://github.com/ros2/system_tests/tree/master/test_communication) packages, provide feature coverage.

> 单元、集成和系统测试位于较高层的堆栈中，例如在[`test_rmw_implementation`](https://github.com/ros2/rmw_implementation/tree/master/test_rmw_implementation)、[`test_rclcpp`](https://github.com/ros2/system_tests/tree/master/test_rclcpp)和[`test_communication`](https://github.com/ros2/system_tests/tree/master/test_communication)包中找到的那些测试，提供功能覆盖。

Nightly CI jobs in [`ci.ros2.org`](https://ci.ros2.org/) and [`build.ros2.org`](https://build.ros2.org/), where `rmw_cyclonedds_cpp` is the default `rmw` implementation, thoroughly exercise this middleware.

> 在[ci.ros2.org](https://ci.ros2.org/)和[build.ros2.org](https://build.ros2.org/)上的每夜 CI 任务中，`rmw_cyclonedds_cpp`作为默认的`rmw`实现，对这个中间件进行了彻底的测试。

### Public API Testing [4.ii]

`rmw_cyclonedds_cpp` implements the ROS middleware public API.

Unit, integration, and system tests higher up in the stack, such as those found in [`test_rmw_implementation`](https://github.com/ros2/rmw_implementation/tree/master/test_rmw_implementation), [`test_rclcpp`](https://github.com/ros2/system_tests/tree/master/test_rclcpp), and [`test_communication`](https://github.com/ros2/system_tests/tree/master/test_communication) packages, ensure compliance with the ROS middleware API specification (see [`rmw`](https://github.com/ros2/rmw) package) and further extend coverage.

> 单元、集成和系统测试位于较高层次的堆栈中，例如在 [`test_rmw_implementation`]、 [`test_rclcpp`] 和 [`test_communication`] 包中找到的测试，确保了与 ROS 中间件 API 规范的符合性（请参见 [`rmw`] 包）并进一步扩展了覆盖范围。

New additions or changes to this API require tests before being added.

> 此 API 的新增功能或更改需要在添加之前进行测试。

### Coverage [4.iii]

`rmw_cyclonedds_cpp` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#code-coverage), and opts to use branch coverage instead of line coverage.

This includes:

> 这包括：

- tracking and reporting line coverage statistics

> - 跟踪和报告行覆盖率统计数据

- achieving and maintaining a reasonable branch line coverage (90-100%)

> 实现并维持合理的分支线覆盖率（90-100%）

- no lines are manually skipped in coverage calculations

> - 在覆盖率计算中不手动跳过任何行

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

> 更改前需要尽最大努力保持或提高覆盖率，但如果经过合理解释并得到审稿人的接受，覆盖率减少是允许的。

Current coverage statistics can be viewed [here](https://ci.ros2.org/job/ci_linux_coverage/lastSuccessfulBuild/cobertura/src_ros2_rmw_cyclonedds_rmw_cyclonedds_cpp_src/).

> 当前覆盖率统计数据可以在[这里] 查看。

A summary of how these statistics are calculated can be found in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#note-on-coverage-runs).

> 这些统计数据的计算摘要可以在 [ROS 2 Developer Guide] 中找到。

This package claims to meet the coverage requirements for the current quality level, even though it doesn't have 95% line coverage.

> 这个软件包声称满足当前质量等级的覆盖要求，尽管它的代码覆盖率没有达到 95%。

### Performance [4.iv]

`rmw_cyclonedds_cpp` does not currently have performance tests.

### Linters and Static Analysis [4.v]

`rmw_cyclonedds_cpp` uses and passes all the standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#linters-and-static-analysis).

Results of the nightly linter tests can be found [here](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rmw_cyclonedds_cpp).

> 夜间 linter 测试的结果可以在[这里]找到。

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]/[5.ii]

`rmw_cyclonedds_cpp` has the following runtime ROS dependencies:

- `rcpputils`: [QUALITY DECLARATION](https://github.com/ros2/rcpputils/blob/master/QUALITY_DECLARATION.md)
- `rcutils`: [QUALITY DECLARATION](https://github.com/ros2/rcutils/blob/master/QUALITY_DECLARATION.md)
- `rmw`: [QUALITY DECLARATION](https://github.com/ros2/rmw/blob/master/rmw/QUALITY_DECLARATION.md)
- `rmw_dds_common`: [QUALITY DECLARATION](https://github.com/ros2/rmw_dds_common/blob/master/rmw_dds_common/QUALITY_DECLARATION.md)
- `rosidl_runtime_c`: [QUALITY DECLARATION](https://github.com/ros2/rosidl/blob/master/rosidl_runtime_c/QUALITY_DECLARATION.md)
- `rosidl_runtime_cpp`: [QUALITY DECLARATION](https://github.com/ros2/rosidl/blob/master/rosidl_runtime_cpp/QUALITY_DECLARATION.md)
- `rosidl_typesupport_introspection_c`: [QUALITY DECLARATION](https://github.com/ros2/rosidl/blob/master/rosidl_typesupport_introspection_c/QUALITY_DECLARATION.md)
- `rosidl_typesupport_introspection_cpp`: [QUALITY DECLARATION](https://github.com/ros2/rosidl/blob/master/rosidl_typesupport_introspection_cpp/QUALITY_DECLARATION.md)
- `tracetools`: [QUALITY DECLARATION](https://gitlab.com/ros-tracing/ros2_tracing/-/blob/master/tracetools/QUALITY_DECLARATION.md)

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

> 它有多个"构建工具"依赖，这些依赖不会影响包的最终质量，因为它们不会对公共库 API 做出贡献。

It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

> 它还具有多个测试依赖项，这些依赖项不影响包的最终质量，因为它们仅用于构建和运行测试代码。

### Direct Runtime Non-ROS Dependencies [5.iii]

`rmw_cyclonedds_cpp` has the following runtime non-ROS dependencies:

- `cyclonedds`: Eclipse Cyclone DDS claims to be Quality Level 2. For more information, please refer to its [quality declaration document](https://github.com/eclipse-cyclonedds/cyclonedds/blob/releases/0.8.x/CYCLONEDDS_QUALITY_DECLARATION.md).

> `cyclonedds`: Eclipse Cyclone DDS 声称其为质量等级 2。欲了解更多信息，请参阅其[质量声明文档](https://github.com/eclipse-cyclonedds/cyclonedds/blob/releases/0.8.x/CYCLONEDDS_QUALITY_DECLARATION.md)。

## Platform Support [6]

`rmw_cyclonedds_cpp` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Currently nightly results can be seen here:

> 目前夜间结果可以在这里查看：

- [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rmw_cyclonedds_cpp/)
- [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rmw_cyclonedds_cpp/)
- [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rmw_cyclonedds_cpp/)
- [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rmw_cyclonedds_cpp/)

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).

> 此软件包符合[REP-2006](https://www.ros.org/reps/rep-2006.html)中的漏洞披露政策。

# Current status Summary

The chart below compares the requirements in the REP-2004 with the current state of the `rmw_cyclonedds_cpp` package.

> 下面的图表比较了 REP-2004 中的要求与`rmw_cyclonedds_cpp`包的当前状态。

| Number  | Requirement                                       | Current state |
| ------- | ------------------------------------------------- | ------------- |
| 1       | **Version policy**                                | ---           |
| 1.i     | Version Policy available                          | ✓             |
| 1.ii    | Stable version                                    | ✓             |
| 1.iii   | Declared public API                               | ✓             |
| 1.iv    | API stability policy                              | ✓             |
| 1.v     | ABI stability policy                              | ✓             |
| 1.vi\_  | API/ABI stable within ros distribution            | ✓             |
| 2       | **Change control process**                        | ---           |
| 2.i     | All changes occur on change request               | ✓             |
| 2.ii    | Contributor origin (DCO, CLA, etc)                | ✓             |
| 2.iii   | Peer review policy                                | ✓             |
| 2.iv    | CI policy for change requests                     | ✓             |
| 2.v     | Documentation policy for change requests          | ✓             |
| 3       | **Documentation**                                 | ---           |
| 3.i     | Per feature documentation                         | ✓             |
| 3.ii    | Per public API item documentation                 | ✓             |
| 3.iii   | Declared License(s)                               | ✓             |
| 3.iv    | Copyright in source files                         | ✓             |
| 3.v.a   | Quality declaration linked to README              | ✓             |
| 3.v.b   | Centralized declaration available for peer review | ✓             |
| 4       | **Testing**                                       | ---           |
| 4.i     | Feature items tests                               | ✓             |
| 4.ii    | Public API tests                                  | ✓             |
| 4.iii.a | Using coverage                                    | ✓             |
| 4.iii.a | Coverage policy                                   | ✓             |
| 4.iv.a  | Performance tests (if applicable)                 | ☓             |
| 4.iv.b  | Performance tests policy                          | ✓             |
| 4.v.a   | Code style enforcement (linters)                  | ✓             |
| 4.v.b   | Use of static analysis tools                      | ✓             |
| 5       | **Dependencies**                                  | ---           |
| 5.i     | Must not have ROS lower level dependencies        | ✓             |
| 5.ii    | Optional ROS lower level dependencies             | ✓             |
| 5.iii   | Justifies quality use of non-ROS dependencies     | ✓             |
| 6       | **Platform support**                              | ---           |
| 6.i     | Support targets Tier1 ROS platforms               | ✓             |
| 7       | **Security**                                      | ---           |
| 7.i     | Vulnerability Disclosure Policy                   | ✓             |
