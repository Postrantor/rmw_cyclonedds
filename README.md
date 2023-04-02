---
translate by baidu@2023-04-28 01:34:54
...

# ROS 2 RMW for Eclipse Cyclone DDS

**Easy, fast, reliable, small [Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds) Tier 1 ROS middleware** for ROS 2. Make your **🐢 run like a 🚀** [Eclipse Cyclone DDS has great adopters](https://iot.eclipse.org/adopters/) and contributors in the ROS community and is an [Eclipse Foundation](https://www.eclipse.org) open source project of [Eclipse IoT](https://iot.eclipse.org) and [OpenADx](https://openadx.eclipse.org) (autonomous driving).

> **简单、快速、可靠、小巧[Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds)用于 ROS 2 的第 1 层 ROS 中间件**。让你的**🐢 像跑步一样 🚀** 【Eclipse Cyclone DDS 拥有出色的采用者】(https://iot.eclipse.org/adopters/)和ROS社区的贡献者，是一个[Eclipse基金会](https://www.eclipse.org)Eclipse IoT 的开源项目(https://iot.eclipse.org)和[OpenADx](https://openadx.eclipse.org)（自动驾驶）。

This package lets [_ROS 2_](https://docs.ros.org/en/rolling/) use [_Eclipse Cyclone DDS_](https://github.com/eclipse-cyclonedds/cyclonedds) as the underlying DDS implementation.

> 此程序包允许[_ROS 2_](https://docs.ros.org/en/rolling/)使用[Eclipse Cyclone DDS\_](https://github.com/eclipse-cyclonedds/cyclonedds)作为底层 DDS 实现。

Cyclone DDS is ready to use. It seeks to give the fastest, easiest, and most robust ROS 2 experience. Let the Cyclone blow you away!

> 旋风除尘器 DDS 已准备就绪。它旨在提供最快、最简单、最强大的 ROS 2 体验。让旋风把你吹走！

1. Install:

> 1.安装：

    ```
    apt install ros-eloquent-rmw-cyclonedds-cpp
    ```

    or

    ```
    apt install ros-dashing-rmw-cyclonedds-cpp
    ```

2. Set env variable and run ROS 2 apps as usual:

> 2.设置 env 变量并像往常一样运行 ROS 2 应用程序：

    `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

3. Confirm RMW: In Eloquent and later, to confirm which RMW you're using:

> 3.确认 RMW：在 Eloquent 和更高版本中，要确认您正在使用的 RMW：

    `ros2 doctor --report`

## Performance recommendations

With large samples (100s of kilobytes), excessive latency can be caused by running out of space in the OS-level receive buffer. For this reason, on Linux, we recommend increasing the buffer size:

> 对于大样本（100 千字节），操作系统级接收缓冲区的空间不足可能会导致过多的延迟。因此，在 Linux 上，我们建议增加缓冲区大小：

- Temporarily (until reboot): `sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608`

> -暂时（直到重新启动）：`sudo sysctl-w net.core.rem_max=8388608 net.core_rem_default=88388608`

- Permanently: `echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf`

> -永久：`echo“net.core.rem_max=8388608\nnet.core_rem_default=88388608\n”|sudo tee/etc/sysctl.d/60-cyconedds.conf`

## Debugging

So Cyclone isn't playing nice or not giving you the performance you had hoped for? That's not good... Please [file an issue against this repository](https://github.com/ros2/rmw_cyclonedds/issues/new)!

> “旋风”打得不好，还是没有给你带来你所希望的表现？这不好。。。请[针对此存储库提交问题](https://github.com/ros2/rmw_cyclonedds/issues/new)!

The `ddsperf` tool distributed with Cyclone DDS can be used to check that communication works _without_ ROS. Run `ddsperf sanity` on two different machines - if the "mean" value is above `100000us`, there are likely network issues.

> 与 Cyclone DDS 一起分发的“ddsperf”工具可用于检查通信是否在没有 ROS 的情况下工作。在两台不同的机器上运行“ddsperf sanity”-如果“平均”值高于“1000000us”，则可能存在网络问题。

If you're having trouble with nodes discovering others or can't use multicast _at all_ on your network setup, you can circumvent discovery:

> 如果您在节点发现其他节点时遇到问题，或者无法在网络设置中使用多播（_at all_），则可以规避发现：

`export CYCLONEDDS_URI='<Discovery><Peers><Peer Address='myroshost.local' /><Peer Address='myroshost2.local' /></></>'`

Here are some ways to generate additional debugging info that can help identify the problem faster, and are helpful on an issue ticket:

> 以下是一些生成额外调试信息的方法，这些信息有助于更快地识别问题，并对问题票据有帮助：

- Configure Cyclone to create richer debugging output:

> -配置 Cyclone 以创建更丰富的调试输出：

- To see the output live:

> -要实时查看输出：

    `export CYCLONEDDS_URI='<Tracing><Verbosity>trace</><Out>stderr</></>'`

- To send to `/var/log/`:

> -要发送到“/var/log/”：

    `export CYCLONEDDS_URI='<Tracing><Verbosity>trace</><Out>/var/log/cyclonedds.${CYCLONEDDS_PID}.log</></>'`

- Create a Wireshark capture:

> -创建 Wireshark 捕获：

`wireshark -k -w wireshark.pcap.gz`

> `wireshark-k-w wireshark.pcap.gz`

## Building from source and contributing

The following branches are actively maintained:

> 以下分支机构得到了积极维护：

- `master`, which targets the upcoming ROS version, [_Foxy_](https://docs.ros.org/en/rolling/Releases/Release-Foxy-Fitzroy.html)

> -“master”，针对即将推出的 ROS 版本[_Foxy_](https://docs.ros.org/en/rolling/Releases/Release-Foxy-Fitzroy.html)

- `dashing-eloquent`, which maintains compatibility with ROS releases [_Dashing_](https://docs.ros.org/en/rolling/Releases/Release-Dashing-Diademata.html) and [_Eloquent_](https://docs.ros.org/en/rolling/Releases/Release-Eloquent-Elusor.html)

> -“冲锋陷阵”，保持与 ROS 释放的兼容性[_dashing_](https://docs.ros.org/en/rolling/Releases/Release-Dashing-Diademata.html)和[_E 对话_](https://docs.ros.org/en/rolling/Releases/Release-Eloquent-Elusor.html)

If building ROS 2 from source ([ros2.repos](https://github.com/ros2/ros2/blob/master/ros2.repos)), you already have this package and Cyclone DDS:

> 如果从来源构建 ROS 2（[ros2.repos](https://github.com/ros2/ros2/blob/master/ros2.repos))，您已经拥有此软件包和 Cyclone DDS：

    cd /opt/ros/master
    rosdep install --from src -i
    colcon build
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

## Quality Declaration

This package claims to be in the **Quality Level 2** category, see the [Quality Declaration](./rmw_cyclonedds_cpp/QUALITY_DECLARATION.md) for more details.

> 此软件包声称属于**质量级别 2**类别，有关更多详细信息，请参阅[质量声明]（./rmw_cyclonedds_cpp/QULITY_Declaration.md）。
