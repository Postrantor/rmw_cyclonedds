---
@translate by baidu@2023-04-28 01:20:47
...

# Using Shared Memory with ROS 2

This rmw_cyclonedds implementation uses [cyclonedds](https://projects.eclipse.org/projects/iot.cyclonedds) which includes support for fast Shared Memory data transfer based on [iceoryx](https://projects.eclipse.org/projects/technology.iceoryx). Shared Memory is disabled by default but can be enabled easily by providing a cyclonedds.xml configuration file.

> 此 rmw_cyclonedds 实现使用[cyclonedds](https://projects.eclipse.org/projects/iot.cyclonedds)其中包括对基于[iceoryx]的快速共享内存数据传输的支持(https://projects.eclipse.org/projects/technology.iceoryx). 共享内存在默认情况下是禁用的，但可以通过提供 cycloneds.xml 配置文件轻松启用。

## Requirements

Currently Shared Memory transport is only supported on Linux. It is available in the rmw_cyclonedds implementation used by the ROS 2 Rolling or Humble Hawksbill distribution.

> 目前共享内存传输仅在 Linux 上受支持。它可以在 ROS 2 Rolling 或 Humble Hawksbill 发行版使用的 rmw_cyclonedds 实现中使用。

Note that the Shared Memory feature is not available on Windows.

## Installation

ROS 2 needs to be installed as described in [Installing ROS 2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Binary.html).

> ROS 2 需要按照[安装 ROS 2 滚动]中的说明进行安装(https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Binary.html).

It can also be build from sources directly [Building ROS 2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html). Using the latest ROS 2 installation [ROS 2 Humble](https://docs.ros.org/en/galactic/Releases.html) is also possible.

> 它也可以直接从来源构建[构建 ROS 2 滚动](https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html). 使用最新的 ROS 2 安装[ROS 2 Humble](https://docs.ros.org/en/galactic/Releases.html)也是可能的。

In both cases rmw_cyclonedds is built with Shared Memory support by default.

> 在这两种情况下，默认情况下，rmw_cyclonedds 都是使用共享内存支持构建的。

## Configuration

In your ROS 2 workspace `ros2_ws` create a configuration file `cyclonedds.xml` with the following content.

> 在 ROS 2 工作区“ros2_ws”中创建一个包含以下内容的配置文件“cycloneds.xml”。

```xml
    <?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
        <Domain id="any">
            <SharedMemory>
                <Enable>true</Enable>
                <LogLevel>info</LogLevel>
            </SharedMemory>
        </Domain>
    </CycloneDDS>
```

Enter your ROS 2 workspace and enable this configuration.

> 进入您的 ROS 2 工作区并启用此配置。

```bash
    cd ros2_ws
    export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

### Configuration file options

Enabling Shared Memory imposes additional restrictions on publishers and subscriptions in ROS applications if Shared Memory is used for communication. In this case the data is received using an internal queue (one queue per subscription).

> 如果共享内存用于通信，启用共享内存会对 ROS 应用程序中的发布者和订阅施加额外的限制。在这种情况下，使用内部队列（每个订阅一个队列）接收数据。
> Note that depending on the QoS settings Shared Memory might not be used (cf. [Restrictions](#Restrictions)).

The _Loglevel_ controls the output of the iceoryx runtime and can be set to verbose, debug, info, warn, error, fatal and off in order of decreasing output level.

> *Loglevel*控制 iceoryx 运行时的输出，可以按输出级别的递减顺序设置为 verbose、debug、info、warn、error、fatal 和 off。

The settings in the configuration file above are also the default values when left unspecified and setting them any higher is currently not possible.

> 当未指定时，上面配置文件中的设置也是默认值，并且当前不可能将其设置得更高。
> Note that further aligment with DDS QoS is work in progress. Currently these options exist alongside QoS settings as additional limits.

Further information about these options can be found in [manual of the cyclonedds project](https://github.com/eclipse-cyclonedds/cyclonedds/blob/iceoryx/docs/manual/options.md#cycloneddsdomainsharedmemory).

> 有关这些选项的更多信息，请参阅[Cycloneds 项目手册](https://github.com/eclipse-cyclonedds/cyclonedds/blob/iceoryx/docs/manual/options.md#cycloneddsdomainsharedmemory).

## Run an Example

We can now run the basic [talker/listener example](https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html#try-some-examples).

> 我们现在可以运行基本的[speaker/listenerexample](https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html#try-一些示例）。

In one terminal start the talker

> 在一个终端中启动扬声器

```bash
    `export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
    . ~/ros2_ws/install/local_setup.bash
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker
```

and in another terminal start the listener

> 在另一个终端启动监听器

```bash
    export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
    . ~/ros2_ws/install/local_setup.bash
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp listener
```

Note that we have to specify the middleware implementation with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

since otherwise `rmw fastrtps cpp` is used by default.

> 因为在其他情况下默认使用“rmw-fastrtps-cpp”。

If the Shared Memory configuration was successfully activated and recognized, you should get some output containing

> 如果已成功激活并识别共享内存配置，则应获得包含以下内容的一些输出

```bash
    [Warning]: RouDi not found - waiting
```

in both terminals. This is because with Shared Memory enabled, the iceoryx middleware daemon RouDi (abbreviation for Routing and Discovery) needs to be present and was not started yet.

> 在两个端子中。这是因为在启用共享内存的情况下，iceoryx 中间件守护程序 RouDi（路由和发现的缩写）需要存在，但尚未启动。

We can do so by simply running

> 我们可以通过简单的跑步来做到这一点

```bash
    . ~/ros2_ws/install/local_setup.bash
    iox-roudi
```

in a third terminal. Now the talker should start sending data to the listener.

> 在第三终端中。现在，说话者应该开始向监听器发送数据。

While this example uses shared memory, it does not benefit from zero-copy since the message type contains a string, which is a dynamically sized data type (cf. [Types](#Types)).

> 虽然此示例使用共享内存，但由于消息类型包含一个字符串，这是一种动态大小的数据类型（参见[Types]（#Types）），因此它不会从零复制中受益。

Note that RouDi is still required whenever we activate a Shared Memory configuration by exporting the configuration file (it is needed by the underlying cyclonedds implementation in this case).

> 请注意，每当我们通过导出配置文件激活共享内存配置时，仍然需要 roudi（在这种情况下，基础旋风的实现是需要的）。

We could also run the listener or talker without exporting the configuration file and it would still receive data (via network interface). By running both without exporting the configuration file we will not need to have RouDi running anymore (this is the regular ROS 2 setup).

> 我们也可以在不导出配置文件的情况下运行侦听器或发送器，它仍然会接收数据（通过网络接口）。通过在不导出配置文件的情况下运行两者，我们将不再需要运行 RouDi（这是常规的 ROS 2 设置）。

### Using Shared Memory in the example

To actually use Shared Memory the talker/listener example needs to be slightly rewritten to use a fixed size data type such as an unsigned integer. Adapting the publisher and subscription to use messages of type `std_msgs::msg::Uint32` instead leads to an example which uses Shared Memory to transport the data. See [Restrictions](#Restrictions) for further information about when Shared Memory transfer will be used.

> 要真正使用共享内存，需要稍微重写 speaker/listener 示例，以使用固定大小的数据类型，如无符号整数。将发布者和订阅调整为使用类型为“std_msgs::msg::Uint32”的消息会导致一个使用共享内存传输数据的示例。有关何时使用共享内存传输的更多信息，请参阅[限制]（#Restrictions）。

In the talker we use

> 在我们使用的谈话器中

```cpp
    std::unique_ptr<std_msgs::msg::Uint32> msg_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // ...
    pub_ = this->create_publisher<std_msgs::msg::Uint32>("chatter", qos);
```

and send the data with

> 并使用发送数据

```cpp
    msg_ = std::make_unique<std_msgs::msg::Uint32>();
    msg_->data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: 'Hello World: %lu'", msg_->data);
    pub_->publish(std::move(msg_));
```

Similarly on the listener side we also change the message type of the subscription

> 类似地，在侦听器端，我们还更改了订阅的消息类型

```cpp
    rclcpp::Subscription<std_msgs::msg::Uint32>::SharedPtr sub_;
    // ...
    sub_ = create_subscription<std_msgs::msg::Uint32>("chatter", 10, callback);
```

and can then receive the data with

> 然后可以用

```cpp
    auto callback =
      [this](const std::shared_ptr<std_msgs::msg::Uint32> msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [Hello World: %lu]", msg->data);
      };
```

Note that by just observing the received data it cannot be determined whether Shared Memory is actually used, but for large data the maximum data rate will be significantly faster compared to the network loopback interface.

> 请注意，仅通过观察接收到的数据，就无法确定是否实际使用共享内存，但是对于大数据，与网络环回接口相比，最大数据速率的速度将要快得多。

## Zero-Copy Publish-Subscribe Communication

Whether true zero-copy publish subscribe communication via Shared Memory is possible depends on several factors. Here zero-copy means no copy or serialization is performed.

> 是否可以通过共享内存进行真正的零拷贝发布-订阅通信取决于几个因素。这里零拷贝意味着不执行拷贝或序列化。

1. Shared Memory has to be enabled in the configuration.

> 1.必须在配置中启用共享内存。

2. The subscription has to be local on the same machine (i.e. same or different process, but no remote subscription).

> 2.订阅必须在同一台机器上是本地的（即相同或不同的进程，但没有远程订阅）。

3. The QoS settings admit Shared Memory transfer (cf. [QoS settings](#QoS-settings))

> 3.QoS 设置允许共享内存传输（参见[QoS 设置]（#QoS 设置））

If these conditions are satisfied we can publish the data in two different ways, but only by using the loan API we will achieve zero-copy transfer.

> 如果满足这些条件，我们可以用两种不同的方式发布数据，但**只有使用 loan API，我们才能实现零拷贝传输**。

### Regular Publish API

We first create a message, populate it before passing it to the publisher.

> 我们首先创建一个消息，在将其传递给发布者之前对其进行填充。

```cpp
    msg_ = std::make_unique<std_msgs::msg::Uint32>();
    msg_->data = count_++;
    pub_->publish(std::move(msg_));
```

If the Shared Memory transfer conditions are met, publish will internally loan a Shared Memory chunk from iceoryx and copy (only for fixed size types) or serialize the message payload into the memory chunk (non-fixed size types). Any connected subscription has read-only access to this message data.

> 如果满足共享内存传输条件，publish 将从 iceoryx 内部借用共享内存块并进行复制（仅适用于固定大小的类型）或将消息负载序列化到内存块中（非固定大小类型）。任何连接的订阅都可以只读访问此消息数据。

While this API will not allow true zero-copy transfer, it still will improve performance for sufficiently large message sizes since it bypasses the loopback interface and does not perform serialization. The actual size where it will outperform the loopback interface primarily depends on the actual hardware and system load.

> 虽然此 API 不允许真正的零拷贝传输，但它仍然可以提高足够大的消息大小的性能，因为它绕过了环回接口并且不执行序列化。它将优于环回接口的实际大小主要取决于实际的硬件和系统负载。

### Loan API

The loan API allows for true zero-copy transfer by directly consctructing the message data in Shared Memory. To do so, we first have to loan this Shared Memory chunk explicitly.

> 贷款 API 允许通过直接在共享内存中构造消息数据实现真正的零拷贝传输。要做到这一点，我们首先必须明确地借用这个共享内存块。

```cpp
    auto loaned_msg = pub_->borrow_loaned_message();
    loaned_msg.get().data = count_++;
    pub_->publish(std::move(loaned_msg));
```

The publish overload here does not need to copy the data to Shared Memory since it already resides there. Hence no copy is performed and the data is transferred to any subscription in constant time (i.e. independent of the message size). Depending on the data type there serialization into the loaned message may still be required and publish will therefore not be independent of message size. This is not the case for [fixed size](#Types) data types.

> 这里的发布重载不需要将数据复制到共享内存，因为它已经驻留在那里了。因此，不执行复制，并且数据在恒定时间内（即，与消息大小无关）传输到任何订阅。根据数据类型的不同，可能仍然需要序列化到借出的消息中，因此发布与消息大小无关。对于[固定大小]（#Types）数据类型，情况并非如此。

Note that to properly use loaning move semantics are essential, i.e. using `std::move` when publishing is required. This allows efficiently transferring ownership of the internal message data back to the middleware without copy overhead. As a consequence read access of `loaned_msg` after it was published is illegal (undefined behavior).

> 请注意，要正确使用借贷移动语义是必不可少的，即需要发布`时使用`sTD :: move'。这允许有效地将内部消息数据的所有权转移回中间件，而无需复制开销。结果，请阅读“ Loaned_msg”发布后的访问是非法的（不确定的行为）。

## Restrictions

The factors listed above govern whether Shared Memory is used, assuming it was enabled in the configuration. Here we summarize those restrictions and provide some additional details.

> 上面列出的因素决定了是否使用共享内存，假设在配置中启用了共享内存。在这里，我们总结了这些限制，并提供了一些额外的细节。

### Local Subscription

Only local subscriptions on the same machine will receive data via Shared Memory and RouDi is required to enable this. Remote subscriptions will always receive the data via the network interface. There can only be one Roudi process running per machine.

> 只有同一台计算机上的本地订阅才会通过共享内存接收数据，并且需要 RouDi 才能启用此功能。远程订阅将始终通过网络接口接收数据。每台机器只能运行一个 Roudi 进程。

### Types

#### Zero-copy

To benefit from zero-copy transfer, the message types used must be of a fixed size, i.e. the size can be determined at compile time and does not refer to memory

> 为了从零拷贝传输中获益，所使用的消息类型必须具有**固定大小，即大小可以在编译时确定，并且不涉及内存**

outside of the message struct itself. This means strings or variable length arrays of the [available message types](http://wiki.ros.org/msg) cannot be used.

> 在消息结构本身之外。这意味着[可用消息类型]的字符串或可变长度数组(http://wiki.ros.org/msg)无法使用。

Nesting types satisfying the fixed size restriction is also possible.

> 满足固定大小限制的嵌套类型也是可能的。

#### Shared Memory Serialization

All non-fixed size types like strings will be serialized into shared memory. This incurs much more overhead for serialization and deserialization but still avoids some copies that would be performed if the network stack is used (i.e. compared to the loopback interface in the same process). The runtime/latency benefit in this case is much smaller compared to zero-copy for fixed size types.

> 所有非固定大小的类型（如字符串）都将被序列化到共享内存中。这为序列化和反序列化带来了更多的开销，但仍然避免了在使用网络堆栈时会执行的一些复制（即，与同一进程中的环回接口相比）。与固定大小类型的零拷贝相比，这种情况下的运行时/延迟优势要小得多。

### QoS settings

Only a subset of QoS settings supports Shared Memory. Those are:

> 只有一部分 QoS 设置支持共享内存。这些是:

1. Liveliness: Automatic

> 1.生动:自动

2. Deadline: Infinity (0)

> 2.截止日期:无限（0）

3. Reliability: Reliable or Best Effort

> 3.可靠性:可靠或尽最大努力

4. Durability: Volatile or Transient Local

> 4.耐久性:挥发性或瞬态局部

5. History: Keep Last

> 5.历史:保持最后

The Keep Last history depth of a writer cannot be larger than the maximum capacity allowed by an iceoryx publisher

> 写入程序的保留上次历史记录深度不能大于 iceoryx 发布程序允许的最大容量

(currently 16). Otherwise the network stack is implictly used.

> （目前为 16）。否则，将隐含地使用网络堆栈。

The ROS 2 default settings Reliable, Volatile and Keep Last(10) are supported and applicable to a wide range of applications.

> ROS 2 默认设置 Reliable、Volatile 和 Keep Last（10）受支持，适用于各种应用程序。

### Number of subscriptions per Process

Currently a process can only have up to 127 subscriptions when Shared Memory is enabled.

> 当前，当启用共享内存时，一个进程最多只能有 127 个订阅。

### Number of active Loans

A single publisher can only hold up to 8 loaned messages simultaneously.

> 单个发布者最多只能同时保存 8 条借出的消息。

To obtain more loaned messages, it needs to publish some of the loaned messages.

> 为了获得更多的借出消息，它需要发布一些借出的消息。

### Iceoryx Shared Memory Configuration

Iceoryx uses configurable memory pools to define different sizes of memory chunks that will be used to store messages in Shared Memory. These will be obtained when data is sent via iceoryx, either when explicitly requested with the [Loan API](#Loan-API) or implicitly by the [Regular Publish API](#Regular-Publish-API).

> Iceoryx 使用可配置的内存池来定义不同大小的内存块，这些内存块将用于在共享内存中存储消息。当数据通过 iceoryx 发送时，可以通过[Loan API（#Loan-API）显式请求，也可以通过[Regular Publish API（#Regular-Publish-API）隐式请求。

Depending on the size and frequency of messages send, the default configuration may not be sufficient to guarantee that memory can be loaned and hence the data sent. In this case it might help to use a custom configuration for the shared memory pools to increase the available Shared Memory. The configuration options are described in the [iceoryx configuration guide](https://github.com/eclipse-iceoryx/iceoryx/blob/master/doc/website/advanced/configuration-guide.md).

> 根据发送消息的大小和频率，默认配置可能不足以保证可以借出内存，从而保证发送数据。在这种情况下，使用共享内存池的自定义配置来增加可用的共享内存可能会有所帮助。[iceoryx 配置指南]中介绍了配置选项(https://github.com/eclipse-iceoryx/iceoryx/blob/master/doc/website/advanced/configuration-guide.md).

Note that currently the internal loan call is blocking, which means if no memory is available it will not return (this will change in the future). This may happen if the configured memory is not sufficient for the overall system load, i.e. the memory needed was not available in the first place or is used by other samples which are currently read or written.

> 请注意，当前内部贷款通话正在阻止，这意味着如果没有可用的内存，它将无法返回（这将来会改变）。如果配置的内存不足以满足整体系统负载，即首先不可用所需的内存，或者当前读取或编写的其他样本使用所需的内存。

## Verifing Shared Memory Usage

It is currently not possible to accurately check whether Shared Memory transfer actually takes place for a specific subscription. If the conditions in [Restrictions](#Restrictions) are met this should be the case. If the data rate or latency are beyond what is achievable using regular network communication this is an indication that at least partially Shared Memory communication via iceoryx is being used.

> 目前无法准确检查共享内存传输是否真的发生在特定订阅中。如果满足[限制]（#Restrictions）中的条件，情况应该是这样的。如果数据速率或延迟超出了使用常规网络通信可以实现的范围，则表明正在使用通过 iceoryx 的至少部分共享内存通信。

Another way to check whether Shared Memory is used is running the iceoryx introspection client, which allows tracking of various statistics of the iceoryx communication, Shared Memory utilization being one of them.

> 另一种检查是否使用共享内存的方法是运行 iceoryx 自省客户端，该客户端允许跟踪 iceoryx 通信的各种统计信息，共享内存利用率就是其中之一。

### Building the iceoryx introspection

The introspection client is not build by default, so we need to do so manually.

> 内省客户端不是默认构建的，所以我们需要手动构建。

The introspection depends on iceoryx_utils and iceoryx_posh and we will build against the libraries already build by the ROS 2 installation.

> 内省依赖于 iceoryx_utils 和 iceoryx_posh，我们将根据 ROS 2 安装已经构建的库进行构建。

After installing ROS 2 as described in [Installation](#Installation) navigate to the ROS 2 workspace and execute

> 按照[安装]（#Installation）中的说明安装 ROS 2 后，导航到 ROS 2 工作区并执行

```bash
    cd ros2_ws
    . ~/ros2_ws/install/setup.bash
```

In the introspection folder of the iceoryx repository (part of the ROS 2 installation) run cmake followed by make to build the introspection.

> 在 iceoryx 存储库（ROS 2 安装的一部分）的内省文件夹中，运行 cmake，然后运行 make 来构建内省。

```bash
    cd src/eclipse-iceoryx/iceoryx/tools/introspection
    cmake -Bbuild
    cd build
    make
```

Afterwards the executable _iox-introspection-client_ should appear in the build folder.

> 之后，可执行文件*iox-introspection-client*应该出现在构建文件夹中。

### Using the iceoryx introspection

The introspection requires RouDi to be running since it receives the statistics information directly from the RouDi middleware daemon by subscribing to built-in topics.

> 内省需要运行 RouDi，因为它通过订阅内置主题直接从 RouDi 中间件守护进程接收统计信息。

Start RouDi and any applications of your system, e.g. [talker and listener](#Using-Shared-Memory-in-the-example).

> 启动 RouDi 和系统的任何应用程序，例如[talker and listener]（示例中的#使用共享内存）。

The introspection is able to track the processes using iceoryx, the subscriptions using Shared Memory (those using network cannot be found here) and Shared Memory utilization data (i.e. the number and size of allocations in Shared Memory).

> 内省能够跟踪使用 iceoryx 的进程、使用共享内存的订阅（此处找不到使用网络的订阅）和共享内存利用率数据（即共享内存中分配的数量和大小）。

Executing

> 正在执行

```bash
    ./iox-introspection-client --h
```

in the folder where the introspection was build provides us with a list of the various options. In the following we will display all statistics by running

> 在构建内省的文件夹中，为我们提供了各种选项的列表。在下文中，我们将通过运行

```bash
    ./iox-introspection-client --all
```

To verify that a particular connection is using Shared Memory we can proceed as follows. In the particular talker and listener example the internal ID of the talker and lister process should appear in the _Processes_ section of the introspection. These IDs are unique but unfortunately cannot be easily traced back to the process.

> 要验证特定连接是否使用共享内存，我们可以按以下步骤进行。在特定的说话者和监听器示例中，说话者和 lister 进程的内部 ID 应出现在内省的*Processes*部分。这些 ID 是唯一的，但不幸的是无法轻易追溯到流程。

From the _Connections_ section we can infer whether a specific topic is offered and whether a subscription to this topic exists. Note that AUTOSAR terminology is used for the displayed data but the topic information is available in the Instance and Event columns. Also note that the built-in topics used by the introspection are also listed.

> 从*Connections*部分，我们可以推断是否提供了特定的主题，以及是否存在对此主题的订阅。请注意，AUTOSAR 术语用于显示的数据，但主题信息在“实例”和“事件”列中可用。还要注意的是，内省使用的内置主题也列出了。

If a publisher which uses Shared Memory publishes data, the memory utilization changes. This is visible in the _MemPool Status_ section in the part listed in Segment Id: 1. If Shared Memory is used, the number of chunks in use should go up until a specific saturation point where it will stagnate or start to fluctuate slightly as chunks are freed by the suscribing party. This is related to the settings in the [configuration](#Configuration-file-options) as these essentially control how much Shared Memory a specific subscribtion can use at most.

> 如果使用共享内存的发布服务器发布数据，则内存利用率会发生变化。这在段 Id:1 中列出的零件的*MemPool Status*部分中可见。如果使用共享内存，则使用中的块的数量应该会增加，直到达到特定的饱和点，在该饱和点，随着支持方释放块，块的数量将停滞或开始略有波动。这与[配置]（#配置文件选项）中的设置有关，因为这些设置基本上控制了特定子分区最多可以使用多少共享内存。

If we can observe memory chunks being used this is a strong indication that data is transferred by Shared Memory. However, this does not rule out that part of the communication (e.g. other subscriptions) is using the regular network tranfer. This cannot be conclusive in general if multiple subscriptions exist since we do not know which of them transfer data via Shared Memory by observing this statistic in isolation.

> 如果我们能够观察到正在使用的内存块，这就有力地表明数据是由共享内存传输的。然而，这并不排除部分通信（例如其他订阅）使用常规网络传输。如果存在多个订阅，这通常不是决定性的，因为我们不知道其中哪一个订阅通过共享内存传输数据，只需单独观察这一统计数据。

The Min Free statistic is also important, as it counts the minimum number of chunks of a specific size that were free in a particular system execution (i.e. this can only decrease monotonically). If this is running low it indicates that there may be not enough Shared Memory available. This can be increased by using a different [Shared Memory configuration](#Iceoryx-Shared-Memory-Configuration).

> 最小空闲统计数据也很重要，因为它统计在特定系统执行中空闲的特定大小的块的最小数量（即，这只能单调减少）。如果内存不足，则表明可能没有足够的共享内存可用。这可以通过使用不同的[共享内存配置]（#Icoryx 共享内存配置）来增加。
