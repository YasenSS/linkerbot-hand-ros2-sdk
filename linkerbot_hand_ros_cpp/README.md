# Linkerbot Hand C++ ROS 2 驱动

`linkerbot_hand_ros_cpp` 使用 `rclcpp_lifecycle` 调用 C++17 Hand SDK，与 Python ROS2
驱动共用 `linkerbot_hand_msgs`。公开接口按 Python ROS2 包补齐；SDK 或具体型号不支持的
功能保留入口并明确报错。详细对应关系见 [接口对照](docs/python_ros2_parity.md)。

## 构建

以 Ubuntu 24.04 / ROS Jazzy 为例，目录应为：

```text
sdk-build/
├── linkerbot-hand-cpp/
├── linkerbot-transport-cpp/
└── linkerbot-hand-ros-sdk/
```

在 ROS 仓库根目录执行。这里统一使用 `install/cpp`，构建完成后才会生成 setup 文件：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths linkerbot_hand_msgs linkerbot_hand_ros_cpp \
  --ignore-src -y --rosdistro "$ROS_DISTRO"

CMAKE_BUILD_PARALLEL_LEVEL=4 colcon --log-base log/cpp build \
  --base-paths linkerbot_hand_msgs linkerbot_hand_ros_cpp \
  --packages-up-to linkerbot_hand_ros_cpp \
  --build-base build/cpp --install-base install/cpp \
  --cmake-args \
    -DLINKERBOT_HAND_SOURCE_DIR="$(realpath ../linkerbot-hand-cpp)" \
    -DLINKERBOT_COMM_SOURCE_DIR="$(realpath ../linkerbot-transport-cpp)" \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source install/cpp/setup.bash
ros2 pkg prefix linkerbot_hand_ros_cpp
```

也可使用已安装的 C++ SDK 0.3.x：将 SDK 安装前缀加入 `CMAKE_PREFIX_PATH`，并将
`LINKERBOT_HAND_SOURCE_DIR` 置空。通信 SDK 导出的 CMake 包名是 `linkerbot-comm`。
本包不依赖 Python Hand SDK。ROS launch 及附带的纯 ROS 客户端工具使用系统 Python；
设备 I/O 和 ROS 驱动节点使用 C++。Humble 需要在对应环境另行验证。

## L30 角度控制

先根据设备要求配置 `can0` 的 CAN FD 和波特率。每个新终端均需加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/linkerbot-hand-ros-sdk/install/cpp/setup.bash

ros2 launch linkerbot_hand_ros_cpp hand.launch.py \
  model:=l30 side:=not_applicable channel:=can0
```

默认自动 configure、activate。启动和恢复不会重放运动目标，也不自动执行标定。
在另一终端查看设备能力、新鲜反馈和实现情况：

```bash
ros2 topic echo --once --qos-durability transient_local /linkerbot_hand/capabilities
ros2 topic echo --once --qos-reliability best_effort /linkerbot_hand/state/angles
ros2 topic echo --once --qos-durability transient_local /linkerbot_hand/interface_support
```

L30 有 17 个关节，顺序以当前 `capabilities.joints` 为准。核对目标后发送一次 percent
命令（下例会移动设备）：

```bash
ros2 run linkerbot_hand_ros_cpp angle_command \
  --values 50 50 50 50 50 50 50 50 50 50 50 50 50 50 50 50 50
```

默认 `--values` 为 percent；加 `--raw` 使用协议原值，不能将角度度数直接当作 percent。
命令状态到达协议完成阶段不代表机械到位，需检查同一连接代的新鲜反馈。
还提供 `angle_controller`、`state_monitor`、`frequency_benchmark`，使用 `--help` 查看参数。
这些工具只收发 ROS 消息，不打开设备。Python 业务代码可使用
`linkerbot_hand_ros_cpp.client.HandCommandClient`；未新增独立 C++ 客户端库或示例包。

## 生命周期与参数

```bash
ros2 launch linkerbot_hand_ros_cpp hand.launch.py autostart:=false
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
ros2 lifecycle set /linkerbot_hand deactivate
ros2 lifecycle set /linkerbot_hand cleanup
```

configure 不打开设备；activate 执行身份校验、初始化及轮询。deactivate 停止接纳命令并关闭
SDK；cleanup 释放设备。公开 Topic/Service 在未配置和 cleanup 后仍保留。未激活时的
业务调用返回 `device_not_ready`。手动 deactivate 不会自动重新激活。

启动参数支持 Python ROS2 的全部公开参数名，另保留 `channel`、`address`、
`host_address`、`command_timeout_s`、`angle_mapping_path` 等 C++ 参数。
使用 `ros2 launch linkerbot_hand_ros_cpp hand.launch.py --show-args` 查看入口；
`params_file` 可加载 YAML。显式 launch 参数覆盖 YAML。

各 `*_hz`、`publish_rate_hz`、`diagnostic_rate_hz` 可运行时更新，0 表示暂停对应项。
型号、端点、队列及 SDK 连接配置在 cleanup 后才能更改。`angle_feedback` 当前仅支持
`query_only`，其他策略明确拒绝。轮询非零范围为 `0.001..1000 Hz`，按 SDK 整数毫秒
周期向上取整。发布只发送新样本或有效性变化；设备采样频率由实际传输决定。

```bash
ros2 param set /linkerbot_hand current_hz 10.0
ros2 param set /linkerbot_hand faults_hz 1.0
ros2 param set /linkerbot_hand force_fn_hz 10.0
```

具体型号不支持的轮询会被拒绝。`auto_reconnect`、重连退避、恢复次数及重试间隔已接入。
独立 `recovery_timeout_s`、运行时 mailbox/route 容量缺少 C++ SDK 配置接口，保留参数名及
兼容默认值；请求非默认配置或动态设置时返回 `unsupported_operation`，不会假装已生效。

## 连接、发现与多手

手动连接支持 SocketCAN / SocketCAN FD、MiniCANFD，以及经典 CAN 的 PCAN。
兼容 Python ROS 的 `transport_json` 字段；`python_can` 表示配置格式，其可用子集映射到
C++ transport，不会导入 Python SDK。任意 Python 插件及未映射选项明确拒绝。

```bash
ros2 launch linkerbot_hand_ros_cpp hand.launch.py model:=l30 side:=not_applicable \
  transport_json:='{"backend":"socketcan","channel":"can0"}'

# 单手发现，必须唯一匹配；在 activate 才执行扫描
ros2 launch linkerbot_hand_ros_cpp hand.launch.py discover:=true channels:=can0

# 静态多手，示例默认 autostart=false，不连接设备
ros2 launch linkerbot_hand_ros_cpp multi_hand.launch.py

# 多手发现并保存部署配置（不会覆盖已有文件）
ros2 launch linkerbot_hand_ros_cpp multi_hand.launch.py \
  discover:=true channels:=can0,can1 output:=/tmp/hands.yaml

# 重新发现并按保存的身份和端点绑定
ros2 launch linkerbot_hand_ros_cpp multi_hand.launch.py config:=/tmp/hands.yaml
```

发现可通过 `discovery_config` 配置 SocketCAN、PCAN、SLCAN、MiniCANFD 的原生 SDK
后端；模板位于 [config](config)。部署文件使用 Python 相同的 schema/version。
多手各自有 namespace，全组 configure 成功后才逐一 activate，失败逆序回滚。
每个物理通道只允许一只手；不要在同一设备上同时启动两套驱动。

## ROS 接口与报错

公开接口使用 Python 相同的名称、消息类型和主要 QoS：命令和状态 best effort/depth 1，
capabilities/device_info transient local/depth 1，command/status reliable/depth 32。
24 个业务 Service 完整保留：设备信息、恢复、stall、9 个 getter、7 个 setter、5 个离散操作。
完整清单、SDK 映射和 mock 规则见 [接口对照](docs/python_ros2_parity.md)。

- 不支持的 Service：`success=false`（SubmitCommand 为 `accepted=false`），
  `error_code=unsupported_operation`。
- 不支持的 Topic 命令：在 `command/status` 返回 FAILED，保留 client/sequence，
  `command_id=0` 表示 SDK 接纳前拒绝，并写入 diagnostics。
- 不支持的状态：保留 Topic，发布 `meta.valid=false`、`meta.stale=true`，通过
  `interface_support` 和 `/diagnostics` 说明错误；不伪造零反馈。
- 不支持的参数或连接策略：配置失败或参数更新失败，reason/日志含 `unsupported_operation`。
- SDK 没有的 metadata 保持未知/`has_*=false`，缺失的测量指标使用 NaN/unknown 并声明不可用。

普通读取 `timeout_s` 必须有限、正数且小于 `shutdown_timeout_s / 2`。
`recover.timeout_s` 不能小于 SDK 的 `shutdown_timeout_s`，否则返回 unsupported_operation；
SDK recover 没有独立的单次超时控制。离散操作 `SubmitCommand` 的请求是空对象 `{}`，
通过返回的 command_id 和 `command/status` 跟踪结果。

## C++ 格式与命名

本目录的 `.clang-format` 与 `.clang-tidy` 只约束本包的 C++ 代码，排版沿用
`linkerbot-hand-cpp` 的配置。统一使用 LLVM 18.1.8；可在独立环境准备工具：

```bash
python3 -m venv /tmp/linkerbot-ros-cpp-style
/tmp/linkerbot-ros-cpp-style/bin/python -m pip install \
  clang-format==18.1.8 clang-tidy==18.1.8
export PATH="/tmp/linkerbot-ros-cpp-style/bin:$PATH"
```

- 使用 4 个空格缩进、120 列行宽，左花括号放在行末，命名空间内缩进。
- `if/else/for/while` 的语句体始终加花括号，保留 `else if` 链式写法。
- 类、结构体、枚举、类型别名和模板类型参数使用 `CamelCase`。
- 函数、变量、参数、命名空间和公开数据成员使用 `snake_case`，类的私有成员加 `_` 后缀。
- 枚举值和非局部常量使用 `kCamelCase`，局部常量保持 `snake_case`。
- 普通函数使用前置返回类型；lambda 优先推导返回类型。
- GoogleTest 的 `SetUp`、`TearDown`、`SetUpTestSuite`、`TearDownTestSuite` 保留框架名称。
  ROS 和第三方定义的接口、消息字段、参数键及生成代码保留原名，不对依赖运行重命名。

在 ROS 仓库根目录运行以下命令。直接扫描包目录，包含尚未纳入 Git 的 C++ 文件：

```bash
mapfile -d '' cpp_files < <(rg --files --null linkerbot_hand_ros_cpp \
  -g '*.cpp' -g '*.hpp' -g '*.h')
clang-format -i "${cpp_files[@]}"
clang-format --dry-run --Werror "${cpp_files[@]}"
```

命名检查仅启用 `readability-identifier-naming`，违规作为错误处理。先生成编译数据库并
运行已有测试，再检查本包的源文件和测试；包内头文件会随引用它们的源文件一并检查：

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_EXPORT_COMPILE_COMMANDS=ON bash tests/cpp/verify.sh \
  ../linkerbot-hand-cpp ../linkerbot-transport-cpp /tmp/linkerbot-ros-cpp-check
rg --files --null linkerbot_hand_ros_cpp/src linkerbot_hand_ros_cpp/test -g '*.cpp' \
  | xargs -0 -r -n 1 clang-tidy \
      -p /tmp/linkerbot-ros-cpp-check/build/linkerbot_hand_ros_cpp
```

## 验证

在 ROS 仓库根目录，加载构建环境后运行：

```bash
colcon --log-base log/cpp-test test \
  --base-paths linkerbot_hand_msgs linkerbot_hand_ros_cpp \
  --packages-select linkerbot_hand_ros_cpp \
  --build-base build/cpp --install-base install/cpp --return-code-on-test-failure
colcon test-result --test-result-base build/cpp/linkerbot_hand_ros_cpp/test_results --verbose
```

也可运行 `bash tests/cpp/verify.sh ../linkerbot-hand-cpp ../linkerbot-transport-cpp` 进行隔离
构建。契约测试只读 Python ROS 源码，默认从仓库根目录的 `linkerbot_hand_ros` 读取；可用
`LINKERBOT_PYTHON_ROS_BASELINE` 指向另一份只读基准。测试用内存 FakeTransport 运行真实
SDK codec/runtime，并启动无硬件 ROS 节点检查图接口、工具及配置失败回滚。
这些检查不替代适配器、CAN FD 与六种型号的真机验收。
