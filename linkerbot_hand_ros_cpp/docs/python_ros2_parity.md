# Python ROS2 公开接口与 C++ 对照

对照范围是本仓库 Python ROS2 包公开的 ROS 图接口、launch、参数、客户端与工具。
行为差异尚未逐项统一。C++ 使用同一消息包，接口存在不表示当前型号支持该操作。

## 公开入口覆盖

| Python ROS2 功能 | C++ ROS2 对应入口 | 实现情况 |
| --- | --- | --- |
| 单手生命周期节点 | `hand_node` / `hand.launch.py` | C++ LifecycleNode；真实 SDK open/close/identity |
| 多手容器 | `hand_container` / `multi_hand.launch.py` | 独立 namespace，分组启动及回滚 |
| 单手发现 | `hand.launch.py discover:=true` | SDK discover + 唯一身份选择 |
| 多手发现、导出及重新绑定 | multi launch 的 discover/config/output | 原生 SDK 发现，保存完整适配器配置及身份匹配 |
| 公开启动/驱动参数名 | `public_parameters.json`、launch 与节点参数 | 全部提供；不支持的配置明确拒绝 |
| 动态采样和发布频率 | `ros2 param set` | 验证、更新真实 polling，失败回滚旧计划 |
| 自动重连与手动恢复 | `auto_reconnect` / `~/recover` | SDK recover + 退避；不重放运动目标 |
| ROS Python 客户端 | `linkerbot_hand_ros_cpp.client.HandCommandClient` | 保留公开方法、签名；仅依赖 ROS 消息 |
| 角度发送/交互/监视/频率工具 | angle_command / angle_controller / state_monitor / frequency_benchmark | 保留工具入口和参数；不依赖 Python 驱动或 Hand SDK |
| GUI | 现有共享 GUI 包 | 使用同一 ROSIDL；未改 GUI，尚未进行真机 GUI 验收 |

Python 源码的静态契约提取结果为 44 个 Topic、24 个业务 Service；C++ 提供其全部名称和
类型，并额外提供 `~/interface_support` 以及 SDK 存在的补充状态/命令入口。
契约测试从源码提取，实际启动 C++ 节点验证图名称、类型和参数；没有导入 Python SDK。

## 命令与服务映射

| 公开接口 | C++ SDK 对应 |
| --- | --- |
| command/angles | set_angles / set_percent_angles，按 representation 分派 |
| command/speeds、accelerations、current_limits、torques_limits、torques、motion_times | 同语义 set 方法，数值通过 capability 校验 |
| command/stall_detection_times、stall_thresholds、stall_hold_currents | 同语义 set 方法 |
| command/limit_compensations、position_kp、position_kd、torque_kd | 同语义 set 方法 |
| get_configured_speeds、get_configured_accelerations、get_current_limits | 对应 SDK 读取方法 |
| get_torques_limits、get_motion_times、get_limit_compensations | 对应 SDK 读取方法 |
| get_position_kp、get_position_kd、get_torque_kd | 对应 SDK 读取方法 |
| set_stall_detection_times、set_stall_thresholds、set_stall_hold_currents | 对应 SDK 命令句柄 |
| set_limit_compensations、set_position_kp、set_position_kd、set_torque_kd | 对应 SDK 命令句柄 |
| get_device_info、get_stall_protection | get_device_info / 三个 stall getter（共享读取截止时间） |
| clear_faults、enable、disable、calibrate、save_parameters | 对应 SDK 离散操作及 CommandHandle |
| recover | HandDevice::recover，重新校验身份并恢复 polling |
| command/status | SDK command history，映射 accepted/dispatched/acknowledged/superseded/failed |

保留 ROS 中既有拼写 `torques_limits`。14 种命令、9 个 getter、7 个 setter 均保留入口。
某个型号没有对应能力时返回错误；不将其他型号的操作借用到当前设备。

## 状态、能力及诊断

| 状态接口 | 数据来源与限制 |
| --- | --- |
| state/angles | SDK AngleSample；raw/percent/physical_rad、valid/stale/generation |
| measured_speeds、configured_speeds、configured_accelerations、currents、current_limits、torques_limits、torques、temperatures、faults、voltages、motion_times、limit_compensations、position_kp、position_kd、torque_kd | SDK 对应缓存；只发布新样本或有效性变化 |
| state/force、force_fn、force_fns、force_ft、force_fts、force_fd、force_fds、force_px | SDK ForceSample/ForceFieldSample；按型号/字段能力可用 |
| state/tactile、tactile_sensors、normal_force_resultant、stall_protection | SDK 类型化缓存；不从触觉点阵推测合力 |
| state/fault_statuses | 使用 SDK 各型号的公开故障位定义；保留 raw 和 reserved_bits |
| capabilities、device_info | 已声明能力和经真实身份验证的 SDK 元数据 |
| /diagnostics | SDK health/session/命令族/通道指标；缺失指标明确 unknown/NaN |
| interface_support（新增） | transient local 实现清单；按当前 SDK、型号和已验证协议声明不可用项 |

fault_statuses 对 L6/O6/L20Lite/L20/L30 使用各自 SDK 的定义。O30 仅在协议版本明确为
0.0.4 时解码；旧协议或未知协议保留 raw，发布 invalid，避免误解码。

## 保留为 mock 或部分实现的功能

| 功能 | 原因 | 调用/读取结果 |
| --- | --- | --- |
| O20 型号 | 当前 C++ HandDevice 无后端 | 配置返回 unsupported_operation，公开图入口仍存在 |
| angle_feedback=adaptive / command_feedback_only | SDK facade 无反馈策略控制 | 配置/参数更新失败；query_only 接真实 polling |
| mailbox_capacity / route_queue_capacity | HandDeviceConfig 无独立 runtime 字段 | 保留兼容默认 256；非默认配置或运行时设置返回 unsupported_operation |
| recovery_timeout_s | SDK recover 共用 shutdown_timeout，没有独立恢复预算 | 保留兼容默认 2.0；非默认配置/动态设置返回 unsupported_operation |
| recover 请求短于 SDK shutdown 预算 | SDK recover 无单次超时参数 | success=false，unsupported_operation |
| transaction_timeout_s=0 | SDK 无 Python 的按协议选默认窗口入口 | 配置返回 unsupported_operation；使用显式正值 |
| 任意 python-can 插件、未映射 transport/discovery 选项 | 只有 SDK 的原生后端可用 | unsupported_operation，禁止静默忽略 |
| 扩展身份 reported_side/sensor_variant 等及触觉窗口 flags/component_count | SDK 未公开对应数据 | has/available/元数据保留未知；额外支持清单声明不可用 |
| SDK 未公开的 bus_errors、pending_rx、utilization、queue_depth、命令反馈样本数/精确来源、逐 force field 实测频率 | 不能从合并计数推测真实值 | 对应诊断 NaN/unknown；支持清单声明不可用 |
| 当前型号不支持的命令、服务、轮询或状态 | capability 不提供 | 命令/服务/配置明确失败；状态 invalid/stale 并有诊断 |

兼容默认值仅为占位，不代表已调节 SDK 对应容量或预算。普通数据结构字段没有独立返回
错误的位置，因此使用协议本身的 valid/has/available 标志，并通过 interface_support
发布不可用原因。Topic 命令拒绝有 FAILED 回执，command_id=0，不生成虚构 SDK ID。

## 验证边界

无硬件测试覆盖真实 L6 SDK 的生命周期、协议报文、命令、状态、服务和恢复；六种型号的
capability/fault 投影及 discovery 配置通过内存测试。ROS 图/launch 测试覆盖全部公开
名称和类型、动态参数、mock 报错、工具导入、静态容器和启动失败回滚。
PCAN/SLCAN/MiniCANFD 及各型号的机械动作、断线重连、触觉数据和 GUI 仍需实际设备验证。
