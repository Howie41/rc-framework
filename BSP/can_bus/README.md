# CAN 移植驱动

`can_bus.hpp` 保留 `CanDevice` 和 `CanBus` 的设备注册、接收分发和发送职责。普通实现为 header-only，`can_bus.cpp` 只定义唯一的 HAL 回调入口。

`CanDevice(bus, rx_id, is_extid)` 在构造时注册到指定总线，只保存接收 ID 和接收帧类型，不保存总线引用或发送 ID。注册失败触发断言，注册过程不操作硬件、不调用设备虚函数。发送报文的 ID 和帧类型由协议设备构建的 `ClassicPack` 指定。

一个 `C620 c620{bus}` 作为 CanDevice 管理 1～8 号电机，只占一个总线设备槽位。`C620Motor motor{c620, motor_id}` 仅继承 MotorBase，构造时登记到 C620，不再直接注册到 CanBus。

## 原接口对应

| 原名称 | 当前名称 |
| --- | --- |
| `canFilterInit` | `can_filter_init` |
| `bspCanInit` | `bsp_can_init` |
| `instanceByIndex` | `instance_by_index` |
| `registerDevice` | `register_device` |
| `addCanMsg` | `add_can_msg` |
| `processRxInterrupt` | `process_rx_interrupt` |
| `hardwareTxQueueEmptySize` | `hardware_tx_queue_empty_size` |
| `isExtId`、`setCanId` | `is_ext_id`、`set_can_id` |
| `matchRx`、`onRx`、`buildTx` | `match_rx`、`on_rx(rx_id, data, len)`、`build_tx(bus)` |

`CanBus::Type` 的枚举项为 `Standard`、`Extended`。`ClassicPack` 成员为 `m_id`、`m_type`、`m_data[8]`。结构体只向 HAL 提供各字段，不直接按内存布局传输，因此移除原来的 `typedef` 和 `packed`。

## 初始化与生命周期

1. 先构造 CanBus，再构造协议设备；设备通过 CanDevice 构造函数自动注册。C620Motor 在所属 C620 构造后创建并登记。全局构造阶段不操作硬件。
2. 由当前工程的 CubeMX 代码初始化 `hfdcan1/2/3`。本驱动只支持这些句柄。
3. 调用 `CanBus::init()`，先建立外设到 Bus 的中断映射，保留构造阶段注册的设备。
4. `init()` 内部调用现有 `can_filter_init()`：标准帧进入 FIFO0，扩展帧进入 FIFO1，均使用全接收掩码，再由设备软件匹配。标准和扩展过滤器各使用索引 0。
5. `init()` 内部调用现有 `bsp_can_init()`，开启接收、错误通知并启动外设。应用无需再单独调用这两个辅助函数。

`init()` 返回 bool，未知外设句柄返回 false；重复初始化返回 true，不重复配置和启动。`register_device()` 同样返回 bool，最多 8 个设备；同一指针重复注册成功，空指针、重复接收 ID/类型、表满及启动后新增设备返回 false。同一数值的标准 ID 与扩展 ID 可分别注册。

过滤器与启动函数保留原 `void` 接口，HAL 初始化失败调用 `Error_Handler()`。配置使用当前项目的 HAL 和消息 RAM，不改写 CubeMX 生成文件。外设应配置为经典 CAN，发送和接收元素至少容纳 8 字节，具备对应的过滤器和 FIFO。

总线、句柄、协议设备及电机须在中断和收发使用期间保持存活，建议按总线、协议设备、电机的顺序静态创建。注册表不拥有对象，也不提供注销；设备和电机禁止复制和移动。所有构造、注册、ID 配置及其他初始化须在 `CanBus::init()` 启用接收中断前完成，运行中不清空注册表。

## 收发行为

- `add_can_msg()` 直接提交固定 8 字节的经典数据帧，使用现有 `AtomicLock` 串行化 HAL 发送。0 表示 HAL 接受，1 表示忙、FIFO 满、非法 ID/类型或 HAL 失败。接受不代表总线交付成功。
- 控制任务每周期调用 `heartbeat()`，总线仅遍历 Device 并调用 `build_tx(*this)`，不再管理共享帧或槽位。该纯虚接口允许协议设备构建零帧或多帧，通过传入 Bus 的 `add_can_msg()` 提交硬件 FIFO，无需保存 Bus 引用。
- `build_tx(bus)` 返回本设备本周期是否全部处理成功；`heartbeat()` 汇总结果，未初始化或任一设备失败返回 false，其他情况返回 true。某个设备失败仍继续调用后续设备，无设备或某设备本周期无需报文时按成功处理。
- C620 内部负责两组编码，每周期分别提交两帧完整控制命令，未绑定或禁用的电机槽位置零。应用无需选择组，也不再使用 `uses_shared_tx()` 或 Bus 内部的共享合帧机制。
- 无软件队列、失败帧缓存、等待或自动重试；下一控制周期由协议设备重新读取最新输出。部分已提交帧不能撤回，硬件 FIFO 中已提交的帧仍可能等待发送。
- 删除原队列专用的 `tx_service`、`process_tx_interrupt`、待处理标志和发送完成/空 FIFO 续发回调，不启用发送续发通知。
- 接收每次回调取一帧，遍历设备匹配后调用纯虚接口 `on_rx(rx_id, data, len)`。C620 按实际接收 CAN ID 将反馈更新到对应电机。回调在中断中运行，不能阻塞，数据指针不能保留到回调之外。
- 接收侧拒绝远程帧、FD 帧和超出 8 字节的 DLC，HAL 读取失败直接返回。内部接收临时缓冲留足 HAL 最大复制长度，避免在检查报文前写越界。
- `g_can_error_state` 保留原错误状态参数记录方式，供调试器观察；错误回调不再死循环。没有新增统计、恢复或同步接口。

设备反馈在中断中更新；任务读取及其他并发访问由调用方保证同步。驱动不新增 `InterruptGuard` 或反馈快照。

C620 注册和分组发送的完整示例见 `MODULE/motor/README.md`。
