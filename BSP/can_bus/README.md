# CAN 移植驱动

`can_bus.hpp` 保留原版 `CanDevice` 和 `CanBus` 的职责、参数形式和返回值，按项目规范调整命名。普通实现为 header-only，`can_bus.cpp` 只定义唯一的 HAL 回调入口。

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
| `matchRx`、`onRx`、`buildTx` | `match_rx`、`on_rx`、`build_tx` |

`CanBus::Type` 的枚举项为 `Standard`、`Extended`。`ClassicPack` 成员为 `m_id`、`m_type`、`m_data[8]`。结构体只向 HAL 提供各字段，不直接按内存布局传输，因此移除原来的 `typedef` 和 `packed`。

## 初始化与生命周期

1. 由当前工程的 CubeMX 代码初始化 `hfdcan1/2/3`。本驱动只支持这些句柄。
2. 调用 `CanBus::init()`，建立 0、1、2 到 CAN1、CAN2、CAN3 的映射并清空设备表。
3. 初始化设备并调用 `register_device()`，最多 6 个设备。同一指针重复注册成功；空指针返回 1，表满返回 2。
4. 调用 `can_filter_init()` 配置所需 ID 过滤器。原版过滤器索引 0 保留：标准和扩展 ID 可各配置一个；同类型重复调用会覆盖之前的索引 0。
5. 调用 `bsp_can_init()` 开启接收、错误通知并启动外设。

过滤器与启动函数保留原 `void` 接口，HAL 初始化失败调用 `Error_Handler()`。配置使用当前项目的 HAL 和消息 RAM，不改写 CubeMX 生成文件。外设应配置为经典 CAN，发送和接收元素至少容纳 8 字节，具备对应的过滤器和 FIFO。

总线、句柄与已注册设备须在中断和收发使用期间保持存活，建议静态创建。注册表不拥有设备，也不提供注销；不要复制或移动已登记对象。`init()`、注册、ID 配置必须在启用接收中断前串行完成，不能在运行中重新清空设备表。

## 收发行为

- `add_can_msg()` 直接提交固定 8 字节的经典数据帧，使用现有 `AtomicLock` 串行化 HAL 发送。0 表示 HAL 接受，1 表示忙、FIFO 满、非法 ID/类型或 HAL 失败。接受不代表总线交付成功。
- 无软件队列、失败帧缓存、等待或自动重试；应用下一控制周期重新组装最新输出。硬件 FIFO 中已提交的帧仍可能等待发送。
- 删除原队列专用的 `tx_service`、`process_tx_interrupt`、待处理标志和发送完成/空 FIFO 续发回调，不启用发送续发通知。
- 接收沿用原版每次回调取一帧、遍历设备匹配后调用 `on_rx()`；回调在中断中运行，不能阻塞，数据指针不能保留到回调之外。
- 接收侧拒绝远程帧、FD 帧和超出 8 字节的 DLC，HAL 读取失败直接返回。内部接收临时缓冲留足 HAL 最大复制长度，避免在检查报文前写越界。
- `g_can_error_state` 保留原错误状态参数记录方式，供调试器观察；错误回调不再死循环。没有新增统计、恢复或同步接口。

设备反馈在中断中更新；任务读取及其他并发访问由调用方保证同步。驱动不新增 `InterruptGuard` 或反馈快照。

C620 注册和分组发送的完整示例见 `MODULE/motor/README.md`。
