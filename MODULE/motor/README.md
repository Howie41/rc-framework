# 电机基类与 C620

`motor_base.hpp` 和 `c620.hpp` 均为 header-only。MotorBase 保存公共反馈、减速比、使能状态和反馈时间，不包含输出命令、限幅或位置速度进程。

## C620 与 C620Motor

- `C620 : CanDevice` 是协议设备，一个实例管理同一总线上的 1～8 号电机。构造时注册到 CanBus，不保存 Bus 引用，不操作硬件。同一总线只创建一个 C620。
- `C620Motor : MotorBase` 保存单个电机的 ID、配置、反馈及 OutputCommand，不再继承 CanDevice，也不负责 CAN 编解码。
- 按 `CanBus → C620 → C620Motor` 顺序构造。电机通过 `C620Motor motor{c620, 1}` 自动登记到 C620，不需要手动注册。
- `C620Motor::id()` 返回电调设置的 1～8 号 ID；CAN 收发 ID、组帧和反馈映射全部由 C620 处理。一个 C620 及八个电机只占一个 Bus 设备槽位。
- 电机和协议设备都需持续存活，不支持复制、移动或注销。所有登记须在 `CanBus::init()` 启动接收前完成。

## 命令与周期发送

`C620Motor::OutputCommand` 使用 `current_ma` 表示毫安电流目标。`set_output_command()` 保存限幅后的目标，不直接发送；非有限值返回 false，原目标保持不变。

`init(reduction_ratio, max_output)` 只设置单电机配置，不重置反馈。默认减速比为 `C620::kDefaultReductionRatio`，电流上限为 `C620::kMaxCurrentMa`。断言要求减速比是正有限值，上限在 0～最大协议电流内。

电机初始为禁用状态。`enable()` 允许输出，`disable()` 清空目标并禁止输出；停机仍需继续周期发送零电流。在线判断不自动改变使能状态。

每个控制周期先设置全部电机的目标，再调用一次 `bus.heartbeat()`：

```text
CanBus::heartbeat()
    → C620::build_tx(bus)
        → 读取八个电机的最新目标并换算协议电流
        → 构建 1～4 号控制帧并调用 bus.add_can_msg()
        → 构建 5～8 号控制帧并调用 bus.add_can_msg()
```

两帧每周期都会构建，未登记或禁用的槽位填零；全部电机禁用时仍发送两帧零电流。应用不需要选择控制组、手动组帧或调用 output_trans()。原 uses_shared_tx()、DjiMotorCanGroup 及手动 pack_dji_motor_can_msg() 接口已移除。

`build_tx(bus)` 返回两帧是否全部提交成功，第一帧失败也会尝试第二帧。`heartbeat()` 汇总所有协议设备的结果，某设备失败仍继续处理后续设备。发送使用现有硬件 FIFO，不新增软件队列或失败帧缓存；部分成功无法撤回，下一周期重新读取最新目标。

## 使用示例

```cpp
#include "c620.hpp"

static CanBus g_can_bus{hfdcan1};
static C620 g_c620{g_can_bus};
static C620Motor g_motor1{g_c620, 1};
static C620Motor g_motor6{g_c620, 6};

// 在 MX_FDCAN1_Init() 之后调用，设备及电机已经在构造时登记。
void init_motors() {
    g_motor1.init();
    g_motor6.init();
    if (!g_can_bus.init()) {
        Error_Handler();
        return;
    }
    g_motor1.enable();
    g_motor6.enable();
}

// 在同一控制任务中周期调用；PID 等控制器的输出单位应为 mA。
[[nodiscard]] bool update_motors(float current1_ma, float current6_ma) {
    const bool first = g_motor1.set_output_command({.current_ma = current1_ma});
    const bool sixth = g_motor6.set_output_command({.current_ma = current6_ma});
    const bool sent = g_can_bus.heartbeat();
    return first && sixth && sent;
}
```

## 反馈与单位

C620 只匹配标准帧反馈范围，Bus 调用 `on_rx(rx_id, data, len)` 后，C620 根据实际 CAN ID 定位电机。未绑定 ID、短帧或非法编码器值不会更新电机状态。反馈解析和换算沿用现有 frame_codec 定义，不改变协议比例。

- 输出端位置为度，速度为 rad/s，温度为摄氏度；首帧建立位置基准，第二帧起更新角度，保留原行为。
- 跨圈判断使用半圈阈值。两帧间运动超过半个转子圈或丢帧后，无法仅凭此算法恢复圈数；累计计数仍使用原 int32 范围。
- 原始力矩字段仍保存反馈电流原始量，力矩继续沿用原有系数和减速比换算，不新增电流或力矩标定。
- 有效反馈更新时间到 MotorBase，中断分发使用 ISR tick 接口。`is_online()` 使用 RTOS 当前毫秒时间和基类公共阈值，不自动停机。
- 中断更新与任务读取之间的同步仍由调用方保证，不新增反馈快照或临界区封装。

## IM6014 构造与初始化

构造函数只接收电机 ID 和配置，不接收或保存 Bus。可以在构造时提供 ID 和输出端前馈力矩上限（Nm）：

```cpp
Im6014Motor motor(0, 5.0f);
```

构造不发送命令。Bus 绑定已有的 `UartPortDma`，须在 RTOS 资源创建前构造。通信开始前调用返回 void 的 `bus.init()` 清空设备表，再检查 `bus.register_device(&motor)` 的 bool 结果；RTOS 资源创建后，由应用调用 `uart.start_receive_to_idle_dma()` 并检查 HAL 返回值。运行时，应用通过 `motor.build_tx(output, size)` 组帧、`bus.add_rs485_msg(bytes, &motor)` 入队，同一个通信任务先调用 `process_rx()` 再调用 `process_tx()`。总线不自动组帧或轮询设备。同一电机对象只注册到一条总线。完整调用说明见 [Rs485Bus 说明](../../BSP/rs485_bus/README.md)。

也可以一次指定完整的初始化配置：

```cpp
Im6014Motor motor(Im6014Motor::Config{
    .id = 0,
    .max_output_nm = 20.0f,
    .reduction_ratio = 38.0f / 3.0f,
    .mode = Im6014Motor::Mode::Stop,
    .timeout_protection = true,
});
```

Config 默认 ID 为 0、前馈力矩上限为 0、减速比为 38/3、模式为停机，并开启电机超时保护。未注册到总线时，也可独立调用 `build_tx(output, size)`、`on_rx()` 或 `feed_rx()` 进行协议编解码。

构造和 `init()` 共用参数校验逻辑：非法构造参数使对象保持未初始化，`id()` 返回 0xFF，`build_tx(output, size)` 返回 false 并将 size 置零。需要直接获取初始化结果时，可默认构造，再调用返回 bool 的 `init(config)` 或 `init(id, max_output_nm)`；失败不改变已有状态，成功则清空命令、反馈和解析半帧，并应用配置。

`OutputCommand` 包含输出端位置 `position_deg`、速度 `speed_rad_s`、前馈力矩 `torque_nm`、位置增益 `kp` 和速度增益 `kd`。`set_output_command()` 校验参数并限幅前馈力矩，返回 bool，不直接发送，也不改变使能状态。

`enable()` 请求 FOC 模式；`disable()` 请求停机并清空命令。禁用时 `build_tx()` 生成停机模式和零目标；软件禁用不会撤回已经入队的旧报文，仍须由应用发送新的停机命令。`get_mode()` 是待发送模式，`get_feedback_mode()` 是设备反馈的实际模式。

有效反馈使用 RTOS 毫秒时间更新在线状态；主控 `is_online()` 使用 MotorBase 的公共 100 ms 阈值。设备自身的超时保护和反馈超时标志与此独立，离线处置由应用决定。
