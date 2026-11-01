# MotorBase 与电机通信设计

初始记录：2026-10-31。更新日期：2026-11-01。状态：已实现，本文按当前接口同步。

本文记录 `MODULE/motor/motor_base.hpp`、C620、IM6014 及其通信接口的职责。具体使用示例见 [电机说明](../MODULE/motor/README.md)、[CAN 说明](../BSP/can_bus/README.md) 和 [RS485 说明](../BSP/rs485_bus/README.md)。

## 1. 职责与范围

MotorBase 保存电机通用属性，包括转速、温度、位置、力矩、减速比、使能状态以及最近反馈时间。型号专有的输出命令由各子类声明和保存。

状态更新和控制命令收发由所属总线管理。电机对象不主动调度通信，不包含 PID 计算。PID 控制器计算输出后形成具体电机的输出命令。

保留 `CanBus`、`Rs485Bus`，不引入 Endpoint 或额外的通用驱动层。CAN 由协议设备构造时注册；RS485 设备由应用显式注册，发送由应用组帧入队。

## 2. 输出命令类型

MotorBase 保持普通的非模板基类，不使用命令类型模板或 CRTP。移除基类中的数值型 `m_output`，各子类自行声明嵌套结构体 `OutputCommand`，并保存结构体成员 `m_output_command`。

`set_output_command()` 和 `get_output_command()` 均放在子类中。MotorBase 不声明命令类型、命令成员或命令访问接口，仍然可以通过统一的 `MotorBase*` 访问公共反馈和使能接口。

```cpp
class MotorBase {
public:
    virtual void enable() noexcept = 0;
    virtual void disable() noexcept = 0;
    // 公共反馈、使能状态和最近反馈时间，见下文。
};

class Im6014Motor : public Rs485Device, public MotorBase {
public:
    struct OutputCommand {
        float position_deg{0};
        float speed_rad_s{0};
        float torque_nm{0};
        float kp{0};
        float kd{0};
    };

    // 命令接口示意；校验失败返回 false，保留原命令。
    [[nodiscard]] bool set_output_command(const OutputCommand& command) noexcept;
    [[nodiscard]] const OutputCommand& get_output_command() const noexcept;

    void enable() noexcept override;
    void disable() noexcept override;

private:
    OutputCommand m_output_command{};
};

Im6014Motor::OutputCommand command{};
```

通过 `Im6014Motor::OutputCommand` 访问型号专有命令类型。C620Motor 同样自行声明 OutputCommand。`C620 : CanDevice` 统一管理八个电机、构建两帧和分发反馈，`C620Motor : MotorBase` 仅保留单电机状态与命令，不继承 CanDevice。使用方式为 `C620 c620{bus}`、`C620Motor motor{c620, motor_id}`。

命令内容按型号区分：

| 电机 | 命令内容 |
| --- | --- |
| C620 系列 | 电流目标，由外部 PID 等控制器计算 |
| IM6014 | `position_deg`、`speed_rad_s`、`torque_nm`、`kp`、`kd` |
| 其他电机 | 由对应型号定义自己的命令结构体 |

基类不统一规定命令字段和数值单位，不提供原先的数值型 `set_motor_output()`。具体命令设置、参数检查以及型号相关的限幅由子类接口决定，不为复用赋值操作引入模板或额外的命令抽象层。

## 3. 使能接口

原方案的 `start()`、`stop()` 改为 `enable()`、`disable()`，均为纯虚函数，由子类实现。

```cpp
virtual void enable() noexcept = 0;
virtual void disable() noexcept = 0;

[[nodiscard]] bool is_enabled() const noexcept;
```

基类保存：

```cpp
bool m_enabled{false};
```

`m_enabled` 表示软件是否允许输出，子类在实现使能接口时维护该状态。它不表示电机已经转动，也不等同于设备反馈的实际工作模式。

C620 根据使能状态决定发送目标电流或零电流。IM6014 根据使能状态生成 FOC 命令或停机模式的零目标报文。`disable()` 清空各型号的输出命令，调用使能接口不直接完成总线发送；RS485 已排队的旧报文也不会被改写，应用仍须发送新的停机命令。

## 4. 公共属性

保留现有通用反馈字段、减速比和反馈 getter，不为本次设计改变反馈单位或换算公式。

| 属性 | 含义 |
| --- | --- |
| `m_reduction_ratio` | 转子到输出端的减速比，初始值为 1 |
| `m_raw_single_pos` | 转子侧单圈位置 |
| `m_raw_sum_pos` | 转子侧累计位置 |
| `m_raw_speed` | 转子侧转速 |
| `m_raw_torque` | 原始力矩反馈量，含义和单位由型号决定 |
| `m_single_pos` | 输出端单圈位置 |
| `m_sum_pos` | 输出端累计位置 |
| `m_speed` | 输出端转速 |
| `m_torque` | 输出端力矩 |
| `m_temperature` | 电机温度，摄氏度 |
| `m_enabled` | 软件使能状态，初始值为 false |
| `m_last_feedback_ms` | 最近有效反馈的毫秒时间，初始值为 `0xFFFFFFFFU` |

反馈数值沿用零初始化。状态字段放在 `protected` 区域，由 getter 对外提供读取。IM6014 在自身反馈解析函数中更新基类字段；C620Motor 将 C620 声明为友元，由 C620 将反馈分发至对应电机。

保留的方法：

```cpp
void set_motor_reduction(float config);

[[nodiscard]] float get_current_single_pos() const;
[[nodiscard]] float get_current_sum_pos() const;
[[nodiscard]] float get_current_speed() const;
[[nodiscard]] float get_current_torque() const;
[[nodiscard]] float get_current_temperature() const;

[[nodiscard]] float get_raw_current_single_pos() const;
[[nodiscard]] float get_raw_current_sum_pos() const;
[[nodiscard]] float get_raw_current_speed() const;
[[nodiscard]] float get_raw_current_torque() const;
```

## 5. 在线状态

与反馈时效相关的成员只保留：

```cpp
std::uint32_t m_last_feedback_ms{0xFFFFFFFFU};
```

不增加 `m_has_feedback`、`m_offline_timeout_ms` 或缓存的 `m_is_online`。`0xFFFFFFFFU` 作为尚无有效反馈的初始标记。

在线判断在任务上下文中读取 `rtos::now_ms()`，使用公共的 100 ms 超时阈值，不在每个对象中保存阈值：

```cpp
static constexpr std::uint32_t kOfflineTimeoutMs{100U};

[[nodiscard]] bool is_online() const noexcept {
    return m_last_feedback_ms != 0xFFFFFFFFU &&
           std::uint32_t(rtos::now_ms() - m_last_feedback_ms) < kOfflineTimeoutMs;
}

[[nodiscard]] std::uint32_t get_last_feedback_ms() const noexcept;
```

仅有效反馈成功更新后才更新时间。C620 在接收中断中使用 ISR tick，IM6014 在通信任务中使用 RTOS 当前时间，均换算成同源毫秒时间。无符号差值允许时间回绕，实际反馈间隔须小于一个完整 uint32_t 毫秒计时周期；`0xFFFFFFFFU` 保留为尚无有效反馈标记。在线判断不自动禁止输出，失联处置由应用实现。

暂不设计反馈更新时间等辅助方法，也不保留独立的 `has_feedback()` 接口。IM6014 反馈中的设备端超时标志属于型号专有反馈，与主控的在线判断分开。

## 6. 移除与不纳入的内容

- 移除基类的 `m_max_output` 和 `set_max_output()`。
- 移除基类的数值型 `m_output` 和 `set_motor_output()`；结构体 `m_output_command`、OutputCommand 及其 setter/getter 放在各子类中。
- 不保存 `m_has_feedback`、`m_offline_timeout_ms`。
- 电机对象按长期存活使用，不设计析构流程，不显式增加析构函数。
- 不在基类中保留特定位置进程算法：`f_vx_buffer()`、`pos_with_speed_control()`、`update_speed_process()`、`get_is_finished()`，以及对应的 `m_k`、`m_tar_sum_pos`、`m_ini_sum_pos`、`m_pos_process`、`m_v`、`m_is_finished`。

位置进程属于外部控制逻辑，当前基类不提供这些接口。

## 7. 当前通信流程

- CAN 按 `CanBus → C620 → C620Motor` 顺序构造；协议设备和电机在构造时登记。`CanBus::init()` 建立中断映射、配置过滤器并启动外设，保留设备表；应用周期调用 `heartbeat()`，由 C620 构建两组控制帧。
- RS485 Bus 在 RTOS 资源创建前构造，由应用在通信前调用 `init()` 清空设备表并显式注册设备。UART DMA 接收由应用在 RTOS 资源创建后启动。
- IM6014 的 `build_tx(output, size)` 生成报文，应用调用 `add_rs485_msg(bytes, &motor)` 入队。同一通信任务先执行 `process_rx()` 再执行 `process_tx()`；总线负责排队、DMA 提交和回复等待，不自动轮询设备。
- MotorBase 是抽象类，IM6014 初始化时直接清空公共状态并重置解析器，不构造临时 MotorBase 对象。命令设置、初始化及设备组帧接口通过 bool 报告成功与失败。

电机反馈、命令和使能状态跨任务或中断访问时，仍由应用保证同步；当前没有统一的反馈快照或临界区封装。总线提交成功不等于硬件已经执行，离线处理和 UART 故障恢复属于应用职责。
