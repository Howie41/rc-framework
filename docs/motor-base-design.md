# MotorBase 基类设计方案

记录日期：2026-10-31。状态：设计记录，尚未修改实现。

本方案在现有 `MODULE/motor/motor_base.hpp` 及电机类的基础上调整，不重建电机驱动结构。当前只确定 MotorBase；总线和具体电机的后续修改另行讨论。

## 1. 职责与范围

MotorBase 保存电机通用属性，包括转速、温度、位置、力矩、减速比、使能状态以及最近反馈时间。型号专有的输出命令由各子类声明和保存。

状态更新和控制命令收发由所属总线管理。电机对象不主动调度通信，不包含 PID 计算。PID 控制器计算输出后形成具体电机的输出命令。

本方案不引入 Endpoint，不新增通用驱动层，不要求重命名 `CanBus`、`Rs485Bus`，也不在本轮重新设计注册机制或继承关系。

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

    // 接口示意；具体错误返回方式在子类适配时确定。
    void set_output_command(const OutputCommand& command);
    [[nodiscard]] const OutputCommand& get_output_command() const noexcept;

    void enable() noexcept override;
    void disable() noexcept override;

private:
    OutputCommand m_output_command{};
};

Im6014Motor::OutputCommand command{};
```

这对应 `Im6014::OutputCommand` 这种访问形式；无需为此重命名现有 `Im6014Motor` 类。C620Motor 同样自行声明 OutputCommand。后续通信方案已调整为 `C620 : CanDevice` 统一管理八个电机、构建两帧和分发反馈，`C620Motor : MotorBase` 仅保留单电机状态与命令，不再继承 CanDevice。使用方式为 `C620 c620{bus}`、`C620Motor motor{c620, motor_id}`。

命令内容按型号区分：

| 电机 | 命令内容 |
| --- | --- |
| C620 系列 | 电流目标，由外部 PID 等控制器计算 |
| IM6014 | 位置、速度、前馈力矩、位置增益、速度增益；可以沿用现有 `HybridCommand` 的字段 |
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

C620 不需要存在独立的硬件启动指令才能实现这组接口。其输出门控，以及 IM6014 的模式和命令处理，留到具体电机与总线设计时确定。调用使能接口不直接完成总线发送。

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

反馈数值沿用现有的零初始化。状态字段放在 `protected` 区域，由现有反馈 getter 对外提供读取；具体总线如何获得写入权限，在后续总线方案中确定。

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

在线判断使用调用方提供的当前时间和超时阈值，不在电机对象中保存阈值：

```cpp
[[nodiscard]]
bool is_online(std::uint32_t now_ms,
               std::uint32_t timeout_ms) const noexcept {
    return m_last_feedback_ms != 0xFFFFFFFFU &&
           std::uint32_t(now_ms - m_last_feedback_ms) < timeout_ms;
}

[[nodiscard]] std::uint32_t get_last_feedback_ms() const noexcept;
```

仅有效反馈成功更新后才更新时间。当前时间和反馈时间使用同一单调毫秒时钟。超时判断、失联后的输出处理、时间回绕及哨兵值边界的具体约束，在后续总线设计中补齐。

暂不设计反馈更新时间等辅助方法，也不保留独立的 `has_feedback()` 接口。IM6014 反馈中的设备端超时标志属于型号专有反馈，与主控的在线判断分开。

## 6. 移除与不纳入的内容

- 移除基类的 `m_max_output` 和 `set_max_output()`。
- 移除基类的数值型 `m_output` 和 `set_motor_output()`；结构体 `m_output_command`、OutputCommand 及其 setter/getter 放在各子类中。
- 不保存 `m_has_feedback`、`m_offline_timeout_ms`。
- 电机对象按长期存活使用，不设计析构流程，不显式增加析构函数。
- 不在基类中保留特定位置进程算法：`f_vx_buffer()`、`pos_with_speed_control()`、`update_speed_process()`、`get_is_finished()`，以及对应的 `m_k`、`m_tar_sum_pos`、`m_ini_sum_pos`、`m_pos_process`、`m_v`、`m_is_finished`。

位置进程属于外部控制逻辑；需要保留该功能时，再随对应控制器迁移，不在本次文档记录中修改代码。

## 7. 后续实施范围

后续落地时，保留 C620 和 IM6014 现有的非模板 MotorBase 继承，分别增加嵌套 OutputCommand、命令存储及访问接口，并实现 enable()/disable()。还需要适配现有 IM6014 初始化代码：加入纯虚使能接口后，MotorBase 将成为抽象类，不能继续使用 `MotorBase{}` 构造临时对象来重置基类状态。

总线对状态的写入权限、命令收发流程、离线处置，以及输出上限在具体型号中的处理方式尚未展开。本文不将这些事项视为已经实现，也不修改现有源码和使用示例。
