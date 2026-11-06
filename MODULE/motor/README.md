# motor

假设我们要控制 CAN1 上的 1 号 C620 电调：每次给它一个电流目标，同时读取速度和位置。

电流单位为 mA。例如 `1000` 表示 1 A，负数表示反方向。这里直接控制电流，不会自动保持某个转速；需要稳速时，可以让 [PID](../pid/README.md) 计算电流目标。

```cpp
#include "c620.hpp"
#include "fdcan.h"

// 放在函数外，整个程序运行期间保留。
CanBus motor_bus{hfdcan1};
C620 c620{motor_bus};
C620Motor motor1{c620, 1}; // 1 是电调上设置的编号

// MX_FDCAN1_Init() 完成后、控制任务开始工作前调用一次。
bool init_motor() {
    motor1.init(19.0f, 5000.0f); // 减速比 19，电流最多 ±5000 mA
    if (!motor_bus.init()) return false;
    motor1.enable();
    return true;
}

// 在同一个控制任务中周期调用。
bool update_motor(float current_ma) {
    if (!motor1.set_output_command({.current_ma = current_ma})) return false;
    bool submitted = motor_bus.periodic(); // 把本周期目标交给 CAN 发送
    float speed = motor1.get_current_speed(); // 输出端速度，rad/s
    (void)speed; // 在这里接入自己的反馈处理
    return submitted;
}
```

先调用并检查 `init_motor()`，再在控制任务里周期调用 `update_motor(1000.0f)`。周期由应用决定，例如每 1 毫秒调用一次。总线接收配置见 [CanBus](../../BSP/can_bus/README.md)。

如果项目里已经创建了 CAN1 的总线和 C620，直接使用已有对象，不要给同一个 CAN 外设再创建一套。以上是接入写法，不是给现有 `robot` 配置重复添加设备。

### C620

管理同一条 CAN 总线上的 C620 电调，控制 1～8 号电机。

- `explicit C620(CanBus& bus)`
  - 传入电调所在的 CAN 总线，例如 `C620 c620{motor_bus};`。
  - 同一条总线只创建一个 C620，可以给它配多个 `C620Motor`。
  - 构造时自动登记到总线，不用手动注册，也不在这里启动硬件。

平时不用调用 C620 的组帧和接收方法。每个控制周期先更新所有电机目标，再调用一次 `motor_bus.periodic()` 即可。

### C620Motor

代表一个 C620 电机。用它设置目标、允许输出、读取反馈。

- `C620Motor(C620& controller, uint8_t motor_id)`
  - 第一项填所属的 C620，第二项填电调上设置的 ID，范围 1～8，同一个 C620 下不能重复。
  - 创建后默认不允许输出，需要调用 `enable()`。
- `void init(float reduction_ratio = 19.0f, float max_output = 20000.0f)`
  - 第一项是减速比，必须大于 0。
  - 第二项是电流上限，单位 mA，范围 0～20000；实际目标会限制在正负这个值以内。
  - 更新配置，不清空已经收到的位置和速度。
- `void enable() noexcept`
  - 允许使用设置的电流目标。实际发送还要继续调用总线的 `periodic()`。
- `void disable() noexcept`
  - 禁止输出，并把目标电流清成 0。
  - 之后仍要继续调用 `periodic()`，把零电流发给电调。
- `bool set_output_command(const OutputCommand& command) noexcept`
  - 保存目标电流，例如 `motor1.set_output_command({.current_ma = 1000.0f});`。
  - 超过配置上限时自动限制，保存成功返回 `true`。
  - 传入 NaN、无穷大时返回 `false`，保留原来的目标。这里只保存目标，不会立即发送。
- `const OutputCommand& get_output_command() const noexcept`
  - 查看当前保存的命令，例如 `motor1.get_output_command().current_ma`。
- `uint8_t id() const noexcept`
  - 返回电调 ID，也就是创建时填的 1～8，不是 CAN 报文 ID。

### C620Motor::OutputCommand

- `float current_ma{0}`
  - 目标电流，单位 mA。正负方向由电机接线和安装方向决定。

### Im6014Motor

控制通过 RS-485 通信的 IM6014 电机。下面以 0 号电机为例，给它输出端速度目标 `1 rad/s`，阻尼设为 `0.1`。

```cpp
#include "im6014.hpp"
#include "rs485_bus.hpp"
#include "usart.h"
#include <array>
#include <span>

// 放在函数外，rtos_resource_init() 之前创建。串口名不要与其他实例重复。
UartPortDma<"im6014-uart2", 64, 128, 512> motor_uart{huart2, {}};
Rs485Bus rs485{motor_uart};
Im6014Motor im_motor;

// 串口和 RTOS 资源准备好后，在通信任务开始时调用一次。
bool init_im_motor() {
    if (!im_motor.init(0, 5.0f)) return false; // ID 为 0，前馈力矩最多 ±5 Nm
    rs485.init();
    if (!rs485.register_device(&im_motor)) return false;
    if (motor_uart.start_receive_to_idle_dma() != HAL_OK) return false;
    im_motor.enable();
    return true;
}

// 由同一个通信任务周期调用。
bool update_im_motor() {
    (void)rs485.process_rx(); // 先处理已经收到的回复
    if (!im_motor.set_output_command({.speed_rad_s = 1.0f, .kd = 0.1f})) return false;

    std::array<uint8_t, 20> bytes{};
    std::size_t length{};
    if (!im_motor.build_tx(bytes, length)) return false;
    bool queued = rs485.add_rs485_msg(std::span{bytes}.first(length), &im_motor);
    (void)rs485.process_tx(); // 推进发送；忙时留到后续调用继续处理
    return queued;
}
```

先检查 `init_im_motor()` 是否成功，再持续调用 `update_im_motor()`。返回 `true` 只表示这条命令排进发送队列，不表示电机已经执行。队列满时返回 `false`，要继续运行收发处理。

UART 要配置硬件方向控制，TX DMA 用 NORMAL、RX DMA 用 CIRCULAR，收发回调按 [UART 接口](../../BSP/uart_port/uart_port.hpp) 接好。总线接入步骤见 [Rs485Bus](../../BSP/rs485_bus/README.md)。不要让其他功能同时使用这个串口。

- `Im6014Motor()`
  - 创建尚未配置的电机，之后调用 `init()` 并检查返回值。
- `Im6014Motor(uint8_t id, float max_output_nm) noexcept`
  - 创建时给定 ID 和前馈力矩上限，默认停机。
  - 也可以写 `Im6014Motor motor{Im6014Motor::Config{...}};` 给出完整配置。
- `bool init(uint8_t id, float max_output_nm) noexcept`
  - ID 范围 0～14，上限为非负数，单位 Nm。
  - 成功返回 `true`，清空命令和反馈，设为停机模式。
  - 参数非法返回 `false`，保留原状态。
- `bool init(const Config& config) noexcept`
  - 使用下面的完整配置。默认构造后再调用这个方法，可以直接知道配置是否成功。
  - 成功时清空命令和反馈，并应用配置中的模式。
- `uint8_t id() const noexcept`
  - 返回电机 ID。尚未成功配置时为 `0xFF`。
- `void enable() noexcept`
  - 请求工作模式，还要组帧并通过总线发送才会作用到电机。
- `void disable() noexcept`
  - 请求停机并清空目标。仍然需要组帧、排队、持续处理发送。
- `bool set_mode(Mode mode) noexcept`
  - `Im6014Motor::Mode::Foc` 与 `enable()` 一样，`Mode::Stop` 与 `disable()` 一样。
  - 支持的模式返回 `true`，其他值返回 `false`。
- `Mode get_mode() const noexcept`
  - 查看下一次打算发送的模式，不代表电机已经进入该模式。
- `bool set_output_command(const OutputCommand& command) noexcept`
  - 保存目标，例如开头的速度和阻尼命令，不改变使能状态，也不发送。
  - 前馈力矩会限制在配置上限内。
  - 非法数字、负的 kp/kd、超过协议能表示的范围时返回 `false`，保留旧命令。
- `const OutputCommand& get_output_command() const noexcept`
  - 查看当前保存的目标。
- `bool build_tx(std::span<uint8_t> output, std::size_t& size) const noexcept`
  - 把命令写进给定数组，至少留 20 字节。
  - 成功返回 `true`，并把实际长度写进第二项；失败返回 `false`，长度为 0。
  - 只生成字节，不发送。像示例一样再调用总线的入队和收发方法。
- `void on_rx(std::span<const uint8_t> bytes)`
  - 自己收发串口、不经过 Rs485Bus 时，把每次收到的原始字节传进来。
  - 允许分多次传入，收齐且检查通过后更新反馈。使用总线时由总线调用，不要再重复传。
- `void set_timeout_protection(bool enabled) noexcept`
  - 设置电机自身的一秒超时保护位，默认开启。
  - 修改后还要发送命令；它和主控的 `is_online()` 不是同一个状态。
- `uint8_t get_feedback_mode() const noexcept`
  - 查看电机回复中的实际模式。
- `bool is_timeout() const noexcept`
  - 查看电机回复的超时标志。
- `float get_winding_temperature() const noexcept`
  - 获取绕组温度，单位摄氏度。
- `float get_voltage() const noexcept`
  - 获取母线电压，单位 V。
- `uint32_t get_error_flags() const noexcept`
  - 获取电机反馈的故障标志，具体含义按电机手册查看。
- `uint8_t get_warning_flags() const noexcept`
  - 获取电机反馈的告警标志，具体含义按电机手册查看。

### Im6014Motor::Config

需要更改减速比、初始模式等选项时使用。下面都是默认值：

- `uint8_t id{0}`
  - 电机 ID，0～14。
- `float max_output_nm{0}`
  - 前馈力矩上限，单位 Nm。默认 0，也就是不允许非零前馈力矩；不是全部控制输出的总力矩上限。
- `float reduction_ratio{38.0f / 3.0f}`
  - 减速比，必须为正数。
- `Mode mode{Mode::Stop}`
  - 初始化后的模式，默认停机。需要允许输出时选 `Mode::Foc`。
- `bool timeout_protection{true}`
  - 是否开启电机自身的一秒超时保护。

### Im6014Motor::OutputCommand

命令里的位置、速度和力矩都按减速后的输出端填写，不用自己换算转子数值。未填写的项为 0。

- `float position_deg{0}`
  - 目标累计位置，单位度，和反馈使用同一个原点。
- `float speed_rad_s{0}`
  - 目标速度，单位 rad/s。
- `float torque_nm{0}`
  - 前馈力矩，单位 Nm，受 `max_output_nm` 限制。
- `float kp{0}`
  - 位置控制的刚度，单位 Nm/rad，必须非负。需要位置控制时设置。
- `float kd{0}`
  - 速度控制的阻尼，单位 Nm/(rad/s)，必须非负。例子通过它配合目标速度使用。

### MotorBase：两种电机都能读取的反馈

不需要自己创建 `MotorBase`。直接在 `motor1` 或 `im_motor` 上调用下面的方法。

- `bool is_enabled() const noexcept`
  - 查看软件是否允许输出，不代表电机已经转动。
- `bool is_online() const noexcept`
  - 在任务中调用。最近 100 毫秒内有有效反馈时返回 `true`，否则返回 `false`。
  - 返回 `false` 不会自动停机，是否停止输出要自己处理。
- `uint32_t get_last_feedback_ms() const noexcept`
  - 最近有效反馈的毫秒时间。`0xFFFFFFFFU` 表示还没有有效反馈。
- `float get_current_single_pos() const`
  - 输出端单圈位置，单位度。
- `float get_current_sum_pos() const`
  - 输出端累计位置，单位度。
- `float get_current_speed() const`
  - 输出端速度，单位 rad/s。
- `float get_current_torque() const`
  - 输出端力矩反馈。IM6014 单位为 Nm；C620 是当前驱动换算出的值，不能直接当作已标定的实际 Nm。
- `float get_current_temperature() const`
  - 温度，单位摄氏度。IM6014 这里是驱动器温度。
- `float get_raw_current_single_pos() const` / `float get_raw_current_sum_pos() const`
  - 获取转子侧单圈位置和累计位置，单位度。
- `float get_raw_current_speed() const`
  - 获取转子侧速度，单位 rad/s。
- `float get_raw_current_torque() const`
  - IM6014 返回转子侧力矩，单位 Nm；C620 返回协议里的原始电流反馈值。

所有总线、协议设备和电机对象都要一直保留，不要临时创建后销毁。C620 的累计位置以首次反馈为起点；两次反馈间转子运动超过半圈时，累计圈数可能不准。反馈更新和控制访问不要在不同任务或中断里随意混用，需要由应用安排好同步。
