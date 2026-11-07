# pid

假设我们要让电机保持 `10 rad/s`。每个控制周期读取实际速度，交给 PID 算出电流目标，再把电流交给电机驱动。

这里先演示计算部分，不直接控制电机。参数只用于展示写法，接入实际电机时需要按设备调整。

```cpp
#include "pid_controller.hpp"

// 放在函数外，整个程序运行期间保留。
PidController speed_pid{"motor1_speed", PidController::Config{
    .kp = 100.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_out = 5000.0f, // 本例把输出当成 mA，限制为 ±5000 mA
    .dead_band = 0.01f,
}};

// 每个控制周期调用，measure_rad_s 传实际测得的输出端速度。
float update_speed(float measure_rad_s) {
    return speed_pid.calculate(measure_rad_s, 10.0f);
}
```

调用前需要先初始化 DWT 计时。本项目的 `robot::init()` 已经做了这一步；独立接入时，在硬件时钟设置好后、计算 PID 之前调用一次：

```cpp
Dwt::instance().init(SystemCoreClock / 1'000'000U);
```

在控制任务中持续调用 `update_speed()`，例如每 1 毫秒一次。第一次调用返回 0，从下一次开始计算。拿到返回值后，自己调用电机的 `set_output_command()` 和总线发送方法，接入方式见 [motor](../motor/README.md)。

### PidController

根据当前测量值和目标值，计算一次控制输出。例子中的 `speed_pid` 只负责计算，不负责读传感器、定时调用或发送电流。

- `PidController()`
  - 使用默认配置。默认参数和输出上限都是 0，需要配置后再使用。
- `explicit PidController(const char* name)`
  - 给控制器起名字，方便在 PID Viewer 中辨认，例如 `"motor1_speed"`。
  - 最多保存 31 字节，不是 31 个汉字。
- `explicit PidController(const Config& config)`
  - 创建时传入配置，不指定名字。
- `PidController(const char* name, const Config& config)`
  - 像开头一样，一次给出名字和配置。
- `float calculate(float measure, float ref)`
  - 第一项填实际测量值，第二项填目标值，别填反。
  - 两项要使用相同单位。例子里都用 rad/s。
  - 返回本次输出，限制在 `-max_out～max_out`。输出按什么单位使用，由你的参数和后面的驱动接口决定。
  - 创建、初始化或重置后的第一次调用返回 0。
- `void configure(const Config& config)`
  - 换一套完整配置，并清空之前的计算状态。
  - 不要在另一个任务正调用 `calculate()` 时执行。
- `void set_gains(float kp, float ki, float kd)`
  - 只改三个参数，不清空之前积累的状态。
  - 例如 `speed_pid.set_gains(120.0f, 1.0f, 0.0f);`，适合运行时调参。
- `void reset()`
  - 清空计算状态，保留配置和已经记录的故障类型。
  - 下一次 `calculate()` 又从 0 输出开始。
- `void init()`
  - 清空计算状态，并清除故障记录，配置保留。
- `Config& config()` / `const Config& config() const`
  - 查看或修改当前配置。
  - 例如 `speed_pid.config().max_out.store(3000.0f);` 修改输出上限。
  - 读取一个值时可以写 `speed_pid.config().kp.load()`。不能把整个配置直接复制给另一个变量。
- `float output() const`
  - 查看最近一次输出，不重新计算。
- `float ref() const` / `float measure() const` / `float err() const`
  - 分别查看最近一次目标、测量值和误差，误差是目标减测量值。
- `float pout() const` / `float iout() const` / `float dout() const` / `float ffout() const`
  - 分别查看比例、积分、微分和前馈各自的输出。需要排查或记录时再读。
- `float dt() const`
  - 查看最近两次计算的间隔，单位秒。
- `uint16_t error_count() const` / `ErrorType error_type() const`
  - 查看错误计数和故障类型，需要先开启下面的 `error_handle`。
  - `PID_ERROR_NONE` 表示没有记录到故障，`MOTOR_BLOCKED` 表示记录了堵转判断。
  - 它只报告状态，不会自动停机。
- `uint8_t id() const` / `const char* name() const`
  - 查看 PID Viewer 使用的编号和名字。

控制器要一直保留，不要在每次更新函数里重新创建。创建时会自动登记到管理器，不支持用完后销毁再注销。

### PidController::Config

先配置 `kp`、`ki`、`kd` 和 `max_out`，其他选项按需要再加。以下数值项都是 `std::atomic<float>`，创建时可以像开头一样直接填数；运行时用 `.store(值)` 修改、`.load()` 读取。不需要先学习这个类型的原理。

- `kp`、`ki`、`kd`（默认均为 0）
  - 比例、积分、微分参数，分别填写。
- `max_out`（默认 0）
  - 输出幅值上限。例子填 5000，输出最多为正负 5000。
  - 不改默认值就没有非零输出。
- `dead_band`（默认 0）
  - 允许的误差范围，单位和测量值一样。误差绝对值不超过它时，输出为 0。
- `integral_limit`（默认 0）
  - 积分输出幅值上限。需要开启同名的改进选项才生效。
- `coef_a`、`coef_b`（默认均为 0）
  - 配合 `changing_integration_rate` 使用，`coef_a` 设正数，`coef_b` 设非负数，单位与误差相同。
  - 在积分继续推大误差时，误差不超过 `coef_b` 仍正常累积；再大时减弱，超过两者之和时暂停累积。
- `output_lpf_rc`、`derivative_lpf_rc`（默认均为 0）
  - 分别设置输出、微分输出的平滑程度，单位秒，设为非负数。
  - 分别配合 `output_filter`、`derivative_filter` 开启，数值越大平滑越强、响应越慢。
- `ff_j`、`ff_b`（默认均为 0）
  - 前馈补偿参数，配合 `feedforward_control` 使用，不需要时保持 0。
- `improvement`（默认全部关闭）
  - 选择下面的额外功能。不要直接填一个看不懂的整数，按下面的代码逐项开启。

例如给速度控制加上积分限幅：

```cpp
speed_pid.config().integral_limit.store(1000.0f);
auto options = speed_pid.config().improvement.load();
options.set(PidController::integral_limit);
speed_pid.config().improvement.store(options);
```

### PidController::Improvement

用 `options.set(PidController::选项名)` 开启，用 `options.reset(PidController::选项名)` 关闭，最后写回 `improvement`。普通使用不必全开。

- `increment_of_out`
  - 改用增量式输出计算。按你的控制需求选择，默认关闭。
- `feedforward_control`
  - 开启前馈补偿，同时配置 `ff_j`、`ff_b`。
- `trapezoid_integral`
  - 使用梯形积分方式，不需要额外参数。
- `changing_integration_rate`
  - 误差较大时减弱或暂停积分累积，同时配置 `coef_a`、`coef_b`。
- `derivative_on_measurement`
  - 微分部分使用测量值的变化，不使用目标值的变化。
- `derivative_filter`
  - 平滑微分输出，同时配置 `derivative_lpf_rc`。
- `integral_limit`
  - 限制积分输出，同时配置 Config 中的 `integral_limit`。
- `output_filter`
  - 平滑最终输出，同时配置 `output_lpf_rc`。
- `error_handle`
  - 开启堵转判断，通过 `error_type()` 查看，不会替你发送停机命令。
- `deadband_remain_iout`
  - 进入允许误差范围时保留已积累的积分值。范围内最终输出仍是 0，不是继续输出积分值。

### PidViewer

需要在电脑上查看曲线、修改 PID 参数时使用。板上计算 PID 不依赖它，不需要上位机时可以跳过这一节。

下面接续开头的 `speed_pid`，给 Viewer 单独使用 USART3：

```cpp
#include "pid_viewer.hpp"
#include "usart.h"

// 函数外创建，且早于 rtos_resource_init()。不要和别的功能共用 USART3。
DMA_BUFFER_BSS UartPortDma<64, 128, 512> viewer_uart{huart3, {}};

// 串口初始化和 RTOS 资源创建完成后，在 Viewer 任务开始时调用一次。
bool init_pid_viewer() {
    if (!PidViewer::set_parameter_limits({
        .kp_min = 0.0f, .kp_max = 1000.0f,
        .ki_min = 0.0f, .ki_max = 1000.0f,
        .kd_min = 0.0f, .kd_max = 1000.0f,
    })) return false;
    PidViewer::bind(viewer_uart);
    return viewer_uart.start_receive_to_idle_dma() == HAL_OK;
}

void update_pid_viewer() { // 初始化成功后，在同一任务中约每 1 毫秒调用
    PidViewer::periodic(rtos::now_ms());
}
```

UART 的 TX DMA 使用 NORMAL、RX DMA 使用 CIRCULAR，并接好项目的串口回调。电脑端使用配套 PID Viewer；这里不需要自己拼接上位机消息。

- `static void bind(UartPortDma<...>& port)`（省略串口的类型参数）
  - 传入已经创建好的 DMA 串口，例如 `PidViewer::bind(viewer_uart);`。
  - 发送缓冲区至少 52 字节，接收缓冲区和接收流缓冲区都要启用。
  - 不会替你启动接收，仍需像示例一样检查 `start_receive_to_idle_dma()` 的结果。
  - Viewer 独占这个串口的接收数据和发送接口，不要再由其他功能读取或发送。
- `static void periodic(uint32_t timestamp_ms)`
  - 在同一个任务里周期调用，建议每 1 毫秒一次，传入当前毫秒时间。
  - 处理电脑发来的指令、发送回复和曲线数据。电脑选好监控通道后，曲线数据每 10 毫秒生成一次。
- `static bool set_parameter_limits(const ParameterLimits& limits)`
  - 设置允许电脑修改的 kp、ki、kd 范围，包含上下界。
  - 成功返回 `true`；有非法数字或最小值大于最大值时返回 `false`，原范围不变。
  - 只限制之后电脑发来的调参请求，不修改已有参数，也不限制板上直接修改配置。

### PidViewer::ParameterLimits

下面每一项都是 `float`。默认允许所有有限的 float 值，需要限制时像示例一样显式填写。

- `kp_min`、`kp_max`
  - 电脑可设置的 kp 最小值和最大值。
- `ki_min`、`ki_max`
  - 电脑可设置的 ki 最小值和最大值。
- `kd_min`、`kd_max`
  - 电脑可设置的 kd 最小值和最大值。

### PidManager

需要按编号查找控制器时使用。控制器创建时已经自动登记，不用手动注册。

- `static PidManager& instance()`
  - 取得管理器，例如 `auto& manager = PidManager::instance();`。
- `PidController* get_pid(uint8_t pid_id)` / `const PidController* get_pid(uint8_t pid_id) const`
  - 按编号查找，找到后才能访问。先写 `if (pid != nullptr)` 再使用返回的 `pid`。
  - 找不到返回 `nullptr`。编号按控制器创建顺序从 0 分配，`0xFF` 表示无效编号。
- `std::size_t size() const`
  - 返回已经登记的控制器数量，最多 255 个。

PID 计算、重置和状态读取由应用安排好调用顺序，不要在不同任务里同时操作同一个控制器的计算状态。`set_gains()` 可以更新参数，但三个值不是一次整体替换；不要假定计算时一定同时看到一整组新参数。
