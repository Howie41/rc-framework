# rs485_bus

假设 USART2 接了一条 RS-485 总线，上面有一台 0 号 IM6014 电机。我们要周期发送速度目标，再读取电机反馈。

这里给输出端速度设为 `1 rad/s`，阻尼设为 `0.1`。具体电机命令由已有的 `Im6014Motor` 生成，不用自己填写报文字节。总线负责排队发送，把回复交回电机对象。

```cpp
#include "rs485_bus.hpp"
#include "im6014.hpp"
#include "usart.h"
#include <array>
#include <span>

// 放在函数外，rtos_resource_init() 之前创建，整个运行期间保留。
UartPortDma<"rs485-uart2", 64, 128, 512> uart{huart2, {}};
Rs485Bus bus{uart};
Im6014Motor motor;

// 串口和 RTOS 资源准备好后，在通信任务开始时调用一次。
bool init_rs485() {
    if (!motor.init(0, 5.0f)) return false; // ID 为 0，前馈力矩上限为 5 Nm
    bus.init();
    if (!bus.register_device(&motor)) return false;
    if (uart.start_receive_to_idle_dma() != HAL_OK) return false;
    motor.enable();
    return true;
}

// 生成一条命令并排队，true 只表示排队成功。
bool queue_motor_command() {
    if (!motor.set_output_command({.speed_rad_s = 1.0f, .kd = 0.1f})) return false;
    std::array<uint8_t, 20> bytes{};
    std::size_t length{};
    if (!motor.build_tx(bytes, length)) return false;
    return bus.add_rs485_msg(std::span{bytes}.first(length), &motor);
}

// 由同一个通信任务持续调用，先处理接收，再推进发送。
void process_rs485() {
    (void)bus.process_rx();
    (void)bus.process_tx();
    float speed = motor.get_current_speed(); // 最近有效反馈的输出端速度，rad/s
    // 在这里使用 speed；还没有反馈时是初始值 0。
    (void)speed;
}
```

先检查 `init_rs485()` 是否成功，再按控制周期调用 `queue_motor_command()`。无论这次是否排队成功，都要持续调用 `process_rs485()`，不能只排队而不处理收发。

接入时需要准备：

1. CubeMX 配置串口、RS-485 硬件 DE 方向控制及相关中断。双方的波特率等参数一致。
2. TX DMA 使用 `NORMAL`，RX DMA 使用 `CIRCULAR`。
3. 像例子一样启用 UART 发送、DMA 接收和任务接收缓冲区。UART 和总线对象都要在 `rtos_resource_init()` 之前创建。
4. 系统调度启动后，在通信任务里执行初始化和收发。项目已经接好 UART 的 HAL 回调，不要重复定义。

一个串口只使用一套对象。如果项目已有 `huart2` 对应的 UART 或总线，复用已有对象，不要照例子再绑定一次。串口用法见 [uart_port](../uart_port/README.md)，电机参数和反馈见 [motor](../../MODULE/motor/README.md)。

### Rs485Bus

安排一条 RS-485 总线上的设备依次通信。它不会自动生成命令，也不会自动定时轮询，需要像示例一样自己排队和处理收发。

- `explicit Rs485Bus(UartPortDma<...>& port)`（省略串口类型参数）
  - 传入已经创建的 DMA 串口，例如 `Rs485Bus bus{uart};`。
  - 这个串口要一直保留，收发都由总线安排，不要再自己直接发送或取走接收缓冲区的数据。
- `void init() noexcept`
  - 开始注册设备前调用一次，清空设备名单。
  - 不启动串口，不创建 RTOS 资源，也不是故障恢复接口。
  - 不要在注册设备、排队或通信开始后再次调用。
- `bool register_device(Rs485Device* device)`
  - 登记一个设备，例如 `bus.register_device(&motor)`，成功返回 `true`。
  - 最多登记 16 个设备，设备地址不能重复。
  - 同一个对象再次登记返回 `true`；空指针、地址重复或没有空位时返回 `false`。
  - 通信前完成登记，一个设备只属于一条总线。和 CAN 设备不同，这里需要自己注册。
- `bool add_rs485_msg(std::span<const uint8_t> bytes, Rs485Device* response_device = nullptr)`
  - 把一条完整命令放进发送队列，成功返回 `true`。
  - 第一项填实际要发送的字节，例如 `std::span{bytes}.first(length)`，不要把数组里没用的尾部也发出去。
  - 第二项填要接收这次回复的设备，例如 `&motor`，必须已经登记。
  - 不需要回复时可以省略第二项或填 `nullptr`，这时收到的数据也不会交给该设备。
  - 队列最多放 4 条命令；每条长度不能超过 UART 发送容量，也不能超过 128 字节。
  - 空命令、长度超限、回复设备没有登记或队列已满时返回 `false`，这次没有排进去。
  - 返回后原数组可以复用，总线已经保存一份副本。
- `bool process_rx()`
  - 取出已经收到的字节，交给当前正在等回复的设备处理。
  - 有数据交给设备时返回 `true`，不代表设备已经解出有效反馈。
  - 没有正在等回复的设备时，收到的数据会被丢弃。
- `bool process_tx()`
  - 推进发送和等待回复的工作，每次最多开始发送队首的一条命令。
  - `true` 表示这次成功开始发送，不表示硬件已经发完或设备已经执行。
  - 没有命令、串口忙、正在等回复、结束超时等待或 HAL 启动失败时，都可能返回 `false`，不必把每次 `false` 都当故障。
  - HAL 启动失败时命令仍保留，后续调用会再次尝试。已经开始发送但没有收到回复的命令不会自动重新发送。

收发处理需要反复调用。同一个通信任务里先 `process_rx()`，再 `process_tx()`；可以由多个任务排队，但不要让多个任务同时调用收发处理方法。

### 回复等待

当前这些等待时间是模块里的固定值，不是调用参数：

- `kResponseTimeoutMs = 1`
  - 通信任务观察到串口发送结束后，最多等 1 毫秒，看有没有回复字节。
  - 没收到就结束这次等待，后续调用继续处理其他命令。
- `kResponseIdleMs = 1`
  - 已经收到字节后，要等至少 1 毫秒没有新字节，再处理下一条命令。
  - 这些等待还要求 UART 发送完成，不能只排队后就立刻切换到下一台设备。

收到字节不等于校验通过，是否更新反馈由设备判断。协议不支持请求序号时，也不能靠这个等待过程判断某条迟到回复一定属于哪一次命令。

### Rs485Device

需要接入新的 RS-485 设备时，按这个接口编写设备类。使用已有的 `Im6014Motor` 时不用自己实现，直接按开头的写法创建、注册和调用即可。

- `uint32_t id() const noexcept`
  - 获取设备地址。地址范围按具体设备协议决定，例子中 IM6014 使用 0～14。
- `bool build_tx(std::span<uint8_t> output, std::size_t& size) const noexcept`
  - 自定义设备时要实现这个方法，把完整命令写进给定数组。
  - 成功返回 `true`，同时把实际写入多少字节填进 `size`。
  - 失败返回 `false`，并把 `size` 置为 0。不要保留 `output` 留到以后再写。
  - 总线不会替应用调用这个方法，应用先组帧，再调用 `add_rs485_msg()`。
- `void on_rx(std::span<const uint8_t> bytes)`
  - 自定义设备时要实现这个方法，处理当前请求收到的原始字节。
  - 可能分多次交付，半帧缓存、完整帧检查和数据保存由设备处理。
  - 可以使用 [FrameParser](../../MODULE/frame_codec/README.md) 收齐后再取值。
  - 使用总线时由 `process_rx()` 自动调用，不要重复传入同一批数据。需要保留字节时当场复制。

### 使用时注意

- UART、总线和设备对象都要一直保留，不能临时注册后销毁，也不要在通信中改设备地址。
- 队列里保存的是排队当时的命令。之后改目标或调用电机 `disable()`，不会改掉已经排队的旧命令；还要生成并发送新的停机命令。
- 电机在线判断不会替你停机，离线后怎么处理由应用决定。
- UART 出错后的终止和重启由应用处理，总线不会自动恢复硬件。重新调用 `bus.init()` 也不能完成恢复。
