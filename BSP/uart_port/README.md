# uart_port

假设电脑通过 USART3 给一盏灯发送占空比和亮灯时长。板子收到后保存两个值，再交给点灯代码执行。

每帧 4 字节，位置从 0 开始，按从左到右的顺序发送。

**亮灯控制帧**

| 帧位置 | 字节 0 | 字节 1 | 字节 2～3 |
| --- | --- | --- | --- |
| 数据 | 帧头 `0x01` | 占空比 | 亮灯时长（毫秒） |
| 数据类型 | `uint8_t` | `uint8_t` | `uint16_t` |

占空比为 `0～100`，时长为 `0～65535` 毫秒，低字节在前。例如 `01 32 E8 03` 表示占空比 50%，亮 1000 毫秒。

```cpp
#include "uart_port.hpp"
#include "frame_parser.hpp"
#include "usart.h"
#include <array>
#include <span>

using LedFrame = frame::Frame<
    frame::Header<0x01>,
    frame::Field<uint8_t>,
    frame::Field<uint16_t>>;
using LedParser = frame::FrameParser<LedFrame>;

// 放在函数外，rtos_resource_init() 之前创建。
UartPortDma<"led-uart3", 64, 64, 256> led_uart{huart3, {}};
uint8_t led_duty{};
uint16_t led_duration_ms{};
LedParser parser({
    [](LedParser&, const LedFrame::DataTuple& data) {
        auto [duty, duration_ms] = data;
        if (duty <= 100) {
            led_duty = duty;
            led_duration_ms = duration_ms;
            // 在这里接入点灯代码。
        }
    }
});

// 串口和 RTOS 资源已准备好后，作为接收任务的工作代码运行。
void receive_led_task() {
    if (led_uart.start_receive_to_idle_dma() != HAL_OK) {
        Error_Handler();
        return;
    }
    std::array<uint8_t, 64> buffer{};
    while (true) {
        auto length = led_uart.block_for_received_chunk(buffer);
        parser.feed_and_parse(std::span<const uint8_t>{buffer.data(), length});
    }
}

// 需要发出同样格式的命令时，在任务中调用。
HAL_StatusTypeDef send_led_command() {
    auto bytes = LedFrame::serialize({uint8_t{50}, uint16_t{1000}});
    return led_uart.start_transmit_dma(bytes);
}
```

这个例子保存数值，不会自动在 1000 毫秒后关灯。点灯和计时由应用完成。`receive_led_task()` 是任务里要执行的工作，不要在调度器启动前直接调用它。

接入时按下面的顺序准备：

1. CubeMX 配置 USART3、引脚、波特率、DMA 和相关中断。通信双方串口参数要相同。
2. 接收 DMA 选 `CIRCULAR`，发送 DMA 选 `NORMAL`；USART 中断也要开启。
3. 在全局创建上面的对象。一个串口只建一个 `UartPort` 对象，`"led-uart3"` 这个名字也不要和其他 DMA 串口实例重复。
4. 本项目在 `MX_FREERTOS_Init()` 中调用 `rtos_resource_init()`；接收任务开始运行后，再启动 DMA 接收。

项目的 `uart_port.cpp` 已经把 HAL 回调接好，不要在别处重复定义同名 HAL 回调。接收的一批字节可能是半帧，也可能包含多帧，所以示例把它们交给同一个 [FrameParser](../../MODULE/frame_codec/README.md)。

### UartPortDma<UniqueId, TxBufferSize = 0, RxBufferSize = 0, RxStreamBufferSize = 0>

DMA 串口。上面的 `64, 64, 256` 分别是发送数组、DMA 接收数组、任务接收缓冲区的大小，单位都是字节。

- `explicit UartPortDma(UART_HandleTypeDef& huart, Callbacks callbacks)`
  - 第一项填项目的串口句柄，例如 `huart3`。第二项填事件处理代码，不需要时写 `{}`。
  - 第一个配置项是唯一名字，像 `"led-uart3"` 这样写。
  - 收发容量默认都是 0，要用哪个就给哪个留空间。任务接收缓冲区也要满足 [StreamBuffer](../../MODULE/rtos_resource_wrapper/README.md) 的最小容量要求，示例使用 256。
- `HAL_StatusTypeDef start_receive_to_idle_dma()`
  - 启动连续接收，正常运行期间只需启动一次，不用每收一批就重启。
  - 成功返回 `HAL_OK`，已经在接收时返回 `HAL_BUSY`，其他错误按 HAL 返回值处理。
- `std::size_t block_for_received_chunk(std::span<uint8_t> output_buffer)`
  - 等到有数据后，写进给定数组，返回实际写入的字节数。
  - 只有数组的前面这么多字节有效，不一定填满数组。
  - 没有数据时会等待，只在任务里调用。没有启用任务接收缓冲区时直接返回 0。
- `HAL_StatusTypeDef start_transmit_dma(std::span<const uint8_t> msg)`
  - 开始发送给定字节，先复制到串口自己的发送数组，返回后原数组可以复用。
  - `HAL_OK` 只表示成功开始发送，不表示已经发完。
  - `HAL_BUSY` 表示上一次还没发完；这次不会排队，需要之后自己重试。
  - 数据超过发送容量时返回 `HAL_ERROR`。不要传空数据。
- `RxStreamBuffer m_rx_stream_buffer`
  - 已启用时，可以用它不等待地读取：`led_uart.m_rx_stream_buffer.receive_to(buffer, 0)`。
  - 返回实际读到多少字节，0 表示这次没有数据。具体读写用法见 [StreamBuffer](../../MODULE/rtos_resource_wrapper/README.md)。
  - 它和 `block_for_received_chunk()` 读的是同一份数据，安排一个地方读取即可。
- `std::atomic<std::size_t> m_rx_stream_dropped_bytes`
  - 任务接收缓冲区放不下时，记录有多少字节没放进去。
  - 用 `led_uart.m_rx_stream_dropped_bytes.load()` 查看；增加时要及时读取或增大缓冲区。

DMA 自带的 `m_tx_buffer`、`m_rx_buffer` 平时不用自己操作，发送和接收使用上面的方法即可。

### UartPort::Callbacks

需要在事件发生时执行代码，就在创建串口时填这些选项。没填的事件不处理，像开头那样全部填 `{}` 也可以正常使用任务接收。

- `on_transmit_complete(UartPort&)`
  - 上一次异步发送真正完成后调用。
- `on_receive_complete(UartPort&)`
  - 普通定长接收完成时调用。例如下面的 IT 接收就使用它。
- `on_receive_to_idle_chunk(UartPort&, std::span<const uint8_t>)`
  - DMA 接收有新增字节时调用，第二项就是本次新增的数据。
  - 可能分多次调用，不代表每次是一整帧。数据只在这次调用期间使用，要保存就当场复制。
  - 即使填了这个处理函数，启用的任务接收缓冲区仍会收到数据，不要重复处理同一批字节。
- `on_receive_to_idle_event(UartPort&, ReceiveToIdleEvent, uint16_t)`
  - 需要区分 DMA 收满、收到一半、接收线空闲这三种事件时使用。
  - 第三项是 DMA 在数组里写到的位置，不是本次新增的字节数。平时处理数据优先用 `on_receive_to_idle_chunk`。
- `on_error(UartPort&, uint32_t)`
  - 串口发生收发错误时调用，第二项是 HAL 错误码。
  - 不会自动替应用重启接收，异常后的处理由应用安排。

这些处理函数在中断里执行，不要延时、长时间等待或做耗时工作。需要任务处理时，优先使用开头的接收缓冲区写法。

### UartPortIt<TxBufferSize = 0, RxBufferSize = 0>

需要少量定长数据、使用 IT 收发时选择它。下面是另一种接收写法，不要与开头的 DMA 例子同时绑定 USART3：

```cpp
#include "uart_port.hpp"
#include "usart.h"
#include <atomic>

std::atomic<bool> received{false};
UartPortIt<16, 4> it_uart{huart3, {
    .on_receive_complete = [](UartPort&) { received.store(true); }
}};

// USART3 初始化完成后，在任务里调用。
void receive_once() {
    received.store(false);
    if (it_uart.start_receive_it(4) != HAL_OK) return;
    // 之后由任务检查 received.load()，为 true 时处理 it_uart.m_rx_buffer。
    // 处理完成后再调用 start_receive_it(4)，开始下一次接收。
}
```

- `explicit UartPortIt(UART_HandleTypeDef& huart, Callbacks callbacks)`
  - 两项分别填串口句柄和事件处理代码，容量单位是字节。
- `HAL_StatusTypeDef start_receive_it(uint16_t size)`
  - 启动一次定长接收，收到 `size` 字节后执行 `on_receive_complete`。
  - `size` 必须是正数，不能超过接收容量。再次接收要重新调用。
- `HAL_StatusTypeDef start_transmit_it(std::span<const uint8_t> msg)`
  - 开始一次发送，先复制到自己的发送数组。
  - `HAL_OK` 表示开始成功，`HAL_BUSY` 表示上一次未结束，超过容量时返回 `HAL_ERROR`。
- `std::array<uint8_t, RxBufferSize> m_rx_buffer`
  - 定长接收完成后，从这里读取本次收到的内容，处理完再开始下一次接收。

### UartPort：两种串口都能用的方法

直接在 `led_uart` 或 `it_uart` 上调用，不必再创建一个 `UartPort`。

- `HAL_StatusTypeDef transmit(std::span<const uint8_t> msg, uint32_t timeout)`
  - 同步发送，会等到发送结束或超时，第二项单位是毫秒。
  - 不要和 IT/DMA 发送同时使用，也不要在中断里调用。
- `HAL_StatusTypeDef receive(std::span<uint8_t> buffer, uint32_t timeout)`
  - 同步接收，会等到给定数组填满或超时，第二项单位是毫秒。
  - 不要和已启动的 IT/DMA 接收同时使用，也不要在中断里调用。
  - 同步收发单次长度不要超过 65535 字节，返回 `HAL_OK` 表示这次完成。
- `bool is_transmitting() const noexcept`
  - 查看 IT/DMA 发送是否仍在进行，正常发完后返回 `false`。
  - 只是查看，不会等待，也不代表下一次调用一定不会忙。
- `uint32_t transmit_complete_count() const noexcept`
  - 查看异步发送完成了多少次。
- `uint32_t error_count() const noexcept`
  - 查看串口错误回调发生了多少次。
- `HAL_StatusTypeDef abort_all_receive()`
  - 终止当前 IT/DMA 接收。之后需要接收时再启动，不要在中断里调用。
- `HAL_StatusTypeDef abort_all_transmit()`
  - 终止当前 IT/DMA 发送。不要在中断里调用。
- `UART_HandleTypeDef& handle() noexcept`
  - 需要调用其他 HAL 串口接口时，取出所绑定的句柄。

串口对象要一直保留。`UartPortManager` 和 HAL 转发方法由项目处理，平时不需要直接调用，也不要绕开发送方法手动修改内部缓冲区。
