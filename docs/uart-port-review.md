# UART Port 剩余问题与参考解决方案

更新时间：2026-09-19

审查范围：

- `BSP/uart_port/uart_port.hpp`
- `BSP/uart_port/uart_port.cpp`
- 当前工程中的 STM32H7 HAL UART/DMA 实现和 UART4 DMA 配置

本文只保留当前尚未解决的问题。标题中的状态表示当前处理策略。

## 1. DMA handle 模板参数没有验证实际绑定关系——不处理

### 现状

`DmaTxHandle` 和 `DmaRxHandle` 当前只参与模板类型和 `static_assert`，真正启动传输时 HAL 使用的是：

```cpp
m_uart_handle.hdmatx
m_uart_handle.hdmarx
```

因此下面这种配置可以通过编译，但模板参数与实际 UART 不匹配：

```cpp
UartPortDma<&hdma_uart4_tx, 128> port{huart1, callbacks};
```

如果 `huart1.hdmatx` 为空或指向其他 DMA stream，模板参数不能提供保护。

### 参考方案 A：删除 handle 模板参数

如果 handle 只由 CubeMX 的 `__HAL_LINKDMA` 负责绑定，可以简化为：

```cpp
template <std::uint16_t TxBufferSize, std::uint16_t RxBufferSize>
class UartPortDma;
```

运行时始终以 `huart.hdmatx/hdmarx` 为准。这能避免模板参数给出并不存在的编译期安全保证。

### 参考方案 B：保留参数并延迟验证

由于全局 C++ 对象通常早于 `MX_UARTx_Init()` 构造，不能在构造函数里验证 DMA 链接。可以提供在 HAL 初始化后调用的 `init()`：

```cpp
HAL_StatusTypeDef init() {
    if constexpr (DmaTxHandle != nullptr) {
        if (m_uart_handle.hdmatx != DmaTxHandle) {
            return HAL_ERROR;
        }
    }

    if constexpr (DmaRxHandle != nullptr) {
        if (m_uart_handle.hdmarx != DmaRxHandle) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}
```

建议同时验证 DMA direction 和 mode。

## 2. 阻塞接口仍会把 `size_t` 静默截断为 `uint16_t`——不解决

模板缓冲区大小已经改成 `std::uint16_t`，因此 IT/DMA 内部缓冲区不会超过 HAL 的长度上限；但基类的阻塞接口仍接受任意长度的 dynamic-extent span：

```cpp
static_cast<std::uint16_t>(buffer.size())
static_cast<std::uint16_t>(msg.size())
```

例如 65536 会被转换成 0。

参考方案：

```cpp
#include <limits>

static bool valid_hal_size(std::size_t size) {
    return size > 0 &&
           size <= std::numeric_limits<std::uint16_t>::max();
}

HAL_StatusTypeDef transmit(
    std::span<const std::uint8_t> msg,
    std::uint32_t timeout
) {
    if (!valid_hal_size(msg.size())) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(
        &m_uart_handle,
        msg.data(),
        static_cast<std::uint16_t>(msg.size()),
        timeout
    );
}
```

`receive()` 应进行相同检查。

## 3. Circular ReceiveToIdle 的 Size 是位置，不是本次新增长度——不解决，TODO

在 circular DMA 模式下，`HAL_UARTEx_RxEventCallback()` 的 `Size` 表示 DMA 缓冲区中当前写入位置，而不是自上次回调以来新收到的字节数。缓冲区回卷时，如果直接把 `Size` 当成长度，会重复处理或遗漏数据。

建议把回调参数命名为 `write_position`，保存上一次位置并处理回卷：

```cpp
std::uint16_t m_last_rx_position{0};

void consume_rx_position(std::uint16_t position) {
    if (position >= m_last_rx_position) {
        process(std::span{
            m_rx_buffer.data() + m_last_rx_position,
            position - m_last_rx_position
        });
    } else {
        process(std::span{
            m_rx_buffer.data() + m_last_rx_position,
            m_rx_buffer.size() - m_last_rx_position
        });
        process(std::span{m_rx_buffer.data(), position});
    }

    m_last_rx_position =
        static_cast<std::uint16_t>(position % m_rx_buffer.size());
}
```

需要为零长度缓冲区单独禁止该接口，避免取模除零。

## 4. 全局 HAL callback 被该模块独占——不解决

`uart_port.cpp` 对以下弱符号提供了全局实现：

- `HAL_UART_TxCpltCallback`
- `HAL_UART_RxCpltCallback`
- `HAL_UARTEx_RxEventCallback`
- `HAL_UART_ErrorCallback`

未注册到 `UartPortManager` 的 UART 事件会直接被忽略。其他模块也不能再定义相同 callback，否则会产生重复符号。

如果项目规定所有 UART 必须通过 `UartPort` 使用，这可以作为明确的架构约束。如果需要兼容其他 UART 使用者，可以考虑：

- 在 manager 中提供 fallback callback。
- 统一由一个项目级 UART callback dispatcher 分发。
- 启用 `USE_HAL_UART_REGISTER_CALLBACKS`，在 HAL 支持范围内按 UART handle 注册回调。

## 5. D-Cache 启用后的 DMA 一致性需要额外处理——不解决，TODO

当前工程尚未启用 Cortex-M7 D-Cache，因此暂时不会触发缓存一致性问题。但 `DMA_BUFFER` 的 32 字节对齐和放入 RAM_D2 本身不等价于 non-cacheable。

将来如果启用 D-Cache，需要选择以下方案之一：

- 使用 MPU 把 `.dma_buffer` 所在区域配置为 non-cacheable。
- TX 启动前 clean cache；RX 读取前 invalidate cache，并严格按 cache line 对齐地址和长度。

否则可能出现 CPU 看到旧 RX 数据，或者 DMA 发送的不是 CPU 最近写入的数据。

## 后续验证项目

如果以后处理上述问题，应至少验证：

1. DMA handle 与 UART handle 绑定错误时，初始化阶段能明确失败。
2. 阻塞接口对 0、最大长度和超长 span 的行为符合约定。
3. Circular DMA 接收跨缓冲区末尾时，所有字节只处理一次。
4. 未注册到 `UartPortManager` 的 UART callback 按预期处理。
5. 启用 D-Cache 后，DMA TX/RX 数据与 CPU 视图保持一致。
