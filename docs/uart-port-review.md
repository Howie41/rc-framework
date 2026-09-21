# UART Port 方案评估与后续建议

更新时间：2026-09-21

审查范围：

- `BSP/uart_port/uart_port.hpp`
- `BSP/uart_port/uart_port.cpp`
- `BSP/memory_map.h`
- `STM32H723XG_FLASH.ld`
- 当前工程的 MPU、D-Cache 和 UART/DMA 配置

本文记录对 2026-09-21 所提解决方案的评估，并保留仍需执行的事项。

## 结论摘要

| 项目 | 评估 | 建议 |
| --- | --- | --- |
| 1. DMA handle 模板参数 | 方案成立，但只解决缓冲区实例隔离，不验证 HAL 绑定 | 可以保留；补充设计约束，最好在 HAL 初始化后校验绑定 |
| 2. 长度统一为 `uint16_t` | 只解决模板容量，尚未解决阻塞接口的窄化 | `receive()`、`transmit()` 在转换前检查 `span.size()` |
| 3. Circular ReceiveToIdle | 尚未解决 | 由 `UartPortDma` 维护上次位置并处理回卷 |
| 4. 全局 HAL callback | 在项目约束下可接受 | 把“所有 UART 必须通过 `UartPort`”写成架构约束 |
| 5. D-Cache 一致性 | 当前不会触发，但启用 D-Cache 后必须处理 | 优先将专用 `RAM_D2` MPU 区域设为 non-cacheable |

## 1. DMA handle 作为模板实例区分键——方案成立，建议补充校验

### 方案评估

`UartPortDma` 的缓冲区是模板类的 `inline static` 成员：

```cpp
inline static DMA_BUFFER std::array<std::uint8_t, TxBufferSize> m_tx_buffer;
inline static DMA_BUFFER std::array<std::uint8_t, RxBufferSize> m_rx_buffer;
```

同一模板特化的所有对象会共享这两个静态成员。因此，让 DMA handle 参与模板参数，可以使绑定不同 DMA handle 的 UART 形成不同模板特化，进而各自拥有一组缓冲区。这个设计动机是成立的。

它成立的前提是项目同时遵守以下约束：

- 一个 DMA handle 只能属于一个 UART 方向，不在多个 UART 间复用。
- 不为同一组 DMA handle 和缓冲区大小构造多个 `UartPortDma` 对象。
- 未配置的方向以 `nullptr, 0` 表示；另一个方向的 handle 仍足以区分实际使用的 UART。

### 仍然存在的边界

模板参数只负责生成不同的 C++ 类型和静态存储，并没有参与 HAL 调用。实际传输使用的仍然是：

```cpp
m_uart_handle.hdmatx
m_uart_handle.hdmarx
```

所以这段代码仍可能编译通过：

```cpp
UartPortDma<&hdma_uart4_tx, 128> port{huart1, callbacks};
```

换言之，模板参数在当前设计里是“缓冲区实例键”，不是“DMA 已正确绑定”的编译期证明。建议在类注释中明确这一语义，避免调用者误解。

### 建议

可以保留现有模板参数。作为额外防线，建议增加一个必须在 `MX_UARTx_Init()` 之后调用的初始化检查，因为全局 C++ 对象构造时 CubeMX 通常还没有执行 `__HAL_LINKDMA`：

```cpp
HAL_StatusTypeDef init() {
    if constexpr (DmaTxHandle != nullptr) {
        if (m_uart_handle.hdmatx != DmaTxHandle) return HAL_ERROR;
        if (DmaTxHandle->Init.Direction != DMA_MEMORY_TO_PERIPH) return HAL_ERROR;
        if (DmaTxHandle->Init.Mode == DMA_CIRCULAR) return HAL_ERROR;
    }

    if constexpr (DmaRxHandle != nullptr) {
        if (m_uart_handle.hdmarx != DmaRxHandle) return HAL_ERROR;
        if (DmaRxHandle->Init.Direction != DMA_PERIPH_TO_MEMORY) return HAL_ERROR;
    }

    return HAL_OK;
}
```

如果不想引入 `init()`，至少应在每次启动 TX/RX 前比较对应 handle，而不只是检查 `hdmatx != nullptr`。

## 2. 长度类型统一为 `uint16_t`——尚未完全解决

### 方案评估

把模板缓冲区容量改为 `std::uint16_t`，可以保证 `TxBufferSize` 和 `RxBufferSize` 不超过 HAL UART API 的 16 位长度上限，因此 IT/DMA 类内部固定缓冲区的容量问题已解决。

但是，阻塞接口仍接收 dynamic-extent `std::span`：

```cpp
HAL_StatusTypeDef receive(std::span<std::uint8_t> buffer, ...);
HAL_StatusTypeDef transmit(std::span<const std::uint8_t> msg, ...);
```

`std::span::size()` 的返回类型是 `std::size_t`，不受库中其他长度类型改为 `uint16_t` 的影响。目前仍存在：

```cpp
static_cast<std::uint16_t>(buffer.size())
static_cast<std::uint16_t>(msg.size())
```

例如，长度 65536 会被窄化为 0，更大的长度会按低 16 位截断。

### 建议

转换前显式检查范围；`receive()` 和 `transmit()` 都应处理。是否允许空 span 可按项目约定决定，下面示例选择拒绝：

```cpp
#include <limits>

static constexpr bool valid_hal_size(std::size_t size) {
    return size > 0 &&
           size <= std::numeric_limits<std::uint16_t>::max();
}

HAL_StatusTypeDef transmit(
    std::span<const std::uint8_t> msg,
    std::uint32_t timeout
) {
    if (!valid_hal_size(msg.size())) return HAL_ERROR;

    return HAL_UART_Transmit(
        &m_uart_handle,
        msg.data(),
        static_cast<std::uint16_t>(msg.size()),
        timeout
    );
}
```

仅把函数的另一个长度形参改为 `uint16_t` 不能解决 dynamic-extent span 自身可能超长的问题；边界处校验是最直接的做法。

## 3. Circular ReceiveToIdle 的 `Size` 是位置——待解决

在 circular DMA 模式下，`HAL_UARTEx_RxEventCallback()` 的 `Size` 表示 DMA 缓冲区中的当前写入位置，不是“本次新增长度”。它的典型取值范围是 `1..RxBufferSize`；缓冲区走到末尾时可能等于 `RxBufferSize`，随后回卷。

如果每次都把 `[0, Size)` 当作新数据：

- 连续 IDLE/HT/TC 事件会重复处理已有数据。
- DMA 回卷后会漏掉上一次位置到缓冲区末尾的数据。

建议由 `UartPortDma` 自己保存上次消费位置，不把这项易错状态分散给每个业务回调：

```cpp
std::uint16_t m_last_rx_position{0};

void consume_rx_position(std::uint16_t position) {
    if (position > RxBufferSize) {
        // 配置或 HAL 状态异常
        return;
    }

    if (position > m_last_rx_position) {
        process(std::span{
            m_rx_buffer.data() + m_last_rx_position,
            position - m_last_rx_position
        });
    } else if (position < m_last_rx_position) {
        process(std::span{
            m_rx_buffer.data() + m_last_rx_position,
            RxBufferSize - m_last_rx_position
        });
        process(std::span{m_rx_buffer.data(), position});
    }

    m_last_rx_position =
        (position == RxBufferSize) ? 0 : position;
}
```

还应明确以下行为：

- 重复收到同一 position 时不产生新数据。
- RX 启动或重启时把 `m_last_rx_position` 重置为 0。
- `RxBufferSize == 0` 时接口在编译期不可用。
- 回调中的处理必须足够快；如果消费者可能落后 DMA 一整圈，仅靠 position 已无法判断覆盖了多少数据，需要增加更上层的环形队列或丢包检测策略。

当前 `on_rx_event(..., size)` 的命名容易暗示“长度”，建议把参数和内部变量命名为 `write_position`。

## 4. 全局 HAL callback 由模块统一接管——接受为架构约束

`uart_port.cpp` 提供了以下 HAL 弱回调的全局实现：

- `HAL_UART_TxCpltCallback`
- `HAL_UART_RxCpltCallback`
- `HAL_UARTEx_RxEventCallback`
- `HAL_UART_ErrorCallback`

既然项目规定所有 UART 必须通过 `UartPort` 使用，这种统一入口设计可以接受。它能确保所有 UART 事件都经过 `UartPortManager` 分发。

建议把以下内容写入项目开发约定：

- 业务模块不得直接调用异步 HAL UART API。
- 业务模块不得再次定义上述全局 callback。
- 每个投入使用的 UART handle 必须且只能注册一个 `UartPort`。
- 达到 `MAX_UART_PORT` 或重复注册属于启动期致命配置错误。

如果未来要引入绕过 `UartPort` 的第三方组件，应先改成项目级 callback dispatcher；在当前约束下无需增加 fallback。

## 5. D-Cache 与 DMA 一致性——原理、风险和建议

### 当前工程状态

当前工程没有调用 `SCB_EnableDCache()`，所以现阶段不会发生 D-Cache 一致性问题。

`DMA_BUFFER` 当前做了两件事：

```cpp
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
```

- 把缓冲区放入 DMA 可访问的 `RAM_D2`，避开 DMA 不可访问的 DTCM。
- 按 Cortex-M7 的 32 字节 cache line 对齐缓冲区起始地址。

这两件事都正确，但“DMA 可访问”和“CPU 不缓存”是两个独立属性。链接到 `RAM_D2` 本身不会自动使该区域 non-cacheable。

### 为什么会出错

Cortex-M7 的 D-Cache 与 DMA 不保持硬件一致性。CPU 访问缓存，DMA 直接访问 SRAM，两者可能看到不同版本的数据。

TX 场景：

1. CPU 把待发送数据写入 `m_tx_buffer`。
2. 新数据可能只存在于 dirty cache line，尚未写回 RAM_D2。
3. DMA 从 RAM_D2 读取旧内容并发送。

因此，表现可能是“代码里明明已经 copy，串口却偶尔发送上一次的数据”。

RX 场景：

1. DMA 把新数据写进 RAM_D2。
2. CPU 对相同地址的读取可能命中旧 cache line。
3. 回调看到旧数据或新旧数据混合。

更危险的是：如果该 cache line 是 dirty 的，CPU 后续回写还可能覆盖 DMA 已写入的新数据。

这些问题通常与时序、缓冲区地址和调试行为有关，会呈现为低概率、难复现的数据错误，而不是稳定崩溃。

### 推荐方案：将专用 DMA 内存区设为 non-cacheable

当前链接脚本已经为 `.dma_buffer` 使用独立的 `RAM_D2`：

```ld
RAM_D2 (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
```

如果这个 32 KiB 区域专门用于 DMA，最简单可靠的方案是在启用 D-Cache 前，用 MPU 把 `0x30000000..0x30007fff` 配置为 non-cacheable：

- MPU region 基地址：`0x30000000`。
- region 大小：32 KiB。
- 基地址满足 32 KiB 对齐要求。
- full access、execute-never、non-cacheable。
- 使用比当前兜底 region 0 更高的 region number，使其属性优先。

建议通过 CubeMX 生成这项 MPU 配置，并检查生成结果。配置完成后，UART DMA 路径无需在每次传输时维护 cache，circular RX 也不会遇到“DMA 正在写，CPU 同时 invalidate 相邻 cache line”的复杂竞态。

代价是整个 32 KiB RAM_D2 的 CPU 访问都不享受 D-Cache。因此应把它明确保留给 DMA buffer，不要把计算密集型普通数据放入该区域。

如果未来不希望牺牲整个 RAM_D2，可以在链接脚本中预留一个满足 MPU 要求的专用块。MPU region 的大小必须是 2 的幂，基地址必须按 region 大小对齐；不能直接拿任意起止地址的 `.dma_buffer` section 配置。

### 备选方案：显式维护 cache

如果必须把 DMA buffer 放在 cacheable 内存中，则需要：

- TX：CPU 写完缓冲区后、启动 DMA 前，对覆盖该数据的 cache line 执行 clean。
- RX：启动 DMA 前先 invalidate，DMA 产生数据后、CPU 读取前再次 invalidate。
- cache 操作范围必须按 32 字节 cache line 向下/向上取整。
- 一个被维护的 cache line 中不能混入其他普通变量，否则 clean/invalidate 可能写回或丢弃不相关数据。
- circular RX 回卷时要分别处理尾段和头段，并认真处理 DMA 仍在写入时的竞态。

概念上的范围计算如下：

```cpp
constexpr std::uintptr_t cache_line = 32;

auto begin = reinterpret_cast<std::uintptr_t>(data) & ~(cache_line - 1U);
auto end = (reinterpret_cast<std::uintptr_t>(data) + size + cache_line - 1U)
         & ~(cache_line - 1U);
auto length = end - begin;
```

然后使用项目 CMSIS 版本提供的 `SCB_CleanDCache_by_Addr()` / `SCB_InvalidateDCache_by_Addr()`。实际调用还要按该 CMSIS 版本的函数签名转换参数，并配合必要的内存屏障。

虽然 `DMA_BUFFER` 已使每个变量起始地址 32 字节对齐，仍应确保分配范围的末尾不会与普通数据共享 cache line。链接脚本目前把整个 `.dma_buffer` 输出 section 的末尾也对齐到 32 字节，这是正确方向；若选择 cache maintenance，还应通过 map 文件确认每个对象的布局。

对当前项目，优先采用 MPU non-cacheable 方案；显式 cache maintenance 更适合必须保留 cache 性能的大型数据区，不建议作为 circular UART RX 的首选。

## 建议执行顺序

1. 先给阻塞 `receive()` / `transmit()` 增加长度边界检查，修改小且能消除确定的截断错误。
2. 在启用 circular ReceiveToIdle 前，实现 position 状态与回卷处理。
3. 保留 DMA handle 模板参数，并补充其“实例键”语义；条件允许时增加初始化后绑定校验。
4. 将 UART 全部通过 `UartPort` 的规定写入项目架构文档和代码注释。
5. 当前保持 D-Cache 关闭；将来启用前，先完成 RAM_D2 的 non-cacheable MPU 配置和验证。

## 后续验证项目

1. 两个使用不同 DMA handle、但缓冲区大小相同的 `UartPortDma`，其缓冲区地址不同。
2. DMA 模板参数与 `huart.hdmatx/hdmarx` 不匹配时，初始化或启动明确失败。
3. 阻塞接口对长度 0、1、65535、65536 的行为符合约定。
4. Circular DMA 在连续 IDLE、HT、TC 和跨缓冲区末尾时，每个字节只处理一次。
5. 未注册或重复注册 UART 时按架构约定失败，而不是静默丢事件。
6. 启用 D-Cache 后，分别用 MPU non-cacheable 和压力传输验证 DMA TX/RX 数据一致性。
