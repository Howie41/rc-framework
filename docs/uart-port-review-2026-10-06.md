# UART Port 代码审查（2026-10-06）

## 范围与验证

审查当前工作区的 `BSP/uart_port/uart_port.hpp`、`uart_port.cpp`，并核对 `MODULE/rtos_resource_wrapper/rtos_stream_buffer.hpp`、UART4 的 CubeMX 配置、HAL 实现及 ARM 链接 map。`uart_port.hpp` 和部分配置文件有未提交修改；下列结论针对审查时的工作区内容。旧文档 `uart-port-review.md` 记录的是 2026-09-21 的版本，不能代替本次结论。

- 流缓冲区改造后，`cmake --build build/codex-uart-queue -j 4` 通过；使用项目 ARM 编译器分别实例化接收流缓冲区容量为 0 和 64 的 UART 端口，语法编译均通过。
- 用实际 `uart_port.hpp` 和可控制容量的流缓冲区替身进行主机行为检查：连续接收、重复 HT 位置、TC、回卷的尾段和头段、部分写入及丢失字节计数均通过。尚未在板上验证 DMA 和 FreeRTOS 的实际运行行为。
- 改造前用户编译的 `build/Debug/ROBOCON27.map` 和 ELF 曾显示旧 UART4 实例的 RX 缓冲区在 `0x30000000`，旧接收队列在 `0x24000000`。该构建早于流缓冲区改造，不能用于验证新实例的地址。
- 本次 GCC 13.3 构建的 map 把 RX 缓冲区和流缓冲区都放在普通 `.bss`；当前 `build/Debug` 配置的 GCC 14.3.1 / Ninja 工具路径在本执行环境不可用，新实例在目标构建中的地址仍待复核。

## 审查发现

### 1. 已核实当前 Debug 构建：RX DMA 缓冲区位于 RAM_D2

流缓冲区改造前的 `build/Debug` map 和 ELF 一致：旧实例 `UartPortDma<"uart4", 64, 64, 1>::m_rx_buffer` 在 RAM_D2 的 `0x30000000..0x3000003f`，旧接收队列在 RAM_D1 的 `0x24000000`。第 1 点原先所称的该份固件 RX 缓冲区落在 DTCMRAM 不成立。流缓冲区改造后的目标构建尚未核对，不能沿用旧地址结论。

证据：[缓冲区声明](../BSP/uart_port/uart_port.hpp#L189-L190)、[段属性](../BSP/memory_map.h#L5-L11)、[内存区域](../STM32H723XG_FLASH.ld#L55-L60)、[DMA 段](../STM32H723XG_FLASH.ld#L244-L262)，以及改造前 `build/Debug/ROBOCON27.map` 和 ELF 的旧实例符号地址。

`m_tx_buffer` 尚未出现在当前实例的 ELF 中，因为发送 DMA 路径没有被使用；它的实际链接地址仍需在启用该路径的固件中核对。GCC 13.3 构建中的 RX 地址不同，不能把旧 Debug 结果推广到所有构建配置。建议用最终目标工具链重新生成 map，确认 RX/TX 缓冲区完整位于 `0x30000000..0x30008000`、流缓冲区位于 RAM_D1；如需运行时防护，可恢复缓冲区地址范围断言。当前 `Core/Src/usart.c` 的 UART4 RX 为 `DMA_CIRCULAR`、TX 为 `DMA_NORMAL`，模式符合本类的断言要求。

### 2. 已修复：禁用接收流缓冲区时可实例化

原队列实现把容量 0 对应的成员类型设为 `void`，无法实例化。现改为 `RxStreamBufferSize = 0` 时使用空类型，并以 `if constexpr` 排除整个发送操作；UART4 使用 64 字节流缓冲区，唤醒阈值采用封装的默认值 1。容量 0 和 64 的实例均通过 ARM 语法编译。

证据：[模板参数与禁用类型](../BSP/uart_port/uart_port.hpp#L177-L245)、[条件发送](../BSP/uart_port/uart_port.hpp#L262-L274)、[UART4 实例](../APP/debug_task/debug_task.cpp#L10)。

任务端通过 `m_rx_stream_buffer.receive()` 读取连续字节。流缓冲区不保留 DMA 回调的分段边界；`on_receive_to_idle_chunk` 回调仍按原先的连续段触发。

### 3. 高：TX DMA 出错后，软件忙标志可能永久保持为真

`start_transmit_dma()` 先设置 `m_is_tx_busy`；正常发送完成或主动调用 `abort_all_transmit()` 才会清除它。HAL 的 `UART_DMAError()` 会结束出错的 TX 传输并调用 `HAL_UART_ErrorCallback()`，但 `UartPortManager::hal_on_error()` 只转发错误，不清除忙标志。此后 HAL 已可能处于 `READY`，而本类所有后续 `start_transmit_dma()` / `start_transmit_it()` 都因 `try_acquire_tx()` 失败返回 `HAL_BUSY`。当前 UART4 的 `on_error` 回调没有恢复动作。

证据：[申请与释放忙标志](../BSP/uart_port/uart_port.hpp#L61-L79)、[DMA 发送](../BSP/uart_port/uart_port.hpp#L215-L233)、[错误转发](../BSP/uart_port/uart_port.hpp#L383-L390)、[HAL 的 DMA 错误路径](../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c#L3838-L3866)。

建议：针对 TX 错误建立明确的状态恢复路径，并验证 DMA 错误后可以再次发送；清除忙标志时须区分 RX 专属错误，避免错误地解除正在进行的 TX 传输。

### 4. 中：阻塞收发的 `span` 长度会截断为 16 位

`receive()` 和 `transmit()` 将 `span.size()` 直接转换为 `uint16_t`。例如长度 65537 会被传给 HAL 为 1，可能导致调用者误以为整个缓冲区已收发。IT/DMA 路径有容量检查，但这两个阻塞接口没有。

证据：[阻塞接收与发送](../BSP/uart_port/uart_port.hpp#L92-L110)。

建议：转换前拒绝大于 `UINT16_MAX` 的输入；另行定义空缓冲区的返回语义。

### 5. 已修复两次复制；仍需处理接收溢出

现在每个新增的连续数据段从 DMA 缓冲区直接写入流缓冲区，按实际数据长度复制，不再创建中断栈上的 `DataChunk<RxBufferSize>` 或再次复制完整队列元素。空间不足时，`send()` 保留能写入的前缀；`m_rx_stream_dropped_bytes` 累计未写入的字节数，任务端应检查该计数并处理协议流中的缺口。DMA 环形缓冲区在两次处理间绕行一整圈以上时，仍无法仅凭 HAL 的 `size` 检出被覆盖的数据。

证据：[流缓冲区写入与计数](../BSP/uart_port/uart_port.hpp#L244-L300)、[封装的部分写入返回值](../MODULE/rtos_resource_wrapper/rtos_stream_buffer.hpp#L128-L160)。

建议：按持续流量和任务调度延迟评估 64 字节容量，并在板上压力测试中监测丢失字节计数与数据连续性。

## 建议处理顺序

1. 第 1 点已在当前 Debug 固件中核实 RX 地址；启用 TX DMA 时继续核对 TX 地址，并按目标工具链检查 map。
2. 容量 0 的实例化和流缓冲区接入已完成；继续验证 TX DMA 错误后的重发。
3. 补齐阻塞长度检查，评估流缓冲区容量与持续接收压力。

本次未进行板上 DMA、错误注入或持续接收压力测试；改造后的目标工具链 map 仍需核对。
