# ETH DMA 内存布局修正步骤

核对日期：2026-10-28。本文记录修正方案和验收条件，不表示相关代码已经修改或实机验证通过。

对应[新主控板配置待办](new-board-cubemx-config.md)中的 ETH DMA 内存布局条目。现有通用内存布局说明见 [memory-layout-and-dma-buffer.md](memory-layout-and-dma-buffer.md)。

## 1. 当前问题

STM32H723 的 ETH 正常帧收发依赖外设内置 DMA，不需要像 USART 一样在 DMA1/DMA2 中分配 Stream。使用阻塞式 ETH API 也仍然通过 DMA 传输。

需要区分以下对象：

| 对象 | 用途 | 当前状态 |
| --- | --- | --- |
| `DMARxDscrTab` / `DMATxDscrTab` | 保存 DMA 使用的数据地址、长度、所有权和收发状态 | CubeMX 已在 `Core/Src/eth.c` 中生成，并交给 `HAL_ETH_Init()` |
| RX/TX 数据缓冲区 | 存放实际收发的以太网帧字节 | 后续由应用层或协议栈提供 |
| `ETH_BufferTypeDef` 节点 | 保存数据指针、长度和下一节点指针，供软件提交数据 | `eth.c` 中的 `Txbuffer[]` 属于这一类，不是实际帧数据数组 |

GCC 分支将描述符数组放入 `.RxDescripSection` 和 `.TxDescripSection`。这些属性只指定输入段名，并未指定目标 RAM。当前 `STM32H723xG_flash.ld` 没有显式收集这两个段，核对时的 Debug map 显示：

- `DMARxDscrTab`：`0x20000414`。
- `DMATxDscrTab`：`0x20000474`。

两者都落在 ETH DMA 无法访问的 DTCM，必须重新安排。地址是当前构建的观察结果，不应作为后续固定地址使用。

`heth.Init.RxBuffLen = 1536` 仅配置接收缓冲区长度，不会分配数据内存。即使将来正确声明数据缓冲区，描述符本身的位置仍需修正。

## 2. 修正步骤

### 步骤 1：使用现有 RAM_D2

- 将两个描述符数组放入链接脚本现有的 `RAM_D2`：起始地址 `0x30000000`，大小 32 KiB。
- 与 UART DMA 缓冲区共用这块 SRAM，由链接器连续分配并检查容量。
- 不直接照抄 `eth.c` 中 IAR/MDK 分支的固定地址，避免覆盖现有 UART DMA 缓冲区。

### 步骤 2：扩展 .dma_buffer_bss 的段收集规则

修改 `STM32H723xG_flash.ld` 中现有的 `.dma_buffer_bss (NOLOAD)` 输出段：

1. 保留 `.dma_buffer_bss` 和 `.dma_buffer_bss.*` 的现有收集规则。
2. 在 `_edma_buffer_bss` 定义之前，显式收集 `.RxDescripSection` 和 `.TxDescripSection`，可用 `KEEP` 保留对应输入段。
3. 分别在收集 RX、TX 描述符段之前执行 32 字节对齐，保证两个数组各自的起始地址按 32 字节对齐。
4. 保留输出段末尾的 32 字节对齐，并在全部内容之后定义 `_edma_buffer_bss`。
5. 保持输出段归属 `RAM_D2`，并保持 `NOLOAD`。

此方案复用现有输出段，不需要改变 `eth.c` 中的描述符声明或 `heth.Init.RxDesc` / `heth.Init.TxDesc` 绑定。对齐要求针对两个数组的起始地址，不应擅自改变 `ETH_DMADescTypeDef` 的结构或数组元素步长。

### 步骤 3：核查启动清零范围

- 检查两个描述符数组均位于 `_sdma_buffer_bss` 到 `_edma_buffer_bss` 的范围内，结束符号表示范围之外的第一个地址。
- `startup_stm32h723xx.s` 已包含该范围的清零逻辑；按上述方案收集后可直接复用，无需另建清零循环。
- 确认清零发生在 `main()` 和 `MX_ETH_Init()` 之前。

### 步骤 4：确认缓存策略

- 当前工程未调用 `SCB_EnableDCache()`，可先维持现状完成内存布局修正。
- 将来启用 D-cache 时，需要同时考虑描述符和实际数据缓冲区的一致性。
- 为相关区域配置适合的 MPU 属性，或按 DMA 读写方向实施缓存维护，并核查 cache line 对齐及所有权交接顺序。
- 32 字节对齐本身不会自动解决缓存一致性。

### 步骤 5：构建并核查 map

分别执行 Debug、Release 编译链接，在各自 map 中核对：

- 两个描述符数组位于 `.dma_buffer_bss` 输出段内。
- 两个数组的起始地址均为 32 字节对齐。
- 两个数组的完整占用范围均位于 `0x30000000` 至 `0x30007FFF`。
- 数组彼此之间、与 UART `.dma_buffer_*` 缓冲区之间没有重叠。
- `_sdma_buffer_bss` / `_edma_buffer_bss` 覆盖两个数组。
- 两个输入段不再被单独放入 DTCM，RAM_D2 总占用没有超过 32 KiB。

不要仅以编译通过作为布局正确的依据，应检查实际符号地址和段范围。

### 步骤 6：实现收发时提供实际数据缓冲区

这一步属于后续 ETH 收发功能实现，不是描述符重定位的前置条件：

- 为 RX 提供 DMA 可访问的数据缓冲区池，并接入当前 HAL 所需的接收分配、接收数据链接回调。
- 为 TX 提供 DMA 可访问的数据内存，并通过发送配置提交数据指针和长度。
- 静态缓冲区可使用 `BSP/memory_map.h` 中现有的 `DMA_BUFFER_BSS`，同时确认总容量和最终地址。
- 保证缓冲区在 DMA 使用期间保持有效，按收发完成状态回收或复用。
- 若使用协议栈分配内存，需要核查实际数据区的位置，不能只检查软件节点或指针变量的位置。

## 3. 验收清单

- [ ] 链接脚本显式收集两个描述符输入段，且不依赖固定地址。
- [ ] 描述符纳入现有 D2 SRAM 启动清零范围。
- [ ] Debug、Release 均编译链接通过。
- [ ] 两种构建的 map 均确认描述符位置、对齐、边界及容量正确。
- [ ] 接入收发后，实际 RX/TX 数据缓冲区的位置、生命周期和缓存策略已核查。
- [ ] PHY 复位时序及参考时钟已就绪，并完成实机收发验证。

描述符布局修正不会自动解决 PHY 复位、参考时钟或应用层收发实现；这些仍需分别完成。

## 4. 硬件参考

- [STM32H723/733、STM32H725/735、STM32H730 参考手册 RM0468](https://www.st.com/resource/en/reference_manual/rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)：核查总线可访问范围、ETH DMA 和描述符机制。
