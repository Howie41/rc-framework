# 新主控板配置待办

调试串口目标为 **USART1（`huart1`），115200、8N1**。本文只保留未完成或待确认的事项；核对日期：2026-10-28。

## 1. 调试任务切换至 USART1

`APP/tasks/debug_task.cpp` 仍绑定 UART4，需要修改：

- [ ] 模板实例标识从 `"uart4"` 改为 `"uart1"`。
- [ ] 构造参数从 `huart4` 改为 `huart1`。
- [ ] 对象名及所有引用从 `uart4_debug_port` 改为 `uart1_debug_port`。
- [ ] 将 `PidViewer::heartbeat(rtos::now_tick())` 改为 `PidViewer::heartbeat(rtos::now_ms())`。

目标声明如下，继续保留现有缓冲区大小：

```cpp
UartPortDma<"uart1", 64, 128, 512> uart1_debug_port(huart1, {});
```

## 2. FreeRTOS FPU 配置同步

`.ioc` 中 `FREERTOS.configENABLE_FPU=1`，但 `Core/Inc/FreeRTOSConfig.h` 中仍为 0。

- [ ] 在 CubeMX 确认 FPU Support 已启用，重新生成后核对 `configENABLE_FPU` 是否为 1。
- [ ] 若仍不一致，检查生成目录和配置来源，避免只修改生成头文件而在下次生成时丢失。

当前 ARM_CM4F 移植本身会启用 VFP；这里需要解决的是配置与生成结果不一致，不能仅凭该宏为 0 认定浮点上下文必然失效。

## 3. ETH DMA 内存布局

`Core/Src/eth.c` 使用 `.RxDescripSection` 和 `.TxDescripSection`，但 `STM32H723XG_FLASH.ld` 尚未显式安排这两个段。此前 map 中描述符落入 DTCM，不能供 ETH DMA 使用。

- [ ] 在链接脚本中将 RX/TX 描述符段放入 ETH DMA 可访问的 SRAM。
- [ ] 核对对齐、初始化方式，以及与 UART `.dma_buffer_*` 的边界，避免重叠。
- [ ] 实际接入以太网收发时，同样检查 RX/TX 数据缓冲的位置。
- [ ] 若启用 D-cache，为 DMA 区域明确 MPU 属性或补齐缓存维护。

不要直接照抄 IAR 分支的固定地址；GCC 链接脚本需要显式预留和安排对应空间。

## 4. ETH PHY 复位与启动时机

`MX_GPIO_Init()` 将 `ETH_RST` 置低，尚未找到释放操作；`main()` 随后调用 `MX_ETH_Init()`，早于 `Robot::init()`。

- [ ] 按原理图确认 PHY 复位极性、复位保持时间、释放后的等待时间和参考时钟来源。
- [ ] 若低电平表示复位，在 ETH 初始化之前释放复位并等待所需时钟就绪。
- [ ] 若暂不使用以太网，明确是否暂缓 `MX_ETH_Init()`，避免外部器件未就绪阻塞启动。

复位时序的具体数值需依据 PHY 型号及硬件资料确定。

## 5. 待确认的可选项

### USB CDC

- [ ] 确认新板是否需要 USB 虚拟串口。不需要时，此项可直接关闭。
- [ ] 若需要，在 USB_DEVICE 中启用 CDC，并启用 OTG_HS 中断，优先级 `5,0`。目标使用内部 FS PHY、Full Speed，USB 时钟 48 MHz。

### 堆大小优化

- [ ] 如需进一步压缩 RAM，核查动态 API 使用并测量实际分配峰值，再决定是否缩小当前 15360 bytes 的 FreeRTOS 堆；不要求恢复为 49152 bytes。
- [ ] C 库堆 `_Min_Heap_Size = 0x1000` 单独评估，不与 FreeRTOS 堆混算。

框架主要采用静态分配。若决定完全禁用动态分配，需要同时处理 CMSIS 动态 API 和 `heap_4.c` 的构建，不能只把堆大小设为 0。

## 6. 后续验证

- [ ] 完成 USART1 代码绑定和 FPU 同步，再处理或暂缓 ETH 初始化。
- [ ] 执行新板配置的 Debug、Release 编译链接，并核对 map 中 ETH 描述符与 UART DMA 缓冲的实际地址。
- [ ] 在新板验证 RTOS 启动、USART1 收发、PID Viewer 和三路 CAN 通信，记录任务栈及动态堆余量。
- [ ] 对实际启用的 ETH、USB CDC、OSPI、ADC、I2S、PWM 等功能分别进行实机验证。
