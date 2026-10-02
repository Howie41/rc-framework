# D1/D2 内存段与初始化

[`BSP/memory_map.h`](../BSP/memory_map.h) 为 D1 AXI SRAM 提供 `RAM_D1_DATA`、`RAM_D1_BSS`，为 DMA 可访问的 D2 SRAM 提供 `DMA_BUFFER_DATA`、`DMA_BUFFER_BSS`。DATA 段的初值保存在 Flash，`Reset_Handler` 启动时复制到 RAM；BSS 段不保存初值，启动时清零。`SystemInit()` 在处理 D2 段前开启 D2 SRAM 时钟。旧宏 `RAM_D1`、`DMA_BUFFER` 分别是对应 BSS 宏的别名。

以下均指带宏的**全局或 `static` 变量**，例如 `RAM_D1_DATA static int a{5};`。D1 与 D2 的初始化规则相同：

| C++ 写法 | 放在 DATA 段 | 放在 BSS 段 | 应选 |
| --- | --- | --- | --- |
| `int a{5};` | 从 Flash 复制，启动后为 5 | 启动时清零，变为 0 | DATA |
| `int a = 5;` | 从 Flash 复制，启动后为 5 | 启动时清零，变为 0 | DATA |
| `int a{};` | 启动后为 0，但占用 Flash 初值空间 | 启动后为 0 | BSS |
| `int a;` | 启动后为 0，但占用 Flash 初值空间 | 启动后为 0 | BSS |

`int a{}` 是空列表初始化；对这里的静态存储期变量，`int a;` 也要求初值为 0。宏只决定所在段，**不会根据初始化写法自动选择 DATA 或 BSS**。
