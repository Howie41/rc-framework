# memory_map

假设要准备一个 DMA 收发缓冲区，再放一组比较大的采样数据。DMA 缓冲区使用 `DMA_BUFFER_*`，普通大数组使用 `RAM_D1_*`。

```cpp
#include "memory_map.h"
#include <cstdint>

// 以下都放在函数外。也可以用于类里的 static 变量。
DMA_BUFFER_BSS static uint8_t rx_buffer[256]{};          // DMA 接收，初始全零
DMA_BUFFER_DATA static uint8_t tx_header[]{0xAA, 0x55}; // DMA 发送，保留这两个初值
RAM_D1_BSS static uint16_t samples[1024]{};             // 普通大数组，初始全零
RAM_D1_DATA static uint16_t values[]{1, 2, 3};          // 普通数据，保留初值
```

这些宏只决定数据放在哪里，不会替你启动 DMA 或收发数据。使用 [UartPortDma](../uart_port/README.md) 时，它已经准备好 DMA 缓冲区，不用再额外建一份。

### 选择哪个宏

本项目的普通全局变量和 `static` 变量默认放在 DTCMRAM，DMA1/DMA2 不能直接使用这里的数据。RAM_D1 配置了 320 KB，较大的普通数组、静态任务栈可以放过去。

| 使用场景 | 选择 |
| --- | --- |
| DMA 直接收发的缓冲区，初始全零 | `DMA_BUFFER_BSS` |
| DMA 直接收发的缓冲区，需要保留非零初值 | `DMA_BUFFER_DATA` |
| 普通大数组、静态任务栈等，初始全零 | `RAM_D1_BSS` |
| 普通数据放到 RAM_D1，需要保留非零初值 | `RAM_D1_DATA` |

没有特殊需求的变量不加宏。以上数据宏只用于全局变量或 `static` 变量，不要加在普通局部变量上。

### DMA_BUFFER_BSS / DMA_BUFFER_DATA

准备 DMA 直接使用的数据。在变量声明前加上对应的宏即可。

- `DMA_BUFFER_BSS`
  - 初始全零时用它，写 `{}` 或不写初值都可以。
- `DMA_BUFFER_DATA`
  - 需要保留非零初值时用它，例如上面的 `tx_header`。
- `DMA_BUFFER`
  - 旧写法，等同于 `DMA_BUFFER_BSS`。新代码写完整名称。

### RAM_D1_BSS / RAM_D1_DATA

把普通数据放到 RAM_D1。较大的采样数组、静态任务栈等可以用这两个宏。

- `RAM_D1_BSS`
  - 初始全零时使用。
- `RAM_D1_DATA`
  - 需要保留非零初值时使用。
- `RAM_D1`
  - 旧写法，等同于 `RAM_D1_BSS`。新代码写完整名称。

### ITCM_FUNCTION

明确需要把某个函数放到 ITCM 时使用。普通函数没有这个需求就不用加。

- `ITCM_FUNCTION`
  - 写在函数返回类型前，例如下面这样。它不负责调用函数。

  ```cpp
  ITCM_FUNCTION int add(int a, int b) {
      return a + b;
  }
  ```

### 特别注意

- DMA 收发缓冲区统一使用 `DMA_BUFFER_*`；普通大数组、静态任务栈再考虑 `RAM_D1_*`。

  ```cpp
  DMA_BUFFER_BSS static uint8_t rx_buffer[256]{}; // DMA 接收用
  RAM_D1_BSS static uint16_t samples[1024]{};     // 普通数据存储用
  ```

- 对这里的整数和数组，使用 `BSS` 时不要填非零初值。需要保留初值就改用 `DATA`，模块不会根据你的写法自动选择。

  ```cpp
  DMA_BUFFER_BSS static uint8_t bad_header[]{0xAA, 0x55}; // 错误：不能保留这两个初值
  DMA_BUFFER_DATA static uint8_t header[]{0xAA, 0x55};   // 正确：保留初值
  ```

- `DATA` 会用 Flash 保存整个变量或数组的初值，不是只保存非零的部分。下面这样可以用，但会多占约 1 KB Flash。

  ```cpp
  DMA_BUFFER_DATA static uint8_t data[1024]{0xAA, 0x55};
  // 保存的是 AA 55 00 00 …… 00，总共 1024 字节。
  ```

- 大缓冲区只有几个字节需要预设时，可以用 `BSS`，再在首次使用前赋值。不用保存整份数组初值，但赋值代码和常量仍占少量 Flash。

  ```cpp
  DMA_BUFFER_BSS static uint8_t tx_buffer[1024]{};
  // 以下赋值写在初始化函数里，启动 DMA 之前执行：
  tx_buffer[0] = 0xAA; tx_buffer[1] = 0x55;
  ```
