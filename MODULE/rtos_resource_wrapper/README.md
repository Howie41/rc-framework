
## 声明与初始化

资源对象声明在 `.cpp` 文件的全局作用域，构造时自动注册，生命周期应覆盖整个程序运行期

在 `freertos.c` 的 `MX_FREERTOS_Init()` 中调用统一初始化接口，必须在调度器启动前、所有资源对象构造后执行

```c
#include "rtos_resource_init.h"

// 在 MX_FREERTOS_Init() 中调用
rtos_resource_init();
```

该接口依次创建队列、流缓冲区和任务，重复调用不会再次创建，初始化后不能再注册新资源

下文的声明放在全局作用域，收发和时间查询示例放在任务函数中，资源初始化完成后再使用

## Task：任务

```cpp
#include "rtos_task.hpp"

rtos::Task<"worker", ([] {
    for (;;) {
        // 在这里执行任务工作
        rtos::delay(10);
    }
}), 128 * 4, rtos::TaskPriority::Normal> worker;
```

模板参数依次为任务名称、无捕获 `void()` Lambda、栈大小（字节）、优先级

- 默认栈大小为 `1024` 字节，默认优先级为 `rtos::TaskPriority::Normal`
- 栈大小至少 `512` 字节，且必须是 `sizeof(StackType_t)` 的整数倍
- 可用优先级为 `Idle`、`Low`、`BelowNormal`、`Normal`、`AboveNormal`、`High`、`Realtime`，还须小于 `configMAX_PRIORITIES`
- `None` 和 `Isr` 不能用于创建任务

也可以将入口 Lambda 单独命名，显式指定栈大小和优先级

```cpp
constexpr auto entry = [] {
    for (;;) {
        rtos::delay(10);
    }
};
rtos::Task<"worker", entry, 2048, rtos::TaskPriority::Low> worker;
```

| 接口 | 含义 |
| --- | --- |
| `worker.handle()` | 获取 `TaskHandle_t`，创建前为 `nullptr` |
| `rtos::delay(ticks)` | 阻塞当前任务指定 tick 数，`0` 不阻塞 |
| `rtos::delay_until(ticks)` | 阻塞到指定的绝对 tick，目标应在未来，不考虑 tick 溢出 |

`delay` 和 `delay_until` 定义在 `rtos_task.hpp`，参数单位均为 tick，不是毫秒，仅在任务上下文中使用

## Queue：定长元素队列

```cpp
#include "rtos_queue.hpp"

rtos::Queue<int, 4> values;
```

模板参数依次为元素类型、元素数量，数量必须大于 `0`，元素类型必须平凡可拷贝且可默认构造

```cpp
bool sent = values.put(42);  // 不等待，返回是否成功
if (auto value = values.get()) {
    int result = *value;
    // 使用 result
}
```


| 接口 | 含义 |
| --- | --- |
| `put(item, timeout_ms = 0)` | 复制元素到队尾，返回 `bool` |
| `get(timeout_ms = 0)` | 取出队首元素，返回 `std::optional<T>`，失败时为空 |
| `peek(timeout_ms = 0)` | 查看队首元素，不移出队列，返回 `std::optional<T>` |
| `overwrite(item)` | 覆盖元素，队列为空时直接放入，仅适用于长度为 `1` 的队列 |
| `handle()` | 获取 `QueueHandle_t`，创建前为 `nullptr` |


`get()` 和 `peek()` 返回的 `std::optional<T>` 表示“可能有一个 `T` 类型的值”。先用 `if (value)` 或 `value.has_value()` 判断是否有值，再用 `*value` 或 `value.value()` 取得元素；如果元素是结构体，也可以用 `value->成员` 访问成员

```cpp
auto value = values.get();
if (value.has_value()) {
    int result = value.value();  // 与 *value 等价，取得队列中的 int
    // 使用 result
}

auto front = values.peek();
int result = front.value_or(-1);  // 有值时取值，否则使用默认值 -1
```

空的 `optional` 不能使用 `*value` 或 `value->成员`，否则行为未定义；调用 `value.value()` 会抛出 `std::bad_optional_access`。嵌入式项目应先判断是否有值，或用 `value_or(默认值)` 处理获取失败的情况。`if (value)` 判断的是是否有值，即使存储的整数是 `0`，条件也为真



只保留最新值时使用单元素队列

```cpp
rtos::Queue<int, 1> latest;  // 全局声明

// 在任务函数中使用
latest.overwrite(42);
```

## StreamBuffer：字节流缓冲区

```cpp
#include "rtos_stream_buffer.hpp"

rtos::StreamBuffer<128> bytes;
```

模板参数依次为可存储字节数、接收唤醒阈值，阈值默认为 `1`，范围为 `1` 到容量

容量至少为 `sizeof(configMESSAGE_BUFFER_LENGTH_TYPE)`，需要等到累计 `16` 字节才唤醒等待接收的任务时，可声明为 `rtos::StreamBuffer<128, 16>`

```cpp
std::uint8_t input[]{1, 2, 3};
std::uint8_t output[8]{};
size_t sent = bytes.send(input);
size_t received = bytes.receive_to(output);
```

| 接口 | 含义 |
| --- | --- |
| `send(std::span<const std::uint8_t> data, timeout_ms = 0)` | 发送字节，返回实际发送字节数 |
| `receive_to(std::span<std::uint8_t> output_buffer, timeout_ms = 0)` | 接收字节，返回实际接收字节数 |
| `handle()` | 获取 `StreamBufferHandle_t`，创建前为 `nullptr` |

流缓冲区不保留消息边界，发送可能只写入部分输入，接收可能不填满输出，需要按返回的字节数处理

## 超时与中断上下文

- 队列和流缓冲区的 `timeout_ms` 单位为毫秒，`0` 表示不等待，`rtos::wait_forever` 表示永久等待
- 收发接口自动选择任务或中断版本，中断上下文中 `timeout_ms` 必须为 `0`
- 流缓冲区零超时收发使用临界区保护，复制的数据越多，对中断延迟的影响越大
- 流缓冲区非零超时收发没有临界区保护，同一缓冲区应遵守单写者、单读者约束

## rtos_utility.hpp：时间与上下文接口

```cpp
#include "rtos_utility.hpp"

auto tick = rtos::now_tick();
auto ms = rtos::now_ms();
auto timeout = rtos::timeout_ms_to_ticks(10);
bool in_irq = rtos::is_irq_context();
```

| 接口 | 返回类型 | 含义 |
| --- | --- | --- |
| `now_tick()` | `TickType_t` | 当前 RTOS tick，在任务上下文中调用 |
| `now_ms()` | `std::uint32_t` | 当前 RTOS 毫秒时间，按 `now_tick() * portTICK_PERIOD_MS` 计算，在任务上下文中调用，允许自然回绕 |
| `timeout_ms_to_ticks(std::uint32_t timeout_ms)` | `TickType_t` | 普通值通过 `pdMS_TO_TICKS` 转换，`wait_forever` 转为 `portMAX_DELAY` |
| `is_irq_context()` | `bool` | 在中断服务中返回 `true`，调度器启动后检测到 `PRIMASK` 或 `BASEPRI` 屏蔽中断时也返回 `true`，调度器尚未启动且不在中断服务中时返回 `false` |
| `wait_forever` | `constexpr std::uint32_t` | 毫秒超时参数的永久等待值，值为 `0xFFFFFFFFU` |

## 管理器接口

`TaskManager`、`QueueManager` 和 `StreamBufferManager` 均提供 `init()` 和 `get_count()`，分别用于初始化已注册资源和获取成功创建的数量，通常只需调用统一入口 `rtos_resource_init()`
