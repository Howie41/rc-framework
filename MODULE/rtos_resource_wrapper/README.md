## 介绍

`rtos_resource_wrapper` 提供 FreeRTOS 常用资源自动静态注册和管理功能，将资源配置集中在对象声明处，减少重复编写控制块、存储数组和创建代码。

### 模块组成

| 文件 | 功能 |
| --- | --- |
| `rtos_task.hpp` | `Task`：封装任务名称、入口、栈大小和优先级，并提供任务延时辅助函数 |
| `rtos_queue.hpp` | `Queue`：传递固定类型的元素，支持入队、出队、查看队首和单元素覆盖 |
| `rtos_stream_buffer.hpp` | `StreamBuffer`：传递连续字节流，支持容量和接收唤醒阈值配置 |
| `rtos_utility.hpp` | 提供时间查询、毫秒到 tick 的超时转换、中断上下文检测和永久等待常量 |
| `rtos_resource_init.h/.cpp` | 提供可从 C 调用的统一资源初始化入口 `rtos_resource_init()` |

### 工作方式

任务栈、队列存储区、流缓冲区和 FreeRTOS 控制块均由对象持有，使用 FreeRTOS 静态创建接口，无需为这些资源动态分配堆内存。模板参数在编译期确定，类型、容量、栈大小等约束通过编译期检查验证

对象构造时自动注册到对应管理器，随后由 `rtos_resource_init()` 依次创建队列、流缓冲区和任务。资源对象应在全局作用域声明，生命周期覆盖整个程序运行期；本模块不负责在对象析构时删除 RTOS 资源

队列和流缓冲区的收发接口自动选择任务或中断版本，超时参数统一使用毫秒，中断上下文中只能使用零超时。队列获取结果使用 `std::optional<T>` 表示成功或失败，字节流收发则返回实际传输的字节数

## 移植

在 `freertos.c` 的 `MX_FREERTOS_Init()` 中调用统一初始化接口，**必须在调度器启动前、所有资源对象构造后执行**

```c
#include "rtos_resource_init.h"

// 在 MX_FREERTOS_Init() 中调用
rtos_resource_init();
```

### 注意事项

该接口依次创建队列、流缓冲区和任务，重复调用不会再次创建，初始化后不能再注册新资源

下文的声明放在全局作用域，收发和时间查询示例放在任务函数中，资源初始化完成后再使用

## Task：任务

### 构造

#### 模板和构造函数参数说明

模板签名（位于 `rtos` 命名空间）

```cpp
template <StaticString Name,
          auto EntryFunction,
          size_t StackBytes = 256 * 4,
          TaskPriority Priority = TaskPriority::Normal>
class Task;
```

| 模板参数 | 类型 / 形式 | 默认值 | 含义与约束 |
| --- | --- | --- | --- |
| `Name` | `StaticString` 编译期字符串 | 无，必填 | 任务名称，可直接传入字符串字面量，例如 `"worker"`，创建时作为 FreeRTOS 任务名 |
| `EntryFunction` | `auto`，无捕获 Lambda 对象 | 无，必填 | 任务入口，必须可转换为 `void (*)()`，即无参数、返回 `void`；应持续运行或自行删除当前任务，不能直接返回 |
| `StackBytes` | `size_t` 编译期整数 | `256 * 4`（1024 字节） | 任务栈大小，单位为字节；至少 512 字节，且必须是 `sizeof(StackType_t)` 的整数倍 |
| `Priority` | `rtos::TaskPriority` | `rtos::TaskPriority::Normal` | 任务优先级；须大于等于 `Idle`、小于 `Isr`，对应数值还须小于 `configMAX_PRIORITIES` |

构造函数签名：`Task()`，无运行时参数；构造时只向管理器注册对象，RTOS 资源由 `rtos_resource_init()` 统一创建

#### 构造示例

```cpp
#include "rtos_task.hpp"

rtos::Task<"worker", ([] {
    for (;;) {
        // 在这里执行任务工作
        rtos::delay(10);
    }
}), 128 * 4, rtos::TaskPriority::Normal> worker;
```

也可以将入口 Lambda 单独命名，显式指定栈大小和优先级（与上例二选一）

```cpp
constexpr auto entry = [] {
    for (;;) {
        rtos::delay(10);
    }
};
rtos::Task<"worker", entry, 2048, rtos::TaskPriority::Low> worker;
```

#### 注意事项

对象应在全局作用域声明，在统一初始化前完成构造，生命周期覆盖整个程序运行期；不支持拷贝构造和拷贝赋值

命名优先级为 `Idle`、`Low`、`BelowNormal`、`Normal`、`AboveNormal`、`High`、`Realtime`，`None` 和 `Isr` 不能用于创建任务。

省略末尾两个模板参数时，使用默认栈大小和优先级，例如 `rtos::Task<"worker", entry> worker;`

任务入口应持续运行或自行删除当前任务，不能直接返回

### 方法

#### 方法签名和含义介绍

| 方法签名 | 含义 |
| --- | --- |
| `TaskHandle_t handle() const` | 获取任务句柄，创建前为 `nullptr` |

`rtos_task.hpp` 还提供以下辅助函数（位于 `rtos` 命名空间）

| 函数签名 | 含义 |
| --- | --- |
| `void delay(uint32_t ticks)` | 阻塞当前任务指定 tick 数，`0` 不阻塞 |
| `void delay_until(uint32_t ticks)` | 阻塞到指定的绝对 tick，目标应在未来，不考虑 tick 溢出 |

`delay` 和 `delay_until` 定义在 `rtos_task.hpp`，参数单位均为 tick，不是毫秒，仅在任务上下文中使用

## Queue：定长元素队列

### 构造

#### 模板和构造函数参数说明

模板签名（位于 `rtos` 命名空间）

```cpp
template <typename T, size_t QueueSize>
class Queue;
```

| 模板参数 | 类型 / 形式 | 默认值 | 含义与约束 |
| --- | --- | --- | --- |
| `T` | 类型参数 | 无，必填 | 队列元素类型，例如 `int` 或结构体；必须满足 `std::is_trivially_copyable_v<T>` 和 `std::is_default_constructible_v<T>`，收发时复制元素字节 |
| `QueueSize` | `size_t` 编译期整数 | 无，必填 | 最多可存储的元素数量，单位是元素个数，必须大于 `0`；调用 `overwrite()` 时必须为 `1` |

构造函数签名：`explicit Queue()`，无运行时参数；构造时只向管理器注册对象，RTOS 资源由 `rtos_resource_init()` 统一创建

#### 构造示例

```cpp
#include "rtos_queue.hpp"

rtos::Queue<int, 4> values;
```

`rtos::Queue<int, 4>` 可存储最多 4 个 `int` 元素

只保留最新值时使用单元素队列

```cpp
rtos::Queue<int, 1> latest;  // 全局声明
```

#### 注意事项

对象应在全局作用域声明，在统一初始化前完成构造，生命周期覆盖整个程序运行期；不支持拷贝构造和拷贝赋值

`overwrite()` 只适用于 `QueueSize == 1` 的队列，元素类型必须平凡可拷贝且可默认构造

### 方法

#### 方法签名和含义介绍

| 方法签名 | 含义 |
| --- | --- |
| `bool put(const T& item_ref, uint32_t timeout_ms = 0)` | 复制元素到队尾，成功返回 `true`，失败返回 `false` |
| `std::optional<T> get(uint32_t timeout_ms = 0)` | 取出队首元素，返回 `std::optional<T>`，失败时为空 |
| `std::optional<T> peek(uint32_t timeout_ms = 0)` | 查看队首元素，不移出队列，失败时返回 `std::nullopt` |
| `bool overwrite(const T& item_ref)` | 覆盖元素，队列为空时直接放入，仅适用于长度为 `1` 的队列；返回是否成功 |
| `QueueHandle_t handle() const` | 获取 `QueueHandle_t`，创建前为 `nullptr` |

#### 方法使用示例

```cpp
// 在任务函数中使用
bool sent = values.put(42);  // 不等待，返回是否成功
if (auto value = values.get()) {
    int result = *value;
    // 使用 result
}
latest.overwrite(42);
```

#### optional 取值

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

空的 `optional` 不能使用 `*value` 或 `value->成员`，否则行为未定义。在启用异常的环境中，对空值调用 `value.value()` 会抛出 `std::bad_optional_access`；本项目工具链使用 `-fno-exceptions`，不能依靠捕获异常处理这种错误，应先判断是否有值，或用 `value_or(默认值)` 处理获取失败的情况。`if (value)` 判断的是是否有值，即使存储的整数是 `0`，条件也为真

## StreamBuffer：字节流缓冲区

### 构造

#### 模板和构造函数参数说明

模板签名（位于 `rtos` 命名空间）

```cpp
template <size_t BufferSize, size_t TriggerLevel = 1>
class StreamBuffer;
```

| 模板参数 | 类型 / 形式 | 默认值 | 含义与约束 |
| --- | --- | --- | --- |
| `BufferSize` | `size_t` 编译期整数 | 无，必填 | 可存储的字节数；必须大于等于 `sizeof(configMESSAGE_BUFFER_LENGTH_TYPE)` 且小于 `SIZE_MAX`；内部存储数组为 `BufferSize + 1` 字节，其中额外 1 字节用于环形缓冲区管理 |
| `TriggerLevel` | `size_t` 编译期整数 | `1` 字节 | 通知等待接收任务的字节数阈值，必须满足 `1 <= TriggerLevel <= BufferSize`；不保证每次接收达到该字节数 |

构造函数签名：`StreamBuffer()`，无运行时参数；构造时只向管理器注册对象，RTOS 资源由 `rtos_resource_init()` 统一创建

#### 构造示例

```cpp
#include "rtos_stream_buffer.hpp"

rtos::StreamBuffer<128> bytes;
```

`rtos::StreamBuffer<128>` 可存储最多 128 字节，省略 `TriggerLevel` 时使用默认阈值 1 字节

#### 注意事项

对象应在全局作用域声明，在统一初始化前完成构造，生命周期覆盖整个程序运行期；不支持拷贝构造和拷贝赋值

容量至少为 `sizeof(configMESSAGE_BUFFER_LENGTH_TYPE)`。可声明 `rtos::StreamBuffer<128, 16>`，让发送操作在缓冲区累计至少 `16` 字节时通知正在等待接收的任务。该阈值不保证每次接收至少 `16` 字节：调用接收时已有数据会直接读取，等待超时后也可能读取到不足 `16` 字节的数据，实际接收量还受输出缓冲区大小限制

流缓冲区不保留消息边界，发送可能只写入部分输入，接收可能不填满输出，需要按返回的字节数处理

### 方法

#### 方法签名和含义介绍

| 方法签名 | 含义 |
| --- | --- |
| `size_t send(std::span<const std::uint8_t> data, uint32_t timeout_ms = 0)` | 发送字节，返回实际发送字节数 |
| `size_t receive_to(std::span<std::uint8_t> output_buffer, uint32_t timeout_ms = 0)` | 接收字节，返回实际接收字节数 |
| `StreamBufferHandle_t handle() const` | 获取 `StreamBufferHandle_t`，创建前为 `nullptr` |

#### 方法使用示例

```cpp
// 在任务函数中使用
std::uint8_t input[]{1, 2, 3};
std::uint8_t output[8]{};
size_t sent = bytes.send(input);
size_t received = bytes.receive_to(output);
```

## 超时与中断上下文

- 队列和流缓冲区的 `timeout_ms` 单位为毫秒，`0` 表示不等待，`rtos::wait_forever` 表示永久等待
- 收发接口自动选择任务或中断版本，中断上下文中 `timeout_ms` 必须为 `0`
- 流缓冲区零超时收发使用临界区保护，复制的数据越多，对中断延迟的影响越大
- 流缓冲区非零超时收发没有临界区保护，同一缓冲区应遵守单写者、单读者约束

## rtos_utility.hpp：时间与上下文接口

### 使用示例

```cpp
#include "rtos_utility.hpp"

auto tick = rtos::now_tick();
auto ms = rtos::now_ms();
auto timeout = rtos::timeout_ms_to_ticks(10);
bool in_irq = rtos::is_irq_context();
```

### 接口说明

| 接口 | 返回类型 | 含义 |
| --- | --- | --- |
| `now_tick()` | `TickType_t` | 当前 RTOS tick，在任务上下文中调用 |
| `now_ms()` | `std::uint32_t` | 当前 RTOS 毫秒时间，按 `now_tick() * portTICK_PERIOD_MS` 计算，在任务上下文中调用，允许自然回绕 |
| `timeout_ms_to_ticks(std::uint32_t timeout_ms)` | `TickType_t` | 普通值通过 `pdMS_TO_TICKS` 转换，`wait_forever` 转为 `portMAX_DELAY` |
| `is_irq_context()` | `bool` | 在中断服务中返回 `true`，调度器启动后检测到 `PRIMASK` 或 `BASEPRI` 屏蔽中断时也返回 `true`，调度器尚未启动且不在中断服务中时返回 `false` |
| `wait_forever` | `constexpr std::uint32_t` | 毫秒超时参数的永久等待值，值为 `0xFFFFFFFFU` |

## 管理器接口

`TaskManager`、`QueueManager` 和 `StreamBufferManager` 均提供 `init()` 和 `get_count()`，分别用于初始化已注册资源和获取成功创建的数量，通常只需调用统一入口 `rtos_resource_init()`
