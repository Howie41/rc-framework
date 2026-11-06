# rtos_resource_wrapper

假设一个任务每隔 500 毫秒发出亮灯要求，另一个任务收到后保存占空比和亮灯时长，交给点灯代码执行。

利用这个模块，可以创建两个任务，中间用一个最多存 4 条消息的队列传数据：

```cpp
#include "rtos_task.hpp"
#include "rtos_queue.hpp"
#include <cstdint>

struct LedCommand {
    uint8_t duty;
    uint16_t duration_ms;
};

// 以下对象都放在函数外，整个程序运行期间一直保留。
rtos::Queue<LedCommand, 4> led_queue;
uint8_t led_duty{};
uint16_t led_duration_ms{};

rtos::Task<"led_sender", [] {
    while (true) {
        bool queued = led_queue.put({50, 1000}, 0);
        // queued 为 false 表示没放进去，这次命令没有送达。
        (void)queued;
        rtos::delay(pdMS_TO_TICKS(500));
    }
}, 1024, rtos::TaskPriority::Normal> sender_task;

rtos::Task<"led_receiver", [] {
    while (true) {
        auto result = led_queue.get(rtos::wait_forever);
        if (result) {
            auto command = result.value();
            led_duty = command.duty;
            led_duration_ms = command.duration_ms;
            // 在这里接入实际点灯代码。
        }
    }
}, 1024, rtos::TaskPriority::Normal> receiver_task;
```

在 `freertos.c` 的 `MX_FREERTOS_Init()` 中创建资源，再启动系统调度。本项目已经有下面的调用，不需要再加一遍：

```c
#include "rtos_resource_init.h"

// 放在 MX_FREERTOS_Init() 中，启动调度之前。
rtos_resource_init();
```

任务、队列、字节缓冲区和订阅者要在这个调用之前创建。不要在任务运行后临时创建这些对象，也不要把它们放进一个很快返回的函数里。

### rtos::Task<Name, EntryFunction, StackBytes, Priority>

创建一个持续执行工作的任务。开头的 `sender_task` 和 `receiver_task` 就是两个任务。

- `Task()`
  - 名字放在第一项，例如 `"led_sender"`。
  - 第二项按例子写 `[] { ... }`，里面放这个任务要执行的代码。
  - 通常用 `while (true)` 持续执行，不要让任务函数直接返回。没事做时要等待消息或调用延时。
  - 第三项是栈大小，单位为字节。默认 1024，至少 512，且必须是 4 的倍数。
  - 第四项是优先级，默认 `rtos::TaskPriority::Normal`。
- `TaskHandle_t handle() const`
  - 需要调用其他 FreeRTOS 任务接口时，用它取出任务句柄。
  - `rtos_resource_init()` 之前还没有真正创建任务，此时返回 `nullptr`。

### rtos::TaskPriority

选择任务优先级，从低到高可以用：

- `Idle`、`Low`、`BelowNormal`、`Normal`、`AboveNormal`、`High`、`Realtime`
  - 普通任务先选 `Normal`，确实需要更高优先级时再调整。
  - `None` 和 `Isr` 不能用于创建任务。

### rtos::Queue<T, QueueSize>

按放入顺序传递一条条数据。例子中的 `T` 是 `LedCommand`，`4` 表示最多暂存 4 条，不是 4 字节。

- `Queue()`
  - 例如 `rtos::Queue<LedCommand, 4> led_queue;`。
  - 消息用整数、小数、普通数组组成的简单结构即可，不要直接放 `std::string` 等对象。
- `bool put(const T& item_ref, uint32_t timeout_ms = 0)`
  - 放入一条消息，队列保存一份副本。
  - 第二项是最多等多少毫秒，默认 0，表示放不进去就立即返回。
  - 放进去返回 `true`，队列满且等不到空位时返回 `false`。
- `std::optional<T> get(uint32_t timeout_ms = 0)`
  - 取走最早放入的一条消息。
  - 先用 `if (result)` 判断是否收到，收到后用 `result.value()` 取出消息。
  - 默认不等待；填 `rtos::wait_forever` 就一直等到收到。
- `std::optional<T> peek(uint32_t timeout_ms = 0)`
  - 查看最早的一条消息，但不拿走。判断和取值方式与 `get()` 一样。
- `bool overwrite(const T& item_ref)`
  - 只用于容量为 1 的队列，例如 `rtos::Queue<LedCommand, 1>`。
  - 有旧消息就替换，没有就放进去。只关心最新值时可以用它。

在中断里使用队列时，等待时间必须是 0，中断优先级也要满足项目中 FreeRTOS 的调用要求。

### rtos::StreamBuffer<BufferSize, TriggerLevel = 1>

传递一串字节，例如把收到的串口数据留给任务读取。它不替你区分哪几个字节属于同一条消息。

```cpp
#include "rtos_stream_buffer.hpp"
#include <array>

rtos::StreamBuffer<256> rx_bytes; // 放在函数外，资源初始化之前创建

void stream_example() { // 资源初始化之后，在任务中调用
    std::array<uint8_t, 3> input{0x01, 0x32, 0x00};
    auto sent = rx_bytes.send(input, 0);
    std::array<uint8_t, 16> output{};
    auto received = rx_bytes.receive_to(output, 0);
    // 只有 output 的前 received 个字节是本次取出的数据。
    (void)sent;
}
```

- `StreamBuffer()`
  - 第一项是最多存多少字节，例如 256。容量至少要达到项目的 `sizeof(configMESSAGE_BUFFER_LENGTH_TYPE)`。
  - 第二项是等到多少字节时唤醒等待接收的任务，默认 1，范围为 1 到容量。
- `size_t send(std::span<const uint8_t> data, uint32_t timeout_ms = 0)`
  - 把字节放进去，返回实际放入的字节数，可能只放进去一部分。
  - 原始数组加长度时，可以传 `std::span<const uint8_t>{buffer, length}`。
  - 没放进去的剩余部分需要自己决定重试还是丢弃。
- `size_t receive_to(std::span<uint8_t> output_buffer, uint32_t timeout_ms = 0)`
  - 把已收到的字节取出来，写进给定数组。
  - 返回实际取出多少字节，不保证填满数组。
  - 原始数组加容量时，可以传 `std::span<uint8_t>{buffer, capacity}`。

等待时间单位是毫秒，0 表示立即返回，`rtos::wait_forever` 表示一直等。中断里只能填 0。使用非零等待时间时，同一个缓冲区只安排一个发送者和一个接收者，不要混入其他收发调用。

### rtos::Publisher<T> / rtos::Subscriber<T, QueueSize>

同一份消息要交给多个接收者时，按主题发送。主题就是双方约好的名字，数据类型和主题名都相同才能收到。

```cpp
#include "pub_sub.hpp"

// 使用开头定义的 LedCommand，以下对象放在函数外。
rtos::Publisher<LedCommand> publisher{"led"};
rtos::Subscriber<LedCommand, 4> display_subscriber{"led"};
rtos::Subscriber<LedCommand, 4> record_subscriber{"led"};

void publish_example() { // 资源初始化后，在任务里调用
    auto delivered = publisher.publish({50, 1000}); // 两边都没满时返回 2
    auto result = display_subscriber.receive();
    if (result) {
        auto command = result.value(); // command.duty 为 50
        (void)command;
    }
    // record_subscriber 里还有它自己收到的那一份，不会被上面的读取拿走。
    (void)delivered;
}
```

- `Publisher(std::string_view topic_id)`
  - 创建发送者，例如 `publisher{"led"}`。主题名不能为空，优先直接用字符串常量。
- `std::size_t Publisher::publish(const T& message) const`
  - 向相同类型、相同主题的所有接收者各放入一份消息。
  - 不等待，返回实际送达几个接收者。某个接收者队列满时跳过它。
- `Subscriber(std::string_view topic_id)`
  - 创建接收者，例如 `Subscriber<LedCommand, 4>{"led"}`，每个接收者最多存 4 条。
  - 接收者要在资源初始化之前创建，并一直保留。
- `std::optional<T> Subscriber::receive(uint32_t timeout_ms = 0)`
  - 取走自己收到的一条消息，先判断成功，再用 `result.value()` 取出。
  - 等待时间用法与队列的 `get()` 一样。
- `std::string_view topic_id() const`
  - 查看发送者或接收者使用的主题名。

不要用很快就会销毁或改写的字符数组当主题名。消息内容与队列一样，使用能直接复制的简单数据。

### AtomicLock

轻量原子锁，没有长时间阻塞获取的功能。只想试一下能不能开始操作，忙就下次再试时，可以用它。

```cpp
#include "atomic_lock.hpp"

AtomicLock lock; // 多次调用使用同一个锁
void try_update() {
    if (!lock.try_acquire()) return;
    // 在这里完成需要保护的短操作。
    lock.release();
}
```

- `bool try_acquire() noexcept`
  - 只尝试一次获取锁，不等待；成功返回 true。
- `void release() noexcept`
  - 当前操作结束后释放锁。只有成功获取锁的一方才能释放。
  - 获取成功后，中途返回也必须先释放。
- `bool is_locked() const noexcept`
  - 只查看现在是否有人占用，不能代替 `try_acquire()`。

### 延时和时间

包含 `rtos_task.hpp` 可用延时函数，包含 `rtos_utility.hpp` 可用时间查询。

- `void rtos::delay(uint32_t ticks)`
  - 当前任务等指定的系统节拍数。这里不是毫秒。
  - 要等 500 毫秒，写 `rtos::delay(pdMS_TO_TICKS(500));`。
- `void rtos::delay_until(uint32_t ticks)`
  - 等到指定的绝对节拍时间，已经到时就直接返回。
  - 例如先记 `auto next = rtos::now_tick() + pdMS_TO_TICKS(10);`，做完工作后调用 `rtos::delay_until(next);`。
- `TickType_t rtos::now_tick() noexcept`
  - 读取当前系统节拍数，在任务里调用。
- `uint32_t rtos::now_ms() noexcept`
  - 读取当前系统运行时间，单位毫秒，在任务里调用。计数很久后会从 0 重新开始。
- `rtos::wait_forever`
  - 队列、订阅接收、字节缓冲区的超时参数填它，表示一直等。
  - 不能把它当毫秒数传给 `rtos::delay()`。

### 资源初始化

- `void rtos_resource_init(void)`
  - 在所有资源对象声明完成后、系统调度启动前调用。
  - 先准备队列和字节缓冲区，再创建任务。重复调用不会再次创建。
  - 不要在它执行后再添加任务、队列、字节缓冲区或订阅者。
- `static size_t rtos::TaskManager::get_count()`
  - 查看实际创建成功的任务数量。
- `static size_t rtos::QueueManager::get_count()`
  - 查看实际创建成功的队列数量，包含订阅者内部使用的队列。
- `static size_t rtos::StreamBufferManager::get_count()`
  - 查看实际创建成功的字节缓冲区数量。
