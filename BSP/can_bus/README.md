# can_bus

假设 CAN1 上有一块灯板，我们要发送占空比和亮灯时长，再接收灯板回复的数值。

这里约定使用标准 CAN 帧：控制帧 ID 为 `0x301`，回复帧 ID 为 `0x302`。两种帧的数据部分都是 8 字节，位置从 0 开始。

**亮灯控制帧，ID：`0x301`**

| 帧位置 | 字节 0 | 字节 1～2 | 字节 3～7 |
| --- | --- | --- | --- |
| 数据 | 目标占空比 | 亮灯时长（毫秒） | 固定填 0 |
| 数据类型 | `uint8_t` | `uint16_t` | `uint8_t[5]` |

**灯板回复帧，ID：`0x302`**

| 帧位置 | 字节 0 | 字节 1～2 | 字节 3～7 |
| --- | --- | --- | --- |
| 数据 | 回复的占空比 | 回复的亮灯时长（毫秒） | 保留，本例不使用 |
| 数据类型 | `uint8_t` | `uint16_t` | `uint8_t[5]` |

占空比为 `0～100`，时长为 `0～65535` 毫秒，低字节在前。例如数据 `32 E8 03 00 00 00 00 00` 表示 50%、1000 毫秒。CAN ID 单独填写，不放进这 8 字节。

```cpp
#include "can_bus.hpp"
#include "fdcan.h"
#include <atomic>

// 自定义设备可以照着这个类填写收发代码。
class LedDevice : public CanDevice {
public:
    explicit LedDevice(CanBus& bus) : CanDevice(bus, 0x302, false) {}

    uint8_t target_duty{0};
    uint16_t target_duration_ms{0};
    std::atomic<uint8_t> feedback_duty{0};
    std::atomic<uint16_t> feedback_duration_ms{0};

    // 收到 ID 为 0x302 的回复时自动执行，不需要自己调用。
    void on_rx(uint32_t, const uint8_t data[8], uint8_t len) override {
        if (len != 8 || data[0] > 100) return;
        feedback_duty.store(data[0]);
        feedback_duration_ms.store(
            static_cast<uint16_t>(data[1] | (uint16_t{data[2]} << 8)));
    }

    // 每次调用 bus.heartbeat() 时自动执行。
    bool build_tx(CanBus& bus) override {
        if (target_duty > 100) return false;
        CanBus::ClassicPack pack{}; // 数据默认全为 0
        pack.m_id = 0x301;
        pack.m_type = CanBus::Type::Standard;
        pack.m_data[0] = target_duty;
        pack.m_data[1] = static_cast<uint8_t>(target_duration_ms);
        pack.m_data[2] = static_cast<uint8_t>(target_duration_ms >> 8);
        return bus.add_can_msg(pack) == 0;
    }
};

// 先创建总线，再创建设备；都放在函数外，整个运行期间保留。
CanBus led_bus{hfdcan1};
LedDevice led{led_bus};

bool init_led_bus() { // MX_FDCAN1_Init() 完成后调用一次
    return led_bus.init();
}

bool update_led() { // 初始化成功后，在同一个任务中周期调用
    led.target_duty = 50;
    led.target_duration_ms = 1000;
    return led_bus.heartbeat();
}

uint8_t read_led_duty() { // 在任务中查看最近收到的值
    return led.feedback_duty.load();
}
```

先检查 `init_led_bus()` 是否成功，再按业务需要周期调用 `update_led()`。代码负责发送控制数据并保存回复，不直接点灯；灯板应自己实现点灯和计时。

接入前需要完成这些配置：

- CubeMX 配置 FDCAN1 的引脚、速率、过滤器空间、接收 FIFO 和发送 FIFO。通信双方速率一致，使用经典 CAN 数据帧。
- 标准帧和扩展帧过滤器各至少留 1 个，相关收发空间至少能放 8 字节数据，打开接收和错误中断。
- 项目已经通过 `can_bus.cpp` 接好 HAL 回调，不要重复定义。
- 如果项目已经创建 CAN1 的总线对象，复用它，不要给同一个外设再创建一份。

已有电机驱动时，不用自己写上面的 `LedDevice`。直接创建对应设备即可，例如 C620 的用法见 [motor](../../MODULE/motor/README.md)。

### CanBus

管理一条 CAN 总线，负责发送数据、把收到的帧交给对应设备。

- `explicit CanBus(FDCAN_HandleTypeDef& handle)`
  - 传入项目生成的 `hfdcan1`、`hfdcan2` 或 `hfdcan3`。目前只支持这三个句柄。
- `bool init()`
  - 硬件初始化完成后调用，配置接收并启动 CAN。
  - 设备必须在它之前创建，启动后不能再添加新设备。
  - 成功返回 `true`；未知句柄返回 `false`。HAL 初始化失败会进入项目的 `Error_Handler()`。
  - 重复调用不会重复启动，也不会清掉已经登记的设备。
- `bool heartbeat()`
  - 每个控制周期调用一次，自动执行各设备的 `build_tx()`，使用最新目标发送。
  - 所有设备本次处理成功时返回 `true`；没初始化或有设备失败时返回 `false`。
  - 不会自动定时调用，需要由自己的任务安排周期。
  - 有一个设备失败时仍会处理其他设备，已经提交的帧不会撤回。
- `uint8_t add_can_msg(const ClassicPack& pack)`
  - 直接发送一帧固定 8 字节的经典 CAN 数据。例子在设备的 `build_tx()` 中调用它。
  - **0 表示提交成功，1 表示失败**，不要把它当成返回 `bool` 的接口。
  - 忙、发送空间满、ID 或类型不合法、HAL 出错时返回 1，不会保存下来自动重试。
  - 提交成功只表示硬件接受了数据，不代表对方已经收到。
- `uint32_t hardware_tx_queue_empty_size()`
  - 查看硬件还剩几个发送位置。只是查看，真正发送仍要检查 `add_can_msg()` 的返回值。

### CanBus::ClassicPack

准备一帧要发送的数据。先写 `CanBus::ClassicPack pack{};`，再填下面三项。

- `uint32_t m_id`
  - 发送 ID，例子填 `0x301`。
- `Type m_type`
  - `CanBus::Type::Standard` 表示标准帧，ID 范围 `0～0x7FF`。
  - `CanBus::Type::Extended` 表示扩展帧，ID 范围 `0～0x1FFFFFFF`。
- `uint8_t m_data[8]`
  - 发送的 8 个字节。用 `pack.m_data[0]` 等逐个填写。
  - 本接口固定发送 8 字节，没有单独填写长度的选项。未使用的字节按双方约定填写，例子里填 0。

### CanDevice

需要为新的 CAN 设备编写收发逻辑时，照开头的 `LedDevice` 写。普通使用已有驱动时，不需要再自己实现。

- `explicit CanDevice(CanBus& bus, uint32_t id = 0, bool is_extid = false)`
  - 在自定义设备的构造函数中填写所属总线、接收 ID、是否接收扩展帧。
  - 例子中的 `CanDevice(bus, 0x302, false)` 表示接收标准 ID `0x302`。
  - 自动登记，不用再调用 `register_device()`。一条总线最多登记 8 个设备，同类型接收 ID 不得重复。
- `void on_rx(uint32_t rx_id, const uint8_t data[8], uint8_t len)`
  - 在自己的设备类里实现，格式照例子写，末尾加 `override`。
  - 收到匹配的帧后自动调用，`rx_id` 是实际收到的 ID，`len` 是实际字节数，先确认长度再读数组。
  - 它在中断里执行，不要延时或长时间等待。需要保存数据时，当场复制，不要留着 `data` 的指针。
- `bool build_tx(CanBus& bus)`
  - 在自己的设备类里实现，按例子填写发送 ID 和数据，再调用 `bus.add_can_msg()`。
  - `heartbeat()` 会自动调用它。可以发零帧、一帧或多帧，全部成功返回 `true`。
  - 设备不需要周期发送时，实现为 `return true;` 即可；需要时由应用直接调用总线的发送接口。
- `uint32_t id() const` / `bool is_ext_id() const`
  - 查看当前配置的接收 ID 和是否为扩展帧。
- `void set_can_id(uint32_t id, bool is_extid)`
  - 修改接收 ID 和类型，必须在总线 `init()` 之前完成，并避免与其他设备重复。
- `bool match_rx(const FDCAN_RxHeaderTypeDef& header) const`
  - 默认只接收构造时填写的那一个 ID 和类型。
  - 只有像 C620 那样一个设备要接收多个 ID 时，才需要自己重写这个方法。

### 使用时注意

- 设备和总线都要一直保留，不支持临时注册后销毁。发送失败需要自己安排重试，或在下个控制周期再次发送。
- 接收只处理经典 CAN 数据帧，不处理 CAN FD 帧和远程帧。
- 例子用 `.store()` 保存中断收到的值，用 `.load()` 读取。占空比和时长分别更新；如果必须保证两个值来自同一帧，需要另外安排整组保存和读取。
- 总线回调、注册辅助函数等由模块调用，普通业务不用自己调用。调试时可查看 `g_can_error_state` 和 HAL 句柄里的错误，但模块不会自动完成总线故障恢复。
