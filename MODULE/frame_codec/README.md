# frame_codec

下面是一个简单的例子，接下来会围绕这个例子来介绍这个模块。

假设我们需要实现这样一个业务：

根据接收到的数据帧控制 RGB LED。

每帧共 4 字节，位置从 0 开始，按从左到右的顺序发送。

**红灯控制帧**

| 帧位置 | 字节 0 | 字节 1 | 字节 2～3 |
| --- | --- | --- | --- |
| 数据 | 帧头 `0x01` | 占空比 | 亮灯时长（毫秒） |
| 数据类型 | `uint8_t` | `uint8_t` | `uint16_t` |

**绿灯控制帧**

| 帧位置 | 字节 0 | 字节 1 | 字节 2～3 |
| --- | --- | --- | --- |
| 数据 | 帧头 `0x02` | 占空比 | 亮灯时长（毫秒） |
| 数据类型 | `uint8_t` | `uint8_t` | `uint16_t` |

**蓝灯控制帧**

| 帧位置 | 字节 0 | 字节 1 | 字节 2～3 |
| --- | --- | --- | --- |
| 数据 | 帧头 `0x03` | 占空比 | 亮灯时长（毫秒） |
| 数据类型 | `uint8_t` | `uint8_t` | `uint16_t` |

占空比范围为 `0～100`，`0` 表示灭灯，`100` 表示全亮。亮灯时长范围为 `0～65535` 毫秒，低字节在前，例如 1000 写成 `E8 03`。

例如 `01 32 E8 03` 表示红灯占空比 50%，亮 1000 毫秒。模块只负责收发这些数值，什么时候关灯由点灯代码处理。

利用这个模块，我们可以这样实现这个业务（不包括具体点灯逻辑）：

```cpp
#include "frame_parser.hpp"
#include <cstdint>

// 三种帧：帧头 + 占空比 + 亮灯时长。占空比为 0～100，时长单位为毫秒。
using RedFrame = frame::Frame<
    frame::Header<0x01>,
    frame::Field<uint8_t>,  // 占空比
    frame::Field<uint16_t>>; // 亮灯时长（毫秒）

using GreenFrame = frame::Frame<
    frame::Header<0x02>,
    frame::Field<uint8_t>,  // 占空比
    frame::Field<uint16_t>>; // 亮灯时长（毫秒）

using BlueFrame = frame::Frame<
    frame::Header<0x03>,
    frame::Field<uint8_t>,  // 占空比
    frame::Field<uint16_t>>; // 亮灯时长（毫秒）

using LedParser = frame::FrameParser<RedFrame, GreenFrame, BlueFrame>;

// 用于保存各灯的占空比和时长，实际控制灯的代码通过读取这些值来执行。
uint8_t red_duty{}, green_duty{}, blue_duty{};
uint16_t red_duration_ms{}, green_duration_ms{}, blue_duration_ms{};

// 解析回调顺序与上面的帧类型顺序一致：红、绿、蓝。
LedParser parser({
    [](LedParser&, const RedFrame::DataTuple& data) {
        // 按顺序从 data 取出两个值：第一个是占空比，第二个是时长。
        auto [duty, duration_ms] = data;
        if (duty <= 100) {
            red_duty = duty;
            red_duration_ms = duration_ms;
        }
    },
    [](LedParser&, const GreenFrame::DataTuple& data) {
        auto [duty, duration_ms] = data;
        if (duty <= 100) {
            green_duty = duty;
            green_duration_ms = duration_ms;
        }
    },
    [](LedParser&, const BlueFrame::DataTuple& data) {
        auto [duty, duration_ms] = data;
        if (duty <= 100) {
            blue_duty = duty;
            blue_duration_ms = duration_ms;
        }
    }
});

void example() {
    // 假设我们收到这样一个帧：“红灯占空比 50%，亮 1000 毫秒”，即01 32 E8 03。
    // 这里用 serialize() 生成这几个字节，模拟收到的数据。
    auto bytes = RedFrame::serialize({uint8_t{50}, uint16_t{1000}});

    // 接收这个帧：执行红灯回调，保存占空比 50 和时长 1000；这里不会直接点亮灯。
    parser.feed(bytes);
}
```

实际接收时，把串口每次收到的数据传给同一个 `parser` 的 `feed()` 方法。没能一次收完也没关系，它会在收齐后再执行对应灯的处理代码。

### `Frame<Elements...>`

定义帧格式。通过 `using 名字 = frame::Frame<...>;` 给帧起名字，后面直接用这个名字调用方法就行。

- `DataTuple`
  - 存放要发送或已经收到的值。可以把它理解成一组按顺序摆好的数据，像数组一样有第一个、第二个位置。
  - 例子里第一个位置放占空比，第二个位置放亮灯时长，和两个 `Field` 的顺序一致。帧头不用填。
  - 这些位置可以是不同类型的值：比如这里的占空比用 `uint8_t`，时长用 `uint16_t`。
  - 填值可以写 `RedFrame::DataTuple data{uint8_t{50}, uint16_t{1000}};`。
  - 取值可以写 `auto [duty, duration_ms] = data;`，两个值就分别放进了 `duty` 和 `duration_ms`。这里不用 `data[0]`、`data[1]`。
- `static constexpr std::array<uint8_t, size()> serialize(const DataTuple& data_tuple) noexcept`
  - 序列化，即把填好的值变成可以发送的一整条消息。
  - 例如 `RedFrame::serialize({uint8_t{50}, uint16_t{1000}})` 生成“红灯 50%，亮 1000 毫秒”。
  - 返回的是字节数组。例子里依次是 `01 32 E8 03`，可以用 `bytes[0]` 等读取单个字节。
  - 只生成数据，不会自动发送；发送要自己调用串口等接口。
- `static constexpr std::expected<DataTuple, Error> deserialize(std::span<const uint8_t> input) noexcept`
  - 反序列化，即手上已经有完整的一条消息时，用它取出数据，例如 `auto result = RedFrame::deserialize(bytes);`。
  - 只传这一条消息的字节，不能多也不能少。上面的红灯消息必须传 4 字节。
  - 先用 `if (result)` 判断是否成功。
  - 成功后用 `auto [duty, duration_ms] = *result;` 取出占空比和时长。
  - 失败时用 `result.error()` 查看错误，此时不要取数据。
  - 像示例那样用 `parser.feed()` 接收时，不需要自己调用它。

### `FrameParser<Frames...>`

接收数据，认出是哪一种帧，再执行对应的处理代码。示例中的 `LedParser` 就是接收红、绿、蓝三种帧的工具。

- `LedParser parser({...});`（示例中的创建写法）
  - 大括号里依次写红灯、绿灯、蓝灯收到消息后要做的事。
  - 顺序要和 `FrameParser<RedFrame, GreenFrame, BlueFrame>` 一致。
  - 只创建一次，后面收到的数据都交给这个 `parser`。
  - 不处理某种消息时，对应位置填 `nullptr`。
- `constexpr bool feed(uint8_t byte)`
  - 每次传入一个收到的字节。
  - 凑齐并认出一条消息时，执行对应的处理代码，并返回 `true`；否则返回 `false`。
- `constexpr std::size_t feed(std::span<const uint8_t> bytes)`
  - 一次传入一批收到的数据，返回这次认出了几条消息。
  - 可以直接传示例里的 `bytes`，即 `parser.feed(bytes);`。
  - 如果手上是接收数组 `buffer` 和收到的字节数 `length`，就写 `parser.feed(std::span<const uint8_t>{buffer, length});`。
  - 这里只使用数组里实际收到的那部分，不要把整个数组的容量当成 `length`。
- `constexpr void reset() noexcept`
  - 扔掉之前收到但还没处理的数据，接下来从头接收。
  - 之前写好的红、绿、蓝灯处理代码不用重写。

使用时还要注意：

- 模块负责调用对应的回调；要保存占空比和时长，需要像示例一样在回调里赋值。
- 不要留着 `data` 等以后再用，需要的值当场取出来保存。
- 不要在多个任务里同时使用同一个 `parser`。
- 不要在灯的处理代码里再次调用它的 `feed()`。

### 回调里要把数据存到自己的对象时

开头把各灯的占空比和时长分别放在函数外。如果想把红灯的数据放在同一个对象里，可以这样写。这里沿用开头的 `LedParser` 和三种帧格式：

```cpp
struct LedSettings {
    uint8_t red_duty{};
    uint16_t red_duration_ms{};
};

LedSettings led_settings;
LedParser settings_parser({
    [](LedParser& parser, const RedFrame::DataTuple& data) {
        auto& settings = *static_cast<LedSettings*>(parser.m_callback_context);
        auto [duty, duration_ms] = data;
        if (duty <= 100) {
            settings.red_duty = duty;
            settings.red_duration_ms = duration_ms;
        }
    },
    nullptr, // 这个例子只保存红灯数据，不处理绿灯。
    nullptr  // 不处理蓝灯。
}, &led_settings);
```

创建 `settings_parser` 时，最后的 `&led_settings` 把这份数据的地址交给它。收到红灯帧后，回调从 `parser.m_callback_context` 取回地址，再把占空比和时长写进去。后续收到的字节交给 `settings_parser.feed(bytes)`，点灯代码读取 `led_settings` 即可。

- `m_callback_context` 就是保存这个地址的地方，同一个解析器的所有回调都能用它。存进去的是 `LedSettings` 的地址，取出来就写 `static_cast<LedSettings*>`。
- 不需要这份额外数据时，仍然可以像开头那样只传回调，地址默认为 `nullptr`；没有设置地址就不要拿它访问数据。
- 解析器只记住地址，不会复制 `led_settings`。这份数据要一直保留，至少在回调还会使用它时不能销毁。
- `m_callbacks` 和 `m_callback_context` 都可以直接修改；先等当前的 `feed()` 执行完，再改处理代码或地址。
- `reset()` 只扔掉还没处理完的字节，不会清除处理代码、保存的地址或 `led_settings` 中的数据。

下面介绍可以放进 `Frame` 的各个元素。示例写法省略了 `frame::`，实际写进代码时要像开头的例子一样加上。

### `Header<Bytes...>`

指定帧头。例子里通过帧头判断要控制红灯、绿灯还是蓝灯。

- `Header<0x01>`
  - 消息以 `01` 开头。示例用 `01` 表示红灯、`02` 表示绿灯、`03` 表示蓝灯。
- `Header<0xAA, 0x01>`
  - 也可以指定多个开头字节，这里是 `AA 01`。
  - 接收多种消息时，开头不能相同，也不能一个是另一个的开头。
  - 例如 `Header<0xAA>` 和 `Header<0xAA, 0x01>` 不能一起用。

### `Field<T, Order = std::endian::little>`

指定帧里要带什么数据。例子里写了两个 `Field`，分别放占空比和亮灯时长。

每写一个 `Field`，发送时就多填一个值，接收时也多取一个值，顺序按定义来。

- `Field<uint8_t>`
  - 放一个 0～255 的整数。示例用它放占空比。
  - “只能填 0～100”需要自己判断，这个模块不会帮你限制。
- `Field<uint16_t>`
  - 放一个 0～65535 的整数。示例用它放亮灯时长，最多 65535 毫秒。
  - 更大的数可以用 `uint32_t`，需要负数可以用 `int16_t` 等。
- `Field<float>`
  - 放一个带小数的数，例如 `12.5f`。
- `Field<std::array<uint8_t, N>>`
  - 放固定 `N` 个字节，发送时必须提供这么大的数组。
- `Field<uint16_t, std::endian::big>`
  - 和对方约定要按“大端”收发多字节数字时这样写；没填第二项时按“小端”收发。
  - 示例中的时长占两个字节，1000 默认按 `E8 03` 收发，改成大端后按 `03 E8` 收发。

### `Footer<Bytes...>`

指定帧尾。开头的亮灯例子没有帧尾，如果和对方约定每帧最后要带固定字节，就把它写在 `Frame` 的最后面。

- `Footer<0x0D, 0x0A>`
  - 每帧以 `0D 0A` 结尾，发送时自动加上，接收时自动检查。
  - 不需要帧尾时，不写这一项。

### `BitField<Bits, T, Order = BitOrder::LsbFirst>`

需要按“几位”而不是“几个字节”放数据时，用它代替 `Field`。例如以后要在亮灯消息里加一个开关状态，只需要放 0 或 1，可以用 `BitField<1, uint8_t>`。

发送和接收的写法与 `Field` 一样：发送时填一个值，接收时取一个值。

- `BitField<1, uint8_t>`
  - 放一个 0 或 1，例如灯的开关状态。
- `BitField<3, uint8_t>`
  - 放一个 0～7 的数。不要填超过范围的数，模块不会报错，也不会自动改成最大值。
- `BitField<4, uint8_t>, BitField<4, uint8_t>`
  - 连着写两个各占 4 位的数，总共只占一个字节；发送和接收时仍是两个值。
- `BitField<4, uint8_t, BitOrder::MsbFirst>`
  - 对方要求从一个字节的高位开始放数据时这样写。
  - 默认用 `BitOrder::LsbFirst`，从低位开始。
  - 连续的一组保持同一种写法；要换顺序，先凑满整个字节。

### `RegionTag<Id> / Begin<Region> / End<Region>`

需要给一段数据填写长度或加校验时，先用这三个写法圈出范围。例如只处理占空比和时长，不包括帧头，就把这两个 `Field` 圈起来。

这些标记本身不占字节，发送时也不用给它们填值。

- `using Payload = frame::RegionTag<0>;`
  - 把一段数据命名为 `Payload`。有多段时分别用不同的编号，例如 `RegionTag<1>`。
- `frame::Begin<Payload>`
  - 写在要圈起来的数据前面。
- `frame::End<Payload>`
  - 写在这段数据后面，和 `Begin<Payload>` 配对。同一个名字在一条消息里只圈一次。

例如，下面把占空比和亮灯时长圈成 `Payload`，共 3 字节：

```cpp
using Payload = frame::RegionTag<0>;
using CheckedLedFrame = frame::Frame<
    frame::Header<0x01>,
    frame::Length<uint8_t, frame::LengthOf<Payload>>,
    frame::Begin<Payload>,
    frame::Field<uint8_t>,  // 占空比
    frame::Field<uint16_t>, // 亮灯时长
    frame::End<Payload>,
    frame::Crc<Payload>,
    frame::Footer<0x0D, 0x0A>>;

// 仍然只填占空比和时长，长度、校验、帧头、帧尾都不用填。
auto bytes = CheckedLedFrame::serialize({uint8_t{50}, uint16_t{1000}});
```

### `Length<T, Meaning, Order = std::endian::little> / LengthOf<Region>`

对方要求帧里带上数据长度时，用它自动填写。上面的例子把占空比和时长圈成了 `Payload`，长度就是 3 字节。

发送时不用自己填长度，接收时模块会检查长度是否等于约定的值。

- `Length<uint8_t, LengthOf<Payload>>`
  - 用一个字节保存 `Payload` 有多少字节，上面的例子会自动填 3。
  - 长度超过 255 时改用 `uint16_t` 等更大的类型。
- `LengthOf<Payload>`
  - 指定要数哪一段。放在 `Length` 的第二项，不要单独放进 `Frame`。
- `Length<uint16_t, LengthOf<Payload>, std::endian::big>`
  - 用两个字节保存长度，并按大端收发。不填最后一项时默认小端。

这个模块每种消息的总长度是固定的。加上 `Length` 也不能让同一种消息一会儿带 3 字节数据、一会儿带 5 字节。

### `Crc<Region, Type = CrcType::Crc16, Order = std::endian::little>`

对方要求帧里带校验值时，用它自动填写。先和对方约定校验哪一段、用哪种 CRC，再选择下面的写法。

发送时不用自己计算或填写。用 `parser.feed()` 接收时，校验没通过就不会执行对应灯的处理代码。

- `Crc<Payload>`
  - 给 `Begin<Payload>` 到 `End<Payload>` 中间的内容加 CRC16 校验，占两个字节。不用自己计算或填写。
- `Crc<Payload, CrcType::Crc8>`
  - 改用 CRC8，占一个字节。
- `Crc<Payload, CrcType::Crc32Mpeg2>`
  - 改用 CRC32-MPEG2，占四个字节。不能拿其他 CRC32 算法的结果直接替换。
- `Crc<Payload, CrcType::Crc32Mpeg2WordLe>`
  - 使用宇树 IM6014 电机协议对应的 CRC32 写法，占四个字节。
  - 每组不足四字节时按补零计算，但不会往消息里多塞字节。
- `Crc<Payload, CrcType::Crc16, std::endian::big>`
  - 把校验值按大端收发。不填最后一项时默认小端。

使用时：

- `CrcType` 就是上面几种校验方式的名字，按和对方约定的方式选。
- 像例子一样，把 `Crc<Payload>` 放在 `End<Payload>` 后面即可。
- 圈起来的范围不能包含这个 CRC 自己，也不能包含写在它后面的其他 CRC。

### `Error`

自己调用 `deserialize()` 时，先用 `if (result)` 判断是否成功。失败时才调用 `result.error()`，对照下面的名字查看原因。

用 `parser.feed()` 时，不需要自己读取这些错误，模块会继续寻找下一条能通过检查的帧。

- `Error::BufferSizeMismatch`
  - 传入的字节数不对，多了或少了。
- `Error::HeaderMismatch`
  - 消息开头和约定的不一样。
- `Error::FooterMismatch`
  - 消息结尾和约定的不一样。
- `Error::LengthMismatch`
  - 消息里写的长度和实际约定的长度不一样。
- `Error::CrcMismatch`
  - 校验没通过。
