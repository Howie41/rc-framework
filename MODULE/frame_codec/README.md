## 介绍

`frame_codec` 提供固定长度二进制帧的序列化和反序列化。通过元素类型声明字段顺序、字节序、长度含义和 CRC 校验范围，自动生成完整帧，或验证输入并还原普通字段数值。

### 模块组成

| 文件 | 功能 |
| --- | --- |
| `frame_codec.hpp` | `Frame`：组织整帧编解码，返回字节数组或解析结果 |
| `frame_element.hpp` | 定义 Field、Header、Footer、Length、Crc、region tag 和区域边界标记，提供元素编解码逻辑 |
| `frame_traits.hpp` | 内部类型识别，区分普通字段、自动字段和区域边界标记 |
| `frame_layout.hpp` | 内部 Layout：计算元素偏移、区域长度，并在编译期检查格式是否合法 |

### 工作方式

所有公开类型位于 `frame` 命名空间。`Frame<Elements...>` 和元素只作为格式类型使用，不需要创建对象，也不保存运行时数值或字节视图。

序列化接收普通 Field 数值组成的 tuple，返回拥有完整字节的 `std::array`；反序列化读取完整候选帧的只读视图，返回 `std::expected<普通字段组成的 tuple, frame::Error>`。元素不会将特殊字段加入 tuple，重复类型的普通字段按声明顺序分别对应 tuple 中的位置。

字段宽度和区域范围均在编译期确定，不使用结构体内存布局，不插入字段对齐填充，也不进行堆分配。数值按各字段指定的大小端编码，默认小端。区域标记不占用字节。

## 移植

将本目录加入头文件搜索路径，包含 `frame_codec.hpp` 即可使用，无需添加 `.cpp` 文件或调用初始化函数。编译器及标准库须支持 C++23，包含 `std::expected`、`std::span`、`std::bit_cast` 等功能；模块不依赖 HAL 或 FreeRTOS。

```cpp
#include "frame_codec.hpp"
```

### 注意事项

- 当前仅支持固定长度帧，Length 字段不会改变后续字段的偏移。
- Frame 每次只处理一帧；查找帧头、缓存未收齐的数据、拆包和粘包处理由调用方完成。
- 输入视图长度必须恰好等于帧长度，多余字节同样会解析失败。
- 返回的字节数组由调用方持有。用于异步发送或 DMA 时，应保证发送完成前数组仍然有效。
- 反序列化结果拥有独立数值，成功返回后可以复用接收缓冲区。
- 数值编码实现要求目标采用纯大端或纯小端字节序，float 字段要求 IEEE 754 binary32。

## Frame：完整帧编解码

### 构造

#### 模板和参数说明

模板形式为 `frame::Frame<Elements...>`，无运行时构造参数。通常使用 `using` 声明协议类型，再直接调用静态方法。

| 模板参数 | 类型 / 形式 | 默认值 | 含义与约束 |
| --- | --- | --- | --- |
| `Elements...` | 元素类型列表 | 可为空 | 按线路字节顺序声明元素；可包含 Field、Header、Footer、Length、Crc、Begin、End |

#### 格式声明示例

```cpp
#include "frame_codec.hpp"

#include <cstdint>
#include <tuple>

using MotorPayload = frame::RegionTag<0>;
using MotorPacket = frame::Frame<
    frame::Header<0xAA, 0x55>,
    frame::Length<std::uint8_t, frame::LengthOf<MotorPayload>>,
    frame::Begin<MotorPayload>,
    frame::Field<std::uint16_t>,
    frame::Field<float>,
    frame::End<MotorPayload>,
    frame::Crc<MotorPayload>,
    frame::Footer<0x0D, 0x0A>>;

static_assert(MotorPacket::size() == 13);
```

该示例的线路布局如下，CRC 只覆盖编号和目标值的六个字节：

| 字节偏移 | 宽度 | 含义 |
| --- | --- | --- |
| 0 | 2 | 固定帧头 `AA 55` |
| 2 | 1 | Payload 长度，固定为 `06` |
| 3 | 2 | 编号，uint16 小端 |
| 5 | 4 | 目标值，float 小端 |
| 9 | 2 | Payload 的 CRC16，小端 |
| 11 | 2 | 固定帧尾 `0D 0A` |

### 方法

#### 方法签名和含义介绍

下表中 `Ts...` 表示按声明顺序提取的所有普通 Field 的 `value_type`，仅用于说明签名；Frame 不提供额外的值类型或数组类型别名。

| 方法签名 | 含义 |
| --- | --- |
| `static constexpr std::size_t size() noexcept` | 获取整帧字节数，空帧返回 `0` |
| `static constexpr std::array<std::uint8_t, size()> serialize(const std::tuple<Ts...>& values) noexcept` | 根据普通字段值生成完整帧，自动填充固定字节、长度和 CRC |
| `static constexpr std::expected<std::tuple<Ts...>, Error> deserialize(std::span<const std::uint8_t> input) noexcept` | 检查完整候选帧并返回普通字段值，失败时返回首个错误 |

#### 方法使用示例

以下代码接续前面的 `MotorPacket` 声明：

```cpp
void motor_packet_example() {
    auto bytes = MotorPacket::serialize(std::tuple{std::uint16_t{42}, 1.5f});
    // bytes 为 std::array<std::uint8_t, 13>，可交给通信接口发送。

    auto result = MotorPacket::deserialize(bytes);
    if (result) {
        auto [id, target] = *result;
        (void)id;
        (void)target;
    } else {
        const auto error = result.error();
        // 按 error 处理无效帧。
        (void)error;
    }
}
```

也可以传入接收缓冲区的切片，调用前应确认已经收到足够字节：

```cpp
void receive_motor_packet(std::span<const std::uint8_t> received) {
    if (received.size() < MotorPacket::size()) return;
    auto result = MotorPacket::deserialize(received.first(MotorPacket::size()));
    // result 只对应第一帧；剩余数据由外层接收逻辑继续处理。
    (void)result;
}
```

#### expected 取值

先用 `if (result)` 或 `result.has_value()` 判断是否成功。成功时用 `*result` 取得 tuple，再通过结构化绑定或 `std::get<Index>` 取值；失败时调用 `result.error()`。不要在失败时解引用结果，也不要在成功时读取错误。

没有普通字段时，序列化传入空 tuple；空帧同样可以使用：

```cpp
using FixedPacket = frame::Frame<frame::Header<0xAA>, frame::Footer<0x55>>;
constexpr auto fixed_bytes = FixedPacket::serialize(std::tuple<>{});
static_assert(fixed_bytes == std::array<std::uint8_t, 2>{0xAA, 0x55});
static_assert(*FixedPacket::deserialize(fixed_bytes) == std::tuple<>{});
static_assert(frame::Frame<>::serialize({}).empty());
static_assert(frame::Frame<>::deserialize({}).has_value());
```

## Field：普通数值字段

### 构造与参数

模板形式为 `Field<T, Order = std::endian::little>`。

| 参数 | 含义与约束 |
| --- | --- |
| `T` | 1、2、4、8 字节整数或 `float`，不允许 bool、const、volatile；float 必须为 IEEE 754 binary32。协议中推荐使用固定宽度整数类型 |
| `Order` | 字节序，可选小端、大端或本机字节序，默认小端 |

例如 `Field<std::int16_t, std::endian::big>` 表示两字节有符号大端字段，`Field<float>` 表示四字节小端浮点字段。每个 Field 对应 tuple 中的一个值，重复字段类型不会合并。

### 方法

| 方法 | 含义 |
| --- | --- |
| `size()` | 返回字段宽度 `sizeof(T)` |
| `serialize(value, output)` | 将数值按目标字节序写入自身切片，返回 void |
| `deserialize(input)` | 从自身切片解码并返回 T |

## Header / Footer：固定帧头和帧尾

### 构造与参数

模板形式分别为 `Header<Bytes...>` 和 `Footer<Bytes...>`。`Bytes...` 为一个或多个 uint8 常量，例如 `Header<0xAA, 0x55>`、`Footer<0x0D, 0x0A>`。

通常分别放在元素列表开头和末尾。没有帧头或帧尾时直接省略相应元素，不使用空字节序列占位。它们不占 tuple 位置。

### 方法

| 方法 | 含义 |
| --- | --- |
| `size()` | 返回固定字节的数量 |
| `serialize(output)` | 将固定字节写入自身切片 |
| `deserialize(input)` | 所有字节均匹配时返回 true，否则返回 false |

## RegionTag / Begin / End：区域定义

### 构造与参数

| 类型 | 用法 |
| --- | --- |
| `RegionTag<Id>` | 定义 region tag，不同区域使用不同 Id，例如 `using Payload = frame::RegionTag<0>` |
| `Begin<RegionTag>` | 标记该区域的起点 |
| `End<RegionTag>` | 标记该区域的终点，区域结束于此标记之前 |

RegionTag 是类型标识，不直接加入 Frame 元素列表。Begin、End 占用零字节，不占 tuple 位置，不提供运行时编解码方法。

### 注意事项

同一个 region tag 在一帧中只能定义一组 Begin、End，区域可以为空或嵌套，但不能交叉闭合。例如 `Begin<A>, Begin<B>, End<B>, End<A>` 合法，`Begin<A>, Begin<B>, End<A>, End<B>` 不合法。

长度和 CRC 可以引用同一个区域，也可以引用不同区域。它们引用的区域必须在同一个 Frame 中声明，字段自身可以位于目标区域之前或之后，但 CRC 还须满足下文的覆盖规则。

## Length / LengthOf：区域长度

### 构造与参数

模板形式为 `Length<T, Meaning, Order = std::endian::little>`。

| 参数 | 含义与约束 |
| --- | --- |
| `T` | 保存长度的无符号整数类型，不允许 bool、const、volatile；必须能表示完整区域长度 |
| `Meaning` | 使用 `LengthOf<RegionTag>` 指定长度所属区域；LengthOf 不直接作为帧元素 |
| `Order` | 长度值的字节序，默认小端 |

### 使用示例

```cpp
using WholeRegion = frame::RegionTag<10>;
using SelfLengthPacket = frame::Frame<
    frame::Begin<WholeRegion>,
    frame::Header<0xAA>,
    frame::Length<std::uint8_t, frame::LengthOf<WholeRegion>>,
    frame::Field<std::uint16_t>,
    frame::End<WholeRegion>>;

constexpr auto self_length_bytes = SelfLengthPacket::serialize(std::tuple{std::uint16_t{0x1234}});
static_assert(self_length_bytes == std::array<std::uint8_t, 4>{0xAA, 4, 0x34, 0x12});
```

该区域长度为四字节，包含帧头、长度字段自身及普通字段。区域内的所有线路字节都计入长度，包括嵌套区域、帧尾和 CRC；Begin、End 不占字节。长度是格式确定的常量，接收到不同的长度值会验证失败，不会据此改变帧大小。

### 方法

| 方法 | 含义 |
| --- | --- |
| `size()` | 返回长度字段宽度 `sizeof(T)` |
| `serialize(output, region_size)` | 将区域字节数写入自身切片 |
| `deserialize(input, region_size)` | 读取长度值，与预期区域字节数比较，返回 bool |

## Crc：区域校验

### 构造与参数

模板形式为 `Crc<RegionTag, Type = CrcType::Crc16, Order = std::endian::little>`。

| 参数 | 含义与约束 |
| --- | --- |
| `RegionTag` | 参与校验的区域标识，须在当前 Frame 内声明 |
| `Type` | 选择 Crc8 或 Crc16，默认 Crc16，字段分别占一或两字节 |
| `Order` | 校验值保存到帧中的字节序，默认小端；不改变计算算法 |

### 算法参数

| 参数 | `CrcType::Crc8` | `CrcType::Crc16` |
| --- | --- | --- |
| 多项式 | `0x07` | `0x1021` |
| 初值 | `0x00` | `0xFFFF` |
| 输入反射 / 输出反射 | 均为否 | 均为否 |
| 最终异或值 | `0x00` | `0x0000` |
| 空区域校验值 | `0x00` | `0xFFFF` |
| ASCII `123456789` 的校验值 | `0xF4` | `0x29B1` |

### 方法

| 方法 | 含义 |
| --- | --- |
| `size()` | 返回校验字段宽度 |
| `serialize(output, region)` | 计算区域 CRC，再按 Order 写入自身切片 |
| `deserialize(input, region)` | 读取保存值，与重新计算的区域 CRC 比较，返回 bool |

### 覆盖规则与示例

CRC 不能覆盖自身或声明在后的其他 CRC 字段，可以覆盖声明在前的 CRC 字段。所有非 CRC 字段写入后，再按声明顺序生成 CRC；不满足依赖顺序的格式在编译期拒绝。

以下格式先计算数据区域的 CRC8，再计算包含该 CRC8 的外层 CRC16：

```cpp
using DataRegion = frame::RegionTag<20>;
using CheckedRegion = frame::RegionTag<21>;
using NestedCrcPacket = frame::Frame<
    frame::Begin<CheckedRegion>,
    frame::Header<0xAA>,
    frame::Begin<DataRegion>,
    frame::Field<std::uint16_t>,
    frame::End<DataRegion>,
    frame::Crc<DataRegion, frame::CrcType::Crc8>,
    frame::End<CheckedRegion>,
    frame::Crc<CheckedRegion>>;

constexpr auto nested_bytes = NestedCrcPacket::serialize(std::tuple{std::uint16_t{42}});
static_assert(NestedCrcPacket::size() == 6);
static_assert(std::get<0>(*NestedCrcPacket::deserialize(nested_bytes)) == 42);
```

CRC 字段也可以位于它所校验的数据区域之前，只要不覆盖尚未生成的其他 CRC。

## 错误处理与编译期检查

### 解析错误

| 错误值 | 含义 |
| --- | --- |
| `Error::BufferSizeMismatch` | 输入字节数与完整帧长度不一致 |
| `Error::HeaderMismatch` | 固定帧头不匹配 |
| `Error::FooterMismatch` | 固定帧尾不匹配 |
| `Error::LengthMismatch` | 长度字段与指定区域的预期长度不一致 |
| `Error::CrcMismatch` | CRC 保存值与实际计算值不一致 |

Frame 先检查输入大小，再解码普通字段到局部 tuple，最后按声明顺序验证自动字段。多个字段同时出错时返回最先遇到的自动字段错误；任何验证失败都不返回部分解析值。单独调用普通 Field 的 deserialize 只进行数值解码，不执行业务取值范围检查。

### 编译期检查

格式实例化时自动检查区域唯一配对和嵌套闭合、Length 与 Crc 的 region tag 引用、长度字段容量及 CRC 覆盖依赖。查询元素偏移时拒绝越界索引，查询区域时拒绝未声明的 region tag。

元素级方法接收自身固定长度切片，直接调用时须提供有效切片及正确的区域参数。Frame 负责准备切片和调度，Layout 负责配置合法性，元素不重复检查这些前提。通常使用 Frame 的整帧接口即可。

## 内部布局接口

`frame::detail::Layout<Elements...>` 为 Frame 提供以下编译期查询，不属于模块使用者的稳定公开 API：

| 方法 | 含义 |
| --- | --- |
| `get_frame_size()` | 返回整帧字节数 |
| `get_element_offset<Index>()` | 返回从零开始的声明索引所对应元素的字节偏移；零字节边界标记也占用索引 |
| `get_region_offset<RegionTag>()` | 返回区域起始字节偏移 |
| `get_region_size<RegionTag>()` | 返回区域字节数，空区域为零 |

`check_region_pairs()`、`check_region_references()`、`check_length_capacity()` 和 `check_crc_coverage()` 均为私有方法，通过 static_assert 自动执行，无需手动调用。
