# Frame 编解码重构设计记录

本文记录本次讨论确定的设计，用于后续重构 `MODULE/frame_codec`。Frame 采用无状态编解码方案，输入大小、错误处理、CRC 算法参数和覆盖规则均已确定。

## 核心职责

- `Frame<Elements...>` 是无状态的帧格式与编解码器，只作为类型使用，不需要创建实例，不保存帧字节、视图或解析值。
- `serialize` 创建并返回完整帧的 `std::array`；`deserialize` 读取输入视图，成功时返回普通字段数值组成的 `std::tuple`。
- 元素负责自身的编码、解码或验证。
- 元素只作为描述规则的类型使用，编解码方法为静态方法，不需要创建元素实例。
- 普通字段的解析值放在本次解析返回的 `std::tuple` 中，不存放在元素或 `Frame` 成员中；自动字段不占用 tuple 的位置。
- `Layout` 负责位置和区域信息的编译期计算。
- traits 负责元素类型识别和必要的类型信息提取。
- 保持实现简单，不引入 `Context`、继承体系或虚函数来统一元素接口。

## 文件划分

文件位于 `MODULE/frame_codec/`：

| 文件 | 职责 |
| --- | --- |
| `frame_element.hpp` | 定义 `Field`、`Header`、`Footer`、`Length`、`Crc`、区域标记等元素，以及元素自身的编解码方法 |
| `frame_traits.hpp` | 识别普通字段、自动字段、区域标记，提取所需类型信息 |
| `frame_layout.hpp` | 计算总长度、元素偏移、区域范围，执行布局合法性检查 |
| `frame_codec.hpp` | 定义无状态 `Frame`，组织编解码，返回数组或解析结果 |

主要依赖方向：

```text
frame_codec.hpp
       ↓
frame_layout.hpp
       ↓
frame_traits.hpp
       ↓
frame_element.hpp
```

traits 和 `Layout` 放在 `frame::detail` 命名空间。元素定义不反向依赖 traits 或 `Layout`；元素自身需要的简单类型约束保持自包含，避免循环包含。

## 视图与存储

首版采用固定长度帧，帧长度、元素偏移和区域范围均在编译期确定。`Length` 写入或验证既定的区域长度，不根据输入中的长度值改变布局。

序列化在函数内创建局部 `std::array`，直接将该数组的元素切片传给各元素，完成后按值返回整个数组。元素不各自生成临时数组再拼接，也不返回指向局部数组的视图。首版不额外提供写入外部缓冲区的重载。

反序列化接收只读字节视图，仅在调用期间读取输入，不保存视图，也不复制整帧字节。解析出的值放入局部 tuple，全部验证通过后按值返回。当前普通字段均为数值类型，返回结果不依赖输入缓冲区的生命周期。

写入切片使用可写字节视图，读取或校验切片使用只读字节视图。具体采用动态还是静态 extent，可在实现时按需要确定。

## Frame 公开接口

```cpp
template <typename... Elements>
class Frame {
public:
    static constexpr std::size_t size() noexcept;

    using Values = /* 按声明顺序提取普通字段的 value_type，组成 std::tuple */;
    using Bytes = std::array<std::uint8_t, size()>;

    [[nodiscard]]
    static Bytes serialize(const Values& values) noexcept;

    [[nodiscard]]
    static std::expected<Values, Error> deserialize(
        std::span<const std::uint8_t> input) noexcept;
};
```

以上是接口示意。`Values` 不包含帧头、帧尾、长度、CRC 和区域标记。序列化仅提供 tuple 输入形式，和反序列化的成功结果使用同一类型。

当前元素的格式配置错误通过编译期检查拒绝，合法数值的序列化没有预期的运行时失败，因此直接返回 `Bytes`。反序列化需要处理外部输入错误，使用 `std::expected<Values, Error>`，失败时不提供部分 tuple。

```cpp
auto bytes = Packet::serialize(Packet::Values{42, 1.5f});
auto result = Packet::deserialize(received_bytes);
if (result) {
    auto [id, value] = *result;
}
```

## 元素接口

以下为调用形式示意，表达参数和职责，不是完整函数声明。

### 普通字段

```cpp
Field::serialize(value, output);
auto value = Field::deserialize(input);
```

`output` 和 `input` 都是字段自身占用的字节切片。字段自行处理数值编码和字节序；`Frame` 不实现这些转换。

### 帧头、帧尾

```cpp
Header::serialize(output);
bool valid = Header::deserialize(input);

Footer::serialize(output);
bool valid = Footer::deserialize(input);
```

序列化写入固定字节，反序列化检查固定字节是否匹配。

### 长度字段

```cpp
Length::serialize(output, region_size);
bool valid = Length::deserialize(input, region_size);
```

自身切片用于存放或读取长度值，`region_size` 是目标区域的预期字节数。保留现有 `LengthOf<Tag>` 的含义：区域内所有线路字节都计入长度，包括区域内的长度字段自身。

### CRC 字段

```cpp
Crc::serialize(output, region);
bool valid = Crc::deserialize(input, region);
```

自身切片用于存放或读取 CRC 值，`region` 是参与校验的只读区域切片。

### 区域标记

`Begin<Tag>` 和 `End<Tag>` 只参与布局计算，占用零字节，不需要提供空的编解码方法。

## Layout 与 Frame 的分工

`Layout<Elements...>` 负责：

- 计算整帧长度、每个元素的偏移和区域的起止位置、长度。
- 检查区域标记是否唯一配对，并按嵌套顺序闭合。
- 检查 `Length`、`Crc` 引用的区域是否存在。
- 检查长度字段能否表示目标区域长度。
- 检查 CRC 覆盖范围，拒绝覆盖自身或声明在后的其他 CRC 字段。

元素按声明索引定位，不能仅按元素类型定位；同一种 `Field` 可以在一帧中重复出现。

`Frame` 根据布局从局部输出数组或输入视图取得元素切片、区域切片或区域长度，直接传给对应元素。元素不需要知道布局如何计算，也不接收包装这些信息的 `Context`。

不同元素允许有不同的函数签名。`Frame` 按元素类别分派调用，不为统一签名增加额外抽象。

## 编解码调度与失败语义

二进制布局始终由模板参数的声明顺序决定；方法执行顺序不改变字段在帧内的位置。

序列化先处理普通字段，再处理自动字段。自动字段内部，先填充 `Header`、`Footer`、`Length`，再计算依赖这些字节的 CRC。

反序列化先检查输入大小，避免越界切片；随后按当前要求先解码普通字段，再验证自动字段。普通字段的结果先保存在局部结果中，全部验证通过后才作为整帧结果返回。

任何自动字段的 `deserialize` 返回 `false`，整个 `Frame` 解析失败，不向调用方提交部分解析结果。

## 输入与 CRC 约束

### 输入大小与错误信息

- 输入视图长度必须恰好等于 `Frame::size()`，多余字节也视为错误；大缓冲区由调用方先切片。
- 错误类别为 `BufferSizeMismatch`、`HeaderMismatch`、`FooterMismatch`、`LengthMismatch`、`CrcMismatch`。
- 返回第一个错误，不额外增加字段索引或多错误收集结构。布局错误仍在编译期拒绝。

### CRC 算法参数

保留 `CrcType`，各枚举值使用以下固定算法参数：

| 参数 | `CrcType::Crc8` | `CrcType::Crc16` |
| --- | --- | --- |
| 多项式 | `0x07` | `0x1021` |
| 初值 | `0x00` | `0xFFFF` |
| 输入反射 | 否 | 否 |
| 输出反射 | 否 | 否 |
| 最终异或值 | `0x00` | `0x0000` |

CRC16 参数与现有 `MODULE/pid/pid_viewer.hpp` 中的 `crc16_ccitt_false` 实现一致。算法参数与 CRC 结果写入帧时的字节序分别定义，字节序继续由 `Order` 控制。未来接入参数不同的协议时，需明确扩展算法类型，不改变现有枚举值的含义。

### CRC 覆盖规则

- 所有非 CRC 字段完成后，按 CRC 在模板参数中的声明顺序计算 CRC。
- 禁止校验区域包含 CRC 自身。
- 允许覆盖声明在前、已经生成的其他 CRC 字段；禁止覆盖声明在后的 CRC 字段，违规配置在编译期拒绝。
- CRC 可以位于其校验区域之前，只要满足上述依赖约束。
- 采用上述规则即可支持头部 CRC 与整帧 CRC 的组合，不提前实现通用依赖图或拓扑排序。
