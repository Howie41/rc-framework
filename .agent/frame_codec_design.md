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

当前已建立 `frame_traits.hpp`，包含 `is_field`、`is_header`、`is_footer`、`is_begin`、`is_end`、`is_length` 和 `is_crc`，Layout 按上述方向包含该文件。`is_region_tag` 保留在 `frame_element.hpp`，供元素自身检查模板参数使用，避免反向依赖。`Frame` 已实现 `size()`、`serialize()`、`deserialize()` 和布局检查触发，不提供 `Values`、`Bytes` 类型别名。

## 视图与存储

首版采用固定长度帧，帧长度、元素偏移和区域范围均在编译期确定。`Length` 写入或验证既定的区域长度，不根据输入中的长度值改变布局。

序列化在函数内创建局部 `std::array`，直接将该数组的元素切片传给各元素，完成后按值返回整个数组。元素不各自生成临时数组再拼接，也不返回指向局部数组的视图。首版不额外提供写入外部缓冲区的重载。

反序列化接收只读字节视图，仅在调用期间读取输入，不保存视图，也不复制整帧字节。解析出的值放入局部 tuple，全部验证通过后按值返回。当前普通字段均为数值类型，返回结果不依赖输入缓冲区的生命周期。

Frame 对外接收动态长度的只读输入视图，检查大小后转为固定长度视图。元素和区域切片均通过 Layout 的编译期偏移及长度生成；写入切片可写，读取或校验时使用只读接口。

## Frame 公开接口

```cpp
template <typename... Elements>
class Frame {
public:
    static constexpr std::size_t size() noexcept;

    [[nodiscard]]
    static constexpr std::array<std::uint8_t, size()> serialize(
        const decltype(make_values())& values) noexcept;

    [[nodiscard]]
    static constexpr std::expected<decltype(make_values()), Error> deserialize(
        std::span<const std::uint8_t> input) noexcept;
};
```

以上是公开接口示意。`make_values()` 是私有辅助函数，按声明顺序提取普通字段类型并构造 tuple，不包含帧头、帧尾、长度、CRC 和区域标记。其类型通过 `decltype` 自动推导，不增加类型别名。序列化仅提供 tuple 输入形式，和反序列化的成功结果使用同一类型；没有普通字段时使用 `std::tuple<>`，空帧也支持编解码。

当前元素的格式配置错误通过编译期检查拒绝，合法数值的序列化没有预期的运行时失败，因此直接返回数组。反序列化需要处理外部输入错误，使用 `std::expected<解析值的 tuple, Error>`，失败时不提供部分 tuple。

```cpp
auto bytes = Packet::serialize(std::tuple{std::uint16_t{42}, 1.5f});
auto result = Packet::deserialize(received_bytes);
if (result) {
    auto [id, value] = *result;
}
```

## 元素接口

以下为调用形式示意，表达参数和职责，不是完整函数声明。

当前 `Field`、`Header`、`Footer`、`Length`、`Crc` 的序列化和反序列化均已实现。五类元素的方法均为 `static constexpr`、`noexcept`；读取或验证方法标注 `[[nodiscard]]`。

元素自身切片使用 `std::span<std::uint8_t, size()>` 或 `std::span<const std::uint8_t, size()>`。Frame 负责在检查整帧输入大小后提供正确切片，Layout 负责区域引用、长度容量等配置检查；元素内部不重复这些检查。

`Field` 和 `Length` 共用 `frame::detail` 中的数值编解码辅助函数，通过 `std::bit_cast` 获取对象位表示并按目标字节序排列，避免非对齐的类型指针访问。有符号整数和 float 的位表示保持不变，浮点数不经过整数数值转换。

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

自身切片用于存放或读取长度值，`region_size` 是目标区域的预期字节数。保留现有 `LengthOf<RegionTag>` 的含义：区域内所有线路字节都计入长度，包括区域内的长度字段自身。

`region_size` 参数使用 `std::size_t`，序列化在 Layout 已保证容量的前提下转换为长度字段类型并编码；反序列化解码字段后与预期区域长度比较，返回 `bool`。

### CRC 字段

```cpp
Crc::serialize(output, region);
bool valid = Crc::deserialize(input, region);
```

自身切片用于存放或读取 CRC 值，`region` 是参与校验的只读区域切片。

CRC 计算位于 `frame_element.hpp` 的 `frame::detail` 命名空间，由 `calculate_crc8()`、`calculate_crc16()` 接收动态长度的只读字节视图，按固定参数逐位计算，不修改输入。空区域分别返回初值 `0x00`、`0xFFFF`。

`Crc` 的私有 `calculate()` 按 `Type` 在编译期选择算法。序列化通过 `serialize_value` 按 `Order` 写入计算值，反序列化通过 `deserialize_value` 读取保存值并与计算值比较。切片大小、区域引用与覆盖范围继续由 Frame、Layout 保证，不在元素内重复检查。

### 区域标记

区域的类型标识统一称为 region tag。代码中类型名和模板参数使用 `RegionTag`，类型别名及 trait 使用 `region_tag`、`is_region_tag`，不使用泛称 `Tag` 或 `tag`。

`Begin<RegionTag>` 和 `End<RegionTag>` 是区域的边界标记，只参与布局计算，占用零字节，不需要提供空的编解码方法。

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

### Layout 实现进度

`Frame::size()` 转发调用 `Layout::get_frame_size()`。`Frame` 通过 `static_assert(sizeof(Layout) > 0)` 强制实例化布局，确保仅实例化 Frame 时也会触发 Layout 内部检查。

| 方法 | 状态 | 职责 |
| --- | --- | --- |
| `check_region_pairs()` | 已实现 | 检查 region tag 的 Begin、End 唯一配对及嵌套闭合，私有方法 |
| `get_frame_size()` | 已实现 | 返回整帧总字节数，由 `Frame::size()` 转发调用 |
| `get_element_offset<Index>()` | 已实现 | 按从零开始的元素声明索引返回字节偏移，区域边界标记也占用索引，支持重复元素类型；越界索引编译失败 |
| `get_region_offset<RegionTag>()` | 已实现 | 按 region tag 返回区域起始字节偏移，空区域也可查询；非法 region tag 类型或未声明区域编译失败 |
| `get_region_size<RegionTag>()` | 已实现 | 按 region tag 返回区域字节数，即 End 与 Begin 的字节偏移之差；包含区域内所有字段，允许空区域；非法 region tag 类型或未声明区域编译失败 |
| `check_region_references()` | 已实现 | 检查 Length、Crc 引用的 region tag 是否对应已声明区域 |
| `check_length_capacity()` | 已实现 | 检查长度字段能否表示目标区域长度 |
| `check_crc_coverage()` | 已实现 | 检查 CRC 是否覆盖自身或声明在后的其他 CRC 字段 |

查询方法提供编译期结果，四个检查方法均为私有方法，通过 `static_assert` 自动执行。检查顺序为区域配对、region tag 引用、长度容量、CRC 覆盖。前置检查失败时，后续检查通过 `if constexpr` 跳过依赖该前提的代码，避免附带查询错误。切片和编解码调度仍由 Frame 完成。

`get_region_size<RegionTag>()` 包含区域内的长度字段自身、帧头、帧尾、CRC 和嵌套区域中的字节，不包含区域外字段。空区域返回零，Begin、End 自身不占字节。

查询方法会在查询时拒绝未声明区域；此外，`check_region_references()` 会在实例化 Layout 时检查所有 Length、Crc 的区域引用，即使没有调用区域查询。长度容量检查复用 `get_region_size()` 并通过 `std::cmp_less_equal` 比较字段上限。CRC 覆盖检查复用元素偏移和区域范围，拒绝自身及声明在后的 CRC 与非空校验区域产生字节交集，不建立依赖图。

`tests/frame_codec/layout_offsets_test.cpp` 已覆盖重复字段、边界标记索引、空区域、嵌套区域、长度字段自身及区域内外字节的计数。正例编译检查通过，8 个非法偏移或长度查询均由编译期断言拒绝。这些用例已接入 CMake/CTest 并通过验证。

`tests/frame_codec/layout_checks_test.cpp` 覆盖长度达到上限、空区域、宽整数长度字段、CRC 位于校验区域之前、覆盖声明在前的 CRC 及重复 CRC 类型。非法用例覆盖未声明区域、非 region tag 类型、长度溢出、CRC 自覆盖、覆盖后续 CRC、循环依赖，以及前置配对检查失败时不产生附带错误。

四组测试文件的正例均通过 GCC C++23 直接编译检查，29 个非法配置均触发编译期断言；其中新增检查的 8 个非法配置各只触发一条断言。旧格式测试已改用 Layout 的区域查询接口。

`tests/frame_codec/element_codec_test.cpp` 验证 1/2/4/8 字节整数的大小端字节序和有符号边界、float 的固定字节向量（含负零、无穷和 NaN 位表示）、帧头帧尾任意字节不匹配、长度匹配及不匹配，并检查非对齐切片与相邻字节不被改写。该测试包含编译期断言及运行时用例，已通过 GCC C++23 编译和运行，并接入 CMake。

`tests/frame_codec/crc_codec_test.cpp` 已接入 CMake，通过 GCC C++23 编译期断言和运行时测试。覆盖 ASCII `123456789` 的已知校验值（CRC8 为 `0xF4`，CRC16 为 `0x29B1`）、空区域、大小端保存、CRC 保存值及区域内容逐位篡改、非对齐切片和区域外字节不参与计算。CRC 与其他元素的运行时测试及现有四组布局正例编译检查均通过。

`tests/frame_codec/frame_codec_test.cpp` 覆盖完整帧的已知字节向量、重复字段、混合数值及大小端、嵌套 CRC、CRC 位于帧首、长度字段自身计数、无普通字段、空帧、输入长短不匹配、各自动字段错误和返回数值的独立生命周期。整帧及元素测试含编译期断言和运行时验证；测试目标统一使用 C++23。MinGW Makefiles 构建及 CTest 全部 36 项检查通过。

## 编解码调度与失败语义

二进制布局始终由模板参数的声明顺序决定；方法执行顺序不改变字段在帧内的位置。

序列化先处理普通字段，再处理自动字段。自动字段内部，先填充 `Header`、`Footer`、`Length`，再计算依赖这些字节的 CRC。

反序列化先检查输入大小，避免越界切片；随后按当前要求先解码普通字段，再验证自动字段。普通字段的结果先保存在局部结果中，全部验证通过后才作为整帧结果返回。

自动字段在反序列化阶段按声明顺序验证，遇到首个失败即停止；CRC 直接使用输入中的完整区域字节，不依赖其他字段的验证结果。重复类型的普通字段通过声明索引及此前普通字段数量，映射到 tuple 中不同的位置。

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
