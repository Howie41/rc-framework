# Frame 位字段设计

日期：2026-11-04。状态：已实现，公开接口及验证方式见 `MODULE/frame_codec/README.md`。

保留 Field，新增 `BitField<Bits, T, Order = BitOrder::LsbFirst>`。两者直接放入 Frame，每个数据字段对应扁平 DataTuple 中的一个值。

## 职责与用法

- Field 处理完整字节的整数、float 和固定字节数组，继续使用 std::endian 指定数值字节序。
- BitField 处理需要紧密打包的整数，以 BitOrder 指定明确的位打包规则。
- 现有 Field、Frame::serialize()、Frame::deserialize() 的公开接口保留；电机和 PID 的协议声明不需要迁移。

```cpp
using Packet = frame::Frame<
    frame::Header<0xAA>,
    frame::BitField<11, std::uint16_t>,
    frame::BitField<5, std::uint8_t>,
    frame::Field<std::uint16_t, std::endian::big>,
    frame::Field<float>,
    frame::Footer<0x55>>;

// DataTuple 为 tuple<uint16_t, uint8_t, uint16_t, float>。
Packet::DataTuple data{0x345, 0x12, 0x1234, 1.0f};
auto bytes = Packet::serialize(data);
// 线路字节：AA 45 93 12 34 00 00 80 3F 55。
```

字节序和位序是不同概念，不把 std::endian::big 与 BitOrder::MsbFirst 当作同义词。MsbFirst 定义字段位与字节内位置的映射；在字节对齐、完整字节宽度的整数上，这套具体映射会产生与大端字节序相同的字节数组。

## 排列规则

1. 连续 BitField 按声明顺序紧密打包，允许跨字节，不对单个字段补齐。
2. LsbFirst 从当前字节的 bit 0 向 bit 7 填入，各字段从数值低位向高位取位。
3. MsbFirst 从当前字节的 bit 7 向 bit 0 填入，各字段从数值高位向低位取位。
4. 未对齐时禁止切换位序，编译期报错；已到达字节边界时允许切换。
5. 遇到 Field、Header、Footer、Length、Crc、Begin、End 或帧结束时，向上补齐到字节边界。未使用位编码时写零，解码时忽略。
6. 零长度数组 Field 也结束当前位字段段，与其他 Field 使用相同的对齐规则。

例如 11 位值 0x345 后接 5 位值 0x12：LsbFirst 得到 `45 93`，MsbFirst 得到 `68 B2`。十六个连续 11 位字段共占 22 字节。单个 11 位字段后接 Field<uint16_t> 时，前者占两字节（含五位填充），后者从下一字节开始。

Begin、End 保证区域边界按字节对齐。Length 统计完整线路字节，CRC 校验这些字节，包括填充位。以上位序描述字节数组布局，与外设发送一个字节内部各位的先后顺序分开处理。

## BitField 类型与数值

首版支持不带 const 或 volatile 的 1、2、4、8 字节无符号整数类型，排除 bool；要求 `1 <= Bits <= sizeof(T) * 8`，在编译期检查。解码零扩展到 T。float 和数组继续交给 Field，有符号窄字段按后续实际需求扩展。

编码保留数值低 Bits 位，不做限幅，业务范围由调用方检查；例如 2048 编码为 11 位时得到零。该行为在公开文档与测试中明确，不改变 Frame 的编码返回类型。

## 实现范围与顺序

1. `frame_element.hpp`：新增 BitOrder 与 BitField，保留 Field 实现。BitField 接收自身覆盖的字节切片和起始位偏移，只修改自身覆盖的位。
2. `frame_layout.hpp`：增加位偏移计算，统一计算元素字节位置、字节内位偏移、覆盖字节数和整帧大小。保留字节区域接口；CRC 覆盖检查改用布局查询，避免对所有元素直接求 size()。
3. `frame_traits.hpp`：新增 is_bit_field 和同时识别 Field、BitField 的 is_data_element，原 is_field 仍只识别 Field。
4. `frame_codec.hpp`：两种字段都进入扁平 DataTuple；分别调用现有字节编解码和新增位编解码。验证切片使用 Layout 提供的覆盖字节数，不对 BitField 套用原来的 E::size()。
5. 补充测试和 README；运行已有协议与 CRC、Parser 回归以及 ARM 编译检查。

FrameParser 的缓存和流处理算法不需要修改。新增代码保持 constexpr、noexcept 和零堆分配。完整 64 位字段不能出现按 64 位移位，未对齐的 64 位字段可能覆盖九字节。

共存新增的分支集中在元素识别、布局及编解码调度；主要复杂度来自位偏移和跨字节读写。已有 Field 的有符号整数、float、数组、大小端实现继续复用，无需扩展到任意位起点，也无需迁移现有调用。

## 验证范围

- 两种位序的已知线路字节，不只依赖往返一致；十六个 11 位字段共 22 字节。
- 共享字节、跨字节、1 位与完整 64 位字段、零值、最大值、填充位、位序切换。
- BitField 与大小端 Field、float、数组、零长度数组混排，以及段尾自动对齐。
- 区域、Length、CRC、Parser 混用，含损坏帧和流式分块。
- 编译期拒绝非法类型、位宽、位序及未对齐的位序切换。
- 纯 Field 帧的帧长、DataTuple 类型和完整线路字节保持一致。

实现位于 frame_element、frame_traits、frame_layout 和 frame_codec 四个头文件，FrameParser 保持现有实现；本地验证用例位于 `tests/frame_codec/bit_field`（沿用 tests 目录的 Git 忽略规则）。
