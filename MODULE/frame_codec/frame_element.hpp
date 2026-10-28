#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <limits>
#include <span>
#include <tuple>
#include <type_traits>

namespace frame {

/// @brief 元素读取验证的失败原因，成功由 std::expected 的有效状态表示。
enum class Error : std::uint8_t {
    HeaderMismatch,
    FooterMismatch,
    LengthMismatch,
    BufferSizeMismatch,
    CrcMismatch,
};

/// @brief 区域的类型标识，用于关联边界标记、长度字段和 CRC 字段。
/// @tparam Id 区域标识值，不同区域使用不同的值。
/// @details 例如 using Payload = RegionTag<0>;，随后使用 Begin<Payload>、End<Payload> 定义区域。
template <std::size_t Id>
struct RegionTag {
    static constexpr std::size_t id = Id;
};

enum class CrcType : std::uint8_t {
    Crc8,
    Crc16,
};

namespace detail {

template <typename T>
inline constexpr bool is_region_tag = false;

template <std::size_t Id>
inline constexpr bool is_region_tag<RegionTag<Id>> = true;

/// @brief 将数值的对象位表示按指定字节序写入字节切片。
/// @tparam T 数值类型，要求可使用 std::bit_cast 转换为同宽字节数组。
/// @tparam Order 输出字节序。
template <typename T, std::endian Order>
constexpr void serialize_value(T value, std::span<std::uint8_t, sizeof(T)> output) noexcept {
    const auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(value);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        output[i] = bytes[Order == std::endian::native ? i : sizeof(T) - 1 - i];
    }
}

/// @brief 按指定字节序读取字节切片并还原数值，供字段和长度元素复用。
/// @tparam T 返回的数值类型，要求可使用 std::bit_cast 从同宽字节数组转换。
/// @tparam Order 输入字节序。序。
template <typename T, std::endian Order>
constexpr T deserialize_value(std::span<const std::uint8_t, sizeof(T)> input) noexcept {
    std::array<std::uint8_t, sizeof(T)> bytes{};
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        bytes[i] = input[Order == std::endian::native ? i : sizeof(T) - 1 - i];
    }
    return std::bit_cast<T>(bytes);
}

} // namespace detail

/// @brief 标记区域起点，不占用帧字节，也不参与运行时编解码。
/// @details 在 Frame 的元素列表中使用 Begin<Payload>，与同一 region tag 的 End<Payload> 配对。
template <typename RegionTag>
struct Begin {
    static_assert(detail::is_region_tag<RegionTag>, "Begin 要求使用 RegionTag 类型");
    using region_tag = RegionTag;
    static constexpr int direction = 1;
    static constexpr std::size_t size() noexcept { return 0; }
};

/// @brief 标记区域终点，不占用帧字节，区域结束于此标记之前。
/// @details 例如 Begin<Payload>, Field<float>, End<Payload> 定义一个四字节区域；区域可嵌套。
template <typename RegionTag>
struct End {
    static_assert(detail::is_region_tag<RegionTag>, "End 要求使用 RegionTag 类型");
    using region_tag = RegionTag;
    static constexpr int direction = -1;
    static constexpr std::size_t size() noexcept { return 0; }
};

/// @brief 指定长度字段表示哪个 region tag 对应区域的字节数，本身不作为帧元素使用。
/// @details 作为 Length 的第二个模板参数，例如 Length<std::uint16_t, LengthOf<Payload>>。
///          区域内所有线路字节均计入长度，包括区域内的长度字段自身。
template <typename RegionTag>
struct LengthOf {
    using region_tag = RegionTag;
    static_assert(detail::is_region_tag<RegionTag>, "LengthOf 要求使用 RegionTag 类型");
};


/// @brief 固定帧头字节序列，序列化时写入，反序列化时逐字节验证。
/// @details 例如 Header<0xAA, 0x55> 占用两个字节，通常置于 Frame 元素列表开头。
///          至少指定一个字节；不需要帧头时直接省略该元素。
template <std::uint8_t... Bytes>
struct Header {
    static_assert(sizeof...(Bytes) > 0, "帧头必须至少包含一个字节");

    inline static constexpr std::array<std::uint8_t, sizeof...(Bytes)> bytes{Bytes...};
    static constexpr std::size_t size() noexcept { return bytes.size(); }

    /// @brief 将固定帧头写入自身切片。
    static constexpr void serialize(std::span<std::uint8_t, size()> output) noexcept {
        for (std::size_t i = 0; i < size(); ++i) output[i] = bytes[i];
    }

    /// @brief 检查自身切片是否与固定帧头匹配。
    [[nodiscard]]
    static constexpr bool deserialize(std::span<const std::uint8_t, size()> input) noexcept {
        for (std::size_t i = 0; i < size(); ++i) {
            if (input[i] != bytes[i]) return false;
        }
        return true;
    }
};

/// @brief 固定帧尾字节序列，序列化时写入，反序列化时逐字节验证。
/// @details 例如 Footer<0x0D, 0x0A> 占用两个字节，通常置于 Frame 元素列表末尾。
///          至少指定一个字节；不需要帧尾时直接省略该元素。
template <std::uint8_t... Bytes>
struct Footer {
    static_assert(sizeof...(Bytes) > 0, "帧尾必须至少包含一个字节");

    inline static constexpr std::array<std::uint8_t, sizeof...(Bytes)> bytes{Bytes...};
    static constexpr std::size_t size() noexcept { return bytes.size(); }

    /// @brief 将固定帧尾写入自身切片。
    static constexpr void serialize(std::span<std::uint8_t, size()> output) noexcept {
        for (std::size_t i = 0; i < size(); ++i) output[i] = bytes[i];
    }

    /// @brief 检查自身切片是否与固定帧尾匹配。
    [[nodiscard]]
    static constexpr bool deserialize(std::span<const std::uint8_t, size()> input) noexcept {
        for (std::size_t i = 0; i < size(); ++i) {
            if (input[i] != bytes[i]) return false;
        }
        return true;
    }
};

/// @brief 自动长度字段，写入或验证指定区域的字节数，不改变固定帧布局。
/// @tparam T 保存长度的无符号整数类型。
/// @tparam Meaning 长度含义，使用 LengthOf<RegionTag> 指定区域。
/// @tparam Order 长度值的字节序，默认为小端。
/// @details 例如 Length<std::uint16_t, LengthOf<Payload>>；调用编解码方法时传入区域长度。
template <typename T, typename Meaning, std::endian Order = std::endian::little>
struct Length {
    static_assert(std::is_unsigned_v<T> && !std::is_same_v<T, bool> &&
                      std::is_same_v<T, std::remove_cv_t<T>>,
                  "Length 要求使用不带 const 或 volatile 限定且非 bool 的无符号整数类型");

    using value_type = T;
    using meaning = Meaning;
    using region_tag = typename Meaning::region_tag;
    static constexpr std::endian byte_order = Order;
    static constexpr std::size_t size() noexcept { return sizeof(T); }

    /// @brief 将 Layout 已验证可表示的区域长度写入自身切片。
    static constexpr void serialize(std::span<std::uint8_t, size()> output,
                                    std::size_t region_size) noexcept {
        detail::serialize_value<T, Order>(static_cast<T>(region_size), output);
    }

    /// @brief 检查自身切片中的长度是否等于预期区域长度。
    [[nodiscard]]
    static constexpr bool deserialize(std::span<const std::uint8_t, size()> input,
                                      std::size_t region_size) noexcept {
        return detail::deserialize_value<T, Order>(input) == region_size;
    }
};

/// @brief 普通字段，在数值或固定长度字节数组与自身切片之间转换，不保存运行时数据。
/// @tparam T 1/2/4/8 字节整数（不含 bool）、IEEE 754 binary32 的 float，
///           或元素为 uint8_t、char、std::byte 的 std::array。
/// @tparam Order 数值字段字节序，默认为小端；字节数组始终保持元素顺序。
/// @details 例如 Field<std::uint16_t, std::endian::big> 定义一个两字节大端字段。
///          使用 serialize(value, output) 写入数值，deserialize(input) 返回解析值。
template <typename T, std::endian Order = std::endian::little>
struct Field {
    static_assert((std::is_integral_v<T> && !std::is_same_v<T, bool> &&
                   (sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8)) ||
                      std::is_same_v<T, float>,
                  "Field 要求使用固定宽度整数类型或 float");
    static_assert(std::is_same_v<T, std::remove_cv_t<T>>, "Field 要求使用不带 const 或 volatile 限定的类型");
    static_assert(!std::is_same_v<T, float> ||
                      (sizeof(float) == 4 && std::numeric_limits<float>::is_iec559 &&
                       std::numeric_limits<float>::digits == 24),
                  "浮点字段要求使用 IEEE 754 binary32 格式");
    using value_type = T;
    static constexpr std::endian byte_order = Order;
    static constexpr std::size_t size() noexcept { return sizeof(T); }

    /// @brief 按指定字节序将数值写入自身切片。
    static constexpr void serialize(value_type value,
                                    std::span<std::uint8_t, size()> output) noexcept {
        detail::serialize_value<T, Order>(value, output);
    }

    /// @brief 按指定字节序从自身切片读取数值。
    [[nodiscard]]
    static constexpr value_type deserialize(std::span<const std::uint8_t, size()> input) noexcept {
        return detail::deserialize_value<T, Order>(input);
    }
};

/// @brief 固定长度字节数组字段，整体对应 tuple 中的一个值，不进行字节序转换。
/// @tparam Byte 数组元素类型，支持 uint8_t、char 和 std::byte。
/// @tparam Size 数组的字节数，允许为零。
/// @tparam Order 保留与数值 Field 一致的模板形式，不影响字节数组的元素顺序。
template <typename Byte, std::size_t Size, std::endian Order>
    requires (std::is_same_v<Byte, std::uint8_t> || std::is_same_v<Byte, char> ||
              std::is_same_v<Byte, std::byte>)
struct Field<std::array<Byte, Size>, Order> {
    using value_type = std::array<Byte, Size>;
    static constexpr std::endian byte_order = Order;

    /// @brief 返回线上字节数，零长度数组不占用字节。
    static constexpr std::size_t size() noexcept { return Size; }

    /// @brief 将全部数组元素按原顺序写入自身切片，不处理字符串终止符或补零。
    /// @param value 待编码的完整数组。
    /// @param output 字段自身的可写字节切片。
    static constexpr void serialize(const value_type& value,
                                    std::span<std::uint8_t, size()> output) noexcept {
        for (std::size_t i = 0; i < Size; ++i) {
            output[i] = std::bit_cast<std::uint8_t>(value[i]);
        }
    }

    /// @brief 按原顺序读取全部字节，返回拥有独立数据的数组。
    /// @param input 字段自身的只读字节切片。
    /// @return 解码后的完整数组。
    [[nodiscard]]
    static constexpr value_type deserialize(std::span<const std::uint8_t, size()> input) noexcept {
        value_type value{};
        for (std::size_t i = 0; i < Size; ++i) {
            value[i] = std::bit_cast<Byte>(input[i]);
        }
        return value;
    }
};

namespace detail {

/// @brief 计算 CRC8：多项式 0x07，初值 0x00，不反射，最终异或值 0x00。
/// @param input 参与校验的字节区域，允许为空。
/// @return 区域的 CRC8，空区域返回 0x00。
[[nodiscard]]
constexpr std::uint8_t calculate_crc8(std::span<const std::uint8_t> input) noexcept {
    std::uint8_t crc = 0x00;
    for (const auto byte : input) {
        crc ^= byte;
        for (std::size_t bit = 0; bit < 8; ++bit) {
            crc = static_cast<std::uint8_t>((crc & 0x80) != 0
                                               ? (crc << 1) ^ 0x07
                                               : crc << 1);
        }
    }
    return crc;
}

/// @brief 计算 CRC16：多项式 0x1021，初值 0xFFFF，不反射，最终异或值 0x0000。
/// @param input 参与校验的字节区域，允许为空。
/// @return 区域的 CRC16，空区域返回 0xFFFF。
[[nodiscard]]
constexpr std::uint16_t calculate_crc16(std::span<const std::uint8_t> input) noexcept {
    std::uint16_t crc = 0xFFFF;
    for (const auto byte : input) {
        crc ^= static_cast<std::uint16_t>(static_cast<std::uint16_t>(byte) << 8);
        for (std::size_t bit = 0; bit < 8; ++bit) {
            crc = static_cast<std::uint16_t>((crc & 0x8000) != 0
                                                ? (crc << 1) ^ 0x1021
                                                : crc << 1);
        }
    }
    return crc;
}

} // namespace detail

/// @brief 自动 CRC 字段，计算指定区域的校验值并写入，或验证已保存的校验值。
/// @tparam RegionTag 校验区域的 region tag，由对应 Begin、End 标记定义。
/// @tparam Type CRC 算法，默认为 Crc16；字段宽度随算法为一或两字节。
/// @tparam Order 校验值写入帧时的字节序，默认为小端，不影响 CRC 计算算法。
/// @details 例如 Crc<Payload, CrcType::Crc16, std::endian::big>；编解码时传入自身切片和校验区域。
template <typename RegionTag, CrcType Type = CrcType::Crc16,
          std::endian Order = std::endian::little>
struct Crc {
    static_assert(detail::is_region_tag<RegionTag>, "Crc 要求使用 RegionTag 类型");

    using region_tag = RegionTag;
    using value_type = std::conditional_t<Type == CrcType::Crc8, std::uint8_t, std::uint16_t>;
    static constexpr CrcType type = Type;
    static constexpr std::endian byte_order = Order;
    static constexpr std::size_t size() noexcept { return sizeof(value_type); }

    /// @brief 计算区域 CRC，并按指定字节序写入自身切片。
    /// @param output CRC 字段自身的可写切片。
    /// @param region Layout 已验证覆盖范围的只读校验区域。
    static constexpr void serialize(std::span<std::uint8_t, size()> output,
                                    std::span<const std::uint8_t> region) noexcept {
        detail::serialize_value<value_type, Order>(calculate(region), output);
    }

    /// @brief 读取自身切片中的 CRC，与重新计算的区域 CRC 比较。
    /// @return 保存值与计算值相等时返回 true。
    [[nodiscard]]
    static constexpr bool deserialize(std::span<const std::uint8_t, size()> input,
                                      std::span<const std::uint8_t> region) noexcept {
        return detail::deserialize_value<value_type, Order>(input) == calculate(region);
    }

private:
    static constexpr value_type calculate(std::span<const std::uint8_t> region) noexcept {
        if constexpr (Type == CrcType::Crc8) {
            return detail::calculate_crc8(region);
        } else {
            return detail::calculate_crc16(region);
        }
    }
};

} // namespace frame
