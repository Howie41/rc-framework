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
};

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

} // namespace detail

template <typename RegionTag>
struct Begin {
    static_assert(detail::is_region_tag<RegionTag>, "Begin 要求使用 RegionTag 类型");
    using region_tag = RegionTag;
    static constexpr int direction = 1;
    static constexpr std::size_t size() noexcept { return 0; }
};

template <typename RegionTag>
struct End {
    static_assert(detail::is_region_tag<RegionTag>, "End 要求使用 RegionTag 类型");
    using region_tag = RegionTag;
    static constexpr int direction = -1;
    static constexpr std::size_t size() noexcept { return 0; }
};

namespace detail {

template <typename T>
inline constexpr bool is_begin = false;

template <typename RegionTag>
inline constexpr bool is_begin<Begin<RegionTag>> = true;

template <typename T>
inline constexpr bool is_end = false;

template <typename RegionTag>
inline constexpr bool is_end<End<RegionTag>> = true;

} // namespace detail

// 区域内所有线路字节都计入长度，包括区域内的长度字段自身。
template <typename RegionTag>
struct LengthOf {
    using region_tag = RegionTag;
    static_assert(detail::is_region_tag<RegionTag>, "LengthOf 要求使用 RegionTag 类型");
};


// 帧头和帧尾通过是否出现在 Frame 中表示可选，不使用空字节序列占位。
template <std::uint8_t... Bytes>
struct Header {
    static_assert(sizeof...(Bytes) > 0, "帧头必须至少包含一个字节");

    inline static constexpr std::array<std::uint8_t, sizeof...(Bytes)> bytes{Bytes...};
    static constexpr std::size_t size() noexcept { return bytes.size(); }
};

template <std::uint8_t... Bytes>
struct Footer {
    static_assert(sizeof...(Bytes) > 0, "帧尾必须至少包含一个字节");

    inline static constexpr std::array<std::uint8_t, sizeof...(Bytes)> bytes{Bytes...};
    static constexpr std::size_t size() noexcept { return bytes.size(); }
};

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
};

// 首版只描述固定宽度整数与 IEEE 754 binary32，不使用结构体内存布局。
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
};

template <typename RegionTag, CrcType Type = CrcType::Crc16,
          std::endian Order = std::endian::little>
struct Crc {
    static_assert(detail::is_region_tag<RegionTag>, "Crc 要求使用 RegionTag 类型");

    using region_tag = RegionTag;
    using value_type = std::conditional_t<Type == CrcType::Crc8, std::uint8_t, std::uint16_t>;
    static constexpr CrcType type = Type;
    static constexpr std::endian byte_order = Order;
    static constexpr std::size_t size() noexcept { return sizeof(value_type); }
};

} // namespace frame
