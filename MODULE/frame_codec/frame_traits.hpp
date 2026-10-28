#pragma once

#include "frame_element.hpp"

namespace frame::detail {

/// @brief 判断类型是否为普通数值或固定长度字节数组字段 Field。
template <typename T>
inline constexpr bool is_field = false;

template <typename T, std::endian Order>
inline constexpr bool is_field<Field<T, Order>> = true;

template <typename T>
inline constexpr bool is_header = false;

template <std::uint8_t... Bytes>
inline constexpr bool is_header<Header<Bytes...>> = true;

template <typename T>
inline constexpr bool is_footer = false;

template <std::uint8_t... Bytes>
inline constexpr bool is_footer<Footer<Bytes...>> = true;

template <typename T>
inline constexpr bool is_begin = false;

template <typename RegionTag>
inline constexpr bool is_begin<Begin<RegionTag>> = true;

template <typename T>
inline constexpr bool is_end = false;

template <typename RegionTag>
inline constexpr bool is_end<End<RegionTag>> = true;

template <typename T>
inline constexpr bool is_length = false;

template <typename T, typename Meaning, std::endian Order>
inline constexpr bool is_length<Length<T, Meaning, Order>> = true;

template <typename T>
inline constexpr bool is_crc = false;

template <typename RegionTag, CrcType Type, std::endian Order>
inline constexpr bool is_crc<Crc<RegionTag, Type, Order>> = true;

} // namespace frame::detail
