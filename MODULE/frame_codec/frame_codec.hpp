#pragma once

#include "frame_layout.hpp"

#include <array>
#include <expected>
#include <span>
#include <tuple>
#include <utility>

namespace frame {

template <typename... Elements> class Frame {
private:
    using Layout = detail::Layout<Elements...>;

    // 强制实例化布局，使仅实例化 Frame 时也会执行布局检查。
    static_assert(sizeof(Layout) > 0);

    // 只为普通字段构造数据 tuple，其他元素不占 tuple 位置。
    static constexpr auto make_data_tuple() noexcept {
        return std::tuple_cat([] {
            if constexpr (detail::is_data_element<Elements>) {
                return std::tuple<typename Elements::value_type>{};
            } else {
                return std::tuple<>{};
            }
        }()...);
    }

    /// @brief 按声明顺序遍历元素。
    /// @param fn 对元素执行的操作，以元素类型和从零开始的索引作为模板参数。
    /// @return 所有调用均返回 true 或元素列表为空时返回 true；遇到 false 时立即停止并返回 false。
    template <typename Fn> static constexpr bool for_each_element(Fn&& fn) {
        return [&]<std::size_t... Indices>(std::index_sequence<Indices...>) {
            return (fn.template operator()<Elements, Indices>() && ...);
        }(std::index_sequence_for<Elements...>{});
    }

    // 元素在 tuple 中的位置等于它之前的普通字段数量。
    template <std::size_t Index> static constexpr std::size_t get_data_tuple_index() noexcept {
        constexpr std::array<bool, sizeof...(Elements)> fields{detail::is_data_element<Elements>...};
        std::size_t count = 0;
        for (std::size_t i = 0; i < Index; ++i)
            count += fields[i];
        return count;
    }

    template <typename RegionTag, typename Byte>
    static constexpr auto get_region(std::span<Byte, Layout::get_frame_size()> bytes) noexcept {
        return bytes.template subspan<Layout::template get_region_offset<RegionTag>(),
                                      Layout::template get_region_size<RegionTag>()>();
    }

public:
    /// @brief 按声明顺序保存 Field 和 BitField 数据的 tuple 类型，不包含自动字段。
    using DataTuple = decltype(make_data_tuple());

    /// @brief 返回完整帧的字节数。
    static constexpr std::size_t size() noexcept { return Layout::get_frame_size(); }

    /// @brief 返回帧起始处连续的 Header 字节，忽略不占字节的区域标记。
    static constexpr auto get_header_prefix() noexcept {
        constexpr auto length = [] {
            std::size_t count = 0;
            for_each_element([&]<typename E, std::size_t>() {
                if constexpr (detail::is_header<E>) count += E::size();
                return detail::is_header<E> || E::size() == 0;
            });
            return count;
        }();
        std::array<std::uint8_t, length> prefix{};
        std::size_t offset = 0;
        for_each_element([&]<typename E, std::size_t>() {
            if constexpr (detail::is_header<E>)
                for (const auto byte : E::bytes) prefix[offset++] = byte;
            return detail::is_header<E> || E::size() == 0;
        });
        return prefix;
    }

    /// @brief 将普通字段的 tuple 编码为完整帧，自动填充固定字段、长度和 CRC。
    /// @param data_tuple 数据字段数值，类型和顺序与元素声明中的 Field、BitField 对应。
    /// @return 拥有完整帧字节的数组，无普通字段时传入 std::tuple<>{}。
    [[nodiscard]]
    static constexpr std::array<std::uint8_t, size()>
    serialize(const DataTuple& data_tuple) noexcept {
        std::array<std::uint8_t, size()> bytes{};
        auto output = std::span{bytes};
        for_each_element([&]<typename E, std::size_t Index>() {
            if constexpr (detail::is_field<E>) {
                E::serialize(std::get<get_data_tuple_index<Index>()>(data_tuple),
                             output.template subspan<Layout::template get_element_offset<Index>(),
                                                     E::size()>());
            } else if constexpr (detail::is_bit_field<E>) {
                E::template serialize<Layout::template get_element_bit_offset<Index>()>(
                    std::get<get_data_tuple_index<Index>()>(data_tuple),
                    output.template subspan<Layout::template get_element_offset<Index>(),
                                            Layout::template get_element_size<Index>()>());
            }
            return true;
        });
        for_each_element([&]<typename E, std::size_t Index>() {
            if constexpr (detail::is_header<E> || detail::is_footer<E>) {
                E::serialize(output.template subspan<Layout::template get_element_offset<Index>(),
                                                     E::size()>());
            } else if constexpr (detail::is_length<E>) {
                E::serialize(output.template subspan<Layout::template get_element_offset<Index>(),
                                                     E::size()>(),
                             Layout::template get_region_size<typename E::region_tag>());
            }
            return true;
        });
        for_each_element([&]<typename E, std::size_t Index>() {
            if constexpr (detail::is_crc<E>) {
                E::serialize(output.template subspan<Layout::template get_element_offset<Index>(),
                                                     E::size()>(),
                             get_region<typename E::region_tag>(output));
            }
            return true;
        });
        return bytes;
    }

    /// @brief 解码完整帧，所有自动字段验证通过后返回普通字段的 tuple。
    /// @param input 长度必须恰好等于 size() 的完整候选帧。
    /// @return 成功时返回独立数值；失败时返回首个错误，不提供部分解析结果。
    [[nodiscard]]
    static constexpr std::expected<DataTuple, Error>
    deserialize(std::span<const std::uint8_t> input) noexcept {
        if (input.size() != size())
            return std::unexpected(Error::BufferSizeMismatch);
        const std::span<const std::uint8_t, size()> bytes{input};
        auto data_tuple = make_data_tuple();
        for_each_element([&]<typename E, std::size_t Index>() {
            if constexpr (detail::is_field<E>) {
                std::get<get_data_tuple_index<Index>()>(data_tuple) = E::deserialize(
                    bytes.template subspan<Layout::template get_element_offset<Index>(),
                                           E::size()>());
            } else if constexpr (detail::is_bit_field<E>) {
                std::get<get_data_tuple_index<Index>()>(data_tuple) =
                    E::template deserialize<Layout::template get_element_bit_offset<Index>()>(
                        bytes.template subspan<Layout::template get_element_offset<Index>(),
                                               Layout::template get_element_size<Index>()>());
            }
            return true;
        });
        Error error{};
        const bool valid = for_each_element([&]<typename E, std::size_t Index>() {
            const auto slice =
                bytes.template subspan<Layout::template get_element_offset<Index>(),
                                       Layout::template get_element_size<Index>()>();
            bool matches = true;
            if constexpr (detail::is_header<E>) {
                matches = E::deserialize(slice);
                if (!matches)
                    error = Error::HeaderMismatch;
            } else if constexpr (detail::is_footer<E>) {
                matches = E::deserialize(slice);
                if (!matches)
                    error = Error::FooterMismatch;
            } else if constexpr (detail::is_length<E>) {
                matches = E::deserialize(
                    slice, Layout::template get_region_size<typename E::region_tag>());
                if (!matches)
                    error = Error::LengthMismatch;
            } else if constexpr (detail::is_crc<E>) {
                matches = E::deserialize(slice, get_region<typename E::region_tag>(bytes));
                if (!matches)
                    error = Error::CrcMismatch;
            }
            return matches;
        });
        if (!valid)
            return std::unexpected(error);
        return data_tuple;
    }
};

} // namespace frame
