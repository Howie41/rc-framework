#pragma once

#include "frame_traits.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <utility>

namespace frame::detail {

template <typename... Elements>
class Layout {
private:
    static constexpr std::size_t total_elements = sizeof...(Elements);

    struct Positions {
        std::array<std::size_t, total_elements> offsets{};
        std::array<std::size_t, total_elements> bit_offsets{};
        std::array<std::size_t, total_elements> sizes{};
        std::size_t frame_size{0};
        bool valid_bit_order{true};
    };

    // 仅位字段连续累计；其他元素（包括零字节区域标记）先结束当前位字段段。
    static constexpr Positions calculate_positions() noexcept {
        Positions positions{};
        std::size_t bit_position = 0;
        [[maybe_unused]] std::size_t index = 0;
        [[maybe_unused]] BitOrder previous_order = BitOrder::LsbFirst;
        ([&] {
            using E = Elements;
            if constexpr (is_bit_field<E>) {
                const auto bit_offset = bit_position % 8;
                if (bit_offset != 0 && previous_order != E::bit_order) {
                    positions.valid_bit_order = false;
                }
                positions.offsets[index] = bit_position / 8;
                positions.bit_offsets[index] = bit_offset;
                positions.sizes[index] = (bit_offset + E::bit_size() + 7) / 8;
                bit_position += E::bit_size();
                previous_order = E::bit_order;
            } else {
                bit_position += (8 - bit_position % 8) % 8;
                positions.offsets[index] = bit_position / 8;
                positions.sizes[index] = E::size();
                bit_position += E::size() * 8;
            }
            ++index;
        }(), ...);
        positions.frame_size = bit_position / 8 + (bit_position % 8 != 0 ? 1 : 0);
        return positions;
    }

    inline static constexpr auto kPositions = calculate_positions();
    static_assert(kPositions.valid_bit_order, "BitField 只能在字节边界切换位序");

public:
    /// @brief 返回完整二进制帧的字节数，区域边界标记不占用字节。
    [[nodiscard]]
    static constexpr std::size_t get_frame_size() noexcept {
        return kPositions.frame_size;
    }

    /// @brief 返回指定声明索引的元素相对于帧起始位置的字节偏移，索引从零开始。
    template <std::size_t Index> [[nodiscard]]
    static constexpr std::size_t get_element_offset() noexcept {
        static_assert(Index < total_elements, "元素索引必须小于元素总数");
        if constexpr (Index < total_elements) {
            return kPositions.offsets[Index];
        }
        return 0;
    }

    /// @brief 返回元素首字节内已使用的位数（0～7），位字段按自身位序解释该偏移。
    template <std::size_t Index> [[nodiscard]]
    static constexpr std::size_t get_element_bit_offset() noexcept {
        static_assert(Index < total_elements, "元素索引必须小于元素总数");
        if constexpr (Index < total_elements) return kPositions.bit_offsets[Index];
        return 0;
    }

    /// @brief 返回元素覆盖的字节数，位字段可能与相邻字段共享首尾字节。
    template <std::size_t Index> [[nodiscard]]
    static constexpr std::size_t get_element_size() noexcept {
        static_assert(Index < total_elements, "元素索引必须小于元素总数");
        if constexpr (Index < total_elements) return kPositions.sizes[Index];
        return 0;
    }

    /// @brief 返回指定 region tag 的区域相对于帧起始位置的字节偏移。
    template <typename RegionTag> [[nodiscard]]
    static constexpr std::size_t get_region_offset() noexcept {
        static_assert(is_region_tag<RegionTag>, "区域偏移查询要求使用 RegionTag 类型");
        if constexpr (is_region_tag<RegionTag>) {
            constexpr auto index = [] {
                constexpr std::array<bool, total_elements> matches{
                    std::is_same_v<Elements, Begin<RegionTag>>...};
                for (std::size_t i = 0; i < total_elements; ++i) {
                    if (matches[i]) return i;
                }
                return total_elements;
            }();
            static_assert(index < total_elements, "RegionTag 未声明区域");
            if constexpr (index < total_elements) {
                return get_element_offset<index>();
            }
        }
        return 0;
    }

    /// @brief 返回指定 region tag 的区域字节数，包含区域内所有字段，允许空区域。
    template <typename RegionTag> [[nodiscard]]
    static constexpr std::size_t get_region_size() noexcept {
        static_assert(is_region_tag<RegionTag>, "区域长度查询要求使用 RegionTag 类型");
        if constexpr (is_region_tag<RegionTag>) {
            constexpr auto index = [] {
                constexpr std::array<bool, total_elements> matches{
                    std::is_same_v<Elements, End<RegionTag>>...};
                for (std::size_t i = 0; i < total_elements; ++i) {
                    if (matches[i]) return i;
                }
                return total_elements;
            }();
            static_assert(index < total_elements, "RegionTag 未声明区域");
            if constexpr (index < total_elements) {
                return get_element_offset<index>() - get_region_offset<RegionTag>();
            }
        }
        return 0;
    }

private:
    // 按声明顺序遍历元素类型。
    template <typename Fn>
    static constexpr bool for_each_element(Fn&& fn) {
        return (static_cast<bool>(fn.template operator()<Elements>()) && ...);
    }

    // 每个 RegionTag 仅允许一组标记，区域必须按嵌套顺序闭合。
    static constexpr bool check_region_pairs() {
        std::array<std::size_t, total_elements> stack{};
        std::size_t depth = 0;
        const bool valid = for_each_element([&]<typename E>() {
            if constexpr (is_begin<E> || is_end<E>) {
                if constexpr (E::direction == 1) {
                    constexpr auto count = (std::size_t{0} + ... +
                                            std::size_t{std::is_same_v<E, Elements>});
                    if (count != 1) return false;
                    stack[depth] = E::region_tag::id;
                    depth += 1;
                } else {
                    if (depth == 0 || stack[depth - 1] != E::region_tag::id) return false;
                    depth -= 1;
                }
            }
            return true;
        });
        return valid && depth == 0;
    }
    static_assert(check_region_pairs(), "Begin、End 必须按相同 RegionTag 唯一配对，并按嵌套顺序闭合");

    // 配对检查通过后，检查所有长度和 CRC 字段引用的区域。
    static constexpr bool check_region_references() {
        if constexpr (!check_region_pairs()) {
            return true;
        } else {
            return for_each_element([]<typename E>() {
                if constexpr (is_length<E> || is_crc<E>) {
                    using RegionTag = typename E::region_tag;
                    if constexpr (!is_region_tag<RegionTag>) {
                        return false;
                    } else {
                        return (std::is_same_v<Begin<RegionTag>, Elements> || ...);
                    }
                }
                return true;
            });
        }
    }
    static_assert(check_region_references(), "Length、Crc 必须引用已声明区域的 region tag");

    // 前置检查失败时不展开区域查询，避免产生附带错误。
    static constexpr bool check_length_capacity() {
        if constexpr (!check_region_pairs() || !check_region_references()) {
            return true;
        } else {
            return for_each_element([]<typename E>() {

                if constexpr (is_length<E>) {
                    return std::cmp_less_equal(
                        get_region_size<typename E::region_tag>(),
                        std::numeric_limits<typename E::value_type>::max());
                }
                return true;
            });
        }
    }
    static_assert(check_length_capacity(), "区域长度超过 Length 字段的可表示范围");

    // CRC 只能依赖非 CRC 字段及声明在前的 CRC 字段。
    static constexpr bool check_crc_coverage() {
        if constexpr (!check_region_pairs() || !check_region_references() ||
                      !check_length_capacity()) {
            return true;
        } else {
            constexpr auto& offsets = kPositions.offsets;
            constexpr auto& sizes = kPositions.sizes;
            constexpr std::array<bool, total_elements> crc_fields{is_crc<Elements>...};
            std::size_t index = 0;
            return for_each_element([&]<typename E>() {
                const auto current = index++;
                if constexpr (is_crc<E>) {
                    constexpr auto begin = get_region_offset<typename E::region_tag>();
                    constexpr auto end = begin + get_region_size<typename E::region_tag>();
                    if constexpr (begin != end) {
                        for (std::size_t other = current; other < total_elements; ++other) {
                            if (crc_fields[other] && offsets[other] < end &&
                                begin < offsets[other] + sizes[other]) {
                                return false;
                            }
                        }
                    }
                }
                return true;
            });
        }
    }
    static_assert(check_crc_coverage(), "CRC 校验区域不能包含自身或声明在后的其他 CRC 字段");
};

} // namespace frame::detail
