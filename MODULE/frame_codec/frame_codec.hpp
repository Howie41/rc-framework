#pragma once

#include "frame_element.hpp"

namespace frame {

template <typename... Elements>
class Frame {
public:
    static constexpr std::size_t size() noexcept {
        return (std::size_t{0} + ... + Elements::size());
    }

private:
    static constexpr std::size_t total_element = sizeof...(Elements);

    // 按声明顺序遍历元素类型
    template <typename Fn>
    static constexpr bool for_each_element(Fn&& fn) {
        return (static_cast<bool>(fn.template operator()<Elements>()) && ...);
    }

    // 每个 RegionTag 仅允许一组标记，区域必须按嵌套顺序闭合。
    static constexpr bool check_region_pairs() {
        std::array<std::size_t, total_element> stack{};
        std::size_t depth = 0;
        const bool valid = for_each_element([&]<typename E>() {
            if constexpr (detail::is_begin<E> || detail::is_end<E>) {
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

};

} // namespace frame
