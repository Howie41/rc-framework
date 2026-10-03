#pragma once

#include "main.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <format>
#include <string_view>
#include <utility>

template <std::size_t Capacity>
class StaticString {
public:
    // 结构化类型要求数据成员公开，供编译期非类型模板参数使用
    std::array<char, Capacity + 1> m_array{};
    std::size_t m_length{0};

    constexpr StaticString() : m_length(0) {
        m_array[0] = '\0';
    }

    constexpr StaticString(const char* str) : m_length(0) {
        while (m_length < Capacity && str[m_length] != '\0') {
            m_array[m_length] = str[m_length];
            m_length += 1;
        }
        m_array[m_length] = '\0';
    }

    StaticString(std::string_view str)
     : m_length(std::min(Capacity, str.size())) {
        str.copy(m_array.data(), m_length);
        m_array[m_length] = '\0';
    }

    template <class... Args>
    StaticString(std::format_string<Args...> format_str, Args&&... args) {
        const auto result = std::format_to_n(
            m_array.data(),
            Capacity,
            format_str,
            std::forward<Args>(args)...
        );
        m_length = std::min(Capacity, static_cast<std::size_t>(result.size));
        m_array[m_length] = '\0';
    }

    [[nodiscard]] constexpr std::size_t size() const {
        return m_length;
    }

    [[nodiscard]] constexpr std::size_t capacity() const {
        return Capacity;
    }

    [[nodiscard]] constexpr bool empty() const {
        return m_length == 0;
    }

    [[nodiscard]] constexpr const char* data() const {
        return m_array.data();
    }

    [[nodiscard]] std::string_view view() const {
        return {m_array.data(), m_length};
    }

    [[nodiscard]] constexpr const char* c_str() const {
        return m_array.data();
    }

    // 可能在编译期计算
    template <std::size_t N>
    [[nodiscard]] constexpr bool operator==(const StaticString<N>& other) const {
        return std::ranges::equal(data(), data() + size(), other.data(), other.data() + other.size());
    }

    void clear() {
        m_length = 0;
        m_array[0] = '\0';
    }

    template <std::size_t N>
    bool try_append(const StaticString<N>& other) {
        const std::size_t count = other.size();
        if (count == 0) { return false; }
        if (count > Capacity - m_length) { return false; }

        std::char_traits<char>::copy(m_array.data() + m_length, other.data(), count);

        m_length += count;
        m_array[m_length] = '\0';
        return true;
    }

    bool try_push_back(char ch) {
        if (m_length == Capacity) {
            return false;
        }
        m_array[m_length++] = ch;
        m_array[m_length] = '\0';
        return true;
    }
};

template <std::size_t N>
StaticString(const char (&)[N]) -> StaticString<N - 1>;
