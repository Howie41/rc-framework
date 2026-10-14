/**
 * @file static_list.hpp
 * @author Badfish
 * @brief 无动态分配的静态非侵入式单链表，仅提供头插和遍历
 * @date 2026-10-13
 *
 * 使用者持有 StaticListNode<T> 实例，节点保存 T&，T 无需继承节点
 */
#pragma once

#include <concepts>
#include <functional>

template <typename T> class StaticList;

template <typename T>
class StaticListNode {
    friend class StaticList<T>;

private:
    T& m_value;  // 节点绑定的实例
    StaticListNode* m_next_node{nullptr};
    bool m_registered{false};  // 标记节点，防止重复注册

public:
    explicit constexpr StaticListNode(T& value) noexcept : m_value(value) {}
    StaticListNode(T&&) = delete;

    StaticListNode(const StaticListNode&) = delete;
    StaticListNode& operator=(const StaticListNode&) = delete;
    StaticListNode(StaticListNode&&) = delete;
    StaticListNode& operator=(StaticListNode&&) = delete;

    [[nodiscard]] T& value() noexcept {
        return m_value;
    }

    [[nodiscard]] const T& value() const noexcept {
        return m_value;
    }
};

template <typename T>
class StaticList {
private:
    StaticListNode<T>* m_head_node{nullptr};

public:
    constexpr StaticList() noexcept = default;
    StaticList(const StaticList&) = delete;
    StaticList& operator=(const StaticList&) = delete;
    StaticList(StaticList&&) = delete;
    StaticList& operator=(StaticList&&) = delete;

    [[nodiscard]] bool push_front(StaticListNode<T>* node) noexcept {
        if (node == nullptr || node->m_registered) {
            return false;
        }
        node->m_next_node = m_head_node;
        node->m_registered = true;
        m_head_node = node;
        return true;
    }

    template <typename Func>
        requires std::invocable<Func&, T&>
    void for_each(Func&& func) {
        for (auto* node = m_head_node; node != nullptr; node = node->m_next_node) {
            std::invoke(func, node->m_value);
        }
    }

    template <typename Func>
        requires std::invocable<Func&, const T&>
    void for_each(Func&& func) const {
        for (const StaticListNode<T>* node = m_head_node; node != nullptr;
             node = node->m_next_node) {
            std::invoke(func, static_cast<const T&>(node->m_value));
        }
    }
};
