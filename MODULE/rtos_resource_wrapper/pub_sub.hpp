#pragma once

#include "asserter.hpp"
#include "rtos_queue.hpp"
#include "static_list.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace rtos {

template <typename T> class Publisher;

namespace detail {

template <typename T>
class SubscriberBase {
    friend class Publisher<T>;

private:
    inline static StaticList<SubscriberBase<T>> m_subscribers{};

    std::string_view m_topic_id;
    StaticListNode<SubscriberBase<T>> m_node{*this};

    virtual bool put_message(const T& message) = 0;

protected:
    explicit SubscriberBase(std::string_view topic_id)
        : m_topic_id(topic_id) {
        RUNTIME_ASSERT(!topic_id.empty());
    }

    // 仅由具体订阅者析构，禁止通过基类指针删除。
    ~SubscriberBase() = default;

    void register_self() {
        if (m_topic_id.empty()) {
            return;
        }
        const bool registered = m_subscribers.push_front(&m_node);
        RUNTIME_ASSERT(registered);
    }

public:
    SubscriberBase(const SubscriberBase&) = delete;
    SubscriberBase& operator=(const SubscriberBase&) = delete;
    SubscriberBase(SubscriberBase&&) = delete;
    SubscriberBase& operator=(SubscriberBase&&) = delete;

    [[nodiscard]] std::string_view topic_id() const {
        return m_topic_id;
    }
};

} // namespace detail

/**
 * @brief 接收指定类型和主题的消息。
 * @tparam T 消息类型，须平凡可拷贝且可默认构造。
 * @tparam QueueSize 接收队列容量，须大于 0。
 * @attention 所有订阅者须在 QueueManager::init() 前构造，并在通信期间持续存活；收发前须完成初始化。
 */
template <typename T, std::size_t QueueSize>
class Subscriber final : public detail::SubscriberBase<T> {
private:
    Queue<T, QueueSize> m_message_queue{};

    bool put_message(const T& message) override {
        return m_message_queue.put(message, 0);
    }

public:
    /**
     * @brief 创建订阅者。
     * @param topic_id 非空主题名，字符串须在对象存活期间有效且内容不变。
     */
    explicit Subscriber(std::string_view topic_id)
        : detail::SubscriberBase<T>(topic_id) {
        // 队列等成员全部构造完成后才将自身加入注册表。
        this->register_self();
    }

    /**
     * @brief 接收一条消息。
     * @param timeout_ms 等待毫秒数；0 表示不等待，rtos::wait_forever 表示永久等待，中断中须为 0。
     * @return 收到的消息；未收到时返回 std::nullopt。
     */
    [[nodiscard]] std::optional<T> receive(std::uint32_t timeout_ms = 0) {
        return m_message_queue.get(timeout_ms);
    }
};

/**
 * @brief 向相同类型和主题的订阅者发布消息。
 * @tparam T 消息类型。
 */
template <typename T>
class Publisher {
private:
    std::string_view m_topic_id;

public:
    /**
     * @brief 创建发布者。
     * @param topic_id 非空主题名，字符串须在对象存活期间有效且内容不变。
     */
    explicit Publisher(std::string_view topic_id)
        : m_topic_id(topic_id) {
        RUNTIME_ASSERT(!topic_id.empty());
    }

    [[nodiscard]] std::string_view topic_id() const {
        return m_topic_id;
    }

    /**
     * @brief 非阻塞发布消息，跳过已满的接收队列。
     * @param message 要发布的消息。
     * @return 成功投递的订阅者数量。
     * @attention 可从任务或中断调用，中断优先级须满足 FreeRTOS 的 FromISR API 要求。
     */
    std::size_t publish(const T& message) const {
        std::size_t delivered{0};
        if (m_topic_id.empty()) {
            return delivered;
        }

        detail::SubscriberBase<T>::m_subscribers.for_each([&](detail::SubscriberBase<T>& subscriber) {
            if (subscriber.topic_id() != m_topic_id) {
                return;
            }
            if (subscriber.put_message(message)) {
                delivered += 1;
            }
        });

        return delivered;
    }
};

} // namespace rtos
