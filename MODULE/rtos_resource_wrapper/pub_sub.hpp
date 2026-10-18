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

/**
 * @brief 同一消息类型的订阅者注册表，与各订阅者的队列容量无关
 * @attention 仅在启动阶段注册，开始发布前必须完成全部注册。
 *            注册后不支持注销，订阅者必须持续存活，不得销毁或移动。
 */
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
 * @brief 按 (T, topic_id) 订阅消息，每个实例持有独立的静态队列
 * @attention 推荐定义为全局或 static 对象，并在 QueueManager::init() 前构造。
 *            收发消息前必须完成 QueueManager::init()。
 *            ID 必须非空；仅保存 string_view，不复制字符串内容。
 *            字符串存储必须在订阅者使用期间有效且内容不变，推荐使用字符串字面量。
 */
template <typename T, std::size_t QueueSize>
class Subscriber final : public detail::SubscriberBase<T> {
private:
    Queue<T, QueueSize> m_message_queue{};

    bool put_message(const T& message) override {
        return m_message_queue.put(message, 0);
    }

public:
    explicit Subscriber(std::string_view topic_id)
        : detail::SubscriberBase<T>(topic_id) {
        // 队列等成员全部构造完成后才将自身加入注册表。
        this->register_self();
    }

    /** @brief 从本订阅者队列取出消息，超时或队列为空时返回 nullopt */
    [[nodiscard]] std::optional<T> receive(std::uint32_t timeout_ms = 0) {
        return m_message_queue.get(timeout_ms);
    }
};

/**
 * @brief 无 Topic 实例的发布者，遍历同一 T 的订阅者并按 ID 投递
 * @note 同一 ID 使用不同 T 时互不通信，不做全局 ID 类型冲突检查。
 *       不缓存消息，没有订阅者时消息直接丢弃。
 * @attention ID 必须非空；仅保存 string_view，字符串存储必须在发布者使用期间
 *            有效且内容不变，推荐使用字符串字面量。
 */
template <typename T>
class Publisher {
private:
    std::string_view m_topic_id;

public:
    explicit Publisher(std::string_view topic_id)
        : m_topic_id(topic_id) {
        RUNTIME_ASSERT(!topic_id.empty());
    }

    [[nodiscard]] std::string_view topic_id() const {
        return m_topic_id;
    }

    /**
     * @brief 非阻塞地向所有匹配订阅者投递；队列满时继续投递其他订阅者
     * @return 成功入队的订阅者数量；无匹配或全部投递失败时为 0
     * @note 耗时为 O(N)，N 为 T 对应的订阅者数量。可从任务或中断调用，
     *       中断优先级须满足 FreeRTOS 的 FromISR API 要求。
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
