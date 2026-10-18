#pragma once

#include "main.h"
#include "FreeRTOS.h"
#include "asserter.hpp"
#include "queue.h"
#include "rtos_utility.hpp"
#include "static_list.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

namespace rtos {

class QueueBase {
    friend class QueueManager;

private:
    StaticListNode<QueueBase> m_node{*this};

protected:
    QueueBase() = default;
    virtual ~QueueBase() = default;
    /**
     * @brief 在 RTOS 内部初始化队列
     * @return 初始化成功返回 `true`，否则返回 `false`
     */
    virtual bool init() = 0;
};

class QueueManager {
    template <typename T, size_t QueueSize>
    friend class Queue;

private:
    inline static StaticList<QueueBase> m_queues;
    inline static bool m_initialized{false};
    inline static size_t m_count{0};

    QueueManager() = delete;

    /**
     * @brief 注册待初始化的队列
     * @param queue 要注册的队列对象指针
     * @attention 必须在管理器初始化之前调用
     */
    static void register_queue(QueueBase* queue) {
        RUNTIME_ASSERT(!m_initialized && queue != nullptr);
        if (m_initialized || queue == nullptr) {
            return;
        }
        const bool registered = m_queues.push_front(&queue->m_node);
        RUNTIME_ASSERT(registered);
    }

public:
    /** @brief 创建所有已注册的队列，重复调用不会再次创建 */
    static void init() {
        if (m_initialized) {
            return;
        }

        m_initialized = true;
        m_queues.for_each([](QueueBase& queue) {
            if (queue.init()) {
                ++m_count;
            }
        });
    }

    /**
     * @brief 获取成功创建的队列数量
     * @return 成功创建的队列数量
     */
    static size_t get_count() {
        return m_count;
    }
};



/**
 * @brief 队列
 * @tparam T 队列元素类型，必须是平凡可拷贝类型，且有默认构造函数
 * @tparam QueueSize 队列大小，必须是正数
 */
template <typename T, size_t QueueSize>
class Queue : QueueBase {
    static_assert(std::is_trivially_copyable_v<T>, "queue elements must be trivially copyable");
    static_assert(std::is_default_constructible_v<T>, "queue elements must be default constructible");
    static_assert(QueueSize > 0, "wrong queue size");

private:
    StaticQueue_t m_control_block{};
    uint8_t m_queue_storage[QueueSize * sizeof(T)];
    QueueHandle_t m_queue_handle{nullptr};

    /**
     * @brief 在 RTOS 内部创建队列，已创建时直接返回
     * @return 句柄有效时返回 `true`，否则返回 `false`
     * @attention 必须在非中断上下文中调用
     */
    bool init() override {
        if (m_queue_handle != nullptr) {
            return true;
        }

        RUNTIME_ASSERT(!is_irq_context());
        m_queue_handle = xQueueCreateStatic(
            static_cast<UBaseType_t>(QueueSize),
            static_cast<UBaseType_t>(sizeof(T)),
            m_queue_storage,
            &m_control_block);

        return m_queue_handle != nullptr;
    }

public:
    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;

    ~Queue() = default;

    /**
     * @brief 构造队列包装对象并注册到管理器
     * @note RTOS 队列由管理器初始化时创建
     */
    explicit Queue() {
        QueueManager::register_queue(this);
    }

    /**
     * @brief 获取队列句柄
     * @return 队列句柄
     */
    QueueHandle_t handle() const {
        return m_queue_handle;
    }

    /**
     * @brief 向队列里放入元素
     * @attention RTOS内部会复制元素字节，元素必须平凡可拷贝
     * @attention 中断上下文中 `timeout_ms` 必须为 0
     * @param item_ref 要放入的元素的引用
     * @param timeout_ms 超时时间，单位毫秒；rtos::wait_forever 表示永久等待
     * @return 在指定时间内成功放入返回 `true`，否则返回 `false`
    */
    bool put(const T& item_ref, uint32_t timeout_ms = 0) {
        RUNTIME_ASSERT(m_queue_handle != nullptr);

        BaseType_t status;

        if (is_irq_context()) {
            RUNTIME_ASSERT(timeout_ms == 0);
            BaseType_t higher_priority_task_woken = pdFALSE;
            status = xQueueSendToBackFromISR(
                m_queue_handle,
                &item_ref,
                &higher_priority_task_woken
            );
            portYIELD_FROM_ISR(higher_priority_task_woken);
        } else {
            status = xQueueSendToBack(
                m_queue_handle,
                &item_ref,
                timeout_ms_to_ticks(timeout_ms)
            );
        }

        return (status == pdPASS);
    }

    /**
     * @brief 覆盖队列中的元素，队列为空时直接放入
     * @attention 仅适用于长度为 1 的队列
     * @param item_ref 要写入的元素的引用
     * @return 覆盖成功返回 `true`，否则返回 `false`
     */
    bool overwrite(const T& item_ref) {
        static_assert(QueueSize == 1, "overwrite is only supported for queues of length 1");
        RUNTIME_ASSERT(m_queue_handle != nullptr);

        BaseType_t status;

        if (is_irq_context()) {
            BaseType_t higher_priority_task_woken = pdFALSE;
            status = xQueueOverwriteFromISR(
                m_queue_handle,
                &item_ref,
                &higher_priority_task_woken
            );
            portYIELD_FROM_ISR(higher_priority_task_woken);
        } else {
            status = xQueueOverwrite(m_queue_handle, &item_ref);
        }

        return (status == pdPASS);
    }

    /**
     * @brief 从队列里获取元素
     * @attention 中断上下文中 `timeout_ms` 必须为 0
     * @param timeout_ms 超时时间，单位毫秒；rtos::wait_forever 表示永久等待
     * @return 返回 `std::optional` 包裹的元素（获取失败时为空）
     * @code 简单食用方法：
     * - `auto result = queue.get();`
     * - 使用 `if (result) {...}` 判断是否获取成功
     * - 获取成功时，用 `result.value()` 获取取出的元素
    */
    [[nodiscard]] std::optional<T> get(uint32_t timeout_ms = 0) {
        RUNTIME_ASSERT(m_queue_handle != nullptr);

        T gotten_item{};
        BaseType_t status;

        if (is_irq_context()) {
            RUNTIME_ASSERT(timeout_ms == 0);
            BaseType_t higher_priority_task_woken = pdFALSE;
            status = xQueueReceiveFromISR(
                m_queue_handle,
                &gotten_item,
                &higher_priority_task_woken
            );
            portYIELD_FROM_ISR(higher_priority_task_woken);
        } else {
            status = xQueueReceive(
                m_queue_handle, 
                &gotten_item, 
                timeout_ms_to_ticks(timeout_ms)
            );
        }

        if (status == pdPASS) {
            return gotten_item;
        } else {
            return std::nullopt;
        }
    }

    /**
     * @brief 查看队首元素但不将其移出队列
     * @attention 中断上下文中 `timeout_ms` 必须为 0
     * @param timeout_ms 超时时间，单位毫秒；rtos::wait_forever 表示永久等待
     * @return 查看成功时返回 `std::optional` 包裹的元素，失败时返回 `std::nullopt`
    */
    [[nodiscard]] std::optional<T> peek(uint32_t timeout_ms = 0) {
        RUNTIME_ASSERT(m_queue_handle != nullptr);

        T peeked_item{};
        BaseType_t status;

        if (is_irq_context()) {
            RUNTIME_ASSERT(timeout_ms == 0);
            status = xQueuePeekFromISR(m_queue_handle, &peeked_item);
        } else {
            status = xQueuePeek(
                m_queue_handle,
                &peeked_item,
                timeout_ms_to_ticks(timeout_ms)
            );
        }

        if (status == pdPASS) {
            return peeked_item;
        } else {
            return std::nullopt;
        }
    }
};

}  // namespace rtos
