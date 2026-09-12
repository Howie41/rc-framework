#pragma once

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <type_traits>

class RtosQueueBase {
    friend class RtosQueueManager;

private:
    RtosQueueBase* m_next{nullptr};

protected:
    RtosQueueBase() = default;
    virtual ~RtosQueueBase() = default;
    virtual bool init() = 0;
};

class RtosQueueManager {
private:
    RtosQueueBase* m_head{nullptr};
    bool m_initialized{false};
    size_t m_count{0};

    RtosQueueManager() = default;

public:
    RtosQueueManager(const RtosQueueManager&) = delete;
    RtosQueueManager& operator=(const RtosQueueManager&) = delete;

    static RtosQueueManager& instance() {
        static RtosQueueManager manager;
        return manager;
    }

    void register_queue(RtosQueueBase* queue) {
        configASSERT(!m_initialized);
        queue->m_next = m_head;
        m_head = queue;
    }

    void create_all() {
        if (m_initialized) {
            return;
        }

        m_initialized = true;
        for (RtosQueueBase* queue = m_head; queue != nullptr; queue = queue->m_next) {
            if (queue->init()) {
                ++m_count;
            }
        }
    }

    size_t get_count() const {
        return m_count;
    }
};

/**
 * @note Instances register their addresses with RtosQueueManager and therefore
 *       must have static storage duration.
 */
template <typename T, size_t QueueSize>
class RtosQueue : RtosQueueBase {
    static_assert(std::is_trivially_copyable_v<T>,
                  "queue elements must be trivially copyable");
    static_assert(std::is_default_constructible_v<T>,
                  "queue elements must be default constructible");
    static_assert(QueueSize > 0, "queue size must be > 0");
private:
    const char* m_name{nullptr};
    StaticQueue_t m_control_block{};
    uint8_t m_queue_storage[QueueSize * sizeof(T)];
    osMessageQueueId_t m_queue_handle{nullptr};

public:
    RtosQueue(const char* name) : m_name(name) {
        RtosQueueManager::instance().register_queue(this);
    }

private:
    bool init() override {
        if (m_queue_handle != nullptr) {
            return true;
        }

        osMessageQueueAttr_t attributes{};
        attributes.name = m_name;
        attributes.cb_mem = &m_control_block;
        attributes.cb_size = sizeof(m_control_block);
        attributes.mq_mem = m_queue_storage;
        attributes.mq_size = sizeof(m_queue_storage);
        m_queue_handle = osMessageQueueNew(static_cast<uint32_t>(QueueSize),
                                    static_cast<uint32_t>(sizeof(T)),
                                    &attributes);
        return m_queue_handle != nullptr;
    }

public:
    // 删除拷贝构造
    RtosQueue(const RtosQueue&) = delete;
    RtosQueue& operator=(const RtosQueue&) = delete;

    ~RtosQueue() = default;

    /**
     * @brief 向队列里放入元素
     * @attention RTOS内部会复制元素字节，元素必须平凡可拷贝
     * @param item_ref 要放入的元素的引用
     * @param timeout_ms 超时时间，单位毫秒
     * @return 状态码，`osOK` 表示成功
     */
    osStatus_t put(const T& item_ref, uint32_t timeout_ms = 0) {
        return osMessageQueuePut(m_queue_handle, &item_ref, 0, timeout_ms);
    }

    /**
     * @brief 从队列里获取元素
     * @param timeout_ms 超时时间，单位毫秒
     * @return 获取成功时返回元素，否则返回 `osStatus_t` 状态码
     * @code 食用方法：
     * - `auto result = queue.get();`
     * - 使用 `if (result) {...}` 判断是否获取成功
     * - 获取成功时，用 `result.value()` 获取取出的元素
     * - 获取失败时，用 `result.error()` 得到 `osStatus_t` 状态码
     */
    [[nodiscard]] std::expected<T, osStatus_t> get(uint32_t timeout_ms = 0) {
        T gotten_item{};
        const osStatus_t status = osMessageQueueGet(m_queue_handle, &gotten_item, nullptr, timeout_ms);

        if (status == osOK) {
            return gotten_item;
        } else {
            return std::unexpected(status);
        }
    }
};
