#pragma once

#include "main.h"
#include "FreeRTOS.h"
#include "asserter.hpp"
#include "stream_buffer.h"
#include "rtos_utility.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos {

class StreamBufferBase {
    friend class StreamBufferManager;

private:
    StreamBufferBase* m_next{nullptr};

protected:
    StreamBufferBase() = default;
    virtual ~StreamBufferBase() = default;
    /**
     * @brief 在 RTOS 内部初始化流缓冲区
     * @return 初始化成功返回 `true`，否则返回 `false`
     */
    virtual bool init() = 0;
};

class StreamBufferManager {
    template <size_t BufferSize, size_t TriggerLevel>
    friend class StreamBuffer;

private:
    inline static StreamBufferBase* m_head{nullptr};
    inline static bool m_initialized{false};
    inline static size_t m_count{0};

    StreamBufferManager() = delete;

    /**
     * @brief 注册待初始化的流缓冲区
     * @param buffer 要注册的流缓冲区对象指针
     * @attention 必须在管理器初始化之前调用
     */
    static void register_stream_buffer(StreamBufferBase* buffer) {
        RUNTIME_ASSERT(!m_initialized);
        buffer->m_next = m_head;
        m_head = buffer;
    }

public:
    /**
     * @brief 创建所有已注册的流缓冲区，重复调用不会再次创建
     */
    static void init() {
        if (m_initialized) {
            return;
        }

        m_initialized = true;
        for (StreamBufferBase* buffer = m_head; buffer != nullptr; buffer = buffer->m_next) {
            if (buffer->init()) {
                ++m_count;
            }
        }
    }

    /**
     * @brief 获取成功创建的流缓冲区数量
     * @return 成功创建的流缓冲区数量
     */
    static size_t get_count() {
        return m_count;
    }
};

/**
 * @brief 静态分配的字节流缓冲区
 * @tparam BufferSize 可存储的字节数，须满足当前 FreeRTOS 静态创建函数的最小长度要求
 * @tparam TriggerLevel 唤醒等待接收任务所需的字节数，范围为 1 到 BufferSize
 * @note 非零超时的同一收发方向须遵守 FreeRTOS 的单写者、单读者约束
 * @note 零超时操作在临界区内复制数据，传输量越大，对中断延迟的影响越大
 */
template <size_t BufferSize, size_t TriggerLevel = 1>
class StreamBuffer : StreamBufferBase {
    static_assert(BufferSize >= sizeof(configMESSAGE_BUFFER_LENGTH_TYPE),
                  "stream buffer size is too small for static creation");
    static_assert(BufferSize < SIZE_MAX, "stream buffer size is too large");
    static_assert(TriggerLevel > 0 && TriggerLevel <= BufferSize, "invalid stream buffer trigger level");

private:
    StaticStreamBuffer_t m_control_block{};
    uint8_t m_storage[BufferSize + 1]{};
    StreamBufferHandle_t m_handle{nullptr};

    /**
     * @brief 在 RTOS 内部创建流缓冲区，已创建时直接返回
     * @return 句柄有效时返回 `true`，否则返回 `false`
     * @attention 必须在非中断上下文中调用
     */
    bool init() override {
        if (m_handle != nullptr) {
            return true;
        }

        RUNTIME_ASSERT(!is_irq_context());
        m_handle = xStreamBufferCreateStatic(
            BufferSize + 1,
            TriggerLevel,
            m_storage,
            &m_control_block);
        return m_handle != nullptr;
    }

public:
    /**
     * @brief 构造流缓冲区包装对象并注册到管理器
     * @note RTOS 流缓冲区由管理器初始化时创建
     */
    StreamBuffer() {
        StreamBufferManager::register_stream_buffer(this);
    }
    StreamBuffer(const StreamBuffer&) = delete;
    StreamBuffer& operator=(const StreamBuffer&) = delete;
    ~StreamBuffer() = default;

    /**
     * @brief 获取流缓冲区句柄
     * @return 流缓冲区句柄，尚未创建时为 `nullptr`
     */
    StreamBufferHandle_t handle() const {
        return m_handle;
    }

    /**
     * @brief 向流缓冲区发送字节
     * @param data 输入缓冲区，要发送的字节从这里读取
     * @param timeout_ms 最长等待时间，单位毫秒；0 表示不等待，rtos::wait_forever 表示永久等待
     * @return 实际发送的字节数，不一定读完输入缓冲区
     *
     * @attention
     * - timeout_ms 为 0 时，会使用临界区保护
     * - timeout_ms 不为 0 时，没有临界区保护，不适合多发送者的情况
     * - 中断上下文中 timeout_ms 必须为 0
     */
    size_t send(std::span<const std::uint8_t> data, uint32_t timeout_ms = 0) {
        RUNTIME_ASSERT(m_handle != nullptr);
        RUNTIME_ASSERT(!is_irq_context() || timeout_ms == 0);
        if (data.empty()) {
            return 0;
        }

        if (is_irq_context()) {
            BaseType_t higher_priority_task_woken = pdFALSE;
            const UBaseType_t saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();
            const size_t sent = xStreamBufferSendFromISR(
                m_handle, data.data(), data.size(), &higher_priority_task_woken);
            taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);
            portYIELD_FROM_ISR(higher_priority_task_woken);
            return sent;
        }

        if (timeout_ms == 0) {
            taskENTER_CRITICAL();
            const size_t sent = xStreamBufferSend(m_handle, data.data(), data.size(), 0);
            taskEXIT_CRITICAL();
            return sent;
        }

        const size_t sent = xStreamBufferSend(m_handle, data.data(), data.size(), timeout_ms_to_ticks(timeout_ms));
        return sent;
    }

    /**
     * @brief 从流缓冲区接收字节
     * @param output_buffer 输出缓冲区，接收到的字节写到这里
     * @param timeout_ms 最长等待时间，单位毫秒；0 表示不等待，rtos::wait_forever 表示永久等待
     * @return 实际接收的字节数，不一定写满输出缓冲区
     *
     * @attention
     * - timeout_ms 为 0 时，会使用临界区保护
     * - timeout_ms 不为 0 时，没有临界区保护，不适合多接收者的情况
     * - 中断上下文中 timeout_ms 必须为 0
     */
    size_t receive_to(std::span<std::uint8_t> output_buffer, uint32_t timeout_ms = 0) {
        RUNTIME_ASSERT(m_handle != nullptr);
        RUNTIME_ASSERT(!is_irq_context() || timeout_ms == 0);
        if (output_buffer.empty()) {
            return 0;
        }

        if (is_irq_context()) {
            BaseType_t higher_priority_task_woken = pdFALSE;
            const UBaseType_t saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();
            const size_t received = xStreamBufferReceiveFromISR(
                m_handle, output_buffer.data(), output_buffer.size(), &higher_priority_task_woken);
            taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);
            portYIELD_FROM_ISR(higher_priority_task_woken);
            return received;
        }

        if (timeout_ms == 0) {
            taskENTER_CRITICAL();
            const size_t received = xStreamBufferReceive(m_handle, output_buffer.data(), output_buffer.size(), 0);
            taskEXIT_CRITICAL();
            return received;
        }

        const size_t received = xStreamBufferReceive(
            m_handle, output_buffer.data(), output_buffer.size(), timeout_ms_to_ticks(timeout_ms));
        return received;
    }
};

}  // namespace rtos
