/**
 * @file uart_port.hpp
 * @author Badfish (Howie41)
 */
#pragma once

#include "asserter.hpp"
#include "atomic_lock.hpp"
#include "main.h"
#include "memory_map.h"
#include "rtos_stream_buffer.hpp"
#include "static_string.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

class UartPortManager;
class UartPort {
    friend class UartPortManager;

public:
    // 根据官方文档，ReceiveToIdleEvent 是 ReceiveToIdle 专用的 API
    // 在 DMA Circular 模式下，ReceiveToIdle_DMA 不会因为 IDLE 事件而停止接收
    enum class ReceiveToIdleEvent : std::uint8_t {
        TransferComplete  = 0U,  // 对应 TC 事件，本轮传输完成
        HalfTransfer      = 1U,  // 对应 HT 事件，本轮传输过半
        IdleLine          = 2U  // 对应 IDLE 事件，串口接收线进入空闲状态
    };

    struct Callbacks {
        void (*on_transmit_complete)(UartPort&){nullptr};
        void (*on_receive_complete)(UartPort&){nullptr};
        /**
        * 对应 start_receive_to_idle_dma
        */
        void (*on_receive_to_idle_event)(
            UartPort&,
            ReceiveToIdleEvent,
            std::uint16_t
        ){nullptr};

        /**
        * 对应 start_receive_to_idle_dma
        *
        * - 有新的数据时就会触发
        * - 一次接收事件可能触发多次
        */
        void (*on_receive_to_idle_chunk)(UartPort&, std::span<const std::uint8_t>){nullptr};

        using ErrorCode = std::uint32_t;
        void (*on_error)(UartPort&, ErrorCode){nullptr};
    };

protected:
    UART_HandleTypeDef& m_uart_handle;
    Callbacks m_callbacks;

    AtomicLock m_tx_lock;

    virtual void handle_receive_to_idle_chunk(std::uint16_t size) {};

public:
    explicit UartPort(UART_HandleTypeDef& huart, Callbacks callbacks);
    // 因为声明顺序的问题，它的构造实现写在了末尾

    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;
    UartPort(UartPort&&) = delete;
    UartPort& operator=(UartPort&&) = delete;

    /** @brief 同步接收数据；耗时操作，不要在中断里调用 */
    HAL_StatusTypeDef receive(std::span<std::uint8_t> buffer, std::uint32_t timeout) {
        // TODO: 转换为 uint16_t 前检查长度，拒绝超过 UINT16_MAX 的输入，避免截断。
        auto result = HAL_UART_Receive(
            &m_uart_handle, 
            static_cast<std::uint8_t*>(buffer.data()),
            static_cast<std::uint16_t>(buffer.size()),
            timeout
        );
        return result;
    }

    /** @brief 同步发送数据；耗时操作，不要在中断里调用 */
    HAL_StatusTypeDef transmit(std::span<const std::uint8_t> msg, std::uint32_t timeout) {
        // TODO: 转换为 uint16_t 前检查长度，拒绝超过 UINT16_MAX 的输入，避免截断。
        auto result = HAL_UART_Transmit(
            &m_uart_handle,
            static_cast<const std::uint8_t*>(msg.data()),
            static_cast<std::uint16_t>(msg.size()),
            timeout
        );
        return result;
    }

    /** @brief 同步终止当前 IT 或 DMA 接收；耗时操作，不要在中断里调用 */
    HAL_StatusTypeDef abort_all_receive() {
        return HAL_UART_AbortReceive(&m_uart_handle);
    }

    /** @brief 同步终止当前 IT 或 DMA 发送；耗时操作，不要在中断里调用 */
    HAL_StatusTypeDef abort_all_transmit() {
        const auto result = HAL_UART_AbortTransmit(&m_uart_handle);
        if (result == HAL_OK) m_tx_lock.release();
        return result;
    }
};


template <
    std::uint16_t TxBufferSize = 0,
    std::uint16_t RxBufferSize = 0
>
class UartPortIt : public UartPort {
public:
    std::array<std::uint8_t, TxBufferSize> m_tx_buffer;
    std::array<std::uint8_t, RxBufferSize> m_rx_buffer;

    explicit UartPortIt(
        UART_HandleTypeDef& huart, 
        Callbacks callbacks
    ) : UartPort(huart, callbacks) {}

    HAL_StatusTypeDef start_receive_it(std::uint16_t size) {
        if (size > RxBufferSize) return HAL_ERROR;

        auto result = HAL_UART_Receive_IT(
            &m_uart_handle, 
            m_rx_buffer.data(), 
            size
        );
        return result;
    }

    HAL_StatusTypeDef start_transmit_it(std::span<const std::uint8_t> msg) {
        if (msg.size() > TxBufferSize) return HAL_ERROR;

        if (!m_tx_lock.try_acquire()) return HAL_BUSY;

        std::ranges::copy(msg, m_tx_buffer.begin());

        auto result = HAL_UART_Transmit_IT(
            &m_uart_handle, 
            static_cast<const std::uint8_t*>(m_tx_buffer.data()),
            static_cast<std::uint16_t>(msg.size())
        );
        if (result != HAL_OK) m_tx_lock.release();
        return result;
    }
};


/**
 * @brief DMA 串口
 *
 * @tparam UniqueId 唯一标识符，建议写 "uart4" "uart5" 之类的，不同实例间的id保持唯一，
 *         防止多个实例共用同一缓冲区
 * @tparam TxBufferSize 发送缓冲区大小
 * @tparam RxBufferSize 接收缓冲区大小
 * @tparam RxStreamBufferSize 接收流缓冲区容量，单位字节，把接收到的数据放到流缓冲区里，
 *         方便在 RTOS 任务里读取；
 *         设置为 0 时不启用，此时对应流缓冲区不会被创建
 *
 * @attention 接收流缓冲区涉及 RTOS API，应该在 RTOS 内核启动后再开始接收
 */
template <
    StaticString UniqueId,
    std::uint16_t TxBufferSize = 0,
    std::uint16_t RxBufferSize = 0,
    std::uint16_t RxStreamBufferSize = 0
>
class UartPortDma : public UartPort {
public:
    DMA_BUFFER_BSS static inline std::array<std::uint8_t, TxBufferSize> m_tx_buffer;
    DMA_BUFFER_BSS static inline std::array<std::uint8_t, RxBufferSize> m_rx_buffer;

    explicit UartPortDma(
        UART_HandleTypeDef& huart, 
        Callbacks callbacks
    ) : UartPort(huart, callbacks) {}

    HAL_StatusTypeDef start_receive_to_idle_dma(){
        RUNTIME_ASSERT(RxBufferSize > 0);
        RUNTIME_ASSERT(m_uart_handle.hdmarx != nullptr);
        // 接收 DMA 应该设置为 CIRCULAR 模式
        RUNTIME_ASSERT(m_uart_handle.hdmarx->Init.Mode == DMA_CIRCULAR);

        // 已运行的环形接收不能重置读取位置
        if (m_uart_handle.RxState != HAL_UART_STATE_READY) return HAL_BUSY;
        m_last_rx_buffer_pos = 0;

        auto result = HAL_UARTEx_ReceiveToIdle_DMA(
            &m_uart_handle,
            static_cast<std::uint8_t*>(m_rx_buffer.data()),
            static_cast<std::uint16_t>(m_rx_buffer.size())
        );
        return result;
    }

    HAL_StatusTypeDef start_transmit_dma(std::span<const std::uint8_t> msg){
        RUNTIME_ASSERT(TxBufferSize > 0);
        RUNTIME_ASSERT(m_uart_handle.hdmatx != nullptr);
        // 发送 DMA 应该设置为 NORMAL 模式
        RUNTIME_ASSERT(m_uart_handle.hdmatx->Init.Mode == DMA_NORMAL);

        if (msg.size() > TxBufferSize) return HAL_ERROR;

        if (!m_tx_lock.try_acquire()) return HAL_BUSY;

        std::ranges::copy(msg, m_tx_buffer.begin());

        auto result = HAL_UART_Transmit_DMA(
            &m_uart_handle, 
            static_cast<const std::uint8_t*>(m_tx_buffer.data()),
            static_cast<std::uint16_t>(msg.size())
        );
        if (result != HAL_OK) m_tx_lock.release();
        return result;
    }

public:
    // RxStreamBufferSize == 0 时不创建流缓冲区
    struct RxStreamBufferDisabled {}; // 没有开启缓冲区时的占位类型

    // 只要满足条件，RxStreamBuffer 的类型就是 rtos::StreamBuffer<>
    // 如果条件不满足，它的类型就是上面声明的无用展位类型
    // 这样做能够在编译期决定 m_rx_stream_buffer 的类型
    using RxStreamBuffer = std::conditional_t<
        (RxStreamBufferSize > 0),
        rtos::StreamBuffer<RxStreamBufferSize>,
        RxStreamBufferDisabled
    >;
    using IsRxStreamBufferEnabled = std::bool_constant<(RxStreamBufferSize > 0)>;

    RAM_D1_BSS static inline RxStreamBuffer m_rx_stream_buffer;
    std::atomic<std::size_t> m_rx_stream_dropped_bytes{0};

    /**
     * @brief 阻塞等待数据块，直到有数据可读
     *
     * @param output_buffer 输出缓冲区
     * @return std::size_t 本次接收的字节数
     *
     * @attention 如果没有启用接收流缓冲区，这个函数会直接返回 0
     */
    std::size_t block_for_received_chunk(std::span<std::uint8_t> output_buffer) {
        if constexpr (IsRxStreamBufferEnabled::value) {
            // tip: 下面这行代码是能通过编译，且功能正常的
            // 但是编辑的时候，语法分析服务器 clangd 会崩溃，也许是 clangd 的 bug
            // 不会影响烧录，也不会影响板子运行
            return m_rx_stream_buffer.receive_to(output_buffer, rtos::wait_forever);
        } else {
            return 0;
        }
    }

protected:
    std::uint16_t m_last_rx_buffer_pos{0};

    // 处理 receive_to_idle 事件产生的新增数据块
    // 这里的 size 实际上表示 DMA 写到了缓冲区的哪个位置
    void handle_receive_to_idle_chunk(std::uint16_t size) override {
        RUNTIME_ASSERT(size > 0 && size <= RxBufferSize);

        const auto new_pos = size;
        const auto last_pos = m_last_rx_buffer_pos;
        const auto callback = m_callbacks.on_receive_to_idle_chunk;

        if (new_pos == last_pos) return;
        m_last_rx_buffer_pos = new_pos;

        // 只复制实际收到的字节，空间不足时保留已写入的前缀
        const auto send_chunk = [&](std::span<const std::uint8_t> chunk_span) {
            if constexpr (IsRxStreamBufferEnabled::value) {
                const auto sent = m_rx_stream_buffer.send(chunk_span);
                if (sent < chunk_span.size()) {
                    m_rx_stream_dropped_bytes.fetch_add(
                        chunk_span.size() - sent,
                        std::memory_order_relaxed
                    );
                }
            }
        };

        if (new_pos > last_pos) {
            // 数据没有回绕，正常处理
            auto chunk_span = std::span<const std::uint8_t>(
                m_rx_buffer.begin() + last_pos,
                new_pos - last_pos
            );
            send_chunk(chunk_span);
            if (callback) { callback(*this, chunk_span); }
        } else {
            // 发生了回绕，分成头尾两段
            if (last_pos < RxBufferSize) {
                auto tail_chunk_span = std::span<const std::uint8_t>(
                    m_rx_buffer.begin() + last_pos,
                    m_rx_buffer.size() - last_pos
                );
                send_chunk(tail_chunk_span);
                if (callback) { callback(*this, tail_chunk_span); }
            }
            if (new_pos > 0) {
                auto head_chunk_span = std::span<const std::uint8_t>(
                    m_rx_buffer.begin(),
                    new_pos
                );
                send_chunk(head_chunk_span);
                if (callback) { callback(*this, head_chunk_span); }
            }
        }
    }
};

class UartPortManager {
    friend class UartPort;

private:
    UartPortManager() = delete;
    
    inline static constexpr std::uint16_t MAX_UART_PORT = 12;
    inline static std::array<UartPort*, MAX_UART_PORT> m_port_ptr_list{};

    inline static bool register_port(UartPort& port) {
        // 寻找列表里有没有已绑定同一 huart 的实例
        auto already_exist = std::ranges::any_of(
            m_port_ptr_list,
            [&](UartPort* p) -> bool {
                return (p != nullptr) &&
                (&(port.m_uart_handle) == &(p->m_uart_handle));
            }
        );
        if (already_exist) return false;
        // 找空位
        auto empty_slot = std::ranges::find_if(
            m_port_ptr_list,
            [&](UartPort* slot) -> bool {
                return (slot == nullptr);
            }
        );
        if (empty_slot == m_port_ptr_list.end()) return false;

        *empty_slot = &port; // 把传入的port指针放到列表里
        return true;
    }

    inline static UartPort* find_uart_port_by_huart(UART_HandleTypeDef* huart) {
        auto port = std::ranges::find_if(
            m_port_ptr_list,
            [&](UartPort* p) -> bool {
                return (p != nullptr) &&
                (huart == &(p->m_uart_handle));
            }
        );
        if (port == m_port_ptr_list.end()) {
            return nullptr;
        } else {
            return *port;
        }
    }

public:
    /** 对接 HAL 的回调函数 */
    inline static void hal_on_tx_complete(UART_HandleTypeDef* huart) {
        auto port = find_uart_port_by_huart(huart);
        if (port == nullptr) return;

        port->m_tx_lock.release(); // 解除忙碌状态

        auto callback = port->m_callbacks.on_transmit_complete;
        if (callback == nullptr) return;
        callback(*port);
    }

    inline static void hal_on_rx_complete(UART_HandleTypeDef* huart) {
        auto port = find_uart_port_by_huart(huart);
        if (port == nullptr) return;

        auto callback = port->m_callbacks.on_receive_complete;
        if (callback == nullptr) return;
        callback(*port);
    }

    inline static void hal_on_rx_event(UART_HandleTypeDef* huart, HAL_UART_RxEventTypeTypeDef event, std::uint16_t size) {
        auto port = find_uart_port_by_huart(huart);
        if (port == nullptr) return;
        port->handle_receive_to_idle_chunk(size);

        auto callback = port->m_callbacks.on_receive_to_idle_event;
        if (callback == nullptr) return;
        callback(*port, static_cast<UartPort::ReceiveToIdleEvent>(event), size);
    }

    inline static void hal_on_error(UART_HandleTypeDef* huart, std::uint32_t error) {
        // TODO: TX DMA 出错并结束传输后，应释放软件发送忙标志，允许再次发送。
        // 需区分 RX 专属错误，避免错误解除仍在进行的 TX 传输。
        auto port = find_uart_port_by_huart(huart);
        if (port == nullptr) return;

        auto callback = port->m_callbacks.on_error;
        if (callback == nullptr) return;
        callback(*port, error);
    }
};

inline UartPort::UartPort(
    UART_HandleTypeDef& huart, 
    Callbacks callbacks
) : m_uart_handle(huart),
m_callbacks(callbacks) {
    auto registered = UartPortManager::register_port(*this);
    RUNTIME_ASSERT(registered); // 这里断言失败说明注册失败了
}
