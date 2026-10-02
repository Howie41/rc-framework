/**
 * @file uart_port.hpp
 * @author Badfish (Howie41)
 */
#pragma once

#include "asserter.hpp"
#include "main.h"
#include "memory_map.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
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
    using ErrorCode = std::uint32_t;

    void (*on_transmit_complete)(UartPort&){nullptr};

    /**
     * 对应 receive / receive_it / receive_dma
     */
    void (*on_receive_complete)(UartPort&){nullptr};

    /**
     * 对应 start_receive_to_idle_it /
     * start_receive_to_idle_dma
     *
     * 每个 HAL ReceiveToIdleEvent 只调用一次
     */
    void (*on_receive_to_idle_event)(
        UartPort&,
        ReceiveToIdleEvent,
        std::uint16_t
    ){nullptr};

    /**
     * 对应 start_receive_to_idle_it /
     * start_receive_to_idle_dma
     *
     * - 有新的数据时就会触发
     * - 一次接收事件可能触发多次
     */
    void (*on_receive_to_idle_chunk)(
        UartPort&,
        std::span<const std::uint8_t>
    ){nullptr};

    void (*on_error)(
        UartPort&,
        ErrorCode
    ){nullptr};
};

protected:
    UART_HandleTypeDef& m_uart_handle;
    Callbacks m_callbacks;

    std::atomic<bool> m_is_tx_busy{false};
    bool try_acquire_tx() {
        bool expected = false;

        return m_is_tx_busy.compare_exchange_strong(
            expected,
            true,
            std::memory_order_acquire,
            std::memory_order_relaxed
        );
    }
    void release_tx() {
        m_is_tx_busy.store(false, std::memory_order_release);
    }

    virtual void handle_receive_to_idle_chunk(std::uint16_t size) {};

public:
    explicit UartPort(UART_HandleTypeDef& huart, Callbacks callbacks);
    // 因为声明顺序的问题，它的构造实现写在了末尾

    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;
    UartPort(UartPort&&) = delete;
    UartPort& operator=(UartPort&&) = delete;

    HAL_StatusTypeDef receive(std::span<std::uint8_t> buffer, std::uint32_t timeout) {
        auto result = HAL_UART_Receive(
            &m_uart_handle, 
            static_cast<std::uint8_t*>(buffer.data()),
            static_cast<std::uint16_t>(buffer.size()),
            timeout
        );
        return result;
    }

    HAL_StatusTypeDef transmit(std::span<const std::uint8_t> msg, std::uint32_t timeout) {
        auto result = HAL_UART_Transmit(
            &m_uart_handle,
            static_cast<const std::uint8_t*>(msg.data()),
            static_cast<std::uint16_t>(msg.size()),
            timeout
        );
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

    HAL_StatusTypeDef start_receive_to_idle_it(std::uint16_t size) {
        if (size > RxBufferSize) return HAL_ERROR;

        auto result = HAL_UARTEx_ReceiveToIdle_IT(
            &m_uart_handle, 
            m_rx_buffer.data(), 
            size
        );
        return result;
    }

    HAL_StatusTypeDef start_transmit_it(std::span<const std::uint8_t> msg) {
        if (msg.size() > TxBufferSize) return HAL_ERROR;

        if (!try_acquire_tx()) return HAL_BUSY;

        std::ranges::copy(msg, m_tx_buffer.begin());

        auto result = HAL_UART_Transmit_IT(
            &m_uart_handle, 
            static_cast<const std::uint8_t*>(m_tx_buffer.data()),
            static_cast<std::uint16_t>(msg.size())
        );
        if (result != HAL_OK) release_tx();
        return result;
    }
};


/**
 * @brief DMA 串口
 * @attention 实例化时必须在前面加上 DMA_BUFFER_BSS 或 DMA_BUFFER_DATA 宏，否则DMA访问不到缓冲区
 * @tparam TxBufferSize 发送缓冲区大小
 * @tparam RxBufferSize 接收缓冲区大小
 */
template <
    std::uint16_t TxBufferSize = 0,
    std::uint16_t RxBufferSize = 0
>
class UartPortDma : public UartPort {
    static bool buffer_in_dma_ram(const std::uint8_t* buffer, std::size_t size) {
        // 与 STM32H723XG_FLASH.ld 中 RAM_D2 的地址和长度保持一致。
        constexpr std::uintptr_t dma_ram_begin = 0x30000000U;
        constexpr std::uintptr_t dma_ram_end = 0x30008000U;
        const auto address = reinterpret_cast<std::uintptr_t>(buffer);
        return address >= dma_ram_begin && address < dma_ram_end &&
               size <= dma_ram_end - address;
    }

public:
    std::array<std::uint8_t, TxBufferSize> m_tx_buffer;
    std::array<std::uint8_t, RxBufferSize> m_rx_buffer;

    explicit UartPortDma(
        UART_HandleTypeDef& huart, 
        Callbacks callbacks
    ) : UartPort(huart, callbacks) {}

    HAL_StatusTypeDef start_receive_to_idle_dma(){
        RUNTIME_ASSERT(RxBufferSize > 0);
        RUNTIME_ASSERT(m_uart_handle.hdmarx != nullptr);
        // 接收 DMA 应该设置为 CIRCULAR 模式
        RUNTIME_ASSERT(m_uart_handle.hdmarx->Init.Mode == DMA_CIRCULAR);
        RUNTIME_ASSERT(buffer_in_dma_ram(m_rx_buffer.data(), m_rx_buffer.size()));

        // 已运行的环形接收不能重置读取位置。
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
        RUNTIME_ASSERT(buffer_in_dma_ram(m_tx_buffer.data(), m_tx_buffer.size()));

        if (msg.size() > TxBufferSize) return HAL_ERROR;

        if (!try_acquire_tx()) return HAL_BUSY;

        std::ranges::copy(msg, m_tx_buffer.begin());

        auto result = HAL_UART_Transmit_DMA(
            &m_uart_handle, 
            static_cast<const std::uint8_t*>(m_tx_buffer.data()),
            static_cast<std::uint16_t>(msg.size())
        );
        if (result != HAL_OK) release_tx();
        return result;
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
        if (callback == nullptr) return;

        if (new_pos > last_pos) {
            auto chunk = std::span<const std::uint8_t>(
                m_rx_buffer.begin() + last_pos,
                new_pos - last_pos
            );
            callback(*this, chunk);
        } else {
            // 发生了回绕
            if (last_pos < RxBufferSize) {
                auto tail_chunk = std::span<const std::uint8_t>(
                    m_rx_buffer.begin() + last_pos,
                    m_rx_buffer.size() - last_pos
                );
                callback(*this, tail_chunk);
            }
            if (new_pos > 0) {
                auto head_chunk = std::span<const std::uint8_t>(
                    m_rx_buffer.begin(),
                    new_pos
                );
                callback(*this, head_chunk);
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
    inline static void hal_on_tx_complete(UART_HandleTypeDef* huart) {
        auto port = find_uart_port_by_huart(huart);
        if (port == nullptr) return;

        port->release_tx(); // 解除忙碌状态

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
