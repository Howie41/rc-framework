/**
 * @file uart_port.hpp
 * @author Badfish (Howie41)
 */
#pragma once

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

class UartPortManager;
class UartPort {
    friend class UartPortManager;

public:
    // 根据官方文档，RxEvent 是 ReceiveToIdle 专用的 API
    // 在 DMA Circular 模式下，ReceiveToIdle_DMA 不会因为 IDLE 事件而停止接收
    enum class RxEvent : std::uint8_t {
        TransferComplete  = 0U,  // 对应 TC 事件，本轮传输完成
        HalfTransfer      = 1U,  // 对应 HT 事件，本轮传输过半
        IdleLine          = 2U  // 对应 IDLE 事件，串口接收线进入空闲状态
    };

    struct Callbacks {
        using ErrorCode = std::uint32_t;

        void(*on_tx_complete)(UartPort&){nullptr};

         /** @attention 只有使用 receive 系列方法才会触发这个回调 */
        void(*on_rx_complete)(UartPort&){nullptr};

        /** @attention 只有使用 start_receive_to_idle 系列方法才会触发这个回调 */
        void(*on_rx_event)(UartPort&, RxEvent, std::uint16_t){nullptr};

        void(*on_error)(UartPort&, ErrorCode){nullptr};
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


template <
    DMA_HandleTypeDef* DmaTxHandle = nullptr,
    std::uint16_t TxBufferSize = 0,
    DMA_HandleTypeDef* DmaRxHandle = nullptr,
    std::uint16_t RxBufferSize = 0
>
class UartPortDma : public UartPort {
    static_assert((DmaTxHandle == nullptr && TxBufferSize == 0) || 
                  (DmaTxHandle != nullptr && TxBufferSize > 0), "wrong tx config");
    static_assert((DmaRxHandle == nullptr && RxBufferSize == 0) || 
                  (DmaRxHandle != nullptr && RxBufferSize > 0), "wrong rx config");

public:
    inline static DMA_BUFFER std::array<std::uint8_t, TxBufferSize> m_tx_buffer;
    inline static DMA_BUFFER std::array<std::uint8_t, RxBufferSize> m_rx_buffer;

    explicit UartPortDma(
        UART_HandleTypeDef& huart, 
        Callbacks callbacks
    ) : UartPort(huart, callbacks) {}

    HAL_StatusTypeDef start_receive_to_idle_dma()
        requires (DmaRxHandle != nullptr && RxBufferSize > 0)
    {
        auto result = HAL_UARTEx_ReceiveToIdle_DMA(
            &m_uart_handle,
            static_cast<std::uint8_t*>(m_rx_buffer.data()),
            static_cast<std::uint16_t>(m_rx_buffer.size())
        );
        return result;
    }

    HAL_StatusTypeDef start_transmit_dma(std::span<const std::uint8_t> msg)
        requires (DmaTxHandle != nullptr && TxBufferSize > 0)
    {
        if (msg.size() > TxBufferSize) return HAL_ERROR;
        if (m_uart_handle.hdmatx == nullptr) return HAL_ERROR;
        if (m_uart_handle.hdmatx->Init.Mode == DMA_CIRCULAR) {
            // 发送 DMA 不能设置成 Circular 模式
            Error_Handler();
            return HAL_ERROR;
        }
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

        // 解除忙碌状态
        port->release_tx();

        auto callback = port->m_callbacks.on_tx_complete;
        if (callback == nullptr) return;
        callback(*port);
    }

    inline static void hal_on_rx_complete(UART_HandleTypeDef* huart) {
        auto port = find_uart_port_by_huart(huart);
        if (port == nullptr) return;

        auto callback = port->m_callbacks.on_rx_complete;
        if (callback == nullptr) return;
        callback(*port);
    }

    inline static void hal_on_rx_event(UART_HandleTypeDef* huart, HAL_UART_RxEventTypeTypeDef event, std::uint16_t size) {
        auto port = find_uart_port_by_huart(huart);
        if (port == nullptr) return;

        auto callback = port->m_callbacks.on_rx_event;
        if (callback == nullptr) return;
        callback(*port, static_cast<UartPort::RxEvent>(event), size);
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
    if (!UartPortManager::register_port(*this)) {
        Error_Handler();
    }
}
