#pragma once

#include "memory_map.h"
#include "rs485_device.hpp"
#include "rtos_queue.hpp"
#include "rtos_utility.hpp"
#include "uart_port.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

/// @brief RS-485 设备注册、原始字节接收分发、发送排队及回复等待。
/// @note 在 RTOS 资源初始化前构造；UART 接收由调用方启动。所有发送均须经由本总线。
///       入队可由多个任务调用，process_rx()/process_tx() 由同一通信任务调用。
///       UART、总线及已注册设备须持续存活，方向控制使用 UART 硬件 DE。
class Rs485Bus {
public:
    static constexpr std::size_t kMaxDevices = 16;
    static constexpr std::size_t kMaxFrameSize = 128;
    static constexpr std::size_t kTxQueueSize = 4;
    static constexpr std::uint32_t kResponseTimeoutMs = 1;
    static constexpr std::uint32_t kResponseIdleMs = 1;

    /// @brief 绑定现有 DMA 串口；DMA 配置、发送互斥及 RTOS 缓冲区由 UART/Queue 自身保证。
    template <StaticString UniqueId, std::uint16_t TxSize, std::uint16_t RxSize, std::uint16_t StreamSize>
    explicit Rs485Bus(UartPortDma<UniqueId, TxSize, RxSize, StreamSize>& port) noexcept
        : m_uart(port), m_tx_capacity(std::min<std::size_t>(TxSize, kMaxFrameSize)) {
        using Port = UartPortDma<UniqueId, TxSize, RxSize, StreamSize>;
        // 仅适配具体 UART 模板类型，不注册额外的中断回调。
        m_transmit = [](UartPort& base, std::span<const std::uint8_t> bytes) {
            return static_cast<Port&>(base).start_transmit_dma(bytes);
        };
        m_receive = [](UartPort& base, std::span<std::uint8_t> bytes) {
            return static_cast<Port&>(base).m_rx_stream_buffer.receive_to(bytes, 0);
        };
    }

    Rs485Bus(const Rs485Bus&) = delete;
    Rs485Bus& operator=(const Rs485Bus&) = delete;
    Rs485Bus(Rs485Bus&&) = delete;
    Rs485Bus& operator=(Rs485Bus&&) = delete;

    /// @brief 清空设备表，仅在注册、入队及通信开始前调用；不初始化或重启 UART。
    void init() noexcept {
        m_devices.fill(nullptr);
    }

    /// @brief 注册设备；同指针重复注册成功，拒绝重复 ID；通信开始前完成注册。
    [[nodiscard]] bool register_device(Rs485Device* device) {
        if (device == nullptr) return false;
        for (auto* existing : m_devices) {
            if (existing == device) return true;
            if (existing != nullptr && existing->id() == device->id()) return false;
        }
        for (auto& slot : m_devices) {
            if (slot == nullptr) {
                slot = device;
                return true;
            }
        }
        return false;
    }

    /// @brief 按值复制一帧报文到发送队列；true 表示入队成功，参数无效或队列满返回 false。
    /// @param response_device 需要等待回复时指定已注册设备；nullptr 表示发送完成即可继续。
    /// @note 不在 Bus 内组帧。队列元素拥有字节数据，调用返回后无需保留 bytes。
    [[nodiscard]] bool add_rs485_msg(std::span<const std::uint8_t> bytes,
                                     Rs485Device* response_device = nullptr) {
        if (bytes.empty() || bytes.size() > m_tx_capacity) {
            return false;
        }
        if (response_device != nullptr &&
            std::find(m_devices.begin(), m_devices.end(), response_device) == m_devices.end()) {
            return false;
        }
        Pack pack{};
        std::copy(bytes.begin(), bytes.end(), pack.data.begin());
        pack.size = bytes.size();
        pack.response_device = response_device;
        return m_tx_queue.put(pack);
    }

    /// @brief 非阻塞读取接收字节并原样交给当前请求设备；有数据分发时返回 true。
    /// @note UART 流可能分片，协议解析由设备负责；无当前请求时丢弃输入。
    bool process_rx() {
        std::array<std::uint8_t, 64> bytes{};
        bool received = false;
        while (const auto size = m_receive(m_uart, bytes)) {
            if (m_response_device == nullptr) continue;
            m_response_device->on_rx(std::span{bytes}.first(size));
            m_last_rx_ms = rtos::now_ms();
            m_state = State::ResponseReceived;
            received = true;
        }
        return received;
    }

    /// @brief 推进回复等待，并至多提交队首一帧；true 表示本次成功提交了发送。
    /// @return 空队列、仍在等待、回复超时或 HAL 提交失败均返回 false。
    /// @note UART TC 前不发送下一帧。指定回复设备时，收到字节后须等待接收静默，或等待回复超时。
    ///       使用 RTOS 毫秒时间；超时从本任务观察到发送结束时开始，不另设发送看门狗。
    ///       HAL 提交失败保留队首，后续可重试；UART 异常的中止/重启交由调用方处理。
    [[nodiscard]] bool process_tx() {
        if (m_uart.is_transmitting()) return false;
        switch (m_state) {
        case State::Transmitting:
            if (m_response_device == nullptr) {
                m_state = State::Idle;
                break;
            }
            m_response_start_ms = rtos::now_ms();
            m_state = State::WaitingResponse;
            return false;
        case State::WaitingResponse:
            if (static_cast<std::uint32_t>(rtos::now_ms() - m_response_start_ms) <
                kResponseTimeoutMs) return false;
            m_response_device = nullptr;
            m_state = State::Idle;
            return false;
        case State::ResponseReceived:
            if (static_cast<std::uint32_t>(rtos::now_ms() - m_last_rx_ms) <
                kResponseIdleMs) return false;
            m_response_device = nullptr;
            m_state = State::Idle;
            break;
        case State::Idle:
            break;
        }

        const auto pack = m_tx_queue.peek();
        if (!pack) return false;
        const auto status = m_transmit(m_uart, std::span{pack->data}.first(pack->size));
        if (status != HAL_OK) return false;
        (void)m_tx_queue.get(); // 只有这个通信任务消费队列，提交成功后才移除。
        m_response_device = pack->response_device;
        m_state = State::Transmitting;
        return true;
    }

private:
    enum class State {
        Idle,             // 可以提交下一帧。
        Transmitting,     // 已提交 UART，尚未开始回复计时。
        WaitingResponse,  // UART 发送结束，等待回复或超时。
        ResponseReceived, // 收到接收字节，等待 UART TC 和接收静默。
    };

    struct Pack {
        std::array<std::uint8_t, kMaxFrameSize> data;
        std::size_t size;
        Rs485Device* response_device;

        Pack() noexcept : data{}, size{0}, response_device{nullptr} {}
    };

    UartPort& m_uart;
    const std::size_t m_tx_capacity;
    HAL_StatusTypeDef (*m_transmit)(UartPort&, std::span<const std::uint8_t>);
    std::size_t (*m_receive)(UartPort&, std::span<std::uint8_t>);
    std::array<Rs485Device*, kMaxDevices> m_devices{};
    rtos::Queue<Pack, kTxQueueSize> m_tx_queue;

    Rs485Device* m_response_device{nullptr};
    State m_state{State::Idle};
    std::uint32_t m_response_start_ms{0};
    std::uint32_t m_last_rx_ms{0};
};
