#pragma once

#include "rs485_device.hpp"
#include "uart_port.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

/// @brief 基于 DMA 串口的 RS-485 主机请求/响应总线；注册及通信处理由同一任务调用。
/// @note 独占所绑定串口的收发。UART、总线和设备须持续存活；heartbeat() 可轮询已注册设备。
class Rs485Bus {
public:
    static constexpr std::size_t kMaxDevices = 16;
    static constexpr std::size_t kMaxFrameSize = 256;

    enum class State : std::uint8_t { Idle, Transmitting, WaitingResponse, CoolingDown, Faulted };

    struct Config {
        std::uint32_t tx_timeout_us{5000}; // 从提交发送起计算的发送看门限，须覆盖实际线速和任务周期。
        std::uint32_t quiet_time_us{100};  // 事务结束后保持的总线静默间隔，收到残留数据时重新计时。
        std::uint32_t response_timeout_us{1000}; // heartbeat() 请求的响应超时，0 表示只发送。
    };

    template <StaticString UniqueId, std::uint16_t TxSize, std::uint16_t RxSize, std::uint16_t StreamSize>
    explicit Rs485Bus(UartPortDma<UniqueId, TxSize, RxSize, StreamSize>& port) noexcept
        : Rs485Bus(port, Config{}) {}

    /// @brief 绑定已有 DMA 串口，不修改其回调；硬件 DE 方向控制由 UART 外设完成。
    template <StaticString UniqueId, std::uint16_t TxSize, std::uint16_t RxSize, std::uint16_t StreamSize>
    Rs485Bus(UartPortDma<UniqueId, TxSize, RxSize, StreamSize>& port, Config config) noexcept
        : m_uart(port), m_config(config), m_tx_capacity(std::min<std::size_t>(TxSize, kMaxFrameSize)) {
        static_assert(TxSize > 0 && RxSize > 0 && StreamSize > 0,
                      "Rs485Bus 要求启用 DMA 收发及接收流缓冲区");
        using Port = UartPortDma<UniqueId, TxSize, RxSize, StreamSize>;
        m_prepare_receive = [](UartPort& base) {
            auto& uart = static_cast<Port&>(base);
            auto& handle = uart.handle();
            if (handle.hdmatx == nullptr || handle.hdmarx == nullptr ||
                handle.hdmatx->Init.Mode != DMA_NORMAL || handle.hdmarx->Init.Mode != DMA_CIRCULAR ||
                uart.m_rx_stream_buffer.handle() == nullptr) return false;
            if (handle.RxState == HAL_UART_STATE_BUSY_RX) return true;
            return uart.start_receive_to_idle_dma() == HAL_OK;
        };
        m_transmit = [](UartPort& base, std::span<const std::uint8_t> bytes) {
            return static_cast<Port&>(base).start_transmit_dma(bytes);
        };
        m_receive = [](UartPort& base, std::span<std::uint8_t> bytes) {
            return static_cast<Port&>(base).m_rx_stream_buffer.receive_to(bytes, 0);
        };
        m_dropped_bytes = [](UartPort& base) {
            return static_cast<Port&>(base).m_rx_stream_dropped_bytes.load(std::memory_order_relaxed);
        };
    }

    Rs485Bus(const Rs485Bus&) = delete;
    Rs485Bus& operator=(const Rs485Bus&) = delete;
    Rs485Bus(Rs485Bus&&) = delete;
    Rs485Bus& operator=(Rs485Bus&&) = delete;

    /// @brief 在 RTOS 流缓冲区创建后调用，检查 DMA 配置并启动环形接收；重复调用不清空设备表。
    [[nodiscard]] std::expected<void, Rs485Error> init() {
        if (m_initialized) return {};
        if (m_config.tx_timeout_us == 0 || m_config.tx_timeout_us > 0x7FFFFFFFU ||
            m_config.quiet_time_us > 0x7FFFFFFFU || m_config.response_timeout_us > 0x7FFFFFFFU) {
            return std::unexpected(Rs485Error::InvalidParameter);
        }
        if (!m_prepare_receive(m_uart)) return std::unexpected(Rs485Error::TransportNotReady);
        m_uart_error_snapshot = m_uart.error_count();
        m_dropped_snapshot = m_dropped_bytes(m_uart);
        m_initialized = true;
        return {};
    }

    /// @brief 显式注册设备；同指针重复注册成功，拒绝同一总线上的重复 ID。
    /// @note 同一设备只注册到一条总线，由调用方保证。
    [[nodiscard]] std::expected<void, Rs485Error> register_device(Rs485Device* device) {
        if (!m_initialized) return std::unexpected(Rs485Error::NotInitialized);
        if (busy()) return std::unexpected(Rs485Error::Busy);
        if (device == nullptr) return std::unexpected(Rs485Error::NullDevice);
        for (auto* existing : m_devices) {
            if (existing == device) return {};
            if (existing != nullptr && existing->id() == device->id()) {
                return std::unexpected(Rs485Error::DuplicateId);
            }
        }
        for (auto& slot : m_devices) {
            if (slot == nullptr) {
                slot = device;
                return {};
            }
        }
        return std::unexpected(Rs485Error::DeviceTableFull);
    }

    /// @brief 生成最新命令并提交异步发送；timeout_us=0 表示本次不等待响应。
    /// @param now_us 单调的 uint32_t 微秒时间，允许回绕，须与 process() 使用同一时间源。
    /// @param timeout_us 观察到 UART TC 完成后的响应等待时间，范围 0～0x7FFFFFFF。
    [[nodiscard]] std::expected<void, Rs485Error>
    request(Rs485Device& device, std::uint32_t now_us, std::uint32_t timeout_us = 1000) {
        if (!m_initialized) return std::unexpected(Rs485Error::NotInitialized);
        if (busy() || m_uart.is_transmitting()) return std::unexpected(Rs485Error::Busy);
        if (timeout_us > 0x7FFFFFFFU) return std::unexpected(Rs485Error::InvalidParameter);
        if (std::find(m_devices.begin(), m_devices.end(), &device) == m_devices.end()) {
            return std::unexpected(Rs485Error::NotRegistered);
        }
        if (auto error = transport_error()) return fail(*error);
        // 开始新事务前丢弃旧数据，避免上一事务的半帧与新响应拼接。
        if (!discard_input()) {
            enter_quiet(now_us);
            return std::unexpected(Rs485Error::Busy);
        }
        const auto size = device.build_tx(std::span{m_tx_buffer}.first(m_tx_capacity));
        if (!size) return std::unexpected(size.error());
        if (*size == 0 || *size > m_tx_capacity) return std::unexpected(Rs485Error::BufferSizeMismatch);
        device.reset_rx();
        m_tx_complete_snapshot = m_uart.transmit_complete_count();
        const auto status = m_transmit(m_uart, std::span{m_tx_buffer}.first(*size));
        if (status == HAL_BUSY) return std::unexpected(Rs485Error::Busy);
        if (status != HAL_OK) return std::unexpected(Rs485Error::UartError);
        m_active_device = &device;
        m_phase_start_us = now_us;
        m_response_timeout_us = timeout_us;
        m_received_response = false;
        m_last_error.reset();
        m_state = State::Transmitting;
        return {};
    }

    /// @brief 非阻塞处理一次收发，单次最多消费 512 字节；true 表示本次成功完成一个事务。
    /// @return 超时、UART 错误和流缓冲溢出通过 unexpected 报告；无完成事件返回 false。
    [[nodiscard]] std::expected<bool, Rs485Error> process(std::uint32_t now_us) {
        if (!m_initialized) return std::unexpected(Rs485Error::NotInitialized);
        if (m_state == State::Faulted) return std::unexpected(*m_last_error);
        if (auto error = transport_error()) return fail(*error);
        if (m_state == State::Transmitting &&
            m_uart.transmit_complete_count() != m_tx_complete_snapshot) {
            m_state = State::WaitingResponse;
            m_phase_start_us = now_us;
        }

        std::array<std::uint8_t, kRxChunkSize> bytes{};
        std::size_t consumed = 0;
        while (consumed < kRxBudget) {
            const auto size = m_receive(m_uart, bytes);
            if (size == 0) break;
            consumed += size;
            if (m_active_device != nullptr) {
                if (m_response_timeout_us != 0 &&
                    m_active_device->feed_rx(std::span{bytes}.first(size)) != 0) m_received_response = true;
            } else {
                enter_quiet(now_us);
            }
        }
        if (auto error = transport_error()) return fail(*error);

        if (m_state == State::Transmitting && expired(now_us, m_config.tx_timeout_us)) {
            return fail(Rs485Error::TxTimeout);
        }
        if (m_state == State::WaitingResponse) {
            if (m_received_response || m_response_timeout_us == 0) {
                finish(now_us);
                return true;
            }
            if (expired(now_us, m_response_timeout_us)) {
                finish(now_us, Rs485Error::ResponseTimeout);
                return std::unexpected(Rs485Error::ResponseTimeout);
            }
        }
        if (m_state == State::CoolingDown && expired(now_us, m_config.quiet_time_us)) {
            m_state = State::Idle;
        }
        return false;
    }

    /// @brief 推进已有事务，并在空闲时按注册顺序轮询下一设备；一次最多提交一个请求。
    /// @param now_us 当前 uint32_t 微秒时间，允许回绕，与 process()、request() 使用同一时间源。
    /// @return true 表示本次完成了一个事务；处理或提交失败返回错误，无事件返回 false。
    /// @note 不自动恢复故障；提交成功后推进轮询位置，提交失败时下次仍尝试当前设备。
    [[nodiscard]] std::expected<bool, Rs485Error> heartbeat(std::uint32_t now_us) {
        const auto result = process(now_us);
        if (!result || busy()) return result;

        for (std::size_t offset = 0; offset < m_devices.size(); ++offset) {
            const auto index = (m_next_device_index + offset) % m_devices.size();
            auto* device = m_devices[index];
            if (device == nullptr) continue;
            const auto sent = request(*device, now_us, m_config.response_timeout_us);
            if (!sent) return std::unexpected(sent.error());
            m_next_device_index = (index + 1) % m_devices.size();
            break;
        }
        return result;
    }

    /// @brief 仅用于故障恢复：同步中止串口收发、重启 RX DMA，保留注册表及设备反馈和目标。
    /// @note 中止失败时保持 Faulted，禁止继续提交发送；正常收发路径不调用阻塞中止函数。
    [[nodiscard]] std::expected<void, Rs485Error> recover(std::uint32_t now_us) {
        if (!m_initialized) return std::unexpected(Rs485Error::NotInitialized);
        if (m_state != State::Faulted) return std::unexpected(Rs485Error::Busy);
        if (m_uart.abort_all_transmit() != HAL_OK || m_uart.abort_all_receive() != HAL_OK ||
            !m_prepare_receive(m_uart)) return std::unexpected(Rs485Error::UartError);
        for (auto* device : m_devices) if (device != nullptr) device->reset_rx();
        m_active_device = nullptr;
        m_received_response = false;
        (void)discard_input();
        m_uart_error_snapshot = m_uart.error_count();
        m_dropped_snapshot = m_dropped_bytes(m_uart);
        m_last_error.reset();
        enter_quiet(now_us);
        return {};
    }

    /// @brief 发送、等待响应、静默间隔或故障状态均为忙，须持续调用 process() 或 heartbeat() 推进。
    [[nodiscard]] bool busy() const noexcept { return m_state != State::Idle; }
    /// @brief 返回当前总线状态，便于上机观察。
    [[nodiscard]] State state() const noexcept { return m_state; }
    /// @brief 返回最近事务或传输故障；新请求被接受及恢复成功时清除。
    [[nodiscard]] std::optional<Rs485Error> last_error() const noexcept { return m_last_error; }

private:
    static constexpr std::size_t kRxChunkSize = 64;
    static constexpr std::size_t kRxBudget = 512;
    UartPort& m_uart;
    Config m_config;
    std::size_t m_tx_capacity;
    bool (*m_prepare_receive)(UartPort&);
    HAL_StatusTypeDef (*m_transmit)(UartPort&, std::span<const std::uint8_t>);
    std::size_t (*m_receive)(UartPort&, std::span<std::uint8_t>);
    std::size_t (*m_dropped_bytes)(UartPort&);
    std::array<Rs485Device*, kMaxDevices> m_devices{};
    std::size_t m_next_device_index{0};
    std::array<std::uint8_t, kMaxFrameSize> m_tx_buffer{};
    Rs485Device* m_active_device{nullptr};
    State m_state{State::Idle};
    bool m_initialized{false};
    bool m_received_response{false};
    std::uint32_t m_phase_start_us{0};
    std::uint32_t m_response_timeout_us{0};
    std::uint32_t m_tx_complete_snapshot{0};
    std::uint32_t m_uart_error_snapshot{0};
    std::size_t m_dropped_snapshot{0};
    std::optional<Rs485Error> m_last_error;

    bool expired(std::uint32_t now_us, std::uint32_t interval_us) const noexcept {
        return static_cast<std::uint32_t>(now_us - m_phase_start_us) >= interval_us;
    }

    void enter_quiet(std::uint32_t now_us) noexcept {
        m_phase_start_us = now_us;
        m_state = m_config.quiet_time_us == 0 ? State::Idle : State::CoolingDown;
    }

    bool discard_input() {
        std::array<std::uint8_t, kRxChunkSize> bytes{};
        for (std::size_t count = 0; count < kRxBudget;) {
            const auto received = m_receive(m_uart, bytes);
            if (received == 0) return true;
            count += received;
        }
        return false;
    }

    std::optional<Rs485Error> transport_error() const {
        if (m_uart.error_count() != m_uart_error_snapshot) return Rs485Error::UartError;
        if (m_dropped_bytes(m_uart) != m_dropped_snapshot) return Rs485Error::RxOverflow;
        return std::nullopt;
    }

    std::unexpected<Rs485Error> fail(Rs485Error error) {
        if (m_active_device != nullptr) m_active_device->reset_rx();
        m_last_error = error;
        m_state = State::Faulted;
        return std::unexpected(error);
    }

    void finish(std::uint32_t now_us, std::optional<Rs485Error> error = std::nullopt) {
        m_active_device->reset_rx();
        m_active_device = nullptr;
        m_received_response = false;
        m_last_error = error;
        enter_quiet(now_us);
    }
};
