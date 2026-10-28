#pragma once

#include "frame_codec.hpp"
#include "pid_controller.hpp"
#include "pid_manager.hpp"
#include "uart_port.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <tuple>

namespace pid_viewer_detail {

using CheckedRegion = frame::RegionTag<0>;
using Byte = frame::Field<uint8_t>;
using Float = frame::Field<float>;

// CRC16/CCITT-FALSE 覆盖编码后的类型及载荷，不包含帧头。
template <uint8_t Type, typename... Payload>
using Packet = frame::Frame<
    frame::Header<0xA5>, frame::Begin<CheckedRegion>,
    frame::Header<static_cast<uint8_t>(Type ^ 0x5A)>, Payload...,
    frame::End<CheckedRegion>, frame::Crc<CheckedRegion>
>;

using ReadPid = Packet<0x01, Byte>;
using SelectChannels = Packet<0x02, Byte, Byte, Byte, Byte>;
using SetPid = Packet<0x03, Byte, Float, Float, Float>;
// 保留原协议 CRC 前的两个零字节。
using Ack = Packet<0x83, Byte, Byte, Byte, Float, Float, Float, frame::Header<0, 0>>;
using Monitor = Packet<0x84, frame::Field<uint32_t>,
    Byte, Float, Float, Byte, Float, Float,
    Byte, Float, Float, Byte, Float, Float>;

using PidName = std::array<char, PidController::kNameSize>;
using PidInfo = Packet<0x81, Byte, Byte, frame::Field<PidName>,
    Float, Float, Float, frame::Header<0, 0>>;

static_assert(ReadPid::size() == 5 && SelectChannels::size() == 8 && SetPid::size() == 17);
static_assert(PidInfo::size() == 52 && Ack::size() == 21 && Monitor::size() == 44);

} // namespace pid_viewer_detail

/**
 * @brief PID Viewer 协议处理器。
 *
 * bind() 绑定启用接收流缓冲区的 DMA 串口，update() 在任务中处理收发和监控。
 * 串口须比 Viewer 活得更久；公开接口应在同一任务调用。
 * 串口 DMA 的初始化与启动由调用方负责。
 */
class PidViewer {
public:
    /**
     * @brief 上位机设置比例、积分和微分参数时使用的允许范围，包含上下界。
     * @note 各上下界须为有限数值，且下界不大于上界；默认允许所有有限 float 值。
     */
    struct ParameterLimits {
        float kp_min{std::numeric_limits<float>::lowest()}; ///< 比例系数最小值。
        float kp_max{std::numeric_limits<float>::max()};    ///< 比例系数最大值。
        float ki_min{std::numeric_limits<float>::lowest()}; ///< 积分系数最小值。
        float ki_max{std::numeric_limits<float>::max()};    ///< 积分系数最大值。
        float kd_min{std::numeric_limits<float>::lowest()}; ///< 微分系数最小值。
        float kd_max{std::numeric_limits<float>::max()};    ///< 微分系数最大值。
    };

    /** @brief 禁止复制单例协议处理器。 */
    PidViewer(const PidViewer &) = delete;
    /** @brief 禁止对单例协议处理器进行拷贝赋值。 */
    PidViewer &operator=(const PidViewer &) = delete;

    /**
     * @brief 绑定 DMA 串口，接管 Viewer 收发；不启动或中止串口 DMA。
     * @tparam UniqueId 串口缓冲区的编译期唯一标识。
     * @tparam TransmitBufferSize DMA 发送缓冲区容量，至少为 52 字节。
     * @tparam ReceiveBufferSize DMA 接收缓冲区容量，须大于零。
     * @tparam ReceiveStreamBufferSize 接收流缓冲区容量，须大于零。
     * @param port 待绑定的串口实例，须在绑定期间持续有效。
     * @note 接收流缓冲区由 Viewer 独占读取，发送接口也应由 Viewer 独占。
     *       重新绑定会清空待发帧、半帧和监控通道。
     *       不清空串口接收流缓冲区，也不取消已提交给 DMA 的发送。
     */
    template <StaticString UniqueId, uint16_t TransmitBufferSize,
              uint16_t ReceiveBufferSize, uint16_t ReceiveStreamBufferSize>
    static void bind(
        UartPortDma<UniqueId, TransmitBufferSize, ReceiveBufferSize, ReceiveStreamBufferSize> &port) {
        static_assert(TransmitBufferSize >= kMaxFrameSize, "PID Viewer TX buffer must hold a complete frame");
        static_assert(ReceiveBufferSize > 0 && ReceiveStreamBufferSize > 0,
                      "PID Viewer requires a DMA RX stream buffer");
        using Port = UartPortDma<UniqueId, TransmitBufferSize, ReceiveBufferSize, ReceiveStreamBufferSize>;
        auto &viewer = instance();
        viewer.m_uart_port = &port;
        viewer.m_uart_receive = [](UartPort *uart, std::span<uint8_t> bytes) {
            return static_cast<Port *>(uart)->m_rx_stream_buffer.receive_to(bytes, 0);
        };
        viewer.m_uart_transmit = [](UartPort *uart, std::span<const uint8_t> bytes) {
            return static_cast<Port *>(uart)->start_transmit_dma(bytes);
        };
        viewer.reset_receive();
        viewer.m_transmit_head = 0;
        viewer.m_transmit_count = 0;
        viewer.m_channels.fill(kUnusedPidId);
        viewer.m_monitor_enabled = false;
        viewer.m_monitor_schedule_valid = false;
    }

    /**
     * @brief 在任务中周期调用，非阻塞处理接收、MONITOR 调度和待发帧。
     * @param timestamp_ms 单调递增的毫秒时间戳，允许 uint32_t 回绕。
     * @note 建议每 1 ms 调用；每次最多读取 256 字节，MONITOR 周期为 10 ms。
     *       发送队列满时暂停读取串口；DMA 发送忙时保留待发帧，其他发送错误丢弃该帧。
     *       未调用 bind() 时直接返回。
     */
    static void update(uint32_t timestamp_ms) {
        auto &viewer = instance();
        if (viewer.m_uart_port == nullptr) return;
        viewer.flush_transmit();
        // 一次读取一个字节，队列满时停止消费，避免丢失命令的应答。
        for (std::size_t i = 0; i < 256 && viewer.m_transmit_count < kTransmitQueueSize; ++i) {
            uint8_t byte;
            if (viewer.m_uart_receive(viewer.m_uart_port, std::span{&byte, 1U}) == 0) break;
            viewer.process_byte(byte);
        }
        if (viewer.m_transmit_count < kTransmitQueueSize) viewer.update_monitor(timestamp_ms);
        viewer.flush_transmit();
    }

    /**
     * @brief 设置上位机调参指令允许的 PID 参数范围。
     * @param limits 比例、积分和微分系数的允许上下界。
     * @return 所有上下界均为有限数值且下界不大于上界时返回 true，否则返回 false。
     * @note 失败时保留原范围；成功时只影响后续调参校验，不修改已有 PID 参数。
     */
    static bool set_parameter_limits(const ParameterLimits &limits) {
        return instance().set_limits(limits);
    }

private:
    static constexpr uint8_t kStartOfFrame = 0xA5U;
    static constexpr uint8_t kTypeXorMask = 0x5AU;
    static constexpr uint8_t kUnusedPidId = PidManager::INVALID_PID_ID;
    static constexpr std::size_t kChannelCount = 4U;
    static constexpr std::size_t kMaxFrameSize = pid_viewer_detail::PidInfo::size();
    static constexpr uint32_t kMonitorPeriodMs = 10U;
    static constexpr std::size_t kTransmitQueueSize = 8U;

    struct PendingFrame {
        std::array<uint8_t, kMaxFrameSize> bytes{};
        std::size_t length{0};
    };

    enum FrameType : uint8_t {
        kReadPid = 0x01U,
        kSelectChannels = 0x02U,
        kSetPid = 0x03U,
    };

    enum PidInfoStatus : uint8_t {
        kPidInfoOk = 0x00U,
        kPidInfoNotFound = 0x01U,
    };

    enum AckStatus : uint8_t {
        kAckOk = 0x00U,
        kAckPidNotFound = 0x01U,
        kAckInvalidParameter = 0x02U,
        kAckInvalidChannel = 0x03U,
    };

    PidViewer() = default;

    /**
     * @brief 获取 PID Viewer 单例，首次调用时创建。
     * @return 整个程序共用的协议处理器引用。
     */
    static PidViewer &instance() {
        static PidViewer viewer;
        return viewer;
    }

    void enqueue_transmit(std::span<const uint8_t> bytes) {
        if (bytes.size() > kMaxFrameSize || m_transmit_count == kTransmitQueueSize) return;
        const auto tail = (m_transmit_head + m_transmit_count) % kTransmitQueueSize;
        auto &pending = m_transmit_queue[tail];
        std::copy(bytes.begin(), bytes.end(), pending.bytes.begin());
        pending.length = bytes.size();
        ++m_transmit_count;
    }

    void flush_transmit() {
        if (m_transmit_count == 0) return;
        const auto &pending = m_transmit_queue[m_transmit_head];
        const auto result = m_uart_transmit(m_uart_port, {pending.bytes.data(), pending.length});
        if (result == HAL_BUSY) return;
        // DMA 接口已复制到串口自有缓冲区；错误帧丢弃，避免堵住后续发送。
        m_transmit_head = (m_transmit_head + 1) % kTransmitQueueSize;
        --m_transmit_count;
    }

    static std::size_t frame_length(uint8_t frame_type) {
        switch (frame_type) {
        case kReadPid:
            return pid_viewer_detail::ReadPid::size();
        case kSelectChannels:
            return pid_viewer_detail::SelectChannels::size();
        case kSetPid:
            return pid_viewer_detail::SetPid::size();
        default:
            return 0U;
        }
    }

    static bool time_reached(uint32_t now, uint32_t target) {
        return static_cast<int32_t>(now - target) >= 0;
    }

    void process_byte(uint8_t byte) {
        if (m_receive_length == 0U) {
            if (byte == kStartOfFrame) {
                m_receive_buffer[0] = byte;
                m_receive_length = 1U;
            }
            return;
        }

        if (m_receive_length == 1U) {
            const uint8_t frame_type = static_cast<uint8_t>(byte ^ kTypeXorMask);
            m_expected_length = frame_length(frame_type);
            if (m_expected_length == 0U) {
                reset_receive();
                if (byte == kStartOfFrame) {
                    m_receive_buffer[0] = byte;
                    m_receive_length = 1U;
                }
                return;
            }

            m_receive_buffer[1] = byte;
            m_receive_length = 2U;
            return;
        }

        if (m_receive_length >= m_receive_buffer.size()) {
            reset_receive();
            return;
        }

        m_receive_buffer[m_receive_length++] = byte;
        if (m_receive_length == m_expected_length) {
            process_frame();
            reset_receive();
        }
    }

    void process_frame() {
        const std::span<const uint8_t> bytes{m_receive_buffer.data(), m_expected_length};
        switch (static_cast<uint8_t>(m_receive_buffer[1] ^ kTypeXorMask)) {
        case kReadPid:
            if (const auto values = pid_viewer_detail::ReadPid::deserialize(bytes)) {
                handle_read_pid(std::get<0>(*values));
            }
            break;
        case kSelectChannels:
            if (const auto values = pid_viewer_detail::SelectChannels::deserialize(bytes)) {
                const auto [channel_0, channel_1, channel_2, channel_3] = *values;
                handle_select_channels({channel_0, channel_1, channel_2, channel_3});
            }
            break;
        case kSetPid:
            if (const auto values = pid_viewer_detail::SetPid::deserialize(bytes)) {
                const auto [pid_id, kp, ki, kd] = *values;
                handle_set_pid(pid_id, kp, ki, kd);
            }
            break;
        default:
            break;
        }
    }

    void handle_read_pid(uint8_t pid_id) {
        send_pid_info(pid_id, PidManager::instance().get_pid(pid_id));
    }

    void handle_select_channels(const std::array<uint8_t, kChannelCount> &requested) {
        AckStatus status = kAckOk;
        for (std::size_t i = 0U; i < kChannelCount && status == kAckOk; ++i) {
            if (requested[i] != kUnusedPidId &&
                PidManager::instance().get_pid(requested[i]) == nullptr) {
                status = kAckPidNotFound;
                break;
            }

            for (std::size_t j = 0U; j < i; ++j) {
                if (requested[i] != kUnusedPidId && requested[i] == requested[j]) {
                    status = kAckInvalidChannel;
                    break;
                }
            }
        }

        if (status == kAckOk) {
            m_channels = requested;
            m_monitor_enabled = false;
            for (uint8_t channel : m_channels) {
                if (channel != kUnusedPidId) {
                    m_monitor_enabled = true;
                    break;
                }
            }
            m_monitor_schedule_valid = false;
        }

        send_ack(kSelectChannels, status, kUnusedPidId, 0.0f, 0.0f, 0.0f);
    }

    void handle_set_pid(uint8_t pid_id, float kp, float ki, float kd) {
        PidController *pid = PidManager::instance().get_pid(pid_id);
        if (pid == nullptr) {
            send_ack(kSetPid, kAckPidNotFound, pid_id, 0.0f, 0.0f, 0.0f);
            return;
        }

        if (!valid_parameter(kp, m_parameter_limits.kp_min, m_parameter_limits.kp_max) ||
            !valid_parameter(ki, m_parameter_limits.ki_min, m_parameter_limits.ki_max) ||
            !valid_parameter(kd, m_parameter_limits.kd_min, m_parameter_limits.kd_max)) {
            const PidController::Config &config = pid->config();
            send_ack(kSetPid, kAckInvalidParameter, pid_id,
                     config.kp.load(std::memory_order_relaxed),
                     config.ki.load(std::memory_order_relaxed),
                     config.kd.load(std::memory_order_relaxed));
            return;
        }

        pid->set_gains(kp, ki, kd);
        send_ack(kSetPid, kAckOk, pid_id, kp, ki, kd);
    }

    static bool valid_parameter(float value, float minimum, float maximum) {
        return std::isfinite(value) && value >= minimum && value <= maximum;
    }

    bool set_limits(const ParameterLimits &limits) {
        if (!valid_limit_pair(limits.kp_min, limits.kp_max) ||
            !valid_limit_pair(limits.ki_min, limits.ki_max) ||
            !valid_limit_pair(limits.kd_min, limits.kd_max)) {
            return false;
        }
        m_parameter_limits = limits;
        return true;
    }

    static bool valid_limit_pair(float minimum, float maximum) {
        return std::isfinite(minimum) && std::isfinite(maximum) && minimum <= maximum;
    }

    void send_pid_info(uint8_t pid_id, const PidController *pid) {
        pid_viewer_detail::PidName name{};
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (pid != nullptr) {
            std::copy_n(pid->name(), name.size(), name.begin());
            const auto &config = pid->config();
            kp = config.kp.load(std::memory_order_relaxed);
            ki = config.ki.load(std::memory_order_relaxed);
            kd = config.kd.load(std::memory_order_relaxed);
        }
        const auto bytes = pid_viewer_detail::PidInfo::serialize(std::tuple{
            static_cast<uint8_t>(pid == nullptr ? kPidInfoNotFound : kPidInfoOk),
            pid_id, name, kp, ki, kd});
        enqueue_transmit(bytes);
    }

    void send_ack(uint8_t command_type, AckStatus status, uint8_t pid_id,
                  float kp, float ki, float kd) {
        const auto bytes = pid_viewer_detail::Ack::serialize(
            std::tuple{command_type, static_cast<uint8_t>(status), pid_id, kp, ki, kd});
        enqueue_transmit(bytes);
    }

    void update_monitor(uint32_t timestamp_ms) {
        if (!m_monitor_enabled) {
            return;
        }

        if (!m_monitor_schedule_valid || time_reached(timestamp_ms, m_next_monitor_ms)) {
            generate_monitor(timestamp_ms);
            m_next_monitor_ms = timestamp_ms + kMonitorPeriodMs;
            m_monitor_schedule_valid = true;
        }
    }

    void generate_monitor(uint32_t timestamp_ms) {
        std::array<std::tuple<uint8_t, float, float>, kChannelCount> values{};
        for (std::size_t i = 0U; i < kChannelCount; ++i) {
            const uint8_t pid_id = m_channels[i];
            const PidController *pid = PidManager::instance().get_pid(pid_id);
            values[i] = {pid_id, pid == nullptr ? 0.0f : pid->ref(),
                                pid == nullptr ? 0.0f : pid->measure()};
        }

        const auto bytes = pid_viewer_detail::Monitor::serialize(std::tuple_cat(
            std::tuple{timestamp_ms}, values[0], values[1], values[2], values[3]));
        enqueue_transmit(bytes);
    }

    void reset_receive() {
        m_receive_length = 0U;
        m_expected_length = 0U;
    }

    std::array<uint8_t, kMaxFrameSize> m_receive_buffer{};
    std::size_t m_receive_length{0U};
    std::size_t m_expected_length{0U};
    std::array<uint8_t, kChannelCount> m_channels{
        kUnusedPidId, kUnusedPidId, kUnusedPidId, kUnusedPidId};
    ParameterLimits m_parameter_limits{};
    UartPort *m_uart_port{nullptr};
    std::size_t (*m_uart_receive)(UartPort *, std::span<uint8_t>){nullptr};
    HAL_StatusTypeDef (*m_uart_transmit)(UartPort *, std::span<const uint8_t>){nullptr};
    std::array<PendingFrame, kTransmitQueueSize> m_transmit_queue{};
    std::size_t m_transmit_head{0};
    std::size_t m_transmit_count{0};
    uint32_t m_next_monitor_ms{0U};
    bool m_monitor_schedule_valid{false};
    bool m_monitor_enabled{false};
};
