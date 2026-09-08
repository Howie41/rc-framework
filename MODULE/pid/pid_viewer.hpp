#pragma once

#include "pid_controller.hpp"
#include "pid_manager.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

/**
 * @brief PID Viewer 协议处理器。
 *
 * 该类只负责 PID Viewer 协议和数据组织，不直接依赖 UART、DMA 或 RTOS。
 * 串口收到的数据通过 receive() 输入；所有待发送帧统一从 transmit() 输出。
 */
class PidViewer {
public:
    using TransmitCallback = void (*)(const uint8_t *data, std::size_t length);

    struct ParameterLimits {
        float kp_min{std::numeric_limits<float>::lowest()};
        float kp_max{std::numeric_limits<float>::max()};
        float ki_min{std::numeric_limits<float>::lowest()};
        float ki_max{std::numeric_limits<float>::max()};
        float kd_min{std::numeric_limits<float>::lowest()};
        float kd_max{std::numeric_limits<float>::max()};
    };

    PidViewer(const PidViewer &) = delete;
    PidViewer &operator=(const PidViewer &) = delete;

    static PidViewer &instance() {
        static PidViewer viewer;
        return viewer;
    }

    /**
     * @brief 输入串口接收到的字节流。
     * @param data 接收缓冲区。
     * @param length 缓冲区长度，可以包含半帧或多帧。
     */
    static void receive(const uint8_t *data, std::size_t length) {
        if (data == nullptr || length == 0U) {
            return;
        }
        instance().process_receive(data, length);
    }

    /**
     * @brief PID Viewer 统一发送出口。
     *
     * 具体 UART/DMA/TX 队列由 set_transmit_callback() 接入。未设置回调时，
     * 数据会被安全丢弃。
     */
    static void transmit(const uint8_t *data, std::size_t length) {
        PidViewer &viewer = instance();
        if (viewer.transmit_callback_ != nullptr && data != nullptr && length != 0U) {
            viewer.transmit_callback_(data, length);
        }
    }

    /**
     * @brief 设置实际发送接口。
     * @param callback 接收完整协议帧的发送回调；传入 nullptr 可禁用发送。
     */
    static void set_transmit_callback(TransmitCallback callback) {
        instance().transmit_callback_ = callback;
    }

    /** @brief 通知接收帧超时并清空当前解析状态。 */
    static void receive_timeout() { instance().reset_receive(); }

    /**
     * @brief 周期处理 MONITOR 发送。
     * @param timestamp_ms 当前 RTOS 毫秒时间戳。
     */
    static void poll(uint32_t timestamp_ms) {
        instance().process_poll(timestamp_ms);
    }

    /**
     * @brief 设置 SET_PID 参数范围。
     * @return 范围有效并设置成功时返回 true。
     */
    static bool set_parameter_limits(const ParameterLimits &limits) {
        return instance().set_limits(limits);
    }

private:
    static constexpr uint8_t SOF = 0xA5U;
    static constexpr uint8_t TYPE_XOR = 0x5AU;
    static constexpr uint8_t UNUSED_PID_ID = PidManager::INVALID_PID_ID;
    static constexpr std::size_t CHANNEL_COUNT = 4U;
    static constexpr std::size_t MAX_FRAME_SIZE = 52U;
    static constexpr uint32_t MONITOR_PERIOD_MS = 10U;

    enum FrameType : uint8_t {
        READ_PID = 0x01U,
        SELECT_CHANNELS = 0x02U,
        SET_PID = 0x03U,
    };

    enum ResponseType : uint8_t {
        PID_INFO = 0x81U,
        MONITOR = 0x84U,
        ACK = 0x83U,
    };

    enum PidInfoStatus : uint8_t {
        PID_INFO_OK = 0x00U,
        PID_INFO_NOT_FOUND = 0x01U,
    };

    enum AckStatus : uint8_t {
        ACK_OK = 0x00U,
        ACK_PID_NOT_FOUND = 0x01U,
        ACK_INVALID_PARAMETER = 0x02U,
        ACK_INVALID_CHANNEL = 0x03U,
    };

    PidViewer() = default;

    static uint8_t encoded_type(uint8_t frame_type) {
        return static_cast<uint8_t>(frame_type ^ TYPE_XOR);
    }

    static uint16_t crc16_ccitt_false(const uint8_t *data, std::size_t length) {
        uint16_t crc = 0xFFFFU;
        for (std::size_t i = 0U; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8U;
            for (uint8_t bit = 0U; bit < 8U; ++bit) {
                crc = (crc & 0x8000U) != 0U
                          ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                          : static_cast<uint16_t>(crc << 1U);
            }
        }
        return crc;
    }

    static uint16_t read_u16(const uint8_t *data) {
        return static_cast<uint16_t>(data[0]) |
               static_cast<uint16_t>(data[1]) << 8U;
    }

    static uint32_t read_u32(const uint8_t *data) {
        return static_cast<uint32_t>(data[0]) |
               static_cast<uint32_t>(data[1]) << 8U |
               static_cast<uint32_t>(data[2]) << 16U |
               static_cast<uint32_t>(data[3]) << 24U;
    }

    static float read_float(const uint8_t *data) {
        const uint32_t bits = read_u32(data);
        float value = 0.0f;
        static_assert(sizeof(value) == sizeof(bits));
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    static void write_u16(uint8_t *data, uint16_t value) {
        data[0] = static_cast<uint8_t>(value & 0xFFU);
        data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    }

    static void write_u32(uint8_t *data, uint32_t value) {
        data[0] = static_cast<uint8_t>(value & 0xFFU);
        data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
        data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
        data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
    }

    static void write_float(uint8_t *data, float value) {
        uint32_t bits = 0U;
        static_assert(sizeof(value) == sizeof(bits));
        std::memcpy(&bits, &value, sizeof(bits));
        write_u32(data, bits);
    }

    static std::size_t frame_length(uint8_t frame_type) {
        switch (frame_type) {
        case READ_PID:
            return 5U;
        case SELECT_CHANNELS:
            return 8U;
        case SET_PID:
            return 17U;
        default:
            return 0U;
        }
    }

    static bool time_reached(uint32_t now, uint32_t target) {
        return static_cast<int32_t>(now - target) >= 0;
    }

    void process_receive(const uint8_t *data, std::size_t length) {
        for (std::size_t i = 0U; i < length; ++i) {
            process_byte(data[i]);
        }
    }

    void process_byte(uint8_t byte) {
        if (receive_length_ == 0U) {
            if (byte == SOF) {
                receive_buffer_[0] = byte;
                receive_length_ = 1U;
            }
            return;
        }

        if (receive_length_ == 1U) {
            const uint8_t frame_type = static_cast<uint8_t>(byte ^ TYPE_XOR);
            expected_length_ = frame_length(frame_type);
            if (expected_length_ == 0U) {
                reset_receive();
                if (byte == SOF) {
                    receive_buffer_[0] = byte;
                    receive_length_ = 1U;
                }
                return;
            }

            receive_buffer_[1] = byte;
            receive_length_ = 2U;
            return;
        }

        if (receive_length_ >= receive_buffer_.size()) {
            reset_receive();
            return;
        }

        receive_buffer_[receive_length_++] = byte;
        if (receive_length_ == expected_length_) {
            process_frame();
            reset_receive();
        }
    }

    void process_frame() {
        if (expected_length_ < 5U) {
            return;
        }

        const uint16_t expected_crc = crc16_ccitt_false(
            receive_buffer_.data() + 1U, expected_length_ - 3U);
        const uint16_t received_crc = read_u16(receive_buffer_.data() + expected_length_ - 2U);
        if (expected_crc != received_crc) {
            return;
        }

        switch (static_cast<uint8_t>(receive_buffer_[1] ^ TYPE_XOR)) {
        case READ_PID:
            handle_read_pid(receive_buffer_[2]);
            break;
        case SELECT_CHANNELS:
            handle_select_channels();
            break;
        case SET_PID:
            handle_set_pid();
            break;
        default:
            break;
        }
    }

    void handle_read_pid(uint8_t pid_id) {
        send_pid_info(pid_id, PidManager::instance().get_pid(pid_id));
    }

    void handle_select_channels() {
        std::array<uint8_t, CHANNEL_COUNT> requested{};
        for (std::size_t i = 0U; i < CHANNEL_COUNT; ++i) {
            requested[i] = receive_buffer_[2U + i];
        }

        AckStatus status = ACK_OK;
        for (std::size_t i = 0U; i < CHANNEL_COUNT && status == ACK_OK; ++i) {
            if (requested[i] != UNUSED_PID_ID &&
                PidManager::instance().get_pid(requested[i]) == nullptr) {
                status = ACK_PID_NOT_FOUND;
                break;
            }

            for (std::size_t j = 0U; j < i; ++j) {
                if (requested[i] != UNUSED_PID_ID && requested[i] == requested[j]) {
                    status = ACK_INVALID_CHANNEL;
                    break;
                }
            }
        }

        if (status == ACK_OK) {
            channels_ = requested;
            monitor_enabled_ = false;
            for (uint8_t channel : channels_) {
                if (channel != UNUSED_PID_ID) {
                    monitor_enabled_ = true;
                    break;
                }
            }
            monitor_schedule_valid_ = false;
        }

        send_ack(SELECT_CHANNELS, status, UNUSED_PID_ID, 0.0f, 0.0f, 0.0f);
    }

    void handle_set_pid() {
        const uint8_t pid_id = receive_buffer_[2];
        PidController *pid = PidManager::instance().get_pid(pid_id);
        if (pid == nullptr) {
            send_ack(SET_PID, ACK_PID_NOT_FOUND, pid_id, 0.0f, 0.0f, 0.0f);
            return;
        }

        const float kp = read_float(receive_buffer_.data() + 3U);
        const float ki = read_float(receive_buffer_.data() + 7U);
        const float kd = read_float(receive_buffer_.data() + 11U);
        if (!valid_parameter(kp, parameter_limits_.kp_min, parameter_limits_.kp_max) ||
            !valid_parameter(ki, parameter_limits_.ki_min, parameter_limits_.ki_max) ||
            !valid_parameter(kd, parameter_limits_.kd_min, parameter_limits_.kd_max)) {
            const PidController::Config &config = pid->config();
            send_ack(SET_PID, ACK_INVALID_PARAMETER, pid_id,
                     config.kp, config.ki, config.kd);
            return;
        }

        pid->set_gains(kp, ki, kd);
        send_ack(SET_PID, ACK_OK, pid_id, kp, ki, kd);
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
        parameter_limits_ = limits;
        return true;
    }

    static bool valid_limit_pair(float minimum, float maximum) {
        return std::isfinite(minimum) && std::isfinite(maximum) && minimum <= maximum;
    }

    void send_pid_info(uint8_t pid_id, const PidController *pid) {
        std::array<uint8_t, 52U> frame{};
        frame[0] = SOF;
        frame[1] = encoded_type(PID_INFO);
        frame[2] = pid == nullptr ? PID_INFO_NOT_FOUND : PID_INFO_OK;
        frame[3] = pid_id;

        if (pid != nullptr) {
            std::memcpy(frame.data() + 4U, pid->name(), PidController::NAME_SIZE);
            const PidController::Config &config = pid->config();
            write_float(frame.data() + 36U, config.kp);
            write_float(frame.data() + 40U, config.ki);
            write_float(frame.data() + 44U, config.kd);
        }

        finish_and_transmit(frame);
    }

    void send_ack(uint8_t command_type, AckStatus status, uint8_t pid_id,
                  float kp, float ki, float kd) {
        std::array<uint8_t, 21U> frame{};
        frame[0] = SOF;
        frame[1] = encoded_type(ACK);
        frame[2] = command_type;
        frame[3] = status;
        frame[4] = pid_id;
        write_float(frame.data() + 5U, kp);
        write_float(frame.data() + 9U, ki);
        write_float(frame.data() + 13U, kd);

        finish_and_transmit(frame);
    }

    template <std::size_t N>
    static void finish_and_transmit(std::array<uint8_t, N> &frame) {
        const uint16_t crc = crc16_ccitt_false(frame.data() + 1U, N - 3U);
        write_u16(frame.data() + N - 2U, crc);
        transmit(frame.data(), frame.size());
    }

    void process_poll(uint32_t timestamp_ms) {
        if (!monitor_enabled_) {
            return;
        }

        if (!monitor_schedule_valid_ || time_reached(timestamp_ms, next_monitor_ms_)) {
            generate_monitor(timestamp_ms);
            next_monitor_ms_ = timestamp_ms + MONITOR_PERIOD_MS;
            monitor_schedule_valid_ = true;
        }
    }

    void generate_monitor(uint32_t timestamp_ms) {
        std::array<uint8_t, 44U> frame{};
        frame[0] = SOF;
        frame[1] = encoded_type(MONITOR);
        write_u32(frame.data() + 2U, timestamp_ms);

        for (std::size_t i = 0U; i < CHANNEL_COUNT; ++i) {
            const std::size_t offset = 6U + i * 9U;
            const uint8_t pid_id = channels_[i];
            frame[offset] = pid_id;

            if (pid_id != UNUSED_PID_ID) {
                const PidController *pid = PidManager::instance().get_pid(pid_id);
                if (pid != nullptr) {
                    write_float(frame.data() + offset + 1U, pid->ref());
                    write_float(frame.data() + offset + 5U, pid->measure());
                }
            }
        }

        finish_and_transmit(frame);
    }

    void reset_receive() {
        receive_length_ = 0U;
        expected_length_ = 0U;
    }

    std::array<uint8_t, MAX_FRAME_SIZE> receive_buffer_{};
    std::size_t receive_length_{0U};
    std::size_t expected_length_{0U};
    std::array<uint8_t, CHANNEL_COUNT> channels_{
        UNUSED_PID_ID, UNUSED_PID_ID, UNUSED_PID_ID, UNUSED_PID_ID};
    ParameterLimits parameter_limits_{};
    TransmitCallback transmit_callback_{nullptr};
    uint32_t next_monitor_ms_{0U};
    bool monitor_schedule_valid_{false};
    bool monitor_enabled_{false};
};
