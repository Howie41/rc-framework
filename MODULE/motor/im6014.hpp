#pragma once

#include "frame_codec.hpp"
#include "frame_parser.hpp"
#include "motor_base.hpp"
#include "rs485_device.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <expected>
#include <limits>
#include <numbers>
#include <span>
#include <tuple>

namespace im6014_detail {

using namespace frame;

using CheckedRegion = RegionTag<0>;

using CommandFrame = Frame<
    Begin<CheckedRegion>, Header<0xFE, 0xEE>,
    Field<std::uint8_t>, Header<0>,
    Field<std::int16_t>, Field<std::int16_t>, Field<std::int32_t>,
    Field<std::int16_t>, Field<std::int16_t>, End<CheckedRegion>,
    Crc<CheckedRegion, CrcType::Crc32Mpeg2WordLe>
>;

using FeedbackFrame = Frame<
    Header<0xFC, 0xEE>, Begin<CheckedRegion>,
    Field<std::uint8_t>, Field<std::int8_t>,
    Field<std::uint8_t>, Field<std::uint8_t>,
    Field<std::int16_t>, Field<std::int16_t>, Field<std::int32_t>,
    Field<std::uint32_t>, Field<std::uint16_t>, Field<std::uint16_t>,
    End<CheckedRegion>, Crc<CheckedRegion, CrcType::Crc32Mpeg2WordLe>
>;

} // namespace im6014_detail

/// @brief IM6014 电机的控制目标、协议编解码与反馈状态，不直接操作串口。
/// @note 输出端位置使用度，速度使用 rad/s，力矩使用 Nm。调用方负责串行访问及通信调度。
class Im6014Motor : public Rs485Device, public MotorBase {
public:
    using CommandFrame = im6014_detail::CommandFrame;
    using FeedbackFrame = im6014_detail::FeedbackFrame;

    using DataTuple = FeedbackFrame::DataTuple;

    static constexpr float kReductionRatio = 38.0f / 3.0f;

    enum class Mode : std::uint8_t { Stop = 0, Foc = 1 };

    enum class Error : std::uint8_t {
        NotInitialized,
        InvalidId,
        InvalidParameter,
        OutOfRange,
        IdMismatch,
        HeaderMismatch,
        FooterMismatch,
        LengthMismatch,
        BufferSizeMismatch,
        CrcMismatch,
    };

    struct HybridCommand {
        float position_deg{0}; // 输出端目标累计位置，度，与电机反馈使用相同原点。
        float speed_rad_s{0};  // 输出端目标速度，rad/s。
        float torque_nm{0};    // 输出端前馈力矩，Nm，按 m_max_output 的绝对值限幅。
        float kp{0};           // 输出端位置刚度，Nm/rad。
        float kd{0};           // 输出端阻尼，Nm/(rad/s)。
    };

    struct Config {
        std::uint8_t id{0};                     // 电机 ID，范围 0～14。
        float max_output_nm{0};                 // 输出端前馈力矩上限，Nm，默认禁止非零前馈力矩。
        float reduction_ratio{kReductionRatio}; // 转子到输出端的减速比，必须为正数。
        Mode mode{Mode::Stop};                  // 初始待发送模式。
        bool timeout_protection{true};          // 是否启用电机一秒超时保护。
    };

    /// @brief 创建未初始化的停机对象，不绑定串口；init() 成功后才可生成命令或更新反馈。
    Im6014Motor() noexcept {
        m_id = kInvalidId;
        set_motor_reduction(kReductionRatio);
        set_max_output(0);
    }

    /// @brief 构造时设置 ID 和输出端力矩上限，使用默认减速比、停机模式和超时保护。
    /// @note 参数非法时保持未初始化，build_tx() 或 on_rx() 返回具体初始化错误。
    Im6014Motor(std::uint8_t id, float max_output_nm) noexcept
        : Im6014Motor(Config{.id = id, .max_output_nm = max_output_nm}) {}

    /// @brief 根据配置构造电机，不绑定串口、不发送命令。
    /// @note 参数非法时保持未初始化，build_tx() 或 on_rx() 返回具体初始化错误。
    explicit Im6014Motor(const Config& config) noexcept : Im6014Motor() {
        const auto result = init(config);
        if (!result) m_initialization_error = result.error();
    }

    /// @brief 设置 ID 和输出端力矩上限，成功时清空控制、反馈及基类位置进程状态。
    /// @param id 电机 ID，范围 0～14，不将广播 ID 作为电机实例。
    /// @param max_output_nm 非负有限力矩上限，不超过默认减速比下协议可表示的正向力矩。
    /// @return 失败时保留原对象状态；成功后为停机模式，默认启用电机一秒超时保护。
    [[nodiscard]]
    std::expected<void, Error> init(std::uint8_t id, float max_output_nm) noexcept {
        return init(Config{.id = id, .max_output_nm = max_output_nm});
    }

    /// @brief 校验并应用全部初始化配置，成功时重置控制、反馈及基类位置进程状态。
    /// @return 参数非法时返回错误并保留原对象状态，所有校验通过后才应用配置。
    [[nodiscard]]
    std::expected<void, Error> init(const Config& config) noexcept {
        if (config.id > 14) return std::unexpected(Error::InvalidId);
        if (!std::isfinite(config.max_output_nm) || config.max_output_nm < 0 ||
            !std::isfinite(config.reduction_ratio) || config.reduction_ratio <= 0 ||
            (config.mode != Mode::Stop && config.mode != Mode::Foc)) {
            return std::unexpected(Error::InvalidParameter);
        }
        if (!in_range<std::int16_t>(
                static_cast<double>(config.max_output_nm) / config.reduction_ratio * 2560.0)) {
            return std::unexpected(Error::OutOfRange);
        }
        // 保留内部 Parser 的所有者引用，仅重置本机状态。
        static_cast<MotorBase&>(*this) = MotorBase{};
        stop();
        reset_rx();
        m_initialization_error = Error::NotInitialized;
        m_feedback_mode = 0;
        m_timeout_triggered = false;
        m_winding_temperature = 0;
        m_voltage = 0;
        m_error_flags = 0;
        m_warning_flags = 0;
        m_has_feedback = false;
        m_id = config.id;
        set_max_output(config.max_output_nm);
        set_motor_reduction(config.reduction_ratio);
        m_mode = config.mode;
        m_timeout_protection = config.timeout_protection;
        return {};
    }

    /// @brief 返回电机 ID；尚未初始化时返回 0xFF。
    [[nodiscard]] std::uint8_t id() const noexcept { return static_cast<std::uint8_t>(m_id); }

    /// @brief 设置待发送的工作模式，不立即发送，也不改变已保存的反馈模式。
    [[nodiscard]]
    std::expected<void, Error> set_mode(Mode mode) noexcept {
        if (mode != Mode::Stop && mode != Mode::Foc) {
            return std::unexpected(Error::InvalidParameter);
        }
        m_mode = mode;
        return {};
    }

    /// @brief 返回待发送的控制模式。
    [[nodiscard]] Mode get_mode() const noexcept { return m_mode; }

    /// @brief 设置命令中的超时保护位；反馈触发超时后，可发送 false 清除电机超时标志。
    void set_timeout_protection(bool enabled) noexcept { m_timeout_protection = enabled; }

    /// @brief 一次设置混合控制目标；先限幅前馈力矩，再检查所有协议数值，不切换模式。
    /// @return 非有限值、负增益或超出协议范围时失败，原控制目标保持不变。
    [[nodiscard]]
    std::expected<void, Error> set_hybrid_command(const HybridCommand& command) noexcept {
        const auto result = make_command_data_tuple(command);
        if (!result) return std::unexpected(result.error());
        m_target_position = command.position_deg;
        m_target_speed = command.speed_rad_s;
        set_motor_output(command.torque_nm);
        m_kp = command.kp;
        m_kd = command.kd;
        return {};
    }

    /// @brief 设置停机模式并清零所有控制目标；调用方仍须发送生成的命令才能作用于电机。
    void stop() noexcept {
        m_mode = Mode::Stop;
        m_output = 0;
        m_target_position = 0;
        m_target_speed = 0;
        m_kp = 0;
        m_kd = 0;
    }

    /// @brief 使用当前控制目标生成独立的 20 字节命令，包含帧头、保留零字节和 CRC。
    /// @note 继承的 set_motor_output() 只修改前馈力矩，不清除位置和速度增益。
    [[nodiscard]]
    std::expected<std::array<std::uint8_t, CommandFrame::size()>, Error> build_tx() const noexcept {
        const auto result = make_command_data_tuple(
            {m_target_position, m_target_speed, m_output, m_kp, m_kd});
        if (!result) return std::unexpected(result.error());
        return CommandFrame::serialize(*result);
    }

    /// @brief Rs485Device 组帧接口，向总线提供当前命令；详细控制错误仍可通过无参 build_tx() 获取。
    [[nodiscard]] std::expected<std::size_t, Rs485Error>
    build_tx(std::span<std::uint8_t> output) const noexcept override {
        if (output.size() < CommandFrame::size()) return std::unexpected(Rs485Error::BufferSizeMismatch);
        const auto bytes = build_tx();
        if (!bytes) return std::unexpected(Rs485Error::BuildFailed);
        std::copy(bytes->begin(), bytes->end(), output.begin());
        return bytes->size();
    }

    /// @brief 接收总线提供的字节流，返回通过校验且 ID 匹配的反馈帧数。
    std::size_t feed_rx(std::span<const std::uint8_t> bytes) override {
        m_parser.m_accepted = 0;
        m_parser.feed(bytes);
        return m_parser.m_accepted;
    }

    /// @brief 丢弃接收半帧，保留已有反馈和控制目标。
    void reset_rx() noexcept override {
        m_parser.reset();
        m_parser.m_accepted = 0;
    }

    /// @brief 验证一帧完整的 26 字节反馈并更新状态；失败时保留已有反馈。
    [[nodiscard]]
    std::expected<void, Error> on_rx(std::span<const std::uint8_t> bytes) noexcept {
        const auto result = FeedbackFrame::deserialize(bytes);
        if (!result) return std::unexpected(from_frame_error(result.error()));
        return on_rx(*result);
    }

    /// @brief 接收 FrameParser 已校验的反馈数据 tuple，检查 ID 后更新状态，不重复计算 CRC。
    /// @note 首帧直接更新累计位置，不设置软件零点；停机、超时和故障反馈同样保存。
    [[nodiscard]]
    std::expected<void, Error> on_rx(const DataTuple& data_tuple) noexcept {
        if (m_id == kInvalidId) return std::unexpected(m_initialization_error);
        if (!std::isfinite(m_reduction_ratio) || m_reduction_ratio <= 0) {
            return std::unexpected(Error::InvalidParameter);
        }
        const auto& [control, driver_temperature, winding_temperature, voltage,
                     torque, speed, position, error_flags, warning_flags, reserved] = data_tuple;
        if ((control & 0x0FU) != m_id) return std::unexpected(Error::IdMismatch);
        (void)reserved;

        const double raw_position = static_cast<double>(position) * 360.0 / 32768.0;
        const double output_position = raw_position / m_reduction_ratio;
        const double raw_speed = static_cast<double>(speed) * kTwoPi / 64.0;
        const double raw_torque = static_cast<double>(torque) / 2560.0;
        if (!in_range<float>(output_position) || !in_range<float>(raw_speed / m_reduction_ratio) ||
            !in_range<float>(raw_torque * m_reduction_ratio)) {
            return std::unexpected(Error::OutOfRange);
        }
        m_raw_sum_pos = static_cast<float>(raw_position);
        m_raw_single_pos = single_position(raw_position);
        m_sum_pos = static_cast<float>(output_position);
        m_single_pos = single_position(output_position);
        m_raw_speed = static_cast<float>(raw_speed);
        m_speed = static_cast<float>(raw_speed / m_reduction_ratio);
        m_raw_torque = static_cast<float>(raw_torque);
        m_torque = static_cast<float>(raw_torque * m_reduction_ratio);
        m_temperature = static_cast<float>(driver_temperature);
        m_winding_temperature = static_cast<float>(winding_temperature);
        m_voltage = static_cast<float>(voltage) * 0.5f;
        m_feedback_mode = static_cast<std::uint8_t>((control >> 4) & 0x07U);
        m_timeout_triggered = (control & 0x80U) != 0;
        m_error_flags = error_flags;
        m_warning_flags = static_cast<std::uint8_t>((warning_flags >> 13) & 0x07U);
        m_has_feedback = true;
        return {};
    }

    /// @brief 判断是否已有通过校验且 ID 匹配的反馈，不表示当前仍在线。
    [[nodiscard]] bool has_feedback() const noexcept { return m_has_feedback; }
    /// @brief 获取反馈中的实际模式，原样保留 0～7 的数值。
    [[nodiscard]] std::uint8_t get_feedback_mode() const noexcept { return m_feedback_mode; }
    /// @brief 获取电机反馈的通信超时状态。
    [[nodiscard]] bool is_timeout() const noexcept { return m_timeout_triggered; }
    /// @brief 获取绕组温度，单位为摄氏度。
    [[nodiscard]] float get_winding_temperature() const noexcept { return m_winding_temperature; }
    /// @brief 获取电机端电压，单位为伏特。
    [[nodiscard]] float get_voltage() const noexcept { return m_voltage; }
    /// @brief 获取完整故障位掩码，保留未知故障位。
    [[nodiscard]] std::uint32_t get_error_flags() const noexcept { return m_error_flags; }
    /// @brief 获取三个警告位的掩码。
    [[nodiscard]] std::uint8_t get_warning_flags() const noexcept { return m_warning_flags; }

private:
    static constexpr std::uint8_t kInvalidId = 0xFF;
    static constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

    struct FeedbackParser : frame::FrameParser<FeedbackFrame, 64> {
        using Base = frame::FrameParser<FeedbackFrame, 64>;
        Im6014Motor& m_motor;
        std::size_t m_accepted{0};

        explicit FeedbackParser(Im6014Motor& motor) noexcept
            : Base({.on_frame = [](Base& parser, const DataTuple& data_tuple) {
                auto& receiver = static_cast<FeedbackParser&>(parser);
                if (receiver.m_motor.on_rx(data_tuple)) ++receiver.m_accepted;
            }}), m_motor(motor) {}
    };

    FeedbackParser m_parser{*this};
    Error m_initialization_error{Error::NotInitialized}; // 仅在尚未初始化时使用。
    Mode m_mode{Mode::Stop};
    bool m_timeout_protection{true};
    float m_target_position{0};
    float m_target_speed{0};
    float m_kp{0};
    float m_kd{0};
    std::uint8_t m_feedback_mode{0};
    bool m_timeout_triggered{false};
    float m_winding_temperature{0};
    float m_voltage{0};
    std::uint32_t m_error_flags{0};
    std::uint8_t m_warning_flags{0};
    bool m_has_feedback{false};

    template <typename T>
    static bool in_range(double number) noexcept {
        return std::isfinite(number) && number >= std::numeric_limits<T>::lowest() &&
               number <= std::numeric_limits<T>::max();
    }

    static float single_position(double position) noexcept {
        auto single = std::fmod(position, 360.0);
        if (single < 0) single += 360.0;
        const auto result = static_cast<float>(single);
        return result >= 360.0f ? 0.0f : result;
    }

    std::expected<CommandFrame::DataTuple, Error>
    make_command_data_tuple(const HybridCommand& command) const noexcept {
        if (m_id == kInvalidId) return std::unexpected(m_initialization_error);
        if (!std::isfinite(m_reduction_ratio) || m_reduction_ratio <= 0 ||
            !std::isfinite(m_max_output)) {
            return std::unexpected(Error::InvalidParameter);
        }
        for (const float number : std::array{command.position_deg, command.speed_rad_s,
                                             command.torque_nm, command.kp, command.kd}) {
            if (!std::isfinite(number)) return std::unexpected(Error::InvalidParameter);
        }
        if (command.kp < 0 || command.kd < 0) return std::unexpected(Error::InvalidParameter);

        const double ratio = m_reduction_ratio;
        const double limit = std::abs(static_cast<double>(m_max_output));
        const double torque = std::clamp(static_cast<double>(command.torque_nm), -limit, limit) /
                              ratio * 2560.0;
        const double speed = static_cast<double>(command.speed_rad_s) * ratio * 64.0 / kTwoPi;
        const double position = static_cast<double>(command.position_deg) * ratio * 32768.0 / 360.0;
        const double kp = static_cast<double>(command.kp) / (ratio * ratio) * 12800.0;
        const double kd = static_cast<double>(command.kd) / (ratio * ratio) * 51200.0;
        if (!in_range<std::int16_t>(torque) || !in_range<std::int16_t>(speed) ||
            !in_range<std::int32_t>(position) || !in_range<std::int16_t>(kp) ||
            !in_range<std::int16_t>(kd)) {
            return std::unexpected(Error::OutOfRange);
        }
        const auto control = static_cast<std::uint8_t>(
            m_id | (static_cast<std::uint8_t>(m_mode) << 4) | (m_timeout_protection ? 0x80U : 0U));
        return CommandFrame::DataTuple{control, static_cast<std::int16_t>(torque),
            static_cast<std::int16_t>(speed), static_cast<std::int32_t>(position),
            static_cast<std::int16_t>(kp), static_cast<std::int16_t>(kd)};
    }

    static Error from_frame_error(frame::Error error) noexcept {
        switch (error) {
        case frame::Error::HeaderMismatch: return Error::HeaderMismatch;
        case frame::Error::FooterMismatch: return Error::FooterMismatch;
        case frame::Error::LengthMismatch: return Error::LengthMismatch;
        case frame::Error::BufferSizeMismatch: return Error::BufferSizeMismatch;
        case frame::Error::CrcMismatch: return Error::CrcMismatch;
        }
        return Error::InvalidParameter;
    }
};
