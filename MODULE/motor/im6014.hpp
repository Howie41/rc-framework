#pragma once

#include "frame_codec.hpp"
#include "frame_parser.hpp"
#include "motor_base.hpp"
#include "rs485_device.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <tuple>

namespace im6014_detail {

using namespace frame;

using CheckedRegion = RegionTag<0>;

using CommandFrame = Frame<
    Begin<CheckedRegion>,                         // CRC 校验区域开始，包含命令帧头。
    Header<0xFE, 0xEE>,                            // 命令帧头。
    Field<std::uint8_t>,                           // 控制字：位 0～3 为 ID，位 4～6 为模式，位 7 为超时保护使能。
    Header<0>,                                    // 固定保留字节，发送 0。
    Field<std::int16_t>,                           // 转子侧目标力矩，原始值 / 2560 为 Nm。
    Field<std::int16_t>,                           // 转子侧目标速度，原始值 / 64 为转/秒。
    Field<std::int32_t>,                           // 转子侧目标累计位置，每转 32768 个单位。
    Field<std::int16_t>,                           // 转子侧位置刚度 Kp，编码系数为 12800。
    Field<std::int16_t>,                           // 转子侧速度阻尼 Kd，编码系数为 51200。
    End<CheckedRegion>,                           // CRC 校验区域结束，不包含 CRC 字段自身。
    Crc<CheckedRegion, CrcType::Crc32Mpeg2WordLe>    // 命令 CRC32，按小端 32 位字处理。
>;

using FeedbackFrame = Frame<
    Header<0xFC, 0xEE>,                            // 反馈帧头。
    Begin<CheckedRegion>,                         // CRC 校验区域开始，不包含反馈帧头。
    Field<std::uint8_t>,                           // 状态字：位 0～3 为 ID，位 4～6 为模式，位 7 为超时标志。
    Field<std::int8_t>,                            // 驱动器温度，摄氏度。
    Field<std::uint8_t>,                           // 电机绕组温度，摄氏度。
    Field<std::uint8_t>,                           // 母线电压，原始值 × 0.5 为 V。
    Field<std::int16_t>,                           // 转子侧反馈力矩，原始值 / 2560 为 Nm。
    Field<std::int16_t>,                           // 转子侧反馈速度，原始值 / 64 为转/秒。
    Field<std::int32_t>,                           // 转子侧反馈累计位置，每转 32768 个单位。
    Field<std::uint32_t>,                          // 故障标志位。
    Field<std::uint16_t>,                          // 告警字段，位 13～15 为告警标志。
    Field<std::uint16_t>,                          // 保留字段。
    End<CheckedRegion>,                           // CRC 校验区域结束，不包含 CRC 字段自身。
    Crc<CheckedRegion, CrcType::Crc32Mpeg2WordLe>    // 反馈 CRC32，按小端 32 位字处理。
>;

} // namespace im6014_detail

/// @brief IM6014 的输出命令、协议编解码和反馈，不直接操作串口。
/// @note 输出端位置使用度，速度使用 rad/s，力矩使用 Nm；由通信任务串行访问。
class Im6014Motor : public Rs485Device, public MotorBase {
public:
    using CommandFrame = im6014_detail::CommandFrame;
    using FeedbackFrame = im6014_detail::FeedbackFrame;
    using DataTuple = FeedbackFrame::DataTuple;

    static constexpr float kReductionRatio = 38.0f / 3.0f;

    enum class Mode : std::uint8_t { Stop = 0, Foc = 1 };

    struct OutputCommand {
        float position_deg{0}; // 输出端目标累计位置，度，与反馈原点相同。
        float speed_rad_s{0};  // 输出端目标速度，rad/s。
        float torque_nm{0};    // 输出端前馈力矩，Nm，按本电机配置的上限限幅。
        float kp{0};           // 输出端位置刚度，Nm/rad。
        float kd{0};           // 输出端阻尼，Nm/(rad/s)。
    };

    struct Config {
        std::uint8_t id{0};                     // 电机 ID，范围 0～14。
        float max_output_nm{0};                 // 非负前馈力矩上限，Nm，默认禁止非零前馈力矩。
        float reduction_ratio{kReductionRatio}; // 转子到输出端的减速比，必须为正数。
        Mode mode{Mode::Stop};                  // 初始软件使能状态对应的模式。
        bool timeout_protection{true};          // 电机自身的一秒超时保护，与主控在线判断独立。
    };

    /// @brief 创建未初始化的禁用对象，不绑定串口、不发送命令。
    Im6014Motor() noexcept {
        m_id = kInvalidId;
        set_motor_reduction(kReductionRatio);
    }

    /// @brief 构造时设置 ID 和前馈力矩上限，参数非法时保持未初始化。
    Im6014Motor(std::uint8_t id, float max_output_nm) noexcept
        : Im6014Motor(Config{.id = id, .max_output_nm = max_output_nm}) {}

    /// @brief 按配置构造，不绑定串口；参数非法时保持未初始化。
    explicit Im6014Motor(const Config& config) noexcept : Im6014Motor() {
        (void)init(config);
    }

    /// @brief 初始化为禁用模式，默认启用电机自身超时保护；失败时保留原状态。
    [[nodiscard]] bool init(std::uint8_t id, float max_output_nm) noexcept {
        return init(Config{.id = id, .max_output_nm = max_output_nm});
    }

    /// @brief 校验配置，成功后清空命令和反馈；失败时保留原状态。
    [[nodiscard]] bool init(const Config& config) noexcept {
        if (config.id > 14 || !std::isfinite(config.max_output_nm) || config.max_output_nm < 0 ||
            !std::isfinite(config.reduction_ratio) || config.reduction_ratio <= 0 ||
            (config.mode != Mode::Stop && config.mode != Mode::Foc)) return false;
        if (!in_range<std::int16_t>(
                static_cast<double>(config.max_output_nm) / config.reduction_ratio * 2560.0)) {
            return false;
        }

        disable();
        m_parser.reset();
        m_raw_single_pos = m_raw_sum_pos = m_raw_speed = m_raw_torque = 0;
        m_single_pos = m_sum_pos = m_speed = m_torque = m_temperature = 0;
        m_last_feedback_ms = 0xFFFFFFFFU;
        m_feedback_mode = 0;
        m_timeout_triggered = false;
        m_winding_temperature = m_voltage = 0;
        m_error_flags = 0;
        m_warning_flags = 0;
        m_id = config.id;
        m_max_output_nm = config.max_output_nm;
        set_motor_reduction(config.reduction_ratio);
        m_timeout_protection = config.timeout_protection;
        if (config.mode == Mode::Foc) enable();
        return true;
    }

    /// @brief 返回电机 ID；尚未初始化时返回 0xFF。
    [[nodiscard]] std::uint8_t id() const noexcept { return static_cast<std::uint8_t>(m_id); }

    /// @brief 请求 FOC 工作模式，不发送命令，也不改变已保存的反馈模式。
    void enable() noexcept override { m_enabled = true; }

    /// @brief 请求停机并清空输出命令；调用方仍需发送停机报文才能作用于硬件。
    void disable() noexcept override {
        m_enabled = false;
        m_output_command = {};
    }

    /// @brief 设置待发送模式；停机等同于 disable()，FOC 等同于 enable()。
    [[nodiscard]] bool set_mode(Mode mode) noexcept {
        if (mode == Mode::Foc) enable();
        else if (mode == Mode::Stop) disable();
        else return false;
        return true;
    }

    /// @brief 从软件使能状态获取待发送模式，与设备反馈模式分开。
    [[nodiscard]] Mode get_mode() const noexcept { return m_enabled ? Mode::Foc : Mode::Stop; }

    /// @brief 设置设备端超时保护位；false 可用于清除设备反馈的超时标志。
    void set_timeout_protection(bool enabled) noexcept { m_timeout_protection = enabled; }

    /// @brief 校验并保存混合控制目标，限幅前馈力矩，不改变使能状态。
    /// @return 非有限值、负增益或协议数值越界时返回 false，原命令保持不变。
    [[nodiscard]] bool set_output_command(const OutputCommand& command) noexcept {
        CommandFrame::DataTuple data{};
        if (!make_command_data_tuple(command, data)) return false;
        m_output_command = command;
        m_output_command.torque_nm = std::clamp(command.torque_nm, -m_max_output_nm, m_max_output_nm);
        return true;
    }

    [[nodiscard]] const OutputCommand& get_output_command() const noexcept { return m_output_command; }

    /// @brief 生成 20 字节控制报文；禁用时始终使用停机模式和零目标。
    /// @return 成功时设置实际长度，失败时长度为零且不改写 output，不直接发送。
    [[nodiscard]] bool build_tx(std::span<std::uint8_t> output,
                                std::size_t& size) const noexcept override {
        size = 0;
        if (output.size() < CommandFrame::size()) return false;
        CommandFrame::DataTuple data{};
        if (!make_command_data_tuple(m_enabled ? m_output_command : OutputCommand{}, data)) return false;
        const auto bytes = CommandFrame::serialize(data);
        std::copy(bytes.begin(), bytes.end(), output.begin());
        size = bytes.size();
        return true;
    }

    /// @brief 拼接原始回复字节，解析并校验 IM6014 反馈帧。
    void on_rx(std::span<const std::uint8_t> bytes) override {
        m_parser.feed(bytes);
    }

    /// @brief 接收已校验的反馈数据，不重复校验 CRC；停机和故障反馈也正常保存。
    [[nodiscard]] bool on_rx(const DataTuple& data) noexcept {
        if (m_id == kInvalidId || !std::isfinite(m_reduction_ratio) || m_reduction_ratio <= 0) return false;
        const auto& [
            control,
            driver_temperature,
            winding_temperature,
            voltage,
            torque,
            speed,
            position,
            error_flags,
            warning_flags,
            reserved
        ] = data;

        if ((control & 0x0FU) != m_id) return false;
        (void)reserved;

        const double raw_position = static_cast<double>(position) * 360.0 / 32768.0;
        const double output_position = raw_position / m_reduction_ratio;
        const double raw_speed = static_cast<double>(speed) * kTwoPi / 64.0;
        const double raw_torque = static_cast<double>(torque) / 2560.0;
        if (!in_range<float>(output_position) || !in_range<float>(raw_speed / m_reduction_ratio) ||
            !in_range<float>(raw_torque * m_reduction_ratio)) return false;

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
        m_last_feedback_ms = rtos::now_ms();
        return true;
    }

    /// @brief 返回反馈中的实际模式，原样保留 0～7 的数值。
    [[nodiscard]] std::uint8_t get_feedback_mode() const noexcept { return m_feedback_mode; }
    /// @brief 返回设备反馈的超时标志，与主控 is_online() 判断分开。
    [[nodiscard]] bool is_timeout() const noexcept { return m_timeout_triggered; }
    [[nodiscard]] float get_winding_temperature() const noexcept { return m_winding_temperature; }
    [[nodiscard]] float get_voltage() const noexcept { return m_voltage; }
    [[nodiscard]] std::uint32_t get_error_flags() const noexcept { return m_error_flags; }
    [[nodiscard]] std::uint8_t get_warning_flags() const noexcept { return m_warning_flags; }

private:
    using FeedbackParser = frame::FrameParser<FeedbackFrame>;
    FeedbackParser m_parser{{[](FeedbackParser& parser, const DataTuple& data) {
        (void)static_cast<Im6014Motor*>(parser.m_callback_context)->on_rx(data);
    }}, this};
    static constexpr std::uint8_t kInvalidId = 0xFF;
    static constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

    OutputCommand m_output_command{};
    float m_max_output_nm{0};
    bool m_timeout_protection{true};
    std::uint8_t m_feedback_mode{0};
    bool m_timeout_triggered{false};
    float m_winding_temperature{0};
    float m_voltage{0};
    std::uint32_t m_error_flags{0};
    std::uint8_t m_warning_flags{0};

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

    bool make_command_data_tuple(const OutputCommand& command, CommandFrame::DataTuple& data) const noexcept {
        if (m_id == kInvalidId || !std::isfinite(m_reduction_ratio) || m_reduction_ratio <= 0) return false;
        for (const float number : std::array{command.position_deg, command.speed_rad_s,
                                             command.torque_nm, command.kp, command.kd}) {
            if (!std::isfinite(number)) return false;
        }
        if (command.kp < 0 || command.kd < 0) return false;

        const double ratio = m_reduction_ratio;
        const double limit = m_max_output_nm;
        const double torque = std::clamp(static_cast<double>(command.torque_nm), -limit, limit) /
                              ratio * 2560.0;
        const double speed = static_cast<double>(command.speed_rad_s) * ratio * 64.0 / kTwoPi;
        const double position = static_cast<double>(command.position_deg) * ratio * 32768.0 / 360.0;
        const double kp = static_cast<double>(command.kp) / (ratio * ratio) * 12800.0;
        const double kd = static_cast<double>(command.kd) / (ratio * ratio) * 51200.0;
        if (!in_range<std::int16_t>(torque) || !in_range<std::int16_t>(speed) ||
            !in_range<std::int32_t>(position) || !in_range<std::int16_t>(kp) ||
            !in_range<std::int16_t>(kd)) return false;
        const auto control = static_cast<std::uint8_t>(
            m_id | (static_cast<std::uint8_t>(get_mode()) << 4) | (m_timeout_protection ? 0x80U : 0U));
        data = CommandFrame::DataTuple{control, static_cast<std::int16_t>(torque),
            static_cast<std::int16_t>(speed), static_cast<std::int32_t>(position),
            static_cast<std::int16_t>(kp), static_cast<std::int16_t>(kd)};
        return true;
    }
};
