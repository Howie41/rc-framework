#pragma once

#include "can_bus.hpp"
#include "motor_base.hpp"
#include "frame_codec.hpp"
#include "asserter.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <tuple>

namespace motor_detail {

using C620Feedback = frame::Frame<
    frame::Field<std::uint16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::uint8_t>, frame::Field<std::uint8_t>>;

using DjiMotorOutputs = frame::Frame<
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>>;


} // namespace motor_detail

class C620Motor;

/** @brief C620 协议设备：统一管理 1～8 号电机，每周期构建两帧控制命令并分发反馈。 */
class C620 : public CanDevice {
public:
    static constexpr std::uint8_t kMinMotorId = 1;
    static constexpr std::uint8_t kMaxMotorId = 8;
    static constexpr std::size_t kMotorCount = kMaxMotorId - kMinMotorId + 1;
    static constexpr std::size_t kMotorsPerFrame = 4;
    static constexpr std::size_t kControlFrameCount = kMotorCount / kMotorsPerFrame;
    static constexpr std::size_t kCanFrameSize = motor_detail::DjiMotorOutputs::size();
    static constexpr std::uint32_t kFeedbackIdBase = 0x200U;
    static constexpr std::uint32_t kTxIdMotors1To4 = 0x200U;
    static constexpr std::uint32_t kTxIdMotors5To8 = 0x1FFU;
    static constexpr float kDefaultReductionRatio = 19.0f;
    static constexpr float kMaxCurrentMa = 20000.0f;
    static constexpr float kMaxProtocolOutput = 16384.0f;

    /** @brief 注册一个协议设备到总线，不操作硬件；同一总线只创建一个 C620。 */
    explicit C620(CanBus& bus) : CanDevice(bus, kFeedbackIdBase + kMinMotorId, false) {}

    /** @brief 电机构造时登记，按电调 ID 保存指针；同指针重复登记成功，拒绝重复 ID。 */
    [[nodiscard]] bool register_motor(C620Motor* motor);

    /** @brief 接收整个 C620 标准帧反馈范围，不要求对应电机已经绑定。 */
    [[nodiscard]] bool match_rx(const FDCAN_RxHeaderTypeDef& header) const override {
        return header.IdType == FDCAN_STANDARD_ID &&
               header.Identifier >= kFeedbackIdBase + kMinMotorId &&
               header.Identifier <= kFeedbackIdBase + kMaxMotorId;
    }

    /** @brief 根据接收 ID 分发并更新对应电机反馈，未绑定电机的反馈直接忽略。 */
    void on_rx(std::uint32_t rx_id, const std::uint8_t data[kCanFrameSize], std::uint8_t len) override;

    /** @brief 读取八个电机目标并分别提交两帧；未绑定或禁用的槽位置零，两帧都成功才返回 true。 */
    [[nodiscard]] bool build_tx(CanBus& bus) override;

private:
    static constexpr std::array<std::uint32_t, kControlFrameCount> kControlTxIds{
        kTxIdMotors1To4, kTxIdMotors5To8};
    static constexpr std::int32_t kEncoderCountsPerTurn = 8192;
    static constexpr std::int32_t kEncoderHalfTurn = kEncoderCountsPerTurn / 2;
    static constexpr float kDegreesPerTurn = 360.0f;
    static constexpr float kEncoderAngleRatio = static_cast<float>(kEncoderCountsPerTurn) / kDegreesPerTurn;
    static constexpr float kRpmToRadiansPerSecond = 0.104719755f;
    static constexpr float kCurrentToTorque = 0.2f;

    std::array<C620Motor*, kMotorCount> m_motors{};
};

/** @brief 单个 C620 电机的状态、配置及输出目标，不继承 CanDevice，不负责 CAN 编解码。 */
class C620Motor : public MotorBase {
public:
    struct OutputCommand {
        float current_ma{0}; // 电流目标，单位 mA，由外部 PID 等控制器计算。
    };

    /** @brief 接收所属 C620 和电调 ID（1～8），构造末尾自动登记，不保存 C620 或 Bus 引用。 */
    C620Motor(C620& controller, std::uint8_t motor_id) : m_id(motor_id) {
        RUNTIME_ASSERT(motor_id >= C620::kMinMotorId && motor_id <= C620::kMaxMotorId);
        set_motor_reduction(C620::kDefaultReductionRatio);
        const bool registered = controller.register_motor(this);
        RUNTIME_ASSERT(registered);
    }

    C620Motor(const C620Motor&) = delete;
    C620Motor& operator=(const C620Motor&) = delete;
    C620Motor(C620Motor&&) = delete;
    C620Motor& operator=(C620Motor&&) = delete;

    /** @brief 获取电调设置的 ID（1～8），不是 CAN 反馈 ID。 */
    [[nodiscard]] std::uint8_t id() const noexcept { return m_id; }

    /** @brief 设置减速比与毫安电流上限，不重置反馈。 */
    void init(float reduction_ratio = C620::kDefaultReductionRatio, float max_output = C620::kMaxCurrentMa) {
        RUNTIME_ASSERT(std::isfinite(reduction_ratio) && reduction_ratio > 0);
        RUNTIME_ASSERT(std::isfinite(max_output) && max_output >= 0 && max_output <= C620::kMaxCurrentMa);
        set_motor_reduction(reduction_ratio);
        m_max_output_ma = max_output;
        m_output_command.current_ma = std::clamp(m_output_command.current_ma, -max_output, max_output);
    }

    /** @brief 允许输出；实际控制帧由 C620 在总线 heartbeat 时构建。 */
    void enable() noexcept override { m_enabled = true; }

    /** @brief 禁止输出并清空电流目标，停机仍需继续周期发送零电流。 */
    void disable() noexcept override {
        m_enabled = false;
        m_output_command = {};
    }

    /** @brief 保存限幅后的电流目标；非有限值返回 false 且保留原命令。 */
    [[nodiscard]] bool set_output_command(const OutputCommand& command) noexcept {
        if (!std::isfinite(command.current_ma)) return false;
        m_output_command.current_ma = std::clamp(command.current_ma, -m_max_output_ma, m_max_output_ma);
        return true;
    }

    [[nodiscard]] const OutputCommand& get_output_command() const noexcept { return m_output_command; }

private:
    friend class C620;

    const std::uint8_t m_id;
    OutputCommand m_output_command{};
    float m_max_output_ma{C620::kMaxCurrentMa};
    bool m_is_encoder_init{false};
    std::uint16_t m_encoder_offset{0};
    std::uint16_t m_last_encoder{0};
    std::int32_t m_round_count{0};
};

inline bool C620::register_motor(C620Motor* motor) {
    if (motor == nullptr) return false;
    const auto index = static_cast<std::size_t>(motor->id() - kMinMotorId);
    auto& slot = m_motors[index];
    if (slot == motor) return true;
    if (slot != nullptr) return false;
    slot = motor;
    return true;
}

inline bool C620::build_tx(CanBus& bus) {
    std::array<std::array<std::int16_t, kMotorsPerFrame>, kControlFrameCount> outputs{};
    for (std::size_t i = 0; i < m_motors.size(); ++i) {
        const auto* motor = m_motors[i];
        if (motor == nullptr || !motor->is_enabled()) continue;
        outputs[i / kMotorsPerFrame][i % kMotorsPerFrame] = static_cast<std::int16_t>(
            motor->get_output_command().current_ma * kMaxProtocolOutput / kMaxCurrentMa);
    }

    bool success = true;
    for (std::size_t i = 0; i < kControlFrameCount; ++i) {
        const auto& slots = outputs[i];
        const auto bytes = motor_detail::DjiMotorOutputs::serialize(
            std::tuple{slots[0], slots[1], slots[2], slots[3]});
        CanBus::ClassicPack pack{};
        pack.m_id = kControlTxIds[i];
        pack.m_type = CanBus::Type::Standard;
        std::copy(bytes.begin(), bytes.end(), pack.m_data);
        if (bus.add_can_msg(pack) != 0) success = false;
    }
    return success;
}

inline void C620::on_rx(std::uint32_t rx_id, const std::uint8_t data[kCanFrameSize], std::uint8_t len) {
    if (rx_id < kFeedbackIdBase + kMinMotorId || rx_id > kFeedbackIdBase + kMaxMotorId ||
        len < motor_detail::C620Feedback::size()) return;
    auto* motor = m_motors[rx_id - kFeedbackIdBase - kMinMotorId];
    if (motor == nullptr) return;
    const auto decoded = motor_detail::C620Feedback::deserialize(
        std::span<const std::uint8_t>{data, motor_detail::C620Feedback::size()});
    if (!decoded) return;
    const auto [encoder, speed, current, temperature, reserved] = *decoded;
    (void)reserved;
    if (encoder >= kEncoderCountsPerTurn || !std::isfinite(motor->m_reduction_ratio) ||
        motor->m_reduction_ratio <= 0) return;

    if (motor->m_is_encoder_init) {
        const auto delta = static_cast<std::int32_t>(encoder) - motor->m_last_encoder;
        if (delta < -kEncoderHalfTurn) ++motor->m_round_count;
        else if (delta > kEncoderHalfTurn) --motor->m_round_count;
        const std::int32_t total_encoder = motor->m_round_count * kEncoderCountsPerTurn +
                                           encoder - motor->m_encoder_offset;
        motor->m_raw_single_pos = static_cast<float>(encoder) / kEncoderAngleRatio;
        motor->m_raw_sum_pos = static_cast<float>(total_encoder) / kEncoderAngleRatio;
        motor->m_sum_pos = motor->m_raw_sum_pos / motor->m_reduction_ratio;
        motor->m_single_pos = std::fmod(motor->m_sum_pos, kDegreesPerTurn);
        if (motor->m_single_pos < 0.0f) motor->m_single_pos += kDegreesPerTurn;
    } else {
        motor->m_encoder_offset = encoder;
        motor->m_is_encoder_init = true;
    }
    motor->m_last_encoder = encoder;
    motor->m_raw_speed = static_cast<float>(speed) * kRpmToRadiansPerSecond;
    motor->m_speed = motor->m_raw_speed / motor->m_reduction_ratio;
    // 保留原有反馈换算，不在此次结构调整中新增电流或力矩标定。
    motor->m_raw_torque = static_cast<float>(current);
    motor->m_torque = motor->m_raw_torque * kCurrentToTorque * motor->m_reduction_ratio;
    motor->m_temperature = static_cast<float>(temperature);
    const auto tick = rtos::is_irq_context() ? xTaskGetTickCountFromISR() : rtos::now_tick();
    motor->m_last_feedback_ms = static_cast<std::uint32_t>(tick * portTICK_PERIOD_MS);
}
