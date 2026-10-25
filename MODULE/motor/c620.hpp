#pragma once

#include "can_bus.hpp"
#include "motor_base.hpp"
#include "frame_codec.hpp"

#include <algorithm>
#include <array>
#include <bit>
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

inline std::int8_t dji_slot_from_ids(std::uint32_t tx_id, std::uint32_t rx_id) {
    if (tx_id == 0x200 && rx_id >= 0x201 && rx_id <= 0x204)
        return static_cast<std::int8_t>(rx_id - 0x201);
    if (tx_id == 0x1FF && rx_id >= 0x205 && rx_id <= 0x208)
        return static_cast<std::int8_t>(rx_id - 0x205);
    return -1;
}

} // namespace motor_detail

enum DjiMotorCanGroup {
    Group1 = 0, // 0x1FF
    Group2      // 0x200
};

/**
 * @brief 按原逻辑聚合四路电机输出，用 frame_codec 编码为 8 字节大端数据。
 * @note 0 ID、非法 ID 和不属于本组的 ID 被忽略，重复槽位保留第一次写入值。
 * @param outputs 与 motor_ids 对应的 int16 协议输出值。
 * @param data 至少 8 字节的输出缓冲区。
 * @param len 固定输出为 8；不发送报文。
 */
inline void pack_dji_motor_can_msg(std::uint32_t tx_id, const std::uint32_t motor_ids[],
                                   const std::int16_t outputs[], std::uint8_t motor_count,
                                   std::uint8_t data[8], std::uint8_t& len) {
    std::array<std::int16_t, 4> slots{};
    std::uint8_t occupied_slots = 0;
    for (std::uint8_t i = 0; i < motor_count; ++i) {
        if (motor_ids[i] == 0) continue;
        const auto slot = motor_detail::dji_slot_from_ids(tx_id, motor_ids[i]);
        if (slot < 0) continue;
        const auto index = static_cast<std::size_t>(slot);
        const auto mask = static_cast<std::uint8_t>(1U << index);
        if ((occupied_slots & mask) != 0) continue;
        slots[index] = outputs[i];
        occupied_slots |= mask;
    }
    const auto bytes = motor_detail::DjiMotorOutputs::serialize(
        std::tuple{slots[0], slots[1], slots[2], slots[3]});
    std::copy(bytes.begin(), bytes.end(), data);
    len = static_cast<std::uint8_t>(bytes.size());
}

/** @brief 原 C620 电调驱动；保留双继承、反馈状态和四路聚合发送方式。 */
class C620Motor : public CanDevice, public MotorBase {
public:
    /** @brief 保存原实现的总线指针、接收和发送 ID 参数，不自动注册设备。 */
    
    C620Motor(CanBus* manager, std::uint32_t id, bool is_extid,
              std::uint32_t tx_id, bool tx_is_extid)
        : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

    /** @brief 设置减速比与输出上限；输出沿用原实现的毫安数值，默认 20000。 */
    void init(float reduction_ratio = 19, float max_output = 20000.0f) {
        set_motor_reduction(reduction_ratio);
        set_max_output(max_output);
    }

    /** @brief 用 frame_codec 解析前 8 字节反馈，保留原版首帧基准和跨圈计算顺序。 */
    void on_rx(const std::uint8_t data[8], std::uint8_t len) override {
        if (len < motor_detail::C620Feedback::size()) return;
        const auto decoded = motor_detail::C620Feedback::deserialize(
            std::span<const std::uint8_t>{data, motor_detail::C620Feedback::size()});
        if (!decoded) return;
        const auto [encoder, speed, current, temperature, reserved] = *decoded;
        (void)reserved;
        m_encoder = encoder;
        if (m_is_encoder_init) {
            const auto delta = static_cast<std::int32_t>(m_encoder) - m_last_encoder;
            if (delta < -4096) ++m_round_count;
            else if (delta > 4096) --m_round_count;
            const std::int32_t total_encoder = m_round_count * 8192 + m_encoder - m_encoder_offset;
            m_raw_single_pos = static_cast<float>(m_encoder) / kEncoderAngleRatio;
            m_raw_sum_pos = static_cast<float>(total_encoder) / kEncoderAngleRatio;
            m_sum_pos = m_raw_sum_pos / m_reduction_ratio;
            m_single_pos = std::fmod(m_sum_pos, 360.0f);
            if (m_single_pos < 0.0f) m_single_pos += 360.0f;
        } else {
            m_encoder_offset = m_encoder;
            m_is_encoder_init = true;
        }
        m_last_encoder = m_encoder;
        m_raw_speed = static_cast<float>(speed) * kRpmToRadiansPerSecond;
        m_speed = m_raw_speed / m_reduction_ratio;
        // 保留原版数值换算；不新增电流标定或改写原有力矩系数。
        m_raw_torque = static_cast<float>(current);
        m_torque = m_raw_torque * kCurrentToTorque * m_reduction_ratio;
        m_temperature = static_cast<float>(temperature);
    }

    /** @brief 按原比例将输出换算为协议值，20000 对应 16384。 */
    [[nodiscard]] float output_trans() { return m_output * 16384.0f / 20000.0f; }

    /** @brief C620 不独立组帧，始终返回 false，长度置零，由应用聚合发送。 */
    [[nodiscard]] bool build_tx(std::uint8_t data[8], std::uint8_t& len) override {
        (void)data;
        len = 0;
        return false;
    }

private:
    bool m_is_encoder_init{false};
    std::uint16_t m_encoder_offset{0};
    std::uint16_t m_encoder{0};
    std::uint16_t m_last_encoder{0};
    std::int32_t m_round_count{0};
    static constexpr float kEncoderAngleRatio = 8192.0f / 360.0f;
    static constexpr float kRpmToRadiansPerSecond = 0.104719755f;
    static constexpr float kCurrentToTorque = 0.2f;
};
