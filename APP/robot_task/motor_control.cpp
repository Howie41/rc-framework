#include "main.h"

#include "c620.hpp"
#include "pid_controller.hpp"
#include "rtos_task.hpp"

#include <cstdint>
#include <numbers>

namespace {

constexpr float kReductionRatio = 19.0f;
constexpr float kMaxOutputMa = 2000.0f;
constexpr std::uint32_t kMotorRxId = 0x201;
constexpr std::uint32_t kMotorTxId = 0x200;
constexpr std::uint32_t kMotorIds[]{kMotorRxId};
constexpr float kRadiansPerSecondToRpm = 60.0f / (2.0f * std::numbers::pi_v<float>);
constexpr TickType_t kControlPeriodTicks = pdMS_TO_TICKS(1);
static_assert(kControlPeriodTicks > 0);

// 总线、电机与控制器配置。
CanBus g_fdcan2_bus(hfdcan2);
C620Motor g_motor(&g_fdcan2_bus, kMotorRxId, false, kMotorTxId, false);
const PidController::Config kSpeedPidConfig{
    .kp = 20.0f,
    .max_out = kMaxOutputMa,
    .improvement = {1ULL << PidController::increment_of_out},
};
PidController g_speed_pid("can2_c620_1_speed", kSpeedPidConfig);

float g_motor_target_speed_rpm{0.0f}; // 输出轴目标转速

// CAN 发送缓冲区。
CanBus::ClassicPack g_motor_tx_pack{
    .m_id = kMotorTxId,
    .m_type = CanBus::Type::Standard,
};
std::int16_t g_motor_commands[1]{};
std::uint8_t g_motor_tx_length{0};

bool motor_init() {
    if (g_fdcan2_bus.init() != 0) return false;
    g_motor.init(kReductionRatio, kMaxOutputMa);
    if (g_fdcan2_bus.register_device(&g_motor) != 0) return false;

    can_filter_init(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, kMotorRxId, 0x7FF);
    bsp_can_init(&hfdcan2);
    return true;
}

void motor_update() {
    float speed_rad_s;
    // CAN2 中断优先级为 5，临界区内只复制速度反馈。
    taskENTER_CRITICAL();
    speed_rad_s = g_motor.get_current_speed();
    taskEXIT_CRITICAL();

    const float speed_rpm = speed_rad_s * kRadiansPerSecondToRpm;
    g_motor.set_motor_output(g_speed_pid.calculate(speed_rpm, g_motor_target_speed_rpm));
}

void motor_send() {
    g_motor_commands[0] = static_cast<std::int16_t>(g_motor.output_trans());
    // 仅填入 1 号电机，其余同组槽位置零；失败帧不缓存。
    pack_dji_motor_can_msg(g_motor_tx_pack.m_id, kMotorIds, g_motor_commands, 1,
                           g_motor_tx_pack.m_data, g_motor_tx_length);
    (void)g_fdcan2_bus.add_can_msg(g_motor_tx_pack);
}

rtos::Task<"motor_control", [] {
    if (!motor_init()) {
        Error_Handler();
        return;
    }

    TickType_t next_wake_tick = rtos::now_tick();
    while (true) {
        motor_update();
        motor_send();
        next_wake_tick += kControlPeriodTicks;
        rtos::delay_until(next_wake_tick);
    }
}, 4096, rtos::TaskPriority::AboveNormal> g_motor_control_task;

} // namespace


