#include "main.h"

#include "c620.hpp"
#include "pid_controller.hpp"
#include "rtos_task.hpp"

#include <cstdint>
#include <numbers>

namespace {

constexpr float kReductionRatio = 19.0f;
constexpr float kMaxOutputMa = 2000.0f;
constexpr float kRadiansPerSecondToRpm = 60.0f / (2.0f * std::numbers::pi_v<float>);
constexpr TickType_t kControlPeriodTicks = pdMS_TO_TICKS(1);
static_assert(kControlPeriodTicks > 0);

CanBus g_can_bus(hfdcan2);
C620Motor g_motor(&g_can_bus, 0x201, false, 0x200, false);
PidController g_speed_pid("can2_c620_1_speed", {
    .kp = 20.0f, // mA/RPM，实机联调初值；Ki、Kd 默认零
    .max_out = kMaxOutputMa,
    .improvement = {1ULL << PidController::increment_of_out},
});

// volatile 用于调试器观察和修改，不作为任务之间的同步机制。
volatile float g_motor_target_speed_rpm{0.0f}; // 输出轴目标转速，由调试器修改
volatile float g_motor_speed_rpm{0.0f}; // 输出轴测量转速
volatile float g_motor_output_ma{0.0f}; // 最近一次速度环输出，单位 mA
volatile std::uint8_t g_motor_send_result{0}; // 最近一次发送结果：0 接受，1 失败或忙

rtos::Task<"motor_control", [] {
    if (g_can_bus.init() != 0) {
        Error_Handler();
        return;
    }
    g_motor.init(kReductionRatio, kMaxOutputMa);
    if (g_can_bus.register_device(&g_motor) != 0) {
        Error_Handler();
        return;
    }

    can_filter_init(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0x201, 0x7FF);
    bsp_can_init(&hfdcan2);

    CanBus::ClassicPack pack{};
    pack.m_id = 0x200;
    pack.m_type = CanBus::Type::Standard;
    const std::uint32_t motor_ids[]{g_motor.id()};
    TickType_t next_wake_tick = rtos::now_tick();

    while (true) {
        float speed_rad_s;
        // CAN2 中断优先级为 5，任务临界区屏蔽其更新，仅在此复制单个速度值。
        taskENTER_CRITICAL();
        speed_rad_s = g_motor.get_current_speed();
        taskEXIT_CRITICAL();

        const float speed_rpm = speed_rad_s * kRadiansPerSecondToRpm;
        const float target_rpm = g_motor_target_speed_rpm;
        const float output_ma = g_speed_pid.calculate(speed_rpm, target_rpm);
        g_motor.set_motor_output(output_ma);
        g_motor_speed_rpm = speed_rpm;
        g_motor_output_ma = g_motor.m_output;

        const std::int16_t outputs[]{static_cast<std::int16_t>(g_motor.output_trans())};
        std::uint8_t len = 0;
        // 仅填入 1 号电机，其余同组槽位置零；失败帧不缓存。
        pack_dji_motor_can_msg(pack.m_id, motor_ids, outputs, 1, pack.m_data, len);
        g_motor_send_result = g_can_bus.add_can_msg(pack);

        next_wake_tick += kControlPeriodTicks;
        rtos::delay_until(next_wake_tick);
    }
}, 4096, rtos::TaskPriority::AboveNormal> g_motor_control_task;

} // namespace


