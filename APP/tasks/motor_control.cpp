#include "asserter.hpp"
#include "pid_controller.hpp"
#include "robot.hpp"
#include "rtos_task.hpp"
#include "rtos_utility.hpp"
#include <atomic>

namespace {
    PidController test_motor_speed{
        "test_motor_speed",
        {
            .kp = 400.0,
            .ki = 3000.0,
            .kd = 0.0,
            .max_out = 3000,
            .improvement = {1U << PidController::Improvement::increment_of_out}
        }
    };

    std::atomic<float> db_des_speed{0};
}

rtos::Task<"pid_control", ([] {
    while (! robot::test_motor.is_online()) { rtos::delay(100); }

    test_motor_speed.init();
    robot::test_motor.enable();

    while (true) {
        auto now = rtos::now_tick();

        RUNTIME_ASSERT(robot::test_motor.is_online());

        (void) robot::test_motor.set_output_command({
            .current_ma = test_motor_speed.calculate(
                robot::test_motor.get_current_speed(),
                db_des_speed
            )
        });

        rtos::delay_until(now + 1);
    }
}), 1024 * 4, rtos::TaskPriority::Realtime> pid_control_task;

rtos::Task<"switch_speed", ([] {
    while (true) {
        db_des_speed.store(10.0);
        rtos::delay(3 * 1000);
        db_des_speed.store(0.0);
        rtos::delay(3 * 1000);
        db_des_speed.store(-10.0);
        rtos::delay(3 * 1000);
        db_des_speed.store(0.0);
        rtos::delay(3 * 1000);
    }
}), 1024 * 4, rtos::TaskPriority::Normal> switch_speed_task;
