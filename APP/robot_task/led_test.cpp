#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "static_list.hpp"

rtos::Task<"led_test", [] {
    while (true) {
        rtos::delay(100);
    }
}, 1024 * 4, rtos::TaskPriority::Normal> led_test_task;
