#include "rtos_task.hpp"
#include "uart_port.hpp"

void test_entry(void* arg) {
    while (true) {
        rtos::delay(100);
    }
}
rtos::Task<&test_entry, 1024 * 4, rtos::TaskPriority::Normal>
led_test_task("led_test");
