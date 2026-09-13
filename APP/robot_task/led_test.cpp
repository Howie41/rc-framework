#include "rtos_asserter.hpp"
#include "rtos_task.hpp"
#include "UartPort.hpp"
#include "memory_map.h"
#include "pid_controller.hpp"
#include "pid_viewer.hpp"
#include "rtos_queue.hpp"

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "usart.h"

void led_test_entry(void* arg) {
    (void)arg;
    for (;;) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
        rtos::delay((250U));
    }
}

RAM_D1 rtos::Task<&led_test_entry, 128 * 4, rtos::TaskPriority::Normal> led_test_task("led_test");




RAM_D1 rtos::Queue<uint8_t, 12> test_queue;

void producer_entry(void* arg) {
    uint8_t counter = 0;
    for (;;) {
        RTOS_ASSERTER_ASSERT(counter <= 2);
        test_queue.put(++counter);
        rtos::delay((1000U));
    }
}
RAM_D1 rtos::Task<&producer_entry, 128 * 100 * 4, rtos::TaskPriority::Normal>
producer_task("producer");

void consumer_entry(void* arg) {
    [[maybe_unused]] static uint8_t latest_received_value = 0;
    for (;;) {
        // latest_received_value = test_queue.get(5000).value_or(0);

        auto result = test_queue.get(500);
        if (result) {
            latest_received_value = result.value();
        } else {
            // 超时处理
            latest_received_value = 0;
        }
    }
}
RAM_D1 rtos::Task<&consumer_entry, 128 * 4, rtos::TaskPriority::Normal> consumer_task("consumer");
