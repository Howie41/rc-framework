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
        osDelay(250);
    }
}

RAM_D1 RtosTask<&led_test_entry, 128 * 4> led_test_task("led_test", osPriorityNormal);




RAM_D1 RtosQueue<uint8_t, 12> test_queue("test_queue");

void producer_entry(void* arg) {
    uint8_t counter = 0;
    for (;;) {
        test_queue.put(++counter);
        osDelay(1000);
    }
}
RAM_D1 RtosTask<&producer_entry, 128 * 100 * 4> producer_task("producer", osPriorityNormal);

void consumer_entry(void* arg) {
    [[maybe_unused]] static uint8_t latest_received_value = 0;
    for (;;) {
        // latest_received_value = test_queue.get(5000).value_or(0);

        auto result = test_queue.get(5000);
        if (result) {
            latest_received_value = result.value();
        } else {
            // 超时处理
            latest_received_value = 0;
        }
        
    }
}
RAM_D1 RtosTask<&consumer_entry, 128 * 4> consumer_task("consumer", osPriorityNormal);
