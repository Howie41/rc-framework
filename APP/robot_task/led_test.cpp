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
