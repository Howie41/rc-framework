#include "task_manager.hpp"

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"

void led_test_entry(void* arg) {
    (void)arg;
    for (;;) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
        osDelay(250);
    }
}

Task<&led_test_entry, 128 * 4> led_test_task("led_test", osPriorityNormal);