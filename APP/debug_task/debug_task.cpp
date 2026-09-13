/**
 * @file debug_task.cpp
 * @brief Export runtime debug globals for Ozone inspection.
 */
#include "cmsis_os2.h"
#include "rtos_task.hpp"

void debug_task_entry(void* arg) {
    (void)arg;
    for (;;) {
        
    }
}
rtos::Task<&debug_task_entry, 128 * 4, rtos::TaskPriority::Low> debug_task("debug_task");
