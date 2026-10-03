#include "memory_map.h"
#include "rtos_task.hpp"

RAM_D1_BSS rtos::Task<"debug_task", [] {
    for (;;) {
        rtos::delay(1000);
    }
}> debug_task;

RAM_D1_BSS rtos::Task<"debug_task", [] {
    for (;;) {
        rtos::delay(1000);
    }
}> debug_task1;
