#include "asserter.hpp"
#include "main.h"
#include "pid_viewer.hpp"
#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "usart.h"

#include <cstdint>

namespace robot {

UartPortDma<"uart4", 64, 128, 512> uart4_debug_port(huart4, {});

} // namespace

rtos::Task<"debug_task", [] {
    PidViewer::bind(robot::uart4_debug_port);

    RUNTIME_ASSERT(robot::uart4_debug_port.start_receive_to_idle_dma() == HAL_OK);

    while (true) {
        PidViewer::heartbeat(rtos::now_tick());
        rtos::delay(1);
    }
}, 4096, rtos::TaskPriority::BelowNormal> debug_task;
