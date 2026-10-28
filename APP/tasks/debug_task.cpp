#include "asserter.hpp"
#include "main.h"
#include "pid_viewer.hpp"
#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "usart.h"

#include <cstdint>

namespace robot {

UartPortDma<"uart1", 64, 128, 512> uart1_debug_port(huart1, {});

} // namespace

rtos::Task<"debug_task", [] {
    PidViewer::bind(robot::uart1_debug_port);

    RUNTIME_ASSERT(robot::uart1_debug_port.start_receive_to_idle_dma() == HAL_OK);

    while (true) {
        PidViewer::heartbeat(rtos::now_ms());
        rtos::delay(1);
    }
}, 4096, rtos::TaskPriority::BelowNormal> debug_task;
