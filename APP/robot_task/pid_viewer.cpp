#include "main.h"

#include "pid_viewer.hpp"
#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "usart.h"

#include <cstdint>

namespace {

constexpr std::uint16_t kTransmitBufferSize = 64;
constexpr std::uint16_t kReceiveBufferSize = 128;
constexpr std::uint16_t kReceiveStreamBufferSize = 512;
constexpr TickType_t kUpdatePeriodTicks = pdMS_TO_TICKS(1);
static_assert(kUpdatePeriodTicks > 0);

UartPortDma<"uart4", kTransmitBufferSize, kReceiveBufferSize, kReceiveStreamBufferSize>
    g_uart4_port(huart4, {});

rtos::Task<"pid_viewer", [] {
    PidViewer::bind(g_uart4_port);
    // 流缓冲区已在 RTOS 初始化时创建，在任务内启动 DMA 接收。
    if (g_uart4_port.start_receive_to_idle_dma() != HAL_OK) {
        Error_Handler();
        return;
    }

    TickType_t next_wake_tick = rtos::now_tick();
    while (true) {
        PidViewer::tick();
        next_wake_tick += kUpdatePeriodTicks;
        rtos::delay_until(next_wake_tick);
    }
}, 4096, rtos::TaskPriority::Normal> g_pid_viewer_task;

} // namespace
