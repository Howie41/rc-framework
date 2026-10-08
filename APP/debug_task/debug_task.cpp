#include "main.h"
#include "memory_map.h"
#include "rtos_task.hpp"
#include "stm32h7xx_hal_def.h"
#include "uart_port.hpp"
#include "usart.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

namespace {

std::atomic<std::size_t> echo_bytes{0};
std::atomic<std::size_t> received_bytes{0};

UartPortDma<"uart4", 64, 64, 64 * 2> uart4_port(
    huart4,
    {
        .on_receive_to_idle_chunk = [](UartPort&, auto chunk_span) {
            received_bytes.fetch_add(chunk_span.size(), std::memory_order_relaxed);
        }
    }
);


auto debug_task_entry = []() {
    uart4_port.start_receive_to_idle_dma();
    while (true) {
        std::array<std::uint8_t, 64> buffer;
        const auto received = uart4_port.block_for_received_chunk(buffer);
        if (received > 0) {
            auto result = uart4_port.transmit(std::span<const std::uint8_t>(buffer.data(), received), HAL_MAX_DELAY);
            if (result == HAL_OK) {
                echo_bytes.fetch_add(received, std::memory_order_relaxed);
            }
        }
    }
};
rtos::Task<"debug_task", debug_task_entry, 512 * 4, rtos::TaskPriority::Low> debug_task;

}
