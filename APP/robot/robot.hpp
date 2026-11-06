#pragma once

#include "asserter.hpp"
#include "main.h"
#include "memory_map.h"
#include "rtos_utility.hpp"
#include "usart.h"
#include "can_bus.hpp"
#include "bsp_dwt.hpp"
#include "rs485_bus.hpp"
#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "c620.hpp"

namespace robot {

inline bool initialized{false};

inline CanBus can1_bus{hfdcan1};
inline CanBus can2_bus{hfdcan2};
inline CanBus can3_bus{hfdcan3};

inline auto can_communication_entry{([] {
    while (true) {
        (void) robot::can1_bus.periodic();
        rtos::delay(1);
    }
})};
RAM_D1_BSS inline rtos::Task<
    "can_communication",
    can_communication_entry,
    2048 * 4,
    rtos::TaskPriority::Realtime
> can_communication_task;

inline C620 can1_c620{can1_bus};
inline C620Motor test_motor{can1_c620, 1};

inline UartPortDma<"uart2", 32, 32, 32 * 2> uart2_port{huart2, {}};
inline Rs485Bus rs4851_bus{uart2_port};
// inline Im6014Motor test_motor{0x00, 0};

/**
 * @brief 初始化 DWT 并启动三路 CAN；CAN 设备已在构造时自动注册。
 * @note 由 MX_FREERTOS_Init() 在 rtos_resource_init() 完成后、调度器启动前调用；
 *       此时外设、队列、流缓冲区和任务已创建，业务任务尚未开始执行。
 * @note 不得调用任务延时或阻塞等待等依赖调度器运行的操作。
 * @note 初始化完成后设置 initialized，重复调用直接返回。
 */
inline void init() {
    if (initialized) return;

    // TODO(ETH): 在链接脚本中将 .RxDescripSection / .TxDescripSection 收入 RAM_D2，
    // 分别按 32 字节对齐并纳入启动清零范围；核对 map，避免与 UART DMA 缓冲重叠。
    // 接入收发时还需检查数据缓冲位置，启用 D-cache 时补齐 MPU 属性或缓存维护。
    // TODO(ETH): 按硬件确认 ETH_RST 极性、复位时序及参考时钟，在 MX_ETH_Init() 前释放复位。
    // MX_ETH_Init() 当前在 main() 中先于本函数执行，复位处理需前移；暂不使用 ETH 时暂缓初始化。

    // CYCCNT 使用 CPU 内核时钟，不能使用二分频
    // 后的 HCLK。
    Dwt::instance().init(SystemCoreClock / 1'000'000U);

    // 各总线依次建立映射、配置过滤器并启动，保留构造阶段注册的设备。
    for (auto *bus : {&can1_bus, &can2_bus, &can3_bus}) {
        const bool ready = bus->init();
        RUNTIME_ASSERT(ready);
    }

    initialized = true;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

};
