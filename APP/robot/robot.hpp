#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 供 C 启动代码调用，在外设初始化完成后初始化机器人。 */
void robot_init(void);

#ifdef __cplusplus
}

#include "can_bus.hpp"
#include "bsp_dwt.hpp"

class Robot {
public:
    Robot() = delete;

    inline static CanBus can1_bus{hfdcan1};
    inline static CanBus can2_bus{hfdcan2};
    inline static CanBus can3_bus{hfdcan3};
    inline static bool initialized{false};

    /** @brief 在外设初始化完成后、调度器启动前初始化 DWT 和三路 CAN。 */
    static void init() {
        if (initialized) return;

        // CYCCNT 使用 CPU 内核时钟，不能使用二分频后的 HCLK。
        Dwt::instance().init(SystemCoreClock / 1'000'000U);

        // 先建立全部映射，再启动外设，保证中断能找到对应的总线。
        for (auto *bus : {&can1_bus, &can2_bus, &can3_bus}) {
            if (bus->init() != 0) {
                Error_Handler();
                return;
            }
        }

        // 接收全部标准及扩展数据帧，由 CanDevice 按 ID 进行软件分发。
        for (auto *handle : {&hfdcan1, &hfdcan2, &hfdcan3}) {
            can_filter_init(handle, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
            can_filter_init(handle, FDCAN_EXTENDED_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
            bsp_can_init(handle);
        }

        initialized = true;
    }
};

#endif // __cplusplus
