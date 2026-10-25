#include "main.h"
#include "robot_init.h"
#include "bsp_dwt.hpp"

void robot_init() {
    // CYCCNT 使用 CPU 内核时钟，不能使用二分频后的 HCLK。
    Dwt::instance().init(SystemCoreClock / 1'000'000U);
}
