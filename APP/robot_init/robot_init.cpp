#include "robot_init.h"

#include "bsp_dwt.hpp"

void robot_init() {
	Dwt::instance().init(HAL_RCC_GetHCLKFreq() / 100'0000U);
}
