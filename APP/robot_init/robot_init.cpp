#include "robot_init.h"

#include "bsp_dwt.hpp"
#include "task_manager.hpp"
#include "pid_controller.hpp"

void robot_init() {
	Dwt::instance().init(HAL_RCC_GetHCLKFreq() / 100'0000U);
	TaskManager::instance().create_all();
}
