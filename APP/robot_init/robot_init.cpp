#include "robot_init.h"

#include "task_manager.hpp"
#include "pid_controller.hpp"

void robot_init() {
	TaskManager::instance().create_all();
}
