#include "rtos_resource_init.h"

#include "rtos_task.hpp"
#include "rtos_queue.hpp"

extern "C" void rtos_resource_init(void) {
    RtosQueueManager::instance().create_all();
    RtosTaskManager::instance().create_all();
}
