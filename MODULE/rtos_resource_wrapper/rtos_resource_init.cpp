#include "rtos_resource_init.h"

#include "rtos_task.hpp"
#include "rtos_queue.hpp"
#include "rtos_stream_buffer.hpp"

extern "C" void rtos_resource_init(void) {
    rtos::QueueManager::init();
    rtos::StreamBufferManager::init();
    rtos::TaskManager::init();
}
