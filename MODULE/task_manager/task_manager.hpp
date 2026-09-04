#pragma once

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "memory_map.h"

#include <cstddef>

class TaskBase {
protected:
    TaskBase() = default;
    virtual ~TaskBase() = default;
    virtual void create() = 0;

private:
    TaskBase* next_{nullptr};
    friend class TaskManager;
};

class TaskManager {
public:
    TaskManager(const TaskManager&) = delete;
    TaskManager& operator=(const TaskManager&) = delete;

    static TaskManager& instance() {
        static TaskManager manager;
        return manager;
    }

    void register_task(TaskBase* task) {
        task->next_ = head_;
        head_ = task;
    }

    void create_all() {
        for (TaskBase* task = head_; task != nullptr; task = task->next_) {
            task->create();
        }
    }

private:
    TaskManager() = default;
    TaskBase* head_ = nullptr;
};

template <void (*entry_function)(void*), size_t stack_bytes = 128 * 4>
class Task : TaskBase {
    static_assert(stack_bytes >= 128 * 4, "stack too small");
    static_assert(stack_bytes % sizeof(StackType_t) == 0, "wrong stack size");
public:
    explicit Task(const char* task_name, osPriority_t priority = osPriorityNormal) {
        task_attribute_.name = task_name;
        task_attribute_.cb_mem = &control_block_;
        task_attribute_.cb_size = sizeof(control_block_);
        task_attribute_.stack_mem = task_stack_;
        task_attribute_.stack_size = sizeof(task_stack_);
        task_attribute_.priority = priority;
        TaskManager::instance().register_task(this);
    }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    ~Task() = default;

    void create() override {
        task_handle_ = osThreadNew(entry_function, nullptr, &task_attribute_);
    }

    osThreadAttr_t get_task_attribute() const {
        return task_attribute_;
    }

    osThreadId_t get_task_handle() const {
        return task_handle_;
    }

private:
    inline static RAM_D1_ATTR StaticTask_t control_block_;
    inline static RAM_D1_ATTR StackType_t task_stack_[stack_bytes / sizeof(StackType_t)];
    osThreadAttr_t task_attribute_{};
    osThreadId_t task_handle_{nullptr};
};
