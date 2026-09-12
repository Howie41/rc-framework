#pragma once

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <cstddef>

class RtosTaskBase {
    friend class RtosTaskManager;

private:
    RtosTaskBase* m_next{nullptr};

protected:
    RtosTaskBase() = default;
    virtual ~RtosTaskBase() = default;
    virtual bool init() = 0;
};

class RtosTaskManager {
private:
    RtosTaskBase* m_head{nullptr};
    bool m_initialized{false};
    size_t m_count{0};

    RtosTaskManager() = default;

public:
    RtosTaskManager(const RtosTaskManager&) = delete;
    RtosTaskManager& operator=(const RtosTaskManager&) = delete;

    static RtosTaskManager& instance() {
        static RtosTaskManager manager;
        return manager;
    }

    void register_task(RtosTaskBase* task) {
        configASSERT(!m_initialized);
        task->m_next = m_head;
        m_head = task;
    }

    void create_all() {
        if (m_initialized) {
            return;
        }

        m_initialized = true;
        for (RtosTaskBase* task = m_head; task != nullptr; task = task->m_next) {
            if (task->init()) {
                ++m_count;
            }
        }
    }

    size_t get_count() const {
        return m_count;
    }
};

/**
 * @note Instances register their addresses with RtosTaskManager and therefore
 *       must have static storage duration.
 */
template <void (*EntryFunction)(void*), size_t StackBytes = 128 * 4>
class RtosTask : RtosTaskBase {
    static_assert(StackBytes >= 128 * 4, "stack too small");
    static_assert(StackBytes % sizeof(StackType_t) == 0, "wrong stack size");
private:
    StaticTask_t m_control_block;
    StackType_t m_task_stack[StackBytes / sizeof(StackType_t)];
    const char* m_name{nullptr};
    const osPriority_t m_priority{osPriorityNormal};
    osThreadId_t m_task_handle{nullptr};

public:
    explicit RtosTask(const char* name, osPriority_t priority = osPriorityNormal)
        : m_name(name), m_priority(priority) {
        RtosTaskManager::instance().register_task(this);
    }

    RtosTask(const RtosTask&) = delete;
    RtosTask& operator=(const RtosTask&) = delete;

    ~RtosTask() = default;

private:
    bool init() override {
        if (m_task_handle != nullptr) {
            return true;
        }

        osThreadAttr_t attributes{};
        attributes.name = m_name;
        attributes.cb_mem = &m_control_block;
        attributes.cb_size = sizeof(m_control_block);
        attributes.stack_mem = m_task_stack;
        attributes.stack_size = sizeof(m_task_stack);
        attributes.priority = m_priority;

        m_task_handle = osThreadNew(EntryFunction, nullptr, &attributes);
        return m_task_handle != nullptr;
    }

public:
    osThreadId_t get_task_handle() const {
        return m_task_handle;
    }
};
