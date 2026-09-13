#pragma once

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include <cstdint>
#include CMSIS_device_header
#include "portmacro.h"
#include "rtos_asserter.hpp"
#include "task.h"

#include <cstddef>

namespace rtos {

/**
 * @brief 判断当前是否处于中断上下文或中断屏蔽状态
 */
[[nodiscard]] inline bool is_irq_context() noexcept {
    if (__get_IPSR() != 0U) {
        return true;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return false;
    }

    return (__get_PRIMASK() != 0U) || (__get_BASEPRI() != 0U);
}

/**
 * @brief 任务优先级
 * @note 考虑到实际需要，删除了一些细分的优先级
 */
enum class TaskPriority : UBaseType_t {
    None         = 0,  // 仅用于初始化，不能用于创建任务
    Idle         = 1,
    Low          = 8,
    BelowNormal  = 16,
    Normal       = 24,
    AboveNormal  = 32,
    High         = 40,
    Realtime     = 48,
    Isr          = 56,  // 仅用于中断服务函数中调用 RTOS，不能用于创建任务
};

class TaskBase {
    friend class TaskManager;

private:
    TaskBase* m_next{nullptr};

protected:
    TaskBase() = default;
    virtual ~TaskBase() = default;
    virtual bool init() = 0;
};

class TaskManager {
    template <void (*EntryFunction)(void*), size_t StackBytes, TaskPriority Priority>
    friend class Task;

private:
    TaskBase* m_head{nullptr};
    bool m_initialized{false};
    size_t m_count{0};

    TaskManager() = default;

    void register_task(TaskBase* task) {
        RTOS_ASSERTER_ASSERT(!m_initialized);
        task->m_next = m_head;
        m_head = task;
    }

public:
    TaskManager(const TaskManager&) = delete;
    TaskManager& operator=(const TaskManager&) = delete;

    static TaskManager& instance() {
        static TaskManager manager;
        return manager;
    }

    void init() {
        if (m_initialized) {
            return;
        }

        m_initialized = true;
        for (TaskBase* task = m_head; task != nullptr; task = task->m_next) {
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
 * @tparam EntryFunction 任务入口函数的指针，必须是 `void (*)(void*)` 类型
 * @tparam StackBytes 任务栈大小（字节），必须是 4 的倍数
 * @tparam Priority 任务优先级，例如 `rtos::TaskPriority::Normal`
 */
template <void (*EntryFunction)(void*), size_t StackBytes = 128 * 4, TaskPriority Priority = TaskPriority::Normal>
class Task : TaskBase {
    static_assert(StackBytes >= 128 * 4, "stack too small");
    static_assert(StackBytes % sizeof(StackType_t) == 0, "wrong stack size");
    static_assert(Priority >= TaskPriority::Idle, "priority too low");
    static_assert(Priority < TaskPriority::Isr, "priority too high");
    static_assert(static_cast<UBaseType_t>(Priority) < configMAX_PRIORITIES, "priority too high");

private:
    StaticTask_t m_control_block;
    StackType_t m_task_stack[StackBytes / sizeof(StackType_t)];
    const char* m_name{nullptr};
    const UBaseType_t m_priority{static_cast<UBaseType_t>(Priority)};

    TaskHandle_t m_task_handle{nullptr};

    // 在 RTOS 内部初始化任务
    bool init() override {
        if (m_task_handle != nullptr) {
            return true;
        }

        RTOS_ASSERTER_ASSERT(!is_irq_context());
        m_task_handle = xTaskCreateStatic(
            EntryFunction,
            m_name,
            static_cast<configSTACK_DEPTH_TYPE>(StackBytes / sizeof(StackType_t)),
            nullptr,
            m_priority,
            m_task_stack,
            &m_control_block);

        return (m_task_handle != nullptr);
    }

public:
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    ~Task() = default;

    /**
     * @brief 创建任务
     * @param name 任务名称，会在 RTOS 内部注册，可以在 Ozone FreeRTOS 页面看到
     */
    explicit Task(const char* name) : m_name(name) {
        TaskManager::instance().register_task(this);
    }

    /**
     * @return 任务句柄
     */
    TaskHandle_t handle() const {
        return m_task_handle;
    }
};

/**
 * @brief 将当前任务阻塞指定的系统节拍数
 */
inline void delay(uint32_t ticks) {
    if (ticks != 0U) {
        vTaskDelay(static_cast<TickType_t>(ticks));
    }
}

/**
 * @brief 将当前任务阻塞到指定的绝对系统节拍
 * @param ticks 目标系统节拍
 * @attention 不考虑 ticks 溢出
 */
inline void delay_until(uint32_t ticks) {
    TickType_t current_ticks = xTaskGetTickCount();
    const TickType_t delay_ticks = static_cast<TickType_t>(ticks) - current_ticks;

    if (delay_ticks != 0U) {
        vTaskDelayUntil(&current_ticks, delay_ticks);
    }
}

} // namespace rtos
