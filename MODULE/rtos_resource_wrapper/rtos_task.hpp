#pragma once

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "asserter.hpp"
#include "main.h"
#include "rtos_utility.hpp"
#include "static_string.hpp"
#include "task.h"

#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace rtos {

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
    template <StaticString Name, auto EntryFunction, size_t StackBytes, TaskPriority Priority>
    friend class Task;

private:
    inline static TaskBase* m_head{nullptr};
    inline static bool m_initialized{false};
    inline static size_t m_count{0};

    TaskManager() = delete;

    static void register_task(TaskBase* task) {
        RUNTIME_ASSERT(!m_initialized);
        task->m_next = m_head;
        m_head = task;
    }

public:
    static void init() {
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

    static size_t get_count() {
        return m_count;
    }
};

/**
 * @tparam Name 编译期任务名称，例如 `"led_test"`
 * @tparam EntryFunction 无捕获 Lambda，签名为 `void()`
 * @tparam StackBytes 任务栈大小（字节），必须是 4 的倍数
 * @tparam Priority 任务优先级，例如 `rtos::TaskPriority::Normal`
 */
template <StaticString Name, auto EntryFunction, size_t StackBytes = 256 * 4, TaskPriority Priority = TaskPriority::Normal>
class Task : TaskBase {
    static_assert(std::is_class_v<decltype(EntryFunction)> &&
                  std::is_convertible_v<decltype(EntryFunction), void (*)()>,
                  "task entry must be a captureless void() lambda");
    static_assert(StackBytes >= 128 * 4, "stack too small");
    static_assert(StackBytes % sizeof(StackType_t) == 0, "wrong stack size");
    static_assert(Priority >= TaskPriority::Idle, "priority too low");
    static_assert(Priority < TaskPriority::Isr, "priority too high");
    static_assert(static_cast<UBaseType_t>(Priority) < configMAX_PRIORITIES, "priority too high");

private:
    static inline void task_entry(void*) {
        EntryFunction();
    }

    StaticTask_t m_control_block;
    StackType_t m_task_stack[StackBytes / sizeof(StackType_t)];
    const UBaseType_t m_priority{static_cast<UBaseType_t>(Priority)};

    TaskHandle_t m_task_handle{nullptr};

    // 在 RTOS 内部初始化任务
    bool init() override {
        if (m_task_handle != nullptr) {
            return true;
        }

        RUNTIME_ASSERT(!is_irq_context());
        m_task_handle = xTaskCreateStatic(
            task_entry,
            Name.c_str(),
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
     */
    Task() {
        TaskManager::register_task(this);
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
