#pragma once

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "asserter.hpp"
#include "main.h"
#include "rtos_utility.hpp"
#include "static_list.hpp"
#include "static_string.hpp"
#include "task.h"

#include <cstdint>
#include <cstddef>
#include <limits>
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
    StaticListNode<TaskBase> m_node{*this};

protected:
    TaskBase() = default;
    virtual ~TaskBase() = default;
    /**
     * @brief 在 RTOS 内部初始化任务
     * @return 初始化成功返回 `true`，否则返回 `false`
     */
    virtual bool init() = 0;
};

class TaskManager {
    template <StaticString Name, auto EntryFunction, size_t StackBytes, TaskPriority Priority>
    friend class Task;

private:
    inline static StaticList<TaskBase> m_tasks;
    inline static bool m_initialized{false};
    inline static size_t m_count{0};

    TaskManager() = delete;

    /**
     * @brief 注册待初始化的任务
     * @param task 要注册的任务对象指针
     * @attention 必须在管理器初始化之前调用
     */
    static void register_task(TaskBase* task) {
        RUNTIME_ASSERT(!m_initialized && task != nullptr);
        if (m_initialized || task == nullptr) {
            return;
        }
        const bool registered = m_tasks.push_front(&task->m_node);
        RUNTIME_ASSERT(registered);
    }

public:
    /** @brief 创建所有已注册的任务，重复调用不会再次创建 */
    static void init() {
        if (m_initialized) {
            return;
        }

        m_initialized = true;
        m_tasks.for_each([](TaskBase& task) {
            if (task.init()) {
                ++m_count;
            }
        });
    }

    /**
     * @brief 获取成功创建的任务数量
     * @return 成功创建的任务数量
     */
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
    /** @brief 调用编译期指定的任务入口函数 */
    static inline void task_entry(void*) {
        EntryFunction();
    }

    StaticTask_t m_control_block;
    StackType_t m_task_stack[StackBytes / sizeof(StackType_t)];
    const UBaseType_t m_priority{static_cast<UBaseType_t>(Priority)};

    TaskHandle_t m_task_handle{nullptr};

    /**
     * @brief 在 RTOS 内部创建任务，已创建时直接返回
     * @return 句柄有效时返回 `true`，否则返回 `false`
     * @attention 必须在非中断上下文中调用
     */
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
     * @brief 获取任务句柄
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
 * @note 目标已到达或过期时直接返回；未来目标距离须小于半个节拍计数周期。
 */
inline void delay_until(uint32_t ticks) {
    TickType_t current_ticks = xTaskGetTickCount();
    const TickType_t delay_ticks = static_cast<TickType_t>(ticks) - current_ticks;

    if (delay_ticks != 0U && delay_ticks <= std::numeric_limits<TickType_t>::max() / 2U) {
        vTaskDelayUntil(&current_ticks, delay_ticks);
    }
}

} // namespace rtos
