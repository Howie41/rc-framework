#pragma once

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

#include <cstdint>

namespace rtos {

/** @brief 毫秒超时参数的永久等待值 */
inline constexpr std::uint32_t wait_forever = 0xFFFFFFFFU;

/** @brief 获取当前 RTOS tick；在任务上下文中调用。 */
[[nodiscard]] inline TickType_t now_tick() noexcept {
    return xTaskGetTickCount();
}

/** @brief 获取当前 RTOS 毫秒时间；在任务上下文中调用，允许 uint32_t 自然回绕。 */
[[nodiscard]] inline std::uint32_t now_ms() noexcept {
    return static_cast<std::uint32_t>(now_tick() * portTICK_PERIOD_MS);
}

/** @brief 将毫秒超时转换为 RTOS tick，保留永久等待语义 */
[[nodiscard]] inline TickType_t timeout_ms_to_ticks(std::uint32_t timeout_ms) noexcept {
    return timeout_ms == wait_forever ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
}

/** @brief 判断当前是否处于中断上下文或中断屏蔽状态 */
[[nodiscard]] inline bool is_irq_context() noexcept {
    if (__get_IPSR() != 0U) {
        return true;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return false;
    }

    return (__get_PRIMASK() != 0U) || (__get_BASEPRI() != 0U);
}

}  // namespace rtos
