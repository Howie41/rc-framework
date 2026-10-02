#pragma once

#include "main.h"

#include <cstdint>

/** @brief PWM 输出控制类。 */
class Pwm {
public:
    constexpr static std::uint32_t channel_1 = TIM_CHANNEL_1;
    constexpr static std::uint32_t channel_2 = TIM_CHANNEL_2;
    constexpr static std::uint32_t channel_3 = TIM_CHANNEL_3;
    constexpr static std::uint32_t channel_4 = TIM_CHANNEL_4;
    constexpr static std::uint32_t channel_5 = TIM_CHANNEL_5;
    constexpr static std::uint32_t channel_6 = TIM_CHANNEL_6;
    constexpr static std::uint32_t channel_all = TIM_CHANNEL_ALL;

private:
    TIM_HandleTypeDef& m_tim_handle;
    std::uint32_t m_channel;
    std::uint32_t m_timer_clock_hz;
public:
    /**
     * @brief 关联已初始化的定时器及其 PWM 通道。
     * @param handle 定时器句柄，使用期间必须保持有效。
     * @param channel PWM 通道，使用 channel_1 至 channel_6 中的一个。
     * @param timer_clock_hz 定时器输入时钟频率，单位为赫兹，指预分频器之前的时钟。
     */
    Pwm(TIM_HandleTypeDef& handle, std::uint32_t channel, std::uint32_t timer_clock_hz)
    : m_tim_handle(handle), m_channel(channel), m_timer_clock_hz(timer_clock_hz) {}

    /**
     * @brief 启动当前通道的 PWM 输出。
     * @return HAL 操作结果。
     */
    HAL_StatusTypeDef start() {
        return HAL_TIM_PWM_Start(&m_tim_handle, m_channel);
    }

    /**
     * @brief 停止当前通道的 PWM 输出。
     * @return HAL 操作结果。
     */
    HAL_StatusTypeDef stop() {
        return HAL_TIM_PWM_Stop(&m_tim_handle, m_channel);
    }

    /**
     * @brief 以毫秒为单位设置 PWM 周期，并保持当前通道的占空比。
     * @details 自动计算预分频器和自动重装载值，取最接近目标的可表示周期。
     *          周期属于整个定时器，修改后同一定时器的其他通道也会受到影响。
     *          此操作会触发更新事件，使新配置立即生效并重新开始计数。
     * @param milliseconds 目标周期，单位为毫秒，必须大于零。
     * @return 成功返回 true；时钟无效或目标周期无法表示时返回 false。
     */
    bool set_period(std::uint32_t milliseconds) {
        if (milliseconds == 0U || m_timer_clock_hz == 0U) {
            return false;
        }

        const std::uint64_t clock_ticks =
            (static_cast<std::uint64_t>(m_timer_clock_hz) * milliseconds + 500U) / 1000U;
        // 为精确表示 100% 占空比，给比较寄存器保留一个计数值。
        const std::uint64_t max_ticks =
            IS_TIM_32B_COUNTER_INSTANCE(m_tim_handle.Instance) != 0U ? 0xFFFFFFFFULL : 0xFFFFULL;
        const std::uint64_t divider = (clock_ticks + max_ticks - 1U) / max_ticks;
        if (divider == 0U || divider > 65536U) {
            return false;
        }
        const std::uint64_t ticks = (clock_ticks + divider / 2U) / divider;
        if (ticks == 0U || ticks > max_ticks) {
            return false;
        }

        if (!set_period_ticks(static_cast<std::uint32_t>(ticks))) {
            return false;
        }
        __HAL_TIM_SET_PRESCALER(&m_tim_handle, static_cast<std::uint32_t>(divider - 1U));
        m_tim_handle.Init.Prescaler = static_cast<std::uint32_t>(divider - 1U);
        // 预分频器在硬件中带缓冲，通过更新事件让新配置同时生效。
        m_tim_handle.Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(&m_tim_handle, TIM_FLAG_UPDATE);
        return true;
    }

    /**
     * @brief 按计数器计数值设置周期，并保持当前通道的占空比。
     * @details 周期等于 ARR 加一，沿用当前预分频值。周期属于整个定时器。
     *          启用预装载时，新周期会在下次更新事件后生效。
     * @param ticks 每个周期的计数值，必须大于零且不超过定时器容量。
     * @return 成功返回 true；计数值或对应比较值无法表示时返回 false。
     */
    bool set_period_ticks(std::uint32_t ticks) {
        if (ticks == 0U ||
            (IS_TIM_32B_COUNTER_INSTANCE(m_tim_handle.Instance) == 0U && ticks > 65536U)) {
            return false;
        }

        const std::uint64_t old_ticks =
            static_cast<std::uint64_t>(__HAL_TIM_GET_AUTORELOAD(&m_tim_handle)) + 1U;
        const std::uint64_t old_compare = __HAL_TIM_GET_COMPARE(&m_tim_handle, m_channel);
        const std::uint64_t compare =
            ((old_compare > old_ticks ? old_ticks : old_compare) * ticks + old_ticks / 2U) / old_ticks;
        const std::uint64_t max_compare =
            IS_TIM_32B_COUNTER_INSTANCE(m_tim_handle.Instance) != 0U ? 0xFFFFFFFFULL : 0xFFFFULL;
        if (compare > max_compare) {
            return false;
        }

        __HAL_TIM_SET_AUTORELOAD(&m_tim_handle, ticks - 1U);
        __HAL_TIM_SET_COMPARE(&m_tim_handle, m_channel, static_cast<std::uint32_t>(compare));
        return true;
    }

    /**
     * @brief 设置当前通道的 PWM 占空比。
     * @details 使用 0.0 至 1.0 的比例值。周期达到定时器最大计数容量时，
     *          比较寄存器无法表示精确的 100% 占空比。
     * @param duty 目标占空比；0.0 表示关闭，1.0 表示全开。
     * @return 成功返回 true；参数超出范围或比较值无法表示时返回 false。
     */
    bool set_duty_cycle(float duty) {
        if (!(duty >= 0.0f && duty <= 1.0f)) {
            return false;
        }

        const std::uint64_t ticks =
            static_cast<std::uint64_t>(__HAL_TIM_GET_AUTORELOAD(&m_tim_handle)) + 1U;
        const std::uint64_t compare =
            static_cast<std::uint64_t>(static_cast<double>(ticks) * duty + 0.5);
        const std::uint64_t max_compare =
            IS_TIM_32B_COUNTER_INSTANCE(m_tim_handle.Instance) != 0U ? 0xFFFFFFFFULL : 0xFFFFULL;
        if (compare > max_compare) {
            return false;
        }

        __HAL_TIM_SET_COMPARE(&m_tim_handle, m_channel, static_cast<std::uint32_t>(compare));
        return true;
    }
};
