#pragma once

#include "stm32h7xx_hal.h"

#include <atomic>
#include <cstdint>
#include <limits>

#if defined(__GNUC__)
#define DWT_ITCM_FUNC __attribute__((section(".itcm_text")))
#else
#define DWT_ITCM_FUNC
#endif

/**
 * @brief DWT 系统时间拆分值。
 */
struct DwtTime {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
};

class Dwt {
public:
    static constexpr uint64_t CYCLE_COUNTER_PERIOD =
        static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) + 1ULL;

    /**
     * @brief 获取 DWT 单例对象。
     * @return DWT 单例引用。
     */
    static Dwt &instance()
    {
        static Dwt instance;
        return instance;
    }

    Dwt(const Dwt &) = delete;
    Dwt &operator=(const Dwt &) = delete;

    /**
     * @brief 检查 DWT CYCCNT 是否回绕，并更新回绕计数。
     *
     * @attention 该函数假设两次调用之间 CYCCNT 最多只发生一次回绕。
     */
    void update_cycle_count()
    {
        if (cnt_update_locker_.test_and_set(std::memory_order_acquire)) {
            return;
        }

        const uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < cyccnt_last_) {
            ++cyccnt_round_count_;
        }

        cyccnt_last_ = cnt_now;
        cnt_update_locker_.clear(std::memory_order_release);
    }

    /**
     * @brief 初始化 DWT 周期计数器。
     * @param cpu_freq_mhz CPU 主频，单位为 MHz。
     */
    void init(uint32_t cpu_freq_mhz)
    {
        sys_time_ = {};
        cyccnt_round_count_ = 0U;
        cyccnt_last_ = 0U;
        cyccnt64_ = 0U;

        if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U) {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->LAR = 0xC5ACCE55U;

            DWT->CYCCNT = 0U;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }

        cpu_freq_hz_ = cpu_freq_mhz * 1000000U;
        cpu_freq_hz_ms_ = cpu_freq_hz_ / 1000U;
        cpu_freq_hz_us_ = cpu_freq_hz_ / 1000000U;
        update_cycle_count();
    }

    /**
     * @brief 获取距离上次调用的时间差。
     * @param cnt_last 上次记录的 CYCCNT 值，函数返回前会更新为当前值。
     * @return 时间差，单位为秒，float 精度。
     */
    DWT_ITCM_FUNC float get_delta_t(uint32_t *cnt_last)
    {
        const uint32_t cnt_now = DWT->CYCCNT;
        const float dt =
            static_cast<float>(static_cast<uint32_t>(cnt_now - *cnt_last)) /
            static_cast<float>(cpu_freq_hz_);

        *cnt_last = cnt_now;
        update_cycle_count();

        return dt;
    }

    /**
     * @brief 获取距离上次调用的时间差。
     * @param cnt_last 上次记录的 CYCCNT 值，函数返回前会更新为当前值。
     * @return 时间差，单位为秒，double 精度。
     */
    double get_delta_t64(uint32_t *cnt_last)
    {
        const uint32_t cnt_now = DWT->CYCCNT;
        const double dt =
            static_cast<double>(static_cast<uint32_t>(cnt_now - *cnt_last)) /
            static_cast<double>(cpu_freq_hz_);

        *cnt_last = cnt_now;
        update_cycle_count();

        return dt;
    }

    /**
     * @brief 更新系统运行时间缓存。
     */
    void sys_time_update()
    {
        update_cycle_count();

        const uint32_t cnt_now = cyccnt_last_;
        cyccnt64_ = static_cast<uint64_t>(cyccnt_round_count_) * CYCLE_COUNTER_PERIOD +
                    static_cast<uint64_t>(cnt_now);

        const uint64_t seconds = cyccnt64_ / cpu_freq_hz_;
        const uint64_t cycles_after_seconds = cyccnt64_ - seconds * cpu_freq_hz_;
        const uint64_t milliseconds = cycles_after_seconds / cpu_freq_hz_ms_;
        const uint64_t cycles_after_milliseconds =
            cycles_after_seconds - milliseconds * cpu_freq_hz_ms_;

        sys_time_.s = static_cast<uint32_t>(seconds);
        sys_time_.ms = static_cast<uint16_t>(milliseconds);
        sys_time_.us = static_cast<uint16_t>(cycles_after_milliseconds / cpu_freq_hz_us_);
    }

    /**
     * @brief 获取系统运行时间。
     * @return 系统运行时间，单位为秒。
     */
    float get_timeline_s()
    {
        sys_time_update();
        return static_cast<float>(sys_time_.s) + static_cast<float>(sys_time_.ms) * 0.001f +
               static_cast<float>(sys_time_.us) * 0.000001f;
    }

    /**
     * @brief 获取系统运行时间。
     * @return 系统运行时间，单位为毫秒。
     */
    float get_timeline_ms()
    {
        sys_time_update();
        return static_cast<float>(sys_time_.s) * 1000.0f + static_cast<float>(sys_time_.ms) +
               static_cast<float>(sys_time_.us) * 0.001f;
    }

    /**
     * @brief 获取系统运行时间。
     * @return 系统运行时间，单位为微秒。
     */
    uint64_t get_timeline_us()
    {
        sys_time_update();
        return static_cast<uint64_t>(sys_time_.s) * 1000000ULL +
               static_cast<uint64_t>(sys_time_.ms) * 1000ULL +
               static_cast<uint64_t>(sys_time_.us);
    }

    /**
     * @brief 使用 DWT 周期计数器进行阻塞延时。
     * @param delay 延时时间，单位为秒。
     */
    void delay(float delay)
    {
        const uint32_t tickstart = DWT->CYCCNT;
        const float wait_cycles = delay * static_cast<float>(cpu_freq_hz_);

        while (static_cast<uint32_t>(DWT->CYCCNT - tickstart) < wait_cycles) {
        }
    }

    /**
     * @brief 获取最近一次缓存的系统时间拆分值。
     * @return 系统时间拆分值。
     */
    DwtTime time() const
    {
        return sys_time_;
    }

private:
    Dwt() = default;

    DwtTime sys_time_{};
    uint32_t cpu_freq_hz_ = 0U;
    uint32_t cpu_freq_hz_ms_ = 0U;
    uint32_t cpu_freq_hz_us_ = 0U;
    uint32_t cyccnt_round_count_ = 0U;
    uint32_t cyccnt_last_ = 0U;
    uint64_t cyccnt64_ = 0U;
    std::atomic_flag cnt_update_locker_ = ATOMIC_FLAG_INIT;
};
