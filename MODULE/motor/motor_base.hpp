#pragma once

#include "rtos_utility.hpp"

#include <cstdint>

/**
 * @brief 电机公共反馈、减速比、使能状态及反馈时间。
 * @note 输出命令及使能操作由子类定义；具体反馈单位沿用各型号约定。
 *       电机对象长期存活，跨任务或中断访问时由调用方保证同步。
 */
class MotorBase {
public:
    /** @brief 公共反馈超时阈值，单位为毫秒。 */
    static constexpr std::uint32_t kOfflineTimeoutMs{100U};

    /** @brief 使用默认参数构造电机基类。 */
    MotorBase() = default;

    /** @brief 请求允许输出，由子类实现并维护 m_enabled。 */
    virtual void enable() noexcept = 0;

    /** @brief 请求禁止输出，由子类实现并维护 m_enabled。 */
    virtual void disable() noexcept = 0;

    /** @brief 获取软件使能状态，不表示电机已经转动或硬件已经执行。 */
    [[nodiscard]] bool is_enabled() const noexcept { return m_enabled; }

    /**
     * @brief 使用当前 RTOS 毫秒时间判断反馈是否仍有效，达到公共阈值即离线。
     * @note 0xFFFFFFFFU 保留为尚无有效反馈的标记；无符号差值允许计时回绕，
     *       实际反馈间隔须小于一个完整的 uint32_t 毫秒计时周期。
     *       在任务上下文中调用，反馈时间须与 rtos::now_ms() 同源。
     */
    [[nodiscard]] bool is_online() const noexcept {
        return m_last_feedback_ms != 0xFFFFFFFFU &&
               static_cast<std::uint32_t>(rtos::now_ms() - m_last_feedback_ms) < kOfflineTimeoutMs;
    }

    /** @brief 获取最近有效反馈时间，0xFFFFFFFFU 表示尚无有效反馈。 */
    [[nodiscard]] std::uint32_t get_last_feedback_ms() const noexcept { return m_last_feedback_ms; }

    /** @brief 设置转子到输出端的减速比，调用方应提供正数。 */
    void set_motor_reduction(const float config) { m_reduction_ratio = config; }

    /** @brief 获取输出端单圈位置。 */
    [[nodiscard]] float get_current_single_pos() const { return m_single_pos; }

    /** @brief 获取输出端累计位置。 */
    [[nodiscard]] float get_current_sum_pos() const { return m_sum_pos; }

    /** @brief 获取输出端速度。 */
    [[nodiscard]] float get_current_speed() const { return m_speed; }

    /** @brief 获取输出端力矩。 */
    [[nodiscard]] float get_current_torque() const { return m_torque; }

    /** @brief 获取电机温度，单位为摄氏度。 */
    [[nodiscard]] float get_current_temperature() const { return m_temperature; }

    /** @brief 获取转子侧单圈位置。 */
    [[nodiscard]] float get_raw_current_single_pos() const { return m_raw_single_pos; }

    /** @brief 获取转子侧累计位置。 */
    [[nodiscard]] float get_raw_current_sum_pos() const { return m_raw_sum_pos; }

    /** @brief 获取转子侧速度。 */
    [[nodiscard]] float get_raw_current_speed() const { return m_raw_speed; }

    /** @brief 获取原始力矩反馈量，含义与单位由派生驱动确定。 */
    [[nodiscard]] float get_raw_current_torque() const { return m_raw_torque; }

protected:
    float m_reduction_ratio{1};   // 转子到输出端的减速比

    float m_raw_single_pos{0};    // 转子侧单圈位置
    float m_raw_sum_pos{0};       // 转子侧累计位置
    float m_raw_speed{0};         // 转子侧速度
    float m_raw_torque{0};        // 原始力矩反馈量，含义与单位由派生驱动确定

    float m_single_pos{0};        // 输出端单圈位置
    float m_sum_pos{0};           // 输出端累计位置
    float m_speed{0};             // 输出端速度
    float m_torque{0};            // 输出端力矩
    float m_temperature{0};       // 电机温度，单位为摄氏度

    bool m_enabled{false};        // 软件是否允许输出，由子类维护
    std::uint32_t m_last_feedback_ms{0xFFFFFFFFU}; // 最近有效反馈时间，初始值表示尚无反馈
};
