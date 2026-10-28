#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>

/**
 * @brief 电机公共输出、状态与位置速度进程，沿用原 MotorBase 的处理逻辑。
 * @note 具体单位由派生驱动确定；跨任务或中断访问时由调用方保证同步。
 */
class MotorBase {
public:
    /** @brief 使用默认参数构造电机基类。 */
    MotorBase() = default;

    /** @brief 设置电机输出，并按最大允许值的绝对值限幅。 */
    void set_motor_output(float output) {
        m_output = std::clamp(output, -std::abs(m_max_output), std::abs(m_max_output));
    }

    /** @brief 设置转子到输出端的减速比，调用方应提供正数。 */
    void set_motor_reduction(const float config) { m_reduction_ratio = config; }

    /** @brief 设置输出上限，限幅时取绝对值。 */
    void set_max_output(const float config) { m_max_output = config; }

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

    /** @brief 根据进程比例计算原实现的线性速度分量 m_k * x。 */
    [[nodiscard]] float f_vx_buffer(float x) { return m_k * x; }

    /**
     * @brief 设置目标位置与速度参数，并重置位置进程。
     * @param pos 目标累计位置，与反馈位置使用相同单位。
     * @param speed 速度系数。
     * @note 目标相对上次目标变化不足 0.01 时，沿用原逻辑直接返回。
     */
    void pos_with_speed_control(float pos, float speed) {
        if (std::fabs(pos - m_tar_sum_pos) < 0.01f) return;
        m_ini_sum_pos = get_current_sum_pos();
        m_tar_sum_pos = pos;
        m_k = speed;
        m_is_finished = false;
        m_pos_process = 0.0f;
    }

    /**
     * @brief 按原顺序检查已有进程，再更新进程和速度值。
     * @return 1.8 加上线性速度分量。
     * @note 未完成时目标位置与起始位置不能相同，调用方需保证该条件。
     */
    [[nodiscard]] float update_speed_process() {
        m_is_finished = get_is_finished();
        m_pos_process = m_is_finished
            ? 1.0f
            : (get_current_sum_pos() - m_ini_sum_pos) / (m_tar_sum_pos - m_ini_sum_pos);
        m_v = 1.8f + f_vx_buffer(m_pos_process);
        return m_v;
    }

    /** @brief 检查已记录的进程是否严格大于阈值，默认阈值为 0.98。 */
    [[nodiscard]] bool get_is_finished(float threshold = 0.98f) {
        return m_pos_process > threshold;
    }

    // 保留原实现的公开成员，供派生驱动更新输出及反馈。
    float m_output{0};            // 当前输出控制量，单位由派生驱动确定
    float m_max_output{99999};    // 输出上限，限幅时取绝对值
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

    float m_k{0.0f};              // 线性速度分量的系数，由目标速度参数设置
    float m_tar_sum_pos{0.0f};    // 本次位置控制的目标累计位置
    float m_ini_sum_pos{0.0f};    // 本次位置控制开始时的累计位置
    float m_pos_process{0.0f};    // 已记录的位移进程比例，按原算法不做限幅
    float m_v{0.0f};              // 最近一次计算的速度值，等于 1.8 加上线性速度分量
    bool m_is_finished{true};     // 最近一次更新速度进程时记录的完成状态
};
