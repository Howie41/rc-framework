#pragma once

#include "bsp_dwt.hpp"
#include "pid_manager.hpp"
#include "memory_map.h"
#include "static_list.hpp"

#include <algorithm>
#include <atomic>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

class PidController {
public:
    static constexpr std::size_t kNameSize = 32U;

    /**
     * @brief PID 控制器可选改进功能。
     *
     * 每个枚举值对应 Config::improvement 中的一个 bit，用于按需启用不同的
     * PID 改进算法或辅助处理逻辑。
     */
    enum Improvement : std::size_t {
        increment_of_out = 0U,         // 使用增量式输出计算
        feedforward_control,           // 启用前馈控制补偿
        trapezoid_integral,            // 使用梯形积分计算积分项
        changing_integration_rate,     // 根据误差大小动态调整积分速率
        derivative_on_measurement,     // 基于测量值变化计算微分项，降低参考值突变带来的微分冲击
        derivative_filter,             // 对微分项进行低通滤波
        integral_limit,                // 对积分项进行限幅并抑制积分饱和
        output_filter,                 // 对最终输出进行低通滤波
        error_handle,                  // 启用 PID 错误检测与处理
        deadband_remain_iout,          // 进入死区时保留积分输出
        improvement_count,             // 没有功能，只是用来表示枚举总数
    };

    enum ErrorType : uint8_t {
        PID_ERROR_NONE = 0U,
        MOTOR_BLOCKED,
    };

    /**
     * @brief 各字段独立更新的原子配置，保留聚合初始化，不支持整体复制。
     * @note 运行时访问使用 relaxed load/store，不保证多个字段属于同一组更新。
     */
    struct Config {
        std::atomic<float> kp{0.0f};
        std::atomic<float> ki{0.0f};
        std::atomic<float> kd{0.0f};
        std::atomic<float> max_out{0.0f};
        std::atomic<float> integral_limit{0.0f};
        std::atomic<float> dead_band{0.0f};
        std::atomic<float> coef_a{0.0f};
        std::atomic<float> coef_b{0.0f};
        std::atomic<float> output_lpf_rc{0.0f};
        std::atomic<float> derivative_lpf_rc{0.0f};
        std::atomic<float> ff_j{0.0f};
        std::atomic<float> ff_b{0.0f};
        std::atomic<std::bitset<improvement_count>> improvement{};
    };

    static_assert(std::atomic<float>::is_always_lock_free,
                  "PID shared floats must be lock-free");
    static_assert(std::atomic<std::bitset<improvement_count>>::is_always_lock_free,
                  "PID improvement flags must be lock-free");

    /**
     * @brief 使用默认配置构造 PID 控制器。
     */
    PidController() { register_self(nullptr); }

    /**
     * @brief 使用名称构造 PID 控制器。
     * @param name PID Viewer 使用的 UTF-8 名称，最多保存 31 个字节。
     */
    explicit PidController(const char *name) { register_self(name); }

    /**
     * @brief 使用指定配置构造 PID 控制器并完成初始化。
     * @param config PID 配置参数。
     */
    explicit PidController(const Config &config) {
        register_self(nullptr);
        configure(config);
    }

    /**
     * @brief 使用名称和指定配置构造 PID 控制器。
     * @param name PID Viewer 使用的 UTF-8 名称，最多保存 31 个字节。
     * @param config PID 配置参数。
     */
    PidController(const char *name, const Config &config) {
        register_self(name);
        configure(config);
    }

    PidController(const PidController &) = delete;
    PidController &operator=(const PidController &) = delete;
    PidController(PidController &&) = delete;
    PidController &operator=(PidController &&) = delete;

    /**
     * @brief 更新 PID 配置并重新初始化运行状态。
     * @param config 新的 PID 配置参数。
     * @note 包含内部状态重置，不得与 calculate() 并发调用。
     */
    void configure(const Config &config) {
        copy_config(config);
        init();
    }

    /**
     * @brief 分别原子更新 PID 的比例、积分和微分参数。
     *
     * 该接口不会清除控制器当前的积分、历史采样和输出状态，适合 Viewer
     * 运行时调参使用。
     */
    void set_gains(float kp, float ki, float kd) {
        m_config.kp.store(kp, std::memory_order_relaxed);
        m_config.ki.store(ki, std::memory_order_relaxed);
        m_config.kd.store(kd, std::memory_order_relaxed);
    }

    /**
     * @brief 初始化控制器运行状态。
     *
     * 清空输出项、误差状态、计时数据和故障状态，同时保留当前配置。
     */
    void init() {
        m_ref.store(0.0f, std::memory_order_relaxed);
        m_iterm = 0.0f;
        m_err = 0.0f;
        m_dwt_cnt = 0U;
        m_dt = 0.0f;
        m_error_count = 0U;
        m_error_type = PID_ERROR_NONE;
        m_pout = 0.0f;
        m_iout = 0.0f;
        m_dout = 0.0f;
        m_ffout = 0.0f;
        m_output = 0.0f;
    }

    /**
     * @brief 计算一次 PID 控制输出。
     * @param measure 当前测量反馈值。
     * @param ref 目标参考值。
     * @return 经过限幅后的控制器输出。
     */
    ITCM_FUNCTION float calculate(float measure, float ref) {
        if (has_improvement(error_handle)) {
            pid_error_handle();
        }

        m_dt = Dwt::instance().get_delta_t(&m_dwt_cnt);
        if (m_dt <= kMinDt) {
            m_dt = kMinDt;
        }

        const bool use_increment_output = has_improvement(increment_of_out);
        const bool use_feedforward = has_improvement(feedforward_control);

        m_measure.store(measure, std::memory_order_relaxed);
        m_ref.store(ref, std::memory_order_relaxed);
        m_err = ref - measure;

        if (std::fabs(m_err) > m_config.dead_band.load(std::memory_order_relaxed)) {
            const float kp = m_config.kp.load(std::memory_order_relaxed);
            const float ki = m_config.ki.load(std::memory_order_relaxed);
            const float kd = m_config.kd.load(std::memory_order_relaxed);
            if (use_increment_output) {
                m_pout = kp * m_err - kp * m_last_err;
                m_iterm = ki * m_err * m_dt;
                m_dout = kd * (m_err + m_earlier_err - 2.0f * m_last_err) / m_dt;
            } else {
                m_pout = kp * m_err;
                m_iterm = ki * m_err * m_dt;
                m_dout = kd * (m_err - m_last_err) / m_dt;
            }

            if (use_feedforward) {
                apply_feedforward();
            }
            if (has_improvement(trapezoid_integral)) {
                apply_trapezoid_integral();
            }
            if (has_improvement(changing_integration_rate)) {
                apply_changing_integration_rate();
            }
            if (has_improvement(derivative_on_measurement)) {
                apply_derivative_on_measurement();
            }
            if (has_improvement(derivative_filter)) {
                apply_derivative_filter();
            }
            if (has_improvement(integral_limit)) {
                apply_integral_limit();
            }

            m_iout += m_iterm;

            if (use_increment_output && use_feedforward) {
                m_output = m_pout + m_iterm + m_dout + m_last_output + m_ffout;
            } else if (use_increment_output) {
                m_output = m_pout + m_iterm + m_dout + m_last_output;
            } else if (use_feedforward) {
                m_output = m_pout + m_iout + m_dout + m_ffout;
            } else {
                m_output = m_pout + m_iout + m_dout;
            }

            if (has_improvement(output_filter)) {
                apply_output_filter();
            }

            output_limit();
            proportion_limit();
        } else {
            m_output = 0.0f;
            if (!has_improvement(deadband_remain_iout)) {
                m_iout = 0.0f;
            }
            // TODO: 改为冻结上一帧的output
        }

        m_earlier_measure = m_last_measure;
        m_last_measure = measure;
        m_last_dout = m_dout;
        m_earlier_err = m_last_err;
        m_last_err = m_err;
        m_last_iterm = m_iterm;
        m_last_output = m_output;

        return m_output;
    }

    /**
     * @brief 重置控制器累计状态。
     *
     * 清空 PID 各项、历史采样值、输出历史和计时计数器，同时保留当前配置和故障类型。
     */
    void reset() {
        m_ref.store(0.0f, std::memory_order_relaxed);
        m_iterm = 0.0f;
        m_err = 0.0f;
        m_pout = 0.0f;
        m_iout = 0.0f;
        m_dout = 0.0f;
        m_output = 0.0f;
        m_last_output = 0.0f;
        m_last_measure = 0.0f;
        m_earlier_measure = 0.0f;
        m_last_err = 0.0f;
        m_earlier_err = 0.0f;
        m_last_iterm = 0.0f;
        m_last_dout = 0.0f;
        m_dwt_cnt = 0U;
    }

    /** @brief 获取 PID 配置的可修改引用。 */
    Config &config() { return m_config; }

    /** @brief 获取 PID 配置的只读引用。 */
    const Config &config() const { return m_config; }

    /** @brief 获取最近一次控制器输出。 */
    float output() const { return m_output; }

    /** @brief 获取最近一次参考值。 */
    float ref() const { return m_ref.load(std::memory_order_relaxed); }

    /** @brief 获取最近一次测量值。 */
    float measure() const { return m_measure.load(std::memory_order_relaxed); }

    /** @brief 获取最近一次控制误差。 */
    float err() const { return m_err; }

    /** @brief 获取最近一次比例输出项。 */
    float pout() const { return m_pout; }

    /** @brief 获取最近一次积分输出项。 */
    float iout() const { return m_iout; }

    /** @brief 获取最近一次微分输出项。 */
    float dout() const { return m_dout; }

    /** @brief 获取最近一次前馈输出项。 */
    float ffout() const { return m_ffout; }

    /** @brief 获取最近一次采样时间间隔。 */
    float dt() const { return m_dt; }

    /** @brief 获取当前连续错误计数。 */
    uint16_t error_count() const { return m_error_count; }

    /** @brief 获取当前 PID 错误类型。 */
    ErrorType error_type() const { return m_error_type; }

    /** @brief 获取 PID Viewer 分配的实例编号。 */
    uint8_t id() const { return m_id; }

    /** @brief 获取 PID Viewer 使用的名称。 */
    const char *name() const { return m_name; }

private:
    static constexpr float kMinDt = 1e-6f;
    static constexpr float kBlockedOutputRatio = 0.001f;
    static constexpr float kBlockedRefMin = 0.0001f;
    static constexpr float kBlockedErrorRatio = 0.95f;
    static constexpr uint16_t kBlockedCountLimit = 500U;

    void copy_config(const Config &config) {
        m_config.kp.store(
            config.kp.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.ki.store(
            config.ki.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.kd.store(
            config.kd.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.max_out.store(
            config.max_out.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.integral_limit.store(
            config.integral_limit.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.dead_band.store(
            config.dead_band.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.coef_a.store(
            config.coef_a.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.coef_b.store(
            config.coef_b.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.output_lpf_rc.store(
            config.output_lpf_rc.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.derivative_lpf_rc.store(
            config.derivative_lpf_rc.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.ff_j.store(
            config.ff_j.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.ff_b.store(
            config.ff_b.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_config.improvement.store(
            config.improvement.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }

    void register_self(const char *name) {
        std::memset(m_name, 0, sizeof(m_name));
        if (name != nullptr) {
            const std::size_t name_length = std::strlen(name);
            const std::size_t copy_length =
                name_length < kNameSize - 1U ? name_length : kNameSize - 1U;
            std::memcpy(m_name, name, copy_length);
        }

        PidManager::instance().register_pid(this);
    }

    /**
     * @brief 检查指定改进功能是否启用。
     * @param improvement 要查询的改进功能。
     * @return 若该功能已启用则返回 true。
     */
    bool has_improvement(Improvement improvement) const {
        return m_config.improvement.load(std::memory_order_relaxed).test(improvement);
    }

    /**
     * @brief 应用前馈补偿。
     *
     * 使用配置中的惯量系数和阻尼系数，根据测量速度和测量值计算前馈输出。
     */
    void apply_feedforward() {
        const float measure = m_measure.load(std::memory_order_relaxed);
        const float ff_j = m_config.ff_j.load(std::memory_order_relaxed);
        const float ff_b = m_config.ff_b.load(std::memory_order_relaxed);
        m_ffout = ff_j * (measure - m_last_measure) / m_dt + ff_b * measure;
    }

    /**
     * @brief 对积分项应用梯形积分。
     */
    void apply_trapezoid_integral() {
        m_iterm = m_config.ki.load(std::memory_order_relaxed) * (m_err + m_last_err) * 0.5f * m_dt;
    }

    /**
     * @brief 根据当前误差大小减小或关闭积分作用。
     *
     * 当误差较大且积分输出与误差同向时，抑制积分累积。
     */
    void apply_changing_integration_rate() {
        if (m_err * m_iout <= 0.0f) {
            return;
        }

        const float coef_a = m_config.coef_a.load(std::memory_order_relaxed);
        const float coef_b = m_config.coef_b.load(std::memory_order_relaxed);
        if (std::fabs(m_err) <= coef_b) {
            return;
        }
        if (std::fabs(m_err) <= coef_a + coef_b) {
            m_iterm *= (coef_a - std::fabs(m_err) + coef_b) / coef_a;
        } else {
            m_iterm = 0.0f;
        }
    }

    /**
     * @brief 限制积分贡献并防止积分继续饱和。
     */
    void apply_integral_limit() {
        const float temp_iout = m_iout + m_iterm;
        const float temp_output = has_improvement(increment_of_out)
                                      ? m_pout + m_iterm + m_dout + m_last_output
                                      : m_pout + m_iout + m_dout;

        const float max_out = std::abs(m_config.max_out.load(std::memory_order_relaxed));
        if (std::abs(temp_output) > max_out && m_err * m_iout > 0.0f) {
            m_iterm = 0.0f;
        }

        const float integral_limit = std::abs(m_config.integral_limit.load(std::memory_order_relaxed));
        if (std::abs(temp_iout) > integral_limit) {
            m_iterm = 0.0f;
            m_iout = std::clamp(temp_iout, -integral_limit, integral_limit);
        }
    }

    /**
     * @brief 根据测量值变化计算微分输出。
     *
     * 避免参考值突变引起的微分冲击。
     */
    void apply_derivative_on_measurement() {
        const float kd = m_config.kd.load(std::memory_order_relaxed);
        const float measure = m_measure.load(std::memory_order_relaxed);
        if (has_improvement(increment_of_out)) {
            m_dout = -kd * (measure + m_earlier_measure - 2.0f * m_last_measure) / m_dt;
        } else {
            m_dout = kd * (m_last_measure - measure) / m_dt;
        }
    }

    /**
     * @brief 对微分输出项应用低通滤波。
     */
    void apply_derivative_filter() {
        const float filter_rc = m_config.derivative_lpf_rc.load(std::memory_order_relaxed);
        m_dout = m_dout * m_dt / (filter_rc + m_dt) +
                 m_last_dout * filter_rc / (filter_rc + m_dt);
    }

    /**
     * @brief 对最终输出应用低通滤波。
     */
    void apply_output_filter() {
        const float filter_rc = m_config.output_lpf_rc.load(std::memory_order_relaxed);
        m_output = m_output * m_dt / (filter_rc + m_dt) +
                   m_last_output * filter_rc / (filter_rc + m_dt);
    }

    /**
     * @brief 将最终输出限制在配置的最大幅值范围内。
     */
    void output_limit() {
        const float max_out = std::abs(m_config.max_out.load(std::memory_order_relaxed));
        m_output = std::clamp(m_output, -max_out, max_out);
    }

    /**
     * @brief 将比例项限制在配置的最大幅值范围内。
     */
    void proportion_limit() {
        const float max_out = std::abs(m_config.max_out.load(std::memory_order_relaxed));
        m_pout = std::clamp(m_pout, -max_out, max_out);
    }

    /**
     * @brief 检测持续的堵转输出错误状态。
     *
     * 当控制器输出有效但测量值长期远离参考值时，递增错误计数。
     */
    void pid_error_handle() {
        const float ref = m_ref.load(std::memory_order_relaxed);
        const float measure = m_measure.load(std::memory_order_relaxed);
        if (std::fabs(m_output) < m_config.max_out.load(std::memory_order_relaxed) * kBlockedOutputRatio ||
            std::fabs(ref) < kBlockedRefMin) {
            return;
        }

        if (std::fabs(ref - measure) / std::fabs(ref) > kBlockedErrorRatio) {
            ++m_error_count;
        } else {
            m_error_count = 0U;
        }

        if (m_error_count > kBlockedCountLimit) {
            m_error_type = MOTOR_BLOCKED;
        }
    }

    Config m_config{};
    std::atomic<float> m_ref{0.0f};
    std::atomic<float> m_measure{0.0f};
    float m_err = 0.0f;
    float m_pout = 0.0f;
    float m_iout = 0.0f;
    float m_dout = 0.0f;
    float m_ffout = 0.0f;
    float m_output = 0.0f;
    float m_iterm = 0.0f;
    float m_last_output = 0.0f;
    float m_last_measure = 0.0f;
    float m_earlier_measure = 0.0f;
    float m_last_err = 0.0f;
    float m_earlier_err = 0.0f;
    float m_last_iterm = 0.0f;
    float m_last_dout = 0.0f;
    float m_dt = 0.0f;
    uint32_t m_dwt_cnt = 0U;
    uint16_t m_error_count = 0U;
    ErrorType m_error_type = PID_ERROR_NONE;

    uint8_t m_id{PidManager::INVALID_PID_ID};
    char m_name[kNameSize]{};
    StaticListNode<PidController> m_node{*this};
    friend class PidManager;
};
