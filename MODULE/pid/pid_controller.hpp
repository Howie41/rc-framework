#pragma once

#include "bsp_dwt.hpp"
#include "pid_manager.hpp"
#include "memory_map.h"

#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

class PidController {
public:
    static constexpr std::size_t NAME_SIZE = 32U;

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

    struct Config {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float max_out = 0.0f;
        float integral_limit = 0.0f;
        float dead_band = 0.0f;
        float coef_a = 0.0f;
        float coef_b = 0.0f;
        float output_lpf_rc = 0.0f;
        float derivative_lpf_rc = 0.0f;
        float ff_j = 0.0f;
        float ff_b = 0.0f;
        std::bitset<improvement_count> improvement{};
    };

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
    explicit PidController(const Config &config) : config_(config) {
        register_self(nullptr);
        init();
    }

    /**
     * @brief 使用名称和指定配置构造 PID 控制器。
     * @param name PID Viewer 使用的 UTF-8 名称，最多保存 31 个字节。
     * @param config PID 配置参数。
     */
    PidController(const char *name, const Config &config) : config_(config) {
        register_self(name);
        init();
    }

    PidController(const PidController &) = delete;
    PidController &operator=(const PidController &) = delete;
    PidController(PidController &&) = delete;
    PidController &operator=(PidController &&) = delete;

    /**
     * @brief 更新 PID 配置并重新初始化运行状态。
     * @param config 新的 PID 配置参数。
     */
    void configure(const Config &config) {
        config_ = config;
        init();
    }

    /**
     * @brief 一次更新 PID 的比例、积分和微分参数。
     *
     * 该接口不会清除控制器当前的积分、历史采样和输出状态，适合 Viewer
     * 运行时调参使用。
     */
    void set_gains(float kp, float ki, float kd) {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }

    /**
     * @brief 初始化控制器运行状态。
     *
     * 清空输出项、误差状态、计时数据和故障状态，同时保留当前配置。
     */
    void init() {
        ref_ = 0.0f;
        iterm_ = 0.0f;
        err_ = 0.0f;
        dwt_cnt_ = 0U;
        dt_ = 0.0f;
        error_count_ = 0U;
        error_type_ = PID_ERROR_NONE;
        pout_ = 0.0f;
        iout_ = 0.0f;
        dout_ = 0.0f;
        ffout_ = 0.0f;
        output_ = 0.0f;
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

        dt_ = Dwt::instance().get_delta_t(&dwt_cnt_);
        if (dt_ <= MIN_DT) {
            dt_ = MIN_DT;
        }

        const bool use_increment_output = has_improvement(increment_of_out);
        const bool use_feedforward = has_improvement(feedforward_control);

        measure_ = measure;
        ref_ = ref;
        err_ = ref_ - measure_;

        if (std::fabs(err_) > config_.dead_band) {
            if (use_increment_output) {
                pout_ = config_.kp * err_ - config_.kp * last_err_;
                iterm_ = config_.ki * err_ * dt_;
                dout_ = config_.kd * (err_ + earlier_err_ - 2.0f * last_err_) / dt_;
            } else {
                pout_ = config_.kp * err_;
                iterm_ = config_.ki * err_ * dt_;
                dout_ = config_.kd * (err_ - last_err_) / dt_;
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

            iout_ += iterm_;

            if (use_increment_output && use_feedforward) {
                output_ = pout_ + iterm_ + dout_ + last_output_ + ffout_;
            } else if (use_increment_output) {
                output_ = pout_ + iterm_ + dout_ + last_output_;
            } else if (use_feedforward) {
                output_ = pout_ + iout_ + dout_ + ffout_;
            } else {
                output_ = pout_ + iout_ + dout_;
            }

            if (has_improvement(output_filter)) {
                apply_output_filter();
            }

            output_limit();
            proportion_limit();
        } else {
            output_ = 0.0f;
            if (!has_improvement(deadband_remain_iout)) {
                iout_ = 0.0f;
            }
        }

        earlier_measure_ = last_measure_;
        last_measure_ = measure_;
        last_dout_ = dout_;
        earlier_err_ = last_err_;
        last_err_ = err_;
        last_iterm_ = iterm_;
        last_output_ = output_;

        return output_;
    }

    /**
     * @brief 重置控制器累计状态。
     *
     * 清空 PID 各项、历史采样值、输出历史和计时计数器，同时保留当前配置和故障类型。
     */
    void reset() {
        ref_ = 0.0f;
        iterm_ = 0.0f;
        err_ = 0.0f;
        pout_ = 0.0f;
        iout_ = 0.0f;
        dout_ = 0.0f;
        output_ = 0.0f;
        last_output_ = 0.0f;
        last_measure_ = 0.0f;
        earlier_measure_ = 0.0f;
        last_err_ = 0.0f;
        earlier_err_ = 0.0f;
        last_iterm_ = 0.0f;
        last_dout_ = 0.0f;
        dwt_cnt_ = 0U;
    }

    /** @brief 获取 PID 配置的可修改引用。 */
    Config &config() { return config_; }

    /** @brief 获取 PID 配置的只读引用。 */
    const Config &config() const { return config_; }

    /** @brief 获取最近一次控制器输出。 */
    float output() const { return output_; }

    /** @brief 获取最近一次参考值。 */
    float ref() const { return ref_; }

    /** @brief 获取最近一次测量值。 */
    float measure() const { return measure_; }

    /** @brief 获取最近一次控制误差。 */
    float err() const { return err_; }

    /** @brief 获取最近一次比例输出项。 */
    float pout() const { return pout_; }

    /** @brief 获取最近一次积分输出项。 */
    float iout() const { return iout_; }

    /** @brief 获取最近一次微分输出项。 */
    float dout() const { return dout_; }

    /** @brief 获取最近一次前馈输出项。 */
    float ffout() const { return ffout_; }

    /** @brief 获取最近一次采样时间间隔。 */
    float dt() const { return dt_; }

    /** @brief 获取当前连续错误计数。 */
    uint16_t error_count() const { return error_count_; }

    /** @brief 获取当前 PID 错误类型。 */
    ErrorType error_type() const { return error_type_; }

    /** @brief 获取 PID Viewer 分配的实例编号。 */
    uint8_t id() const { return id_; }

    /** @brief 获取 PID Viewer 使用的名称。 */
    const char *name() const { return name_; }

private:
    static constexpr float MIN_DT = 1e-6f;
    static constexpr float BLOCKED_OUTPUT_RATIO = 0.001f;
    static constexpr float BLOCKED_REF_MIN = 0.0001f;
    static constexpr float BLOCKED_ERROR_RATIO = 0.95f;
    static constexpr uint16_t BLOCKED_COUNT_LIMIT = 500U;

    void register_self(const char *name) {
        std::memset(name_, 0, sizeof(name_));
        if (name != nullptr) {
            const std::size_t name_length = std::strlen(name);
            const std::size_t copy_length =
                name_length < NAME_SIZE - 1U ? name_length : NAME_SIZE - 1U;
            std::memcpy(name_, name, copy_length);
        }

        PidManager::instance().register_pid(this);
    }

    /**
     * @brief 检查指定改进功能是否启用。
     * @param improvement 要查询的改进功能。
     * @return 若该功能已启用则返回 true。
     */
    bool has_improvement(Improvement improvement) const {
        return config_.improvement.test(improvement);
    }

    /**
     * @brief 应用前馈补偿。
     *
     * 使用配置中的惯量系数和阻尼系数，根据测量速度和测量值计算前馈输出。
     */
    void apply_feedforward() {
        ffout_ = config_.ff_j * (measure_ - last_measure_) / dt_ + config_.ff_b * measure_;
    }

    /**
     * @brief 对积分项应用梯形积分。
     */
    void apply_trapezoid_integral() {
        iterm_ = config_.ki * (err_ + last_err_) * 0.5f * dt_;
    }

    /**
     * @brief 根据当前误差大小减小或关闭积分作用。
     *
     * 当误差较大且积分输出与误差同向时，抑制积分累积。
     */
    void apply_changing_integration_rate() {
        if (err_ * iout_ <= 0.0f) {
            return;
        }

        if (std::fabs(err_) <= config_.coef_b) {
            return;
        }
        if (std::fabs(err_) <= config_.coef_a + config_.coef_b) {
            iterm_ *= (config_.coef_a - std::fabs(err_) + config_.coef_b) / config_.coef_a;
        } else {
            iterm_ = 0.0f;
        }
    }

    /**
     * @brief 限制积分贡献并防止积分继续饱和。
     */
    void apply_integral_limit() {
        const float temp_iout = iout_ + iterm_;
        const float temp_output = has_improvement(increment_of_out)
                                      ? pout_ + iterm_ + dout_ + last_output_
                                      : pout_ + iout_ + dout_;

        if (std::fabs(temp_output) > config_.max_out && err_ * iout_ > 0.0f) {
            iterm_ = 0.0f;
        }

        if (temp_iout > config_.integral_limit) {
            iterm_ = 0.0f;
            iout_ = config_.integral_limit;
        }
        if (temp_iout < -config_.integral_limit) {
            iterm_ = 0.0f;
            iout_ = -config_.integral_limit;
        }
    }

    /**
     * @brief 根据测量值变化计算微分输出。
     *
     * 避免参考值突变引起的微分冲击。
     */
    void apply_derivative_on_measurement() {
        if (has_improvement(increment_of_out)) {
            dout_ = -config_.kd * (measure_ + earlier_measure_ - 2.0f * last_measure_) / dt_;
        } else {
            dout_ = config_.kd * (last_measure_ - measure_) / dt_;
        }
    }

    /**
     * @brief 对微分输出项应用低通滤波。
     */
    void apply_derivative_filter() {
        dout_ = dout_ * dt_ / (config_.derivative_lpf_rc + dt_) +
                last_dout_ * config_.derivative_lpf_rc / (config_.derivative_lpf_rc + dt_);
    }

    /**
     * @brief 对最终输出应用低通滤波。
     */
    void apply_output_filter() {
        output_ = output_ * dt_ / (config_.output_lpf_rc + dt_) +
                  last_output_ * config_.output_lpf_rc / (config_.output_lpf_rc + dt_);
    }

    /**
     * @brief 将最终输出限制在配置的最大幅值范围内。
     */
    void output_limit() {
        if (output_ > config_.max_out) {
            output_ = config_.max_out;
        }
        if (output_ < -config_.max_out) {
            output_ = -config_.max_out;
        }
    }

    /**
     * @brief 将比例项限制在配置的最大幅值范围内。
     */
    void proportion_limit() {
        if (pout_ > config_.max_out) {
            pout_ = config_.max_out;
        }
        if (pout_ < -config_.max_out) {
            pout_ = -config_.max_out;
        }
    }

    /**
     * @brief 检测持续的堵转输出错误状态。
     *
     * 当控制器输出有效但测量值长期远离参考值时，递增错误计数。
     */
    void pid_error_handle() {
        if (std::fabs(output_) < config_.max_out * BLOCKED_OUTPUT_RATIO ||
            std::fabs(ref_) < BLOCKED_REF_MIN) {
            return;
        }

        if (std::fabs(ref_ - measure_) / std::fabs(ref_) > BLOCKED_ERROR_RATIO) {
            ++error_count_;
        } else {
            error_count_ = 0U;
        }

        if (error_count_ > BLOCKED_COUNT_LIMIT) {
            error_type_ = MOTOR_BLOCKED;
        }
    }

    Config config_{};
    float ref_ = 0.0f;
    float measure_ = 0.0f;
    float err_ = 0.0f;
    float pout_ = 0.0f;
    float iout_ = 0.0f;
    float dout_ = 0.0f;
    float ffout_ = 0.0f;
    float output_ = 0.0f;
    float iterm_ = 0.0f;
    float last_output_ = 0.0f;
    float last_measure_ = 0.0f;
    float earlier_measure_ = 0.0f;
    float last_err_ = 0.0f;
    float earlier_err_ = 0.0f;
    float last_iterm_ = 0.0f;
    float last_dout_ = 0.0f;
    float dt_ = 0.0f;
    uint32_t dwt_cnt_ = 0U;
    uint16_t error_count_ = 0U;
    ErrorType error_type_ = PID_ERROR_NONE;

    uint8_t id_{PidManager::INVALID_PID_ID};
    char name_[NAME_SIZE]{};
    PidController *next_{nullptr};

    friend class PidManager;
};
