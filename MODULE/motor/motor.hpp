#pragma once

/**
 * @brief 电机基类，保存命令、反馈和位置速度控制状态。
 */
class MotorBase {
public:
    // 电机原始输出命令（速度、位置或电流，具体含义由电机类型决定）。
    constexpr static float k_max_cmd{99999.0f};  // 命令绝对值上限
    float m_cmd{0.0f};

    // 转子侧到输出端的减速比。
    float m_reduction_ratio{1.0f};

    // 转子侧原始反馈
    float m_raw_single_pos{0.0f};  // 单圈位置
    float m_raw_sum_pos{0.0f};     // 累计位置
    float m_raw_speed{0.0f};       // 速度
    float m_raw_torque{0.0f};      // 力矩

    // 按减速比换算后的输出端反馈
    float m_single_pos{0.0f};  // 单圈位置
    float m_sum_pos{0.0f};     // 累计位置
    float m_speed{0.0f};       // 速度
    float m_torque{0.0f};      // 力矩
    float m_temperature{0.0f};  // 电机温度，单位为摄氏度

    // 位置与速度控制状态。
    float m_speed_buffer_gain{0.0f};  // 速度缓冲曲线系数，对应参考实现的 k_
    float m_target_sum_pos{0.0f};    // 目标累计位置
    float m_initial_sum_pos{0.0f};   // 起始累计位置
    float m_pos_progress{0.0f};      // 位置进度
    float m_speed_limit{0.0f};       // 当前速度上限，对应参考实现的 v_
    bool m_is_finished{true};        // 运动是否完成
};
