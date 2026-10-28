#pragma once

#include "static_list.hpp"

#include <cstddef>
#include <cstdint>

class PidController;

/**
 * @brief PID 控制器实例注册表。
 *
 * PidController 在构造时自动注册到该单例中。实例编号按照构造顺序从
 * 0 开始连续分配，0xFF 保留给 Viewer 表示未使用的通道。
 * 注册须在开始查询前完成；实例须在注册表使用期间存活，不支持注销。
 */
class PidManager {
public:
    static constexpr uint8_t INVALID_PID_ID = 0xFFU;
    static constexpr std::size_t MAX_PID_COUNT = 0xFFU;

    PidManager(const PidManager &) = delete;
    PidManager &operator=(const PidManager &) = delete;

    static PidManager &instance();

    bool register_pid(PidController *pid);

    PidController *get_pid(uint8_t pid_id);
    const PidController *get_pid(uint8_t pid_id) const;

    std::size_t size() const;

private:
    PidManager() = default;

    StaticList<PidController> m_pids;
    std::size_t m_size{0U};
};
