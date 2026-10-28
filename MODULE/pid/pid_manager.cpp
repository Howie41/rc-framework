#include "pid_manager.hpp"

#include "pid_controller.hpp"

PidManager &PidManager::instance() {
    static PidManager manager;
    return manager;
}

bool PidManager::register_pid(PidController *pid) {
    if (pid == nullptr || pid->m_id != INVALID_PID_ID || m_size >= MAX_PID_COUNT) {
        return false;
    }

    if (!m_pids.push_front(&pid->m_node)) return false;

    pid->m_id = static_cast<uint8_t>(m_size);
    ++m_size;
    return true;
}

PidController *PidManager::get_pid(uint8_t pid_id) {
    if (pid_id == INVALID_PID_ID) {
        return nullptr;
    }

    PidController *result = nullptr;
    m_pids.for_each([&](PidController &pid) {
        if (pid.m_id == pid_id) result = &pid;
    });
    return result;
}

const PidController *PidManager::get_pid(uint8_t pid_id) const {
    if (pid_id == INVALID_PID_ID) {
        return nullptr;
    }

    const PidController *result = nullptr;
    m_pids.for_each([&](const PidController &pid) {
        if (pid.m_id == pid_id) result = &pid;
    });
    return result;
}

std::size_t PidManager::size() const { return m_size; }
