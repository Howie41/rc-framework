#include "pid_manager.hpp"

#include "pid_controller.hpp"

PidManager &PidManager::instance() {
    static PidManager manager;
    return manager;
}

bool PidManager::register_pid(PidController *pid) {
    if (pid == nullptr || pid->id_ != INVALID_PID_ID || size_ >= MAX_PID_COUNT) {
        return false;
    }

    pid->id_ = static_cast<uint8_t>(size_);
    pid->next_ = nullptr;

    if (tail_ == nullptr) {
        head_ = pid;
        tail_ = pid;
    } else {
        tail_->next_ = pid;
        tail_ = pid;
    }

    ++size_;
    return true;
}

PidController *PidManager::get_pid(uint8_t pid_id) {
    if (pid_id == INVALID_PID_ID) {
        return nullptr;
    }

    for (PidController *pid = head_; pid != nullptr; pid = pid->next_) {
        if (pid->id_ == pid_id) {
            return pid;
        }
    }
    return nullptr;
}

const PidController *PidManager::get_pid(uint8_t pid_id) const {
    if (pid_id == INVALID_PID_ID) {
        return nullptr;
    }

    for (const PidController *pid = head_; pid != nullptr; pid = pid->next_) {
        if (pid->id_ == pid_id) {
            return pid;
        }
    }
    return nullptr;
}

std::size_t PidManager::size() const { return size_; }
