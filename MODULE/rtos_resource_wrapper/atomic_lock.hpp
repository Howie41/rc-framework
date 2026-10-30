#pragma once

#include <atomic>

/**
 * @brief 轻量原子锁，没有长时间阻塞获取的功能
 */
class AtomicLock {
private:
    std::atomic<bool> m_is_locked{false};

public:
    AtomicLock() = default;

    AtomicLock(const AtomicLock&) = delete;
    AtomicLock& operator=(const AtomicLock&) = delete;
    AtomicLock(AtomicLock&&) = delete;
    AtomicLock& operator=(AtomicLock&&) = delete;

    /** @brief 只尝试一次获取锁，不等待；成功返回 true。 */
    [[nodiscard]] bool try_acquire() noexcept {
        bool expected = false;
        return m_is_locked.compare_exchange_strong(
            expected,
            true,
            std::memory_order_acquire,
            std::memory_order_relaxed
        );
    }

    /** @brief 释放锁，发布此前操作的内存写入。 */
    void release() noexcept {
        m_is_locked.store(false, std::memory_order_release);
    }

    /// @brief 读取当前忙状态，仅用于观察，不获取锁。
    [[nodiscard]] bool is_locked() const noexcept {
        return m_is_locked.load(std::memory_order_acquire);
    }
};

