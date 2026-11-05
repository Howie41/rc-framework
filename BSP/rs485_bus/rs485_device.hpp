#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

/// @brief RS-485 设备地址、命令组帧和原始字节接收接口。
class Rs485Device {
public:
    virtual ~Rs485Device() = default;

    Rs485Device(const Rs485Device&) = delete;
    Rs485Device& operator=(const Rs485Device&) = delete;
    Rs485Device(Rs485Device&&) = delete;
    Rs485Device& operator=(Rs485Device&&) = delete;

    /// @brief 获取设备地址，地址含义及有效范围由具体协议确定。
    [[nodiscard]] std::uint32_t id() const noexcept { return m_id; }
    /// @brief 将当前命令写入 output；成功返回 true 并设置实际长度，失败返回 false 且长度置零。
    /// @note 不得保留 output 视图，成功时 size 须在 1～output.size() 范围内。
    [[nodiscard]] virtual bool build_tx(std::span<std::uint8_t> output,
                                        std::size_t& size) const noexcept = 0;

    /// @brief 接收当前请求的原始回复字节，协议校验和分片拼接由设备实现。
    /// @note 不得保留 bytes 视图；可能被多次调用以交付同一次回复。
    virtual void on_rx(std::span<const std::uint8_t> bytes) = 0;

protected:
    Rs485Device() = default;
    std::uint32_t m_id{0xFFFFFFFFU};
};
