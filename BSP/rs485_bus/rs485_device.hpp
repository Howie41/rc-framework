#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

/// @brief RS-485 设备接口，不保存总线关联；设备负责协议编解码和流式解析。
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
    /// @brief 输入接收流；至少接受一帧通过校验且匹配本设备的反馈时返回 true。
    /// @note 仅有半帧或无有效反馈时返回 false，保留半帧以供后续拼接。
    virtual bool feed_rx(std::span<const std::uint8_t> bytes) = 0;
    /// @brief 清空接收半帧；用于回复超时或显式重置，不清除已有反馈。
    virtual void reset_rx() noexcept = 0;

protected:
    Rs485Device() = default;
    std::uint32_t m_id{0xFFFFFFFFU};
};
