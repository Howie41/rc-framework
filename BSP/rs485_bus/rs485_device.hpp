#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

enum class Rs485Error : std::uint8_t {
    NotInitialized,
    Busy,
    InvalidParameter,
    NullDevice,
    DuplicateId,
    DeviceTableFull,
    NotRegistered,
    BuildFailed,
    BufferSizeMismatch,
    TransportNotReady,
    UartError,
    RxOverflow,
    TxTimeout,
    ResponseTimeout,
};

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
    /// @brief 将当前命令写入 output，返回实际长度；不得保留 output 视图。
    [[nodiscard]] virtual std::expected<std::size_t, Rs485Error>
    build_tx(std::span<std::uint8_t> output) const noexcept = 0;
    /// @brief 输入接收流，返回通过校验且匹配本设备的反馈帧数。
    virtual std::size_t feed_rx(std::span<const std::uint8_t> bytes) = 0;
    /// @brief 清空半帧；总线在开始及结束事务时调用。
    virtual void reset_rx() noexcept = 0;

protected:
    Rs485Device() = default;
    std::uint32_t m_id{0xFFFFFFFFU};
};
