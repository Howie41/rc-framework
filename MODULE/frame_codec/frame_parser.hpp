#pragma once

#include "frame_codec.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace frame {

/// @brief 从连续字节流中提取固定长度帧，缓存半帧并在验证失败后逐字节重新同步。
/// @tparam FrameType 提供 size()、DataTuple 和 deserialize() 的帧类型。
/// @tparam BufferSize 缓冲容量，至少为一帧；较长的数据块会分段处理。
/// @note 同一实例由调用方串行访问；回调内可调用 reset()，不可重入 feed()。
template <typename FrameType, std::size_t BufferSize = FrameType::size()>
class FrameParser {
    static_assert(FrameType::size() > 0, "FrameParser 不支持零长度帧");
    static_assert(BufferSize >= FrameType::size(), "FrameParser 缓冲区必须至少容纳一帧");

public:
    using DataTuple = typename FrameType::DataTuple;

    struct Callbacks {
        /// @brief 每成功解析一帧同步调用；data_tuple 引用仅在本次回调期间有效。
        void (*on_frame)(FrameParser&, const DataTuple&){nullptr};
    };

    /// @brief 保存回调配置，创建空缓存；不绑定或读取串口。
    explicit constexpr FrameParser(Callbacks callbacks = {}) noexcept : m_callbacks(callbacks) {}

    /// @brief 输入一个字节；成功解析出一帧时返回 true，即使未配置回调。
    constexpr bool feed(std::uint8_t byte) {
        return feed(std::span<const std::uint8_t>{&byte, 1}) != 0;
    }

    /// @brief 输入任意长度的数据块，按顺序交付完整帧并保留未处理完的尾部。
    /// @param bytes 本次收到的数据，仅在调用期间使用，不保存输入视图。
    /// @return 本次成功解析的帧数；空输入返回零，未配置回调时仍正常计数。
    constexpr std::size_t feed(std::span<const std::uint8_t> bytes) {
        std::size_t frame_count = 0;
        while (!bytes.empty()) {
            const auto count = std::min(bytes.size(), BufferSize - m_buffered_size);
            for (std::size_t i = 0; i < count; ++i) {
                m_buffer[m_buffered_size + i] = bytes[i];
            }
            m_buffered_size += count;
            bytes = bytes.subspan(count);
            frame_count += parse_buffer();
        }
        return frame_count;
    }

    /// @brief 丢弃所有缓存字节，保留回调配置；回调内调用时也会丢弃已缓存的后续数据。
    constexpr void reset() noexcept { m_buffered_size = 0; }

    /// @brief 返回尚未消费的缓存字节数，可能包含噪声；feed() 返回后小于帧长度。
    [[nodiscard]]
    constexpr std::size_t buffered_size() const noexcept { return m_buffered_size; }

private:
    std::array<std::uint8_t, BufferSize> m_buffer{};
    std::size_t m_buffered_size{0};
    Callbacks m_callbacks;

    // 删除已消费的前缀，将剩余字节移到缓冲区开头；前缀长度由解析流程保证有效。
    constexpr void discard_prefix(std::size_t count) noexcept {
        if (count == 0) return;
        m_buffered_size -= count;
        for (std::size_t i = 0; i < m_buffered_size; ++i) {
            m_buffer[i] = m_buffer[count + i];
        }
    }

    constexpr std::size_t parse_buffer() {
        std::size_t offset = 0;
        std::size_t frame_count = 0;
        while (m_buffered_size - offset >= FrameType::size()) {
            const auto result = FrameType::deserialize(
                std::span<const std::uint8_t>{m_buffer}.subspan(offset, FrameType::size()));
            if (!result) {
                ++offset;
                continue;
            }
            // 先消费成功帧，再调用回调，允许回调安全地重置缓存。
            discard_prefix(offset + FrameType::size());
            offset = 0;
            ++frame_count;
            if (m_callbacks.on_frame != nullptr) {
                m_callbacks.on_frame(*this, *result);
            }
        }
        discard_prefix(offset);
        return frame_count;
    }
};

} // namespace frame
