#pragma once

#include "frame_codec.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <tuple>
#include <utility>

namespace frame {

/// @brief 从连续字节流中提取多种定长帧，每种帧对应一个回调。
/// @tparam BufferSize 缓冲容量，至少容纳最长帧；较长的数据块会分段处理。
/// @tparam Frames 帧类型，任意两种帧的固定帧头不得相同或互为前缀。
/// @note 同一实例由调用方串行访问；回调内可调用 reset()，不可重入 feed_and_parse()。
template <std::size_t BufferSize, typename... Frames>
class FrameParserWithBuffer {
    static_assert(sizeof...(Frames) > 0, "FrameParser 至少需要一种帧类型");
    static_assert(((Frames::size() > 0) && ...), "FrameParser 不支持零长度帧");
    static_assert(((BufferSize >= Frames::size()) && ...), "FrameParser 缓冲区必须至少容纳最长帧");

    template <typename F>
    inline static constexpr auto header = F::get_header_prefix(); // F 的固定帧头前缀，用于筛选候选帧类型。

    static constexpr bool valid_headers() {
        const std::array<std::span<const std::uint8_t>, sizeof...(Frames)> headers{header<Frames>...};
        for (std::size_t i = 0; i < headers.size(); ++i) {
            for (std::size_t j = i + 1; j < headers.size(); ++j) {
                const auto count = std::min(headers[i].size(), headers[j].size());
                if (std::ranges::equal(headers[i].first(count), headers[j].first(count)))
                    return false;
            }
        }
        return true;
    }
    static_assert(valid_headers(), "FrameParser 要求不同帧的固定帧头互不为前缀");

public:
    /// @brief 回调顺序与帧类型顺序对应；引用仅在回调期间有效，可用 nullptr 跳过。
    using Callbacks = std::tuple<void (*)(FrameParserWithBuffer&, const typename Frames::DataTuple&)...>;

    // 仅在 feed() 未执行时修改配置，回调顺序与 Frames 一致。
    Callbacks m_callbacks;
    void* m_callback_context{nullptr}; // 所有回调共享的上下文，不管理其生命周期。

    /// @brief 保存回调配置，创建空缓存；不绑定或读取串口。
    explicit constexpr FrameParserWithBuffer(Callbacks callbacks = {}, void* context = nullptr) noexcept
        : m_callbacks(callbacks), m_callback_context(context) {}

    /// @brief 输入一个字节；成功解析出一帧时返回 true，即使未配置回调。
    constexpr bool feed_and_parse(std::uint8_t byte) {
        return feed_and_parse(std::span<const std::uint8_t>{&byte, 1}) != 0;
    }

    /// @brief 输入任意长度的数据块，按顺序交付完整帧并保留未处理完的尾部。
    /// @param bytes 本次收到的数据，仅在调用期间使用，不保存输入视图。
    /// @return 本次成功解析的帧数；空输入返回零，未配置回调时仍正常计数。
    constexpr std::size_t feed_and_parse(std::span<const std::uint8_t> bytes) {
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

    /// @brief 返回尚未消费的缓存字节数；feed_and_parse() 返回后小于最长帧长度。
    [[nodiscard]]
    constexpr std::size_t buffered_size() const noexcept { return m_buffered_size; }

private:
    std::array<std::uint8_t, BufferSize> m_buffer{}; // 所有帧类型共用的字节缓存，保存半帧及待验证数据。
    std::size_t m_buffered_size{0};                 // 缓存开头尚未消费的有效字节数。

    // 删除已消费的前缀，将剩余字节移到缓冲区开头；前缀长度由解析流程保证有效。
    constexpr void discard_prefix(std::size_t count) noexcept {
        if (count == 0) return;
        m_buffered_size -= count;
        for (std::size_t i = 0; i < m_buffered_size; ++i) {
            m_buffer[i] = m_buffer[count + i];
        }
    }

    enum class TryResult {
        Mismatch,   // 帧头或完整帧校验失败，继续尝试其他类型或下一个字节起点。
        Incomplete, // 已有字节与帧头相容，但不足一帧，保留并等待后续输入。
        Parsed,     // 已解出并消费一帧，且已调用该类型配置的回调（若有）。
    };

    template <std::size_t Index>
    constexpr TryResult try_frame(std::span<const std::uint8_t> bytes, std::size_t offset) {
        using F = std::tuple_element_t<Index, std::tuple<Frames...>>;
        const auto count = std::min(bytes.size(), header<F>.size());
        if (!std::ranges::equal(bytes.first(count), std::span{header<F>}.first(count)))
            return TryResult::Mismatch;
        if (bytes.size() < F::size()) return TryResult::Incomplete;
        const auto result = F::deserialize(bytes.first(F::size()));
        if (!result) return TryResult::Mismatch;
        discard_prefix(offset + F::size());
        const auto callback = std::get<Index>(m_callbacks);
        if (callback != nullptr) callback(*this, *result);
        return TryResult::Parsed;
    }

    template <std::size_t... Indices>
    constexpr TryResult try_at(std::size_t offset, std::index_sequence<Indices...>) {
        const auto bytes = std::span<const std::uint8_t>{m_buffer}
                               .first(m_buffered_size).subspan(offset);
        TryResult TryResult = TryResult::Mismatch;
        const auto try_one = [&]<std::size_t Index>() {
            TryResult = try_frame<Index>(bytes, offset);
            return TryResult != TryResult::Mismatch;
        };
        (void)(try_one.template operator()<Indices>() || ...);
        return TryResult;
    }

    constexpr std::size_t parse_buffer() {
        std::size_t offset = 0;
        std::size_t frame_count = 0;
        while (offset < m_buffered_size) {
            const auto TryResult = try_at(offset, std::index_sequence_for<Frames...>{});
            if (TryResult == TryResult::Incomplete) break;
            if (TryResult == TryResult::Mismatch) {
                ++offset;
                continue;
            }
            offset = 0;
            ++frame_count;
        }
        discard_prefix(offset);
        return frame_count;
    }
};

/// @brief 共用一块恰好容纳最长帧的缓存，按类型顺序配置回调。
template <typename... Frames>
using FrameParser = FrameParserWithBuffer<std::max({std::size_t{0}, Frames::size()...}), Frames...>;

} // namespace frame
