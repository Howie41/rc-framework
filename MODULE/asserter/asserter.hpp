#pragma once

#include "main.h"

#include <cstdint>

struct AssertionInfo {
    const char* expression{nullptr};
    const char* file{nullptr};
    std::uint32_t line{0};
};

class Asserter final {
private:
    inline static AssertionInfo m_source{};

public:
    Asserter() = delete;

    [[nodiscard]] static const AssertionInfo& source() noexcept {
        return m_source;
    }

    static void fail(
        const char* expression,
        const char* file,
        std::uint32_t line
    ) noexcept {
        m_source = {
            .expression = expression,
            .file = file,
            .line = line,
        };
        Error_Handler();
    }
};

#ifndef ENABLE_RUNTIME_ASSERT
#define ENABLE_RUNTIME_ASSERT 0
#endif

#if ENABLE_RUNTIME_ASSERT
#define RUNTIME_ASSERT(condition)                                      \
    do {                                                               \
        if (!static_cast<bool>(condition)) {                            \
            Asserter::fail(#condition, __FILE_NAME__, __LINE__);       \
        }                                                              \
    } while (false)
#else
#define RUNTIME_ASSERT(condition) ((void)0)
#endif
