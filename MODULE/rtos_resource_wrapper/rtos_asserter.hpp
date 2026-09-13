#pragma once

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>


namespace rtos {

struct AssertionInfo {
    const char* expression{nullptr};
    const char* file{nullptr};
    std::uint32_t line{0};
};

class Asserter final {
private:
    inline static AssertionInfo m_info{};

public:
    Asserter() = delete;

    static void runtime_assert(
        bool condition,
        const char* expression,
        const char* file,
        std::uint32_t line
    ) noexcept {
        if (condition) {
            return;
        }

        m_info = {
            .expression = expression,
            .file = file,
            .line = line,
        };

        taskDISABLE_INTERRUPTS();
        for (;;) {
        }
    }
};

}  // namespace rtos

#ifndef ENABLE_ASSERTER_ASSERT
#define ENABLE_ASSERTER_ASSERT 0
#endif

#if ENABLE_ASSERTER_ASSERT
#define RTOS_ASSERTER_ASSERT(condition)                                  \
    do {                                                                 \
        rtos::Asserter::runtime_assert(                                  \
            static_cast<bool>(condition), #condition, __FILE_NAME__,     \
            __LINE__);                                                    \
    } while (false)
#else
#define RTOS_ASSERTER_ASSERT(condition) ((void)0)
#endif
