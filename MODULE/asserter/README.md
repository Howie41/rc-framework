# asserter

假设我们需要检查电机编号是否在 `1～8` 内。编号不对就停止往下执行，并留下出错的位置，方便在调试器里查看。

利用这个模块，可以这样写：

```cpp
#include "asserter.hpp"
#include <cstdint>

void select_motor(uint8_t id) {
    RUNTIME_ASSERT(id >= 1 && id <= 8);
    // 检查通过后，在这里使用 id。
}

void example() {
    select_motor(3); // 通过检查
    select_motor(9); // 开启检查时，记录出错位置并进入 Error_Handler()
}
```

项目根目录 `CMakeLists.txt` 中的 `ENABLE_RUNTIME_ASSERT` 控制是否开启检查，目前设为 `ON`。改动后需要重新配置并编译。

### RUNTIME_ASSERT(condition)

检查一个条件。不需要创建对象，包含 `asserter.hpp` 后直接使用。

- `RUNTIME_ASSERT(condition)`
  - `condition` 填希望成立的条件，例如 `id >= 1 && id <= 8`。
  - 开启检查时，条件为真就继续；为假就保存出错信息，并调用项目的 `Error_Handler()`。
  - 关闭检查时，不会报告错误，但条件里的表达式仍会执行。
  - 不能用它代替必须执行的参数检查。需要返回失败时，自己写 `if` 和 `return`。

例如，无论是否开启运行时检查，都要求拒绝非法编号，可以这样写：

```cpp
bool valid_motor(uint8_t id) {
    if (id < 1 || id > 8) return false;
    return true;
}
```

### Asserter

保存最近一次检查失败的位置。平时用 `RUNTIME_ASSERT`，不需要自己创建这个类，也不用手动调用 `fail()`。

- `static AssertionInfo source`
  - 最近一次失败的信息，主要在调试器里查看。
  - `Asserter::source.expression`：失败的条件文字，例如 `"id >= 1 && id <= 8"`。
  - `Asserter::source.file`：出错文件名。
  - `Asserter::source.line`：出错行号。

### AssertionInfo

一条出错记录。

- `const char* expression`
  - 失败的条件文字。还没有记录时为 `nullptr`。
- `const char* file`
  - 出错文件名。还没有记录时为 `nullptr`。
- `uint32_t line`
  - 出错行号。还没有记录时为 0。
