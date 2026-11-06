# static_string

假设我们要生成一行电机状态文字：`motor=1 speed=100`，再把它交给显示或发送接口。

利用这个模块，可以给这行文字留 32 个字符的位置：

```cpp
#include "static_string.hpp"

void example() {
    StaticString<32> text{"motor={} speed={}", 1, 100};
    bool added = text.try_push_back('!'); // true，变成 "motor=1 speed=100!"

    const char* message = text.c_str();  // 交给需要字符串的显示或发送接口
    auto length = text.size();           // 当前字符数：18
    // 这里只准备文字，没有实际显示或发送。
}
```

### StaticString<Capacity>

存一段有长度上限的文字。例子中的 `32` 表示最多放 32 个字符，不包括结尾的 `\0`。

- `StaticString()`
  - 创建空字符串，例如 `StaticString<32> text;`。
- `StaticString(const char* str)`
  - 从已有文字创建，例如 `StaticString<32> text{"motor"};`。
  - 太长时只保留前 32 个字符。不要传 `nullptr`。
- `StaticString(std::string_view str)`
  - 从已经带长度的文字创建，也会截断超出容量的部分。
  - 例如 `StaticString<32> text{std::string_view{"motor"}};`。
- `template <class... Args> StaticString(std::format_string<Args...> format_str, Args&&... args)`
  - 用 `{}` 依次放入后面给的值。
  - 例如 `StaticString<32>{"motor={} speed={}", 1, 100}` 得到 `motor=1 speed=100`。
  - 结果太长时只保留前 32 个字符。
- `std::size_t size() const`
  - 返回当前字符数，不算末尾的 `\0`。
- `std::size_t capacity() const`
  - 返回最多能放多少字符，例子中是 32。
- `bool empty() const`
  - 没有内容时返回 `true`。
- `const char* c_str() const`
  - 取出文字，交给需要 C 字符串的接口。
  - 返回的指针指向这个对象里的文字，不是另一份副本。对象销毁后不要再用这个指针。
- `const char* data() const`
  - 也可以取出文字，和 `c_str()` 指向同一个位置，只能通过它读取。
- `std::string_view view() const`
  - 取出一份带长度的文字，用于接收 `std::string_view` 的接口。
  - 不复制文字，对象销毁后不要再用。
- `template <std::size_t N> bool operator==(const StaticString<N>& other) const`
  - 用 `a == b` 比较两段文字，相同返回 `true`。
  - 两边容量可以不同。
- `void clear()`
  - 清空文字，之后 `size()` 为 0。
- `template <std::size_t N> bool try_append(const StaticString<N>& other)`
  - 把另一段文字接到末尾，例如 `text.try_append(StaticString{" ready"});`。
  - 放得下时返回 `true`。
  - 放不下或对方为空时返回 `false`，原内容不变，不会只追加一半。
- `bool try_push_back(char ch)`
  - 追加一个字符，例如 `text.try_push_back('!');`。
  - 成功返回 `true`；已经放满时返回 `false`，原内容不变。

`StaticString{"ready"}` 可以省略容量，但容量会刚好等于这段文字的长度，后面没有空位再追加。需要继续加内容时，像开头一样写 `StaticString<32>`。

这里的容量按字节数计算。示例都是英文字符；放中文时，不要把 32 理解成一定能放 32 个汉字。
