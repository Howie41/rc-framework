# Rs485Bus

`Rs485Bus` 绑定已有 `UartPortDma`，管理半双工主机的请求/响应事务。电机负责组帧、搜帧和解析，总线负责 DMA 发送、消费接收流、等待响应和超时处理。方向由 UART 硬件 DE 控制。

## 绑定与调用

```cpp
UartPortDma<"rs4851-usart2", 64, 128, 512> uart(huart2, {});
Rs485Bus bus(uart);
Im6014Motor motor(Im6014Motor::Config{
    .id = 0,
    .max_output_nm = 5.0f,
    .mode = Im6014Motor::Mode::Stop,
});
```

Motor 构造只接收电机配置，不接收或保存 Bus。上述对象需持续存活。在 RTOS 资源创建完成后，依次检查 `bus.init()` 和 `bus.register_device(&motor)` 的返回值，由 Bus 保存设备指针；这一步可在调度器启动前或 RTOS 任务中执行。构造不会发送命令，也不会自动注册电机。

同一电机对象只注册到一条总线，由应用保证。`request()` 拒绝未在当前总线注册的设备，返回 `NotRegistered`；同一指针重复注册成功，不同设备的 ID 重复则返回 `DuplicateId`。

任务持续调用 `bus.process(now_us)`；空闲时调用 `bus.request(motor, now_us, timeout_us)`，生成当前目标并异步发送。一次只处理一个设备的事务，多电机由应用依次请求。注册、设置目标、请求和处理反馈由同一任务执行；其他任务读取多项反馈时需自行同步。该 UART 的收发由总线独占。

也可周期调用 `bus.heartbeat(now_us)`，统一完成处理和轮询：内部先调用原有 `process()`，成功且空闲时再调用 `request()`，按注册顺序依次访问设备，循环回到第一台。一次最多提交一个请求，等待发送、响应和静默间隔期间不会重复提交；没有注册设备时只处理通信状态。返回值与 `process()` 相同，`true` 表示本次完成一个事务，不表示刚提交的请求已经完成。一次事务完成后若立即提交下一请求失败，则优先返回提交错误。

```cpp
// 在 RTOS 任务中初始化、注册之后，每个周期调用；时间单位为微秒。
const std::uint32_t now_us = rtos::now_ms() * 1000U;
if (const auto result = bus.heartbeat(now_us); !result) {
    // 按应用需求记录或处理 result.error()；Faulted 时可显式调用 recover()。
}
```

`Config::response_timeout_us` 配置 heartbeat 的响应超时，默认 1000 μs，0 表示只发送，最大 0x7FFFFFFF。例如 1 ms 任务首次通信测试可设置为 3000 μs。该配置不改变手动 `request()` 的超时参数。处理错误时本次不发起下一请求，后续继续调用 heartbeat；响应超时后静默期结束会轮询下一设备，传输故障须先恢复。提交失败时保留当前轮询位置，下次重试该设备。手动调度与自动轮询应由同一任务按需选择，手动请求不会推进自动轮询位置。

`process()` 返回 `std::expected<bool, Rs485Error>`：`true` 表示一次事务完成，`false` 表示仍在等待或当前无事发生；错误由 `unexpected` 返回。`busy()` 包含发送、等待响应、事务后的静默间隔及故障状态。

所有时间参数单位为微秒，`now_us` 使用同一单调 `uint32_t` 时间源，允许回绕。响应超时从 `process()` 观察到 UART 发送完成时开始；缓冲区中已有的有效反馈先于超时判定处理。传入响应超时 0 时只等待发送完成。

响应超时后进入静默间隔，随后可请求下一次事务。UART 错误、接收流溢出或发送超时进入 `Faulted`，由应用检查 `recover(now_us)` 结果后恢复；恢复会同步中止并重启 DMA，正常收发过程非阻塞。协议无事务序号时，静默间隔不能保证区分同 ID 的迟到响应。
