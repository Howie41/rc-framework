# Rs485Bus

`Rs485Bus` 绑定已有 `UartPortDma`，负责设备注册、发送排队、接收分发及回复等待。设备负责组帧、校验和流式解析，应用决定发送顺序和周期。方向由 UART 硬件 DE 控制。

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

上述对象须在 `rtos_resource_init()` 创建队列和流缓冲区之前构造，并在通信期间持续存活。Motor 构造只接收配置，不保存 Bus，不发送命令，也不自动注册。

初始化顺序为：CubeMX 配置 UART/DMA，创建 RTOS 资源，再完成设备注册并启动 UART 接收。TX DMA 使用 NORMAL，RX DMA 使用 CIRCULAR，UART 模板须启用收发缓冲区和接收流缓冲区。

```cpp
// 在 RTOS 资源创建后、通信开始前调用。
void init_rs485() {
    bus.init();
    if (!bus.register_device(&motor) ||
        uart.start_receive_to_idle_dma() != HAL_OK) {
        Error_Handler();
    }
}
```

`init()` 返回 void，只清空设备表，不配置 UART、不启动 DMA，也不清空发送队列或进行故障恢复；只能在注册、入队和通信开始前调用。`register_device()` 返回 bool，最多注册 16 个设备，同指针重复注册成功；空指针、重复 ID 或表满返回 false。同一设备只属于一条总线，由应用保证。

## 组帧、入队与周期处理

设备先通过 `build_tx(output, size)` 生成报文，再调用 `add_rs485_msg(bytes, response_device)` 入队。总线按值复制报文，队列容量为 4；单帧上限为 `min(UART发送缓冲区大小, 128)` 字节。空报文、长度超限、回复设备未注册或队列满时返回 false。

```cpp
// 在控制或通信任务中按应用周期调用；true 只表示报文已入队。
[[nodiscard]] bool queue_motor_command() {
    std::array<std::uint8_t, Im6014Motor::CommandFrame::size()> bytes{};
    std::size_t size = 0;
    if (!motor.build_tx(bytes, size)) return false;
    return bus.add_rs485_msg(std::span{bytes}.first(size), &motor);
}

// 由同一个通信任务持续调用，先接收再发送。
void process_rs485() {
    (void)bus.process_rx();
    (void)bus.process_tx();
}

// 需要输出时由应用设置命令和使能，例如：
// motor.enable();
// if (!motor.set_output_command({.speed_rad_s = 1.0f, .kd = 0.1f})) { ... }
// if (!queue_motor_command()) { ... }
```

- `process_rx()` 非阻塞读取最多 64 字节，分发给全部注册设备。设备自行校验地址及协议，半帧保留在设备解析器中；至少一个设备接受有效反馈时返回 true。
- `process_tx()` 每次至多提交队首一帧，成功提交 HAL 后返回 true 并移除队首。空队列、UART 忙、等待回复、回复超时或 HAL 提交失败均返回 false；HAL 失败时保留队首供后续重试。返回值不表示硬件已完成发送或对方已收到。
- `response_device == nullptr` 表示不等待回复，但仍须等 UART TC 完成后才能发送下一帧。指定设备时，须收到该设备有效反馈或等待超时后才能继续。
- 回复超时使用 RTOS 毫秒时间，固定为 `kResponseTimeoutMs = 1`，从通信任务观察到 UART 发送结束时开始计算。回复已收到时直接继续；超时时清空对应设备半帧、结束等待，本次不发送，后续调用再处理队列。

## 生命周期与应用职责

- 多任务可以调用入队接口；`process_rx()`、`process_tx()` 由同一个通信任务串行调用。电机命令、组帧、反馈等共享状态仍由应用保证同步。
- 总线保存的是入队时的报文字节。之后修改命令或调用 `disable()` 不会改写已排队报文；禁用后还须生成并发送停机命令。
- 总线不自动轮询设备，不接管 UART 回调，不提供发送看门狗、静默间隔、故障状态或自动恢复。UART 异常、DMA 中止及重启由应用处理。
- 协议没有事务序号，当前等待逻辑不能保证区分同 ID 的迟到回复。在线判断只反映反馈时效，不自动停止输出。

`Rs485Device::build_tx(output, size)` 成功时返回 true 并设置长度，失败时长度为零。`feed_rx(bytes)` 返回是否至少接受一帧有效反馈；`reset_rx()` 清空半帧，保留已有反馈和命令。
