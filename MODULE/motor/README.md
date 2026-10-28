# MotorBase 与 C620 移植

`motor_base.hpp` 和 `c620.hpp` 均为 header-only。`C620Motor` 保留原版 `CanDevice`、`MotorBase` 双继承和公开状态成员；未加入看门狗、Dwt、离线自动归零、统计或新的发送管理对象。

电机基类方法按项目命名规范改为蛇形，成员增加 `m_` 前缀。原 `cmd` 相关名称统一改为 `output`，例如 `set_motor_output()`、`set_max_output()`、`m_output`、`m_max_output` 和 `output_trans()`。位置速度进程保留原算法和更新顺序。

## 反馈和输出

- `init(float reduction_ratio = 19, float max_output = 20000.0f)` 只设置减速比与输出上限，不重置反馈状态，原离线时间和消抖参数已删除。
- `set_motor_output()` 沿用当前 MotorBase 的绝对值限幅，不发送。C620 的输出使用原版毫安数值，`output_trans()` 保留 `m_output * 16384 / 20000` 的协议换算。
- `on_rx()` 使用 `frame_codec` 读取 8 字节：大端 uint16 编码器、int16 转速、int16 电流原始量、uint8 温度和保留字节。
- 位置单位为度，速度为 rad/s，温度为摄氏度。首帧只建立位置基准并更新速度、电流及温度，第二帧起更新角度，保留原行为。
- 跨圈判断保留 ±4096 阈值；两帧间运动超过半个转子圈或丢帧后，无法仅凭此算法正确恢复圈数。圈数和累计计数保持原 int32 范围。
- `m_raw_torque` 仍保存反馈 int16 电流原始量，`m_torque` 保留原算式 `m_raw_torque * 0.2f * m_reduction_ratio`。原代码电流注释与实际算式存在单位不一致，本次没有改变比例或新增标定接口，不能直接据此认定其数值为实测 Nm。
- `build_tx()` 保留返回 false 且长度置零的行为。四路输出继续由 `pack_dji_motor_can_msg()` 聚合，以 `frame_codec` 编码为大端数据。

`pack_dji_motor_can_msg()` 保留原签名形式：反馈 ID 数组、协议输出数组、元素数量和调用方的 8 字节缓冲。0 ID、非法 ID、其他组 ID 被忽略；同槽位重复时保留首次值；未填槽位清零，长度固定为 8。不增加协议输出范围检查或错误返回。

`DjiMotorCanGroup::Group1` 保留原枚举数值 0（表示 0x1FF），`Group2` 为 1（表示 0x200）；组帧函数接收实际 CAN ID，不接收枚举序号。

## 使用示例

```cpp
#include "c620.hpp"

static CanBus g_can_bus(hfdcan1);
static C620Motor g_motor(&g_can_bus, 0x201, false, 0x200, false);

// 在 MX_FDCAN1_Init() 后、开启接收中断前调用。
void init_motor() {
    if (g_can_bus.init() != 0) {
        Error_Handler();
        return;
    }
    g_motor.init();
    if (g_can_bus.register_device(&g_motor) != 0) {
        Error_Handler();
        return;
    }
    can_filter_init(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0x201, 0x7FF);
    bsp_can_init(&hfdcan1);
}

// 由控制任务周期调用，返回 1 时交给调用方处理，下一周期提交最新输出。
[[nodiscard]] std::uint8_t send_motor_output(float output_ma) {
    g_motor.set_motor_output(output_ma);
    const std::uint32_t ids[]{g_motor.id()};
    const std::int16_t outputs[]{static_cast<std::int16_t>(g_motor.output_trans())};
    CanBus::ClassicPack pack{};
    pack.m_id = g_motor.m_tx_id;
    pack.m_type = CanBus::Type::Standard;
    std::uint8_t len = 0;
    pack_dji_motor_can_msg(pack.m_id, ids, outputs, 1, pack.m_data, len);
    return g_can_bus.add_can_msg(pack);
}
```

同组的所有受控电机输出应一次聚合成一帧；分别为同组电机组帧会把未填槽位置零。ID 1～4 的反馈为 0x201～0x204，发送 ID 为 0x200；ID 5～8 的反馈为 0x205～0x208，发送 ID 为 0x1FF。

调用方应提供有限输出值，C620 输出上限绝对值不超过 20000；减速比为正数。保持原版整数转换前提，避免超范围 float 转 int16。跨任务或中断读取多项反馈时，由调用方保证同步；不自动添加临界区或快照。停止输出需要显式设置零并发送，不执行离线自动停机。

## 验证

```powershell
cmake -S tests/c620 -B build/c620-tests -G Ninja
cmake --build build/c620-tests
ctest --test-dir build/c620-tests --output-on-failure
```

测试使用独立 HAL 替身，覆盖负值大端编解码、原版首帧/跨圈/减速比算式、槽位跳过及重复处理、多总线中断分发、硬件忙、HAL 失败和失败后使用最新帧，以及多翻译单元包含。

实机仍需验证过滤器、实际反馈频率、总线拥堵及 bus-off。当前工程禁用硬件自动重发，HAL 接受不保证成功交付；本次不修改该配置，也不自动恢复总线。
