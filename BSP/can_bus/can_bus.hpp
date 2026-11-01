#pragma once

#include "main.h"
#include "fdcan.h"
#include "atomic_lock.hpp"
#include "asserter.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace can_detail {

inline std::uint8_t dlc_to_bytes(std::uint32_t dlc) {
    switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    case FDCAN_DLC_BYTES_8: return 8;
    default: return 0xFF;
    }
}

inline std::int8_t bus_index_from_handle(FDCAN_HandleTypeDef* handle) {
    if (handle == &hfdcan1) return 0;
    if (handle == &hfdcan2) return 1;
    if (handle == &hfdcan3) return 2;
    return -1;
}

} // namespace can_detail

/**
 * @brief 配置索引 0 的掩码过滤器；标准和扩展 ID 各有独立索引空间。
 * @param fifo 使用 FDCAN_FILTER_TO_RXFIFO0 或 FDCAN_FILTER_TO_RXFIFO1。
 * @note 在启动外设之前调用；初始化失败交给 Error_Handler。
 */
inline void can_filter_init(FDCAN_HandleTypeDef* handle, std::uint32_t id_type,
                            std::uint32_t fifo, std::uint32_t id, std::uint32_t mask_id) {
    FDCAN_FilterTypeDef filter{};
    filter.IdType = id_type;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = fifo;
    filter.FilterID1 = id;
    filter.FilterID2 = mask_id;
    if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK) {
        Error_Handler();
        return;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }
}

/** @brief 开启原有接收、错误通知并启动外设；不再启用软件队列续发中断。 */
inline void bsp_can_init(FDCAN_HandleTypeDef* handle) {
    if (HAL_FDCAN_ActivateNotification(handle,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
            FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE, 0) != HAL_OK) {
        Error_Handler();
        return;
    }
    if (HAL_FDCAN_Start(handle) != HAL_OK) Error_Handler();
}

class CanBus;

/** @brief CAN 协议设备，构造时注册到总线，负责反馈处理及本周期报文构建。 */
class CanDevice {
public:
    /** @brief 保存接收配置并注册到尚未启动的总线，不保存总线引用，不操作硬件。 */
    explicit CanDevice(CanBus& bus, std::uint32_t id = 0, bool is_extid = false);

    virtual ~CanDevice() = default;

    CanDevice(const CanDevice&) = delete;
    CanDevice& operator=(const CanDevice&) = delete;
    CanDevice(CanDevice&&) = delete;
    CanDevice& operator=(CanDevice&&) = delete;

    /** @brief 获取设备接收 ID。 */
    [[nodiscard]] std::uint32_t id() const { return m_id; }
    /** @brief 判断接收 ID 是否为扩展 ID。 */
    [[nodiscard]] bool is_ext_id() const { return m_is_extid; }
    /** @brief 设置接收 ID 与类型；接收中断启用前调用。 */
    void set_can_id(std::uint32_t id, bool is_extid) {
        m_id = id;
        m_is_extid = is_extid;
    }
    /** @brief 精确匹配接收帧 ID 与标准、扩展类型。 */
    [[nodiscard]] virtual bool match_rx(const FDCAN_RxHeaderTypeDef& header) const {
        return (header.IdType == FDCAN_EXTENDED_ID) == m_is_extid && header.Identifier == m_id;
    }
    /** @brief 接收 CAN ID 及反馈数据，以便协议设备分发多电机反馈；数据仅在回调期间有效。 */
    virtual void on_rx(std::uint32_t rx_id, const std::uint8_t data[8], std::uint8_t len) = 0;

    /** @brief 构建本周期的零帧或多帧命令并提交给 bus；全部提交成功返回 true。 */
    [[nodiscard]] virtual bool build_tx(CanBus& bus) = 0;

    bool m_is_extid;
    std::uint32_t m_id;
};

/**
 * @brief 管理一个 FDCAN 外设及最多 8 个设备，沿用原版注册和分发方式。
 * @note 总线、设备和 HAL 句柄须在使用期间存活，不能移动总线对象。
 */
class CanBus {
public:
    static constexpr std::size_t kMaxDevices = 8;
    static constexpr std::size_t kClassicFrameSize = 8;
    static constexpr std::size_t kMaxRxFrameSize = 64;
    static constexpr std::uint32_t kStandardIdMax = 0x7FFU;
    static constexpr std::uint32_t kExtendedIdMax = 0x1FFFFFFFU;
    static constexpr std::uint32_t kAcceptAllFilterId = 0;
    static constexpr std::uint32_t kAcceptAllFilterMask = 0;

    enum class Type : std::uint8_t { Standard = 0, Extended = 1 };

    // 只作为软件参数，不直接发送结构体内存，因此不保留 packed 属性。
    struct ClassicPack {
        std::uint32_t m_id{0};
        Type m_type{Type::Standard};
        std::uint8_t m_data[kClassicFrameSize]{};
    };

    /** @brief 保存当前项目 CubeMX 生成的外设句柄引用。 */
    explicit CanBus(FDCAN_HandleTypeDef& handle) : m_handle(handle) {}

    /** @brief 按 0、1、2 获取 CAN1、CAN2、CAN3 对应的总线对象。 */
    [[nodiscard]] static CanBus* instance_by_index(std::uint8_t index) {
        return index < m_map.size() ? m_map[index] : nullptr;
    }

    /**
     * @brief CubeMX 初始化外设后调用：建立映射、配置过滤器、开启通知并启动 CAN。
     * @note 保留已注册设备；重复调用成功返回，不重复启动。HAL 初始化失败沿用辅助函数的 Error_Handler()。
     */
    [[nodiscard]] bool init() {
        if (m_initialized) return true;
        const auto index = can_detail::bus_index_from_handle(&m_handle);
        if (index < 0) return false;
        m_map[static_cast<std::size_t>(index)] = this;
        can_filter_init(&m_handle, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0,
                        kAcceptAllFilterId, kAcceptAllFilterMask);
        can_filter_init(&m_handle, FDCAN_EXTENDED_ID, FDCAN_FILTER_TO_RXFIFO1,
                        kAcceptAllFilterId, kAcceptAllFilterMask);
        bsp_can_init(&m_handle);
        m_initialized = true;
        return true;
    }

    /** @brief 设备构造时注册；拒绝空指针、重复接收 ID/类型、表满及启动后新增设备。 */
    [[nodiscard]] bool register_device(CanDevice* device) {
        if (device == nullptr) return false;
        for (auto* existing : m_devices) {
            if (existing == device) return true;
            if (existing != nullptr && existing->id() == device->id() &&
                existing->is_ext_id() == device->is_ext_id()) return false;
        }
        if (m_initialized) return false;
        for (auto& existing : m_devices) {
            if (existing == nullptr) {
                existing = device;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 直接提交一个固定 8 字节的经典 CAN 数据帧；不缓存、不等待或重试。
     * @return 0 表示 HAL 接受；1 表示互斥忙、FIFO 满、参数非法或 HAL 失败。
     */
    [[nodiscard]] std::uint8_t add_can_msg(const ClassicPack& pack) {
        if ((pack.m_type != Type::Standard && pack.m_type != Type::Extended) ||
            pack.m_id > (pack.m_type == Type::Standard ? kStandardIdMax : kExtendedIdMax)) return 1;
        if (!m_tx_lock.try_acquire()) return 1;
        if (hardware_tx_queue_empty_size() == 0) {
            m_tx_lock.release();
            return 1;
        }
        FDCAN_TxHeaderTypeDef header{};
        header.Identifier = pack.m_id;
        header.IdType = pack.m_type == Type::Extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_8;
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        const auto result = HAL_FDCAN_AddMessageToTxFifoQ(&m_handle, &header, pack.m_data);
        m_tx_lock.release();
        return result == HAL_OK ? 0 : 1;
    }

    /**
     * @brief 每周期调用各协议设备的 build_tx(*this)，具体组帧和报文数量由设备决定。
     * @return 初始化后所有设备处理成功返回 true；无设备时返回 true。
     * @note 某设备失败仍继续处理其他设备，不缓存失败帧，部分已提交帧不能撤回。
     */
    [[nodiscard]] bool heartbeat() {
        if (!m_initialized) return false;
        bool success = true;
        for (auto* device : m_devices) {
            if (device != nullptr && !device->build_tx(*this)) success = false;
        }
        return success;
    }

    /** @brief 按原实现从指定 FIFO 取一帧，遍历设备匹配后调用 on_rx。 */
    void process_rx_interrupt(std::uint32_t fifo) {
        FDCAN_RxHeaderTypeDef header{};
        // HAL 按报文长度复制数据，留足空间后再拒绝非经典帧，避免写越界。
        std::uint8_t data[kMaxRxFrameSize]{};
        if (HAL_FDCAN_GetRxMessage(&m_handle, fifo, &header, data) != HAL_OK) return;
        const auto len = can_detail::dlc_to_bytes(header.DataLength);
        if (len > kClassicFrameSize || header.FDFormat != FDCAN_CLASSIC_CAN ||
            header.RxFrameType != FDCAN_DATA_FRAME) return;
        for (auto* device : m_devices) {
            if (device != nullptr && device->match_rx(header)) device->on_rx(header.Identifier, data, len);
        }
    }

    /** @brief 获取硬件发送 FIFO 的空闲元素数量。 */
    [[nodiscard]] std::uint32_t hardware_tx_queue_empty_size() {
        return HAL_FDCAN_GetTxFifoFreeLevel(&m_handle);
    }

private:
    FDCAN_HandleTypeDef& m_handle;
    AtomicLock m_tx_lock;
    inline static std::array<CanBus*, 3> m_map{};
    std::array<CanDevice*, kMaxDevices> m_devices{};
    bool m_initialized{false};
};

inline CanDevice::CanDevice(CanBus& bus, std::uint32_t id, bool is_extid)
    : m_is_extid(is_extid), m_id(id) {
    // 注册仅读取基类字段并保存指针，派生类构造完成前不调用虚函数、不启动接收。
    const bool registered = bus.register_device(this);
    RUNTIME_ASSERT(registered);
}

// 沿用原版错误状态记录，供调试器观察；不是线程间同步接口。
inline std::uint32_t g_can_error_state{0};
