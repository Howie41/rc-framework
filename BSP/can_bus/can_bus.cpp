#include "can_bus.hpp"

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, std::uint32_t events) {
    if ((events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
    const auto index = can_detail::bus_index_from_handle(handle);
    auto* bus = index >= 0 ? CanBus::instance_by_index(static_cast<std::uint8_t>(index)) : nullptr;
    if (bus != nullptr) bus->process_rx_interrupt(FDCAN_RX_FIFO0);
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* handle, std::uint32_t events) {
    if ((events & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0) return;
    const auto index = can_detail::bus_index_from_handle(handle);
    auto* bus = index >= 0 ? CanBus::instance_by_index(static_cast<std::uint8_t>(index)) : nullptr;
    if (bus != nullptr) bus->process_rx_interrupt(FDCAN_RX_FIFO1);
}

extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* handle) {
    // 不沿用原版错误中断中的死循环，HAL 错误仍保存在句柄内。
    (void)handle;
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* handle, std::uint32_t events) {
    (void)handle;
    g_can_error_state = events;
}
