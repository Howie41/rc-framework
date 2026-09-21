#include "uart_port.hpp"

#include "main.h"

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    UartPortManager::hal_on_tx_complete(huart);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    UartPortManager::hal_on_rx_complete(huart);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    UartPortManager::hal_on_rx_event(huart, HAL_UARTEx_GetRxEventType(huart), Size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    UartPortManager::hal_on_error(huart, HAL_UART_GetError(huart));
}