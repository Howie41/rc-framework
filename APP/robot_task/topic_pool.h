#pragma once
#include "fdcan.h"
#include "topics.hpp"
#include "usart.h"

#include <cstdint>
#include <stdbool.h>

#pragma pack(1)

typedef struct {
  UART_HandleTypeDef *huart; // 串口句柄
  uint16_t len;              // 数据长度
  void *data_addr; // 数据地址，使用时把地址赋值给这个指针，数值强转为uint8_t
} UART_TxMsg;

#pragma pack()
