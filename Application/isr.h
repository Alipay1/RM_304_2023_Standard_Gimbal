#pragma once

#include "main.h"

extern uint8_t u1_rx_buf[256];
extern uint8_t u6_rx_buf[256];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
