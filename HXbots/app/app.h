#pragma once
#include "stm32f4xx_hal.h" 

void app_init(void);
void app_loop(void);
void app_hil_uart_rx_callback(UART_HandleTypeDef *huart);