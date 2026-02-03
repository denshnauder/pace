#ifndef CA_COM_H
#define CA_COM_H

#include "drv_uart.h"
#include "stm32f4xx_hal.h"
#include "usefulFunction.h"
#include "remote_control.h"

void Communication_Call_Back(uint8_t *Buffer, uint16_t Length);
void send_data_to_a1(UART_HandleTypeDef *huart, uint8_t *buffer);
void send_data_to_a2(UART_HandleTypeDef *huart, uint8_t *buffer);

#endif
