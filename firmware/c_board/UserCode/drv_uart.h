/**
 * @file drv_uart.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_UART_H
#define DRV_UART_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*UART_Call_Back)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief UART通信处理结构体
 */
typedef struct Struct_UART_Manage_Object {
  UART_HandleTypeDef *UART_Handler;
  uint8_t *Rx_Buffer;
  uint16_t Rx_Buffer_Size;
  UART_Call_Back Callback_Function;
} STRUCT_UART_MANAGEMENT;

/* Exported variables --------------------------------------------------------*/

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;

extern STRUCT_UART_MANAGEMENT UART6_Manage_Object;
extern STRUCT_UART_MANAGEMENT UART1_Manage_Object;

extern uint8_t usart6_tx_buffer[10];
extern uint8_t usart6_rx_buffer[10];
extern uint8_t usart1_tx_buffer[10];
extern uint8_t usart1_rx_buffer[10];
/* Exported function declarations --------------------------------------------*/

void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer,
               uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function);

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data,
                       uint16_t Length);

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
