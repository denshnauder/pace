/**
 * @file drv_uart.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

STRUCT_UART_MANAGEMENT UART6_Manage_Object = {0};
STRUCT_UART_MANAGEMENT UART1_Manage_Object = {0};
uint8_t usart6_tx_buffer[10] = {0xAB};
uint8_t usart6_rx_buffer[10];
uint8_t usart1_tx_buffer[10] = {0xAB};
uint8_t usart1_rx_buffer[10];
/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Rx_Buffer 接收缓冲区
 * @param Rx_Buffer_Size 接收缓冲区长度
 * @param Callback_Function 处理回调函数
 */
void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer,
               uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function)
{

  if (huart->Instance == USART6)
  {
    UART6_Manage_Object.Rx_Buffer = Rx_Buffer;
    UART6_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
    UART6_Manage_Object.UART_Handler = huart;
    UART6_Manage_Object.Callback_Function = Callback_Function;
  }
  if (huart->Instance == USART1)
  {
    UART1_Manage_Object.Rx_Buffer = Rx_Buffer;
    UART1_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
    UART1_Manage_Object.UART_Handler = huart;
    UART1_Manage_Object.Callback_Function = Callback_Function;
  }
  HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer, Rx_Buffer_Size);
}
/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data,
                       uint16_t Length)
{
  return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // 选择回调函数

  if (huart->Instance == USART6)
  {
    UART6_Manage_Object.Callback_Function(UART6_Manage_Object.Rx_Buffer,
                                          UART6_Manage_Object.Rx_Buffer_Size);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer,
                                 UART6_Manage_Object.Rx_Buffer_Size);
  }

  if (huart->Instance == USART1)
  {
    UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer,
                                          UART1_Manage_Object.Rx_Buffer_Size);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer,
                                 UART1_Manage_Object.Rx_Buffer_Size);
  }
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
