/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          遥控器协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// remote control data
// 遥控器控制变量
RC_ctrl_t rc_ctrl;

// receive data, 18 bytes one frame, but set 36 bytes
// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief          remote control init
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          遥控器初始化
 * @param[in]      none
 * @retval         none
 */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
 * @brief          get remote control data point
 * @param[in]      none
 * @retval         remote control data point
 */
/**
 * @brief          获取遥控器数据指针
 * @param[in]      none
 * @retval         遥控器数据指针
 */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

int16_t Switch_left;
int16_t Switch_right;
int16_t switch_A;
int16_t switch_B;
int16_t switch_C;
int16_t switch_D;
int16_t buf[15];
int global_switch;
int test;
int left_remote;

static uint16_t SBUS_thoroughfare[16] = {0};
static void sbus_to_rc_l(volatile const uint8_t *buf, RC_ctrl_t *rc_ctrl)
{

    SBUS_thoroughfare[0] = ((int16_t)buf[1] >> 0 | ((int16_t)buf[2] << 8)) & 0x07FF;
    SBUS_thoroughfare[1] = ((int16_t)buf[2] >> 3 | ((int16_t)buf[3] << 5)) & 0x07FF;
    SBUS_thoroughfare[2] = ((int16_t)buf[3] >> 6 | ((int16_t)buf[4] << 2) | (int16_t)buf[5] << 10) & 0x07FF;
    SBUS_thoroughfare[3] = ((int16_t)buf[5] >> 1 | ((int16_t)buf[6] << 7)) & 0x07FF;
    SBUS_thoroughfare[4] = ((int16_t)buf[6] >> 4 | ((int16_t)buf[7] << 4)) & 0x07FF;
    SBUS_thoroughfare[5] = ((int16_t)buf[7] >> 7 | ((int16_t)buf[8] << 1) | (int16_t)buf[9] << 9) & 0x07FF;
    SBUS_thoroughfare[6] = ((int16_t)buf[9] >> 2 | ((int16_t)buf[10] << 6)) & 0x07FF;
    SBUS_thoroughfare[7] = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3)) & 0x07FF;
    SBUS_thoroughfare[8] = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8)) & 0x07FF;
    SBUS_thoroughfare[9] = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5)) & 0x07FF;
}

// 串口中断
void USART3_IRQHandler(void)
{
    if (huart3.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // 处理遥控器数据
                sbus_to_rc_l(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

static void sbus_to_rc(volatile const uint8_t *buf, RC_ctrl_t *rc_ctrl)
{
    if (buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = ((int16_t)buf[1] >> 0 | ((int16_t)buf[2] << 8)) & 0x07FF;                         //!< Channel 0
    rc_ctrl->rc.ch[1] = ((int16_t)buf[2] >> 3 | ((int16_t)buf[3] << 5)) & 0x07FF;                         //!< Channel 1
    rc_ctrl->rc.ch[2] = ((int16_t)buf[3] >> 6 | ((int16_t)buf[4] << 2) | (int16_t)buf[5] << 10) & 0x07FF; //!< Channel 2
    rc_ctrl->rc.ch[3] = ((int16_t)buf[5] >> 1 | ((int16_t)buf[6] << 7)) & 0x07FF;                         //!< Channel 3
    rc_ctrl->rc.ch[4] = ((int16_t)buf[6] >> 4 | ((int16_t)buf[7] << 4)) & 0x07FF;                         //!< Channel 1
    rc_ctrl->rc.ch[5] = ((int16_t)buf[7] >> 7 | ((int16_t)buf[8] << 1) | (int16_t)buf[9] << 9) & 0x07FF;  //!< Channel 5
    rc_ctrl->rc.ch[6] = ((int16_t)buf[9] >> 2 | ((int16_t)buf[10] << 6)) & 0x07FF;                        //!< Channel 6
    rc_ctrl->rc.ch[7] = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3)) & 0x07FF;                       //!< Channel 7
    rc_ctrl->rc.ch[8] = ((int16_t)buf[12] >> 0 | ((int16_t)buf[13] << 8)) & 0x07FF;                       //!< Channel 8
    rc_ctrl->rc.ch[9] = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5)) & 0x07FF;                       //!< Channel 9
}
