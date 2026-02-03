#include "ca_com.h"

void Communication_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_RESET);
    uint8_t temp_data[] = "00000000";
    UART_Send_Data(&huart1, temp_data, 8);
    UART_Send_Data(&huart6, temp_data, 8);
}

/**
 * @brief 发送数据到A1
 * @param huart: UART串口句柄
 * @param buffer: 数据缓冲区地址
 * @note 该函数将根据遥控器数据生成指令并发送到A1
 */
void send_data_to_a1(UART_HandleTypeDef *huart, uint8_t *buffer)
{
    // 发送指令说明
    // buffer[0] 0-刹车 1-运动
    // buffer[1] 0-左转 1-右转 2-不转向
    // buffer[2] 0-前进 1-后退 2-左移 3-右移
    // buffer[3] 0-抬升 1-下降
    // buffer[4] 可选项 0-低速 1-高速
		// buffer[5] 视觉控制 0-禁止 1-允许

    // 初始化缓冲区
    strcpy(buffer, "00000000"); // 默认值

    // 获取遥控器数据
    const RC_ctrl_t *rc_data = get_remote_control_point();
    if (rc_data->rc.ch[1] == 0 && rc_data->rc.ch[2] == 0 &&
        rc_data->rc.ch[3] == 0 && rc_data->rc.ch[4] == 0 &&
        rc_data->rc.ch[5] == 0 && rc_data->rc.ch[6] == 0 &&
        rc_data->rc.ch[7] == 0 && rc_data->rc.ch[8] == 0 &&
        rc_data->rc.ch[9] == 0)
    {
        return; // 无效数据不发送信号
    }
		
		// 判断是否开启视觉控制
    if (rc_data->rc.ch[7] <= 400)
    {
			buffer[6] = '0'; // 不开启视觉控制
    }
    else if (rc_data->rc.ch[7] >= 1200)
    {
      buffer[6] = '1'; // 开启视觉控制
    }
    else
    {
      return; // 无效数据不发送信号
    }
		
    // 判断低速高速
    if (rc_data->rc.ch[8] <= 400)
    {
        buffer[4] = '0'; // 低速
    }
    else if (rc_data->rc.ch[8] >= 1200)
    {
        buffer[4] = '1'; // 高速
			
    }
    else
    {
        return; // 无效数据不发送信号
    }
    // 判断是否抬升
    if (rc_data->rc.ch[4] <= 360)
    {
			  buffer[3] = '1'; // 上升
    }
    else if (rc_data->rc.ch[4] >= 1600)
    {
        buffer[3] = '2'; // 下降
    }
		else
		{
			  buffer[3] = '0'; // 停止
		}
		
		// 控制俯仰角
    if (rc_data->rc.ch[1] <= 400)
    {
			 buffer[5] = '1'; // 射球俯仰角上升
		 
    }
    else if (rc_data->rc.ch[1] >= 1200)
    {
        buffer[5] = '2'; // 射球俯仰角下降
			  		
    }
		else
		{
			  buffer[5] = '0'; // 射球俯仰角固定
		}
  
    // 左转右转
    if (rc_data->rc.ch[0] >= 1600)
    {
        strcpy(buffer, "10"); // 左转
        UART_Send_Data(huart, buffer, 8);
			  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_SET);
        return;
    }
    else if (rc_data->rc.ch[0] <= 360)
    {
        strcpy(buffer, "11"); // 右转
        UART_Send_Data(huart, buffer, 8);
			  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_SET);
        return;
    }  
    // 前进后退左移右移
    if (rc_data->rc.ch[2] <= 400)
    {
        strcpy(buffer, "120"); // 前进
        UART_Send_Data(huart, buffer, 8);
			  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_SET);
        return;
    }
    else if (rc_data->rc.ch[2] >= 1200)
    {
        strcpy(buffer, "121"); // 后退
        UART_Send_Data(huart, buffer, 8);
			  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_SET);
        return;
    }
    else if (rc_data->rc.ch[3] >= 1600)
    {
        strcpy(buffer, "122"); // 左移
        UART_Send_Data(huart, buffer, 8);
			  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_SET);
        return;
    }
    else if (rc_data->rc.ch[3] <= 360)
    {
        strcpy(buffer, "123"); // 右移
        UART_Send_Data(huart, buffer, 8);
			  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_SET);
        return;
    }
    // 刹车
    buffer[0] = '0'; // 刹车
    UART_Send_Data(huart, buffer, 8);
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_RESET);
    return;
}

/**
 * @brief 发送数据到A2
 * @param huart: UART串口句柄
 * @param buffer: 数据缓冲区地址
 * @note 该函数将根据遥控数据生成指令并发送到A2
 */
void send_data_to_a2(UART_HandleTypeDef *huart, uint8_t *buffer)
{
    // 发送指令说明
    // buffer[0] 1-射球 0-不射球

    strcpy(buffer, "00000000"); // 默认不射球

    // 获取遥控器数据
    const RC_ctrl_t *rc_data = get_remote_control_point();
    if (rc_data->rc.ch[1] == 0 && rc_data->rc.ch[2] == 0 &&
        rc_data->rc.ch[3] == 0 && rc_data->rc.ch[4] == 0 &&
        rc_data->rc.ch[5] == 0 && rc_data->rc.ch[6] == 0 &&
        rc_data->rc.ch[7] == 0 && rc_data->rc.ch[8] == 0 &&
        rc_data->rc.ch[9] == 0)
    {
        return; // 无效数据不发送信号
    }

    // 判断是否射球
    if (rc_data->rc.ch[9] >= 1600)
    {
        buffer[0] = '1'; // 射球
        UART_Send_Data(huart, buffer, 8);
			  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_SET);
        return;
    }		    			   
    UART_Send_Data(huart, buffer, 8);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_RESET);		
    return;
}
