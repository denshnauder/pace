#include "drv_communication.h"

#include "drv_motion.h"

uint8_t tx_buffer[8] = {0xAB};  // 发送缓冲区（按需调整）
uint8_t rx_buffer[8];           // 接收32位数据（4字节）
uint8_t NUC_rx_buffer[8];

uint16_t Course_Angle = 0;  // NUC回传航向角像素
uint16_t Angle_Index = 0;   // 角度环数组索引值
uint16_t Angle_Array[100] = {0};
//uint8_t vision_control_flag = 0; //视觉控制标志位 0 不允许 1 允许

GPIO_PinState Char2GPIO_State(char message)
{
  if (message == '0') {
    return GPIO_PIN_SET;
  }
  else {
    return GPIO_PIN_RESET;
  }
}

void Trans_LED_Status()
{
  // 使用rx_buffer控制LED（PG1~PG8）
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, Char2GPIO_State(rx_buffer[0]));  // 位0 -> PG1
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Char2GPIO_State(rx_buffer[1]));  // 位1 -> PG2
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, Char2GPIO_State(rx_buffer[2]));  // 位2 -> PG3
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, Char2GPIO_State(rx_buffer[3]));  // 位3 -> PG4
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, Char2GPIO_State(rx_buffer[4]));  // 位4 -> PG5
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, Char2GPIO_State(rx_buffer[5]));  // 位5 -> PG6
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, Char2GPIO_State(rx_buffer[6]));  // 位6 -> PG7
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, Char2GPIO_State(rx_buffer[7]));  // 位7 -> PG8
}

void Trans_LED_Status_Vision()
{
  // 使用NUC_rx_buffer控制LED（PG1~PG8）
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, Char2GPIO_State(NUC_rx_buffer[0]));  // 位0 -> PG1
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Char2GPIO_State(NUC_rx_buffer[1]));  // 位1 -> PG2
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, Char2GPIO_State(NUC_rx_buffer[2]));  // 位2 -> PG3
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, Char2GPIO_State(NUC_rx_buffer[3]));  // 位3 -> PG4
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, Char2GPIO_State(NUC_rx_buffer[4]));  // 位4 -> PG5
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, Char2GPIO_State(NUC_rx_buffer[5]));  // 位5 -> PG6
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, Char2GPIO_State(NUC_rx_buffer[6]));  // 位6 -> PG7
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, Char2GPIO_State(NUC_rx_buffer[7]));  // 位7 -> PG8
}

/* 字符串指令解析运动函数 */
void Parse_Motion(uint8_t * cmd, uint8_t * NUC)
{
  // 0. 视觉使能
  if (cmd[6] == '1') {
    Course_Angle = (NUC[0] - '0') * 100 + (NUC[1] - '0') * 10 + (NUC[2] - '0') * 1;  //航向角解码
    Angle_Index = (NUC[2] - '0') + (NUC[1] - '0') * 10;  //俯仰角索引值解码
                                                         //俯仰角索引值匹配
    switch (NUC[0]) {
      case '0':
        Motor_Brake();
        break;
      case '1':
        Motor_Left_Rotation_Vision();
        break;
      case '2':
        Motor_Right_Rotation_Vision();
        break;
      default:
        break;
    }
    if (NUC[0] == 0) {
      MOTOR_ANGLE = Angle_Array[Angle_Index];
    }

    /*  UART6_Tx_Data[0] = '6';
				UART6_Tx_Data[1] = '5';
				UART6_Tx_Data[2] = '4';
				UART6_Tx_Data[3] = '7';
				UART6_Tx_Data[4] = '2';
				UART_Send_Data(&huart6, UART6_Tx_Data, 5);
				HAL_Delay(1000); */
  }

  // 1. 运动状态解析
  else if (cmd[0] == '0') {
    Motor_Brake();
    // return; // 刹车状态直接返回
  }
  else {
    // 2. 运动方向解析 (cmd[1]运动与cmd[4]高低速)
    switch (cmd[4]) {
      case '0': {  //低速模式
        switch (cmd[1]) {
          case '0':  // 左转
          {
            Motor_Left_Rotation();
          }  // 更新LED状态
          break;
          case '1':  // 右转
          {
            Motor_Right_Rotation();
          }  // 更新LED状态
          break;
          case '2': {  // 直线运动 (cmd[2]决定具体方向)
            switch (cmd[2]) {
              case '0': {
                Motor_Forward();
              } break;
              case '1': {
                Motor_Back();
              } break;
              case '2': {
                Motor_Left_Move();
              } break;
              case '3': {
                Motor_Right_Move();
              } break;
              default:
                break;
            }
          } break;
        }
      } break;
      case '1': {  // 高速模式
        switch (cmd[1]) {
          case '0':  // 高速左转
          {
            High_Motor_Left_Rotation();
          } break;
          case '1':  // 高速右转
          {
            High_Motor_Right_Rotation();
          } break;
          case '2': {  // 高速直线运动 (cmd[2]决定具体方向)
            switch (cmd[2]) {
              case '0': {
                High_Motor_Forward();
              } break;
              case '1': {
                High_Motor_Back();
              } break;
              case '2': {
                High_Motor_Left_Move();
              } break;
              case '3': {
                High_Motor_Right_Move();
              } break;
              default:
                break;
            }
          } break;
        }
      } break;
    }
  }

  // 3. 抬升机构解析 (cmd[3])
  if (cmd[3] == '1') {
    Lift_Up();
  }
  else if (cmd[3] == '2') {
    Lift_Down();
  }
  else if (cmd[3] == '0') {
    ;  // 步进停止
  }

  // 4. 俯仰角机构解析 (cmd[5])
  if (cmd[5] == '1') {
    Angle_up_manual();
  }
  else if (cmd[5] == '2') {
    Angle_down_manual();
  }
  else if (cmd[5] == '0') {
    Pitch_Stop();
  }
}

void Parse_Chassis_Command(uint8_t * cmd)
{
  robot_state = 0;  // 重置状态

  // 1. 运动状态解析
  if (cmd[0] == '0') {
    robot_state |= STATE_BRAKE;
    // Trans_LED_Status();
    return;  // 刹车状态直接返回
  }

  // 2. 运动方向解析 (cmd[1])
  switch (cmd[1]) {
    case '0':  // 左转
      robot_state |= STATE_LEFT_ROT;
      // Trans_LED_Status();
      break;
    case '1':  // 右转
      robot_state |= STATE_RIGHT_ROT;
      // Trans_LED_Status();
      break;
    case '2':  // 直线运动 (cmd[2]决定具体方向)
      switch (cmd[2]) {
        case '0': {
          robot_state |= STATE_FORWARD;
        } break;
        case '1': {
          robot_state |= STATE_BACK;
        } break;
        case '2': {
          robot_state |= STATE_LEFT_MOVE;
        } break;
        case '3': {
          robot_state |= STATE_RIGHT_MOVE;
        } break;
        default:
          break;
      }
      break;
  }

  // 3. 抬升机构解析 (cmd[3])
  if (cmd[3] == '0') {
    robot_state |= STATE_UP;
    // Trans_LED_Status();
  }
  else if (cmd[3] == '1') {
    robot_state |= STATE_DOWN;
    // Trans_LED_Status();
  }

  // 4. 速度模式解析 (cmd[4])
  if (cmd[4] == '1') {
    // 将常规速度转换为高速模式
    if (robot_state & STATE_FORWARD) {
      robot_state &= ~STATE_FORWARD;
      robot_state |= STATE_FORWARD_HIGH;
      // Trans_LED_Status();
    }
    if (robot_state & STATE_BACK) {
      robot_state &= ~STATE_BACK;
      robot_state |= STATE_BACK_HIGH;
      // Trans_LED_Status();
    }
    if (robot_state & STATE_LEFT_ROT) {
      robot_state &= ~STATE_LEFT_ROT;
      robot_state |= STATE_LEFT_ROT_HIGH;
      // Trans_LED_Status();
    }
    if (robot_state & STATE_RIGHT_ROT) {
      robot_state &= ~STATE_RIGHT_ROT;
      robot_state |= STATE_RIGHT_ROT_HIGH;
      // Trans_LED_Status();
    }
    if (robot_state & STATE_LEFT_MOVE) {
      robot_state &= ~STATE_LEFT_MOVE;
      robot_state |= STATE_LEFT_MOVE_HIGH;
      //  Trans_LED_Status();
    }
    if (robot_state & STATE_RIGHT_MOVE) {
      robot_state &= ~STATE_RIGHT_MOVE;
      robot_state |= STATE_RIGHT_MOVE_HIGH;
      // Trans_LED_Status();
    }
  }
}

void MoveOrder_Call_Back(uint8_t * rx_buffer, uint16_t Length)
{
  // 指令有效性检查
  if (Length != 8) return;

  Parse_Chassis_Command(rx_buffer);  // 解析有效载荷
                                     /* if (rx_buffer[6] == '1') {
				vision_control_flag = 1;
			}else{
				vision_control_flag = 0;
			}*/
  Trans_LED_Status();
}

// NUC 回调函数

void NUC_Control_Call_Back(uint8_t * rx_buffer, uint16_t Length)
{
  /*  if (!vision_control_flag){
			return;
		}else{
			do something;
		}  */
  //if(Length != 8) return;
  // Trans_LED_Status_Vision();
}
