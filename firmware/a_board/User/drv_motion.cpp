#include "drv_motion.h"

#include "drv_act.h"

uint16_t Rx_Omega1, Rx_Omega2, Rx_Omega3, Rx_Omega4, Rx_Omega5, Rx_Omega6, Rx_Omega7, Rx_Omega8;

float Target_Angle1, Now_Angle1, Target_Omega1, Now_Omega1, Target_Angle2, Now_Angle2,
  Target_Omega2, Now_Omega2, Target_Angle3, Now_Angle3, Target_Omega3, Now_Omega3, Target_Angle4,
  Now_Angle4, Target_Omega4, Now_Omega4, Target_Angle5, Now_Angle5, Target_Omega5, Now_Omega5,
  Target_Angle6, Now_Angle6, Target_Omega6, Now_Omega6, Target_Angle7, Now_Angle7, Target_Omega7,
  Now_Omega7, Target_Angle8, Now_Angle8, Target_Omega8, Now_Omega8;

uint32_t Counter = 0;
uint32_t Counter_Ball = 0;

uint32_t Output1, Output2, Output3, Output4, Output5, Output6, Output7, Output8;
// 1. 定义 C++ 类对象
Class_Motor_C620 motor1;
Class_Motor_C620 motor2;
Class_Motor_C620 motor3;
Class_Motor_C620 motor4;
Class_Motor_C620 motor5;
Class_Motor_C620 motor6;
Class_Motor_C620 motor7;
Class_Motor_C620 motor8;

// 2. 定义普通变量并赋初值
uint16_t MOTOR_ANGLE = 0;  // 之前你在头文件里写了赋值，要移到这里
uint16_t robot_state = 0;
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer * Rx_Buffer)
{
  switch (Rx_Buffer->Header.StdId) {
    case (0x201): {
      motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
    case (0x202): {
      motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
    case (0x203): {
      motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
    case (0x204): {
      motor4.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
    case (0x205): {
      motor5.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
    case (0x206): {
      motor6.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
    case (0x207): {
      motor7.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
    case (0x208): {
      motor8.CAN_RxCpltCallback(Rx_Buffer->Data);
    } break;
  }
}

void Motor_Forward()
{
  /* motor1.Set_Target_Omega(-30.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(-30.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(30.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(30.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Back()
{
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Left_Rotation()
{
  /*motor1.Set_Target_Omega(30.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(30.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(30.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(30.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Right_Rotation()
{
  /* Counter = 0;
	    while(Counter <= 10)
	{
		  Counter++ ;
	    motor1.Set_Target_Omega(-30.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(-30.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(-30.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(-30.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();
		
		  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
		  HAL_Delay(0) ;
	}*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}
void High_Motor_Left_Rotation()
{
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void High_Motor_Right_Rotation()
{
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Right_Move()
{
  /*Counter = 0;
	    while(Counter <= 50)
	{
		  Counter++ ;
	    motor1.Set_Target_Omega(-30.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(30.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(30.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(-30.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();
		
		  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
		  HAL_Delay(0) ;
	}*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Left_Move()
{
  /*Counter = 0;
	    while(Counter <= 50)
	{
		  Counter++ ;
	    motor1.Set_Target_Omega(30.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(-30.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(-30.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(30.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();
		
		  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
		  HAL_Delay(0) ;
	}*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void High_Motor_Forward()
{
  /*Counter = 0;
	    while(Counter <= 5)
	{
		  Counter++;
	    motor1.Set_Target_Omega(-60.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(-60.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(60.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(60.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();
		
		  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
		  HAL_Delay(0) ;
	}	*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void High_Motor_Back()
{
  /*Counter = 0;
	    while(Counter <= 5)
	{
		  Counter++ ;
	    motor1.Set_Target_Omega(60.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(60.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(-60.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(-60.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();
		
		  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
		  HAL_Delay(0) ;
	}	*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void High_Motor_Left_Move()
{
  /*Counter = 0;
	    while(Counter <= 5)
	{
		  Counter++ ;
	    motor1.Set_Target_Omega(60.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(-60.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(-60.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(60.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();
		
		  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
		  HAL_Delay(0) ;
	}*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void High_Motor_Right_Move()
{
  /*Counter = 0;
	    while(Counter <= 5)
	{
		  Counter++ ;
	    motor1.Set_Target_Omega(-60.0f * PI);
      motor1.TIM_PID_PeriodElapsedCallback();
		  motor2.Set_Target_Omega(60.0f * PI);
      motor2.TIM_PID_PeriodElapsedCallback();
		  motor3.Set_Target_Omega(60.0f * PI);
      motor3.TIM_PID_PeriodElapsedCallback();
		  motor4.Set_Target_Omega(-60.0f * PI);
      motor4.TIM_PID_PeriodElapsedCallback();
		
		  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
		  HAL_Delay(0) ;
	}*/
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(-MOTOR_HIGH_SPEED);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(MOTOR_HIGH_SPEED);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Stop()
{
  Counter = 0;
  while (Counter <= 50) {
    //������ݵ����
    Counter++;
    /*CAN1_0x200_Tx_Data[0] = 0;
        CAN1_0x200_Tx_Data[1] = 0;
		    CAN1_0x200_Tx_Data[2] = 0;
        CAN1_0x200_Tx_Data[3] = 0;
		    CAN1_0x200_Tx_Data[4] = 0;
        CAN1_0x200_Tx_Data[5] = 0;
		    CAN1_0x200_Tx_Data[6] = 0;
        CAN1_0x200_Tx_Data[7] = 0;
        CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);*/
    CAN1_0x1ff_Tx_Data[0] = 0;
    CAN1_0x1ff_Tx_Data[1] = 0;
    CAN1_0x1ff_Tx_Data[2] = 0;
    CAN1_0x1ff_Tx_Data[3] = 0;
    CAN1_0x1ff_Tx_Data[4] = 0;
    CAN1_0x1ff_Tx_Data[5] = 0;
    CAN1_0x1ff_Tx_Data[6] = 0;
    CAN1_0x1ff_Tx_Data[7] = 0;

    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    HAL_Delay(1);
  }
}

void Motor_Brake()
{
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(0);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(0);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(0);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(0);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

////////////////////////*射球机构 *////////////////////////
///*射球角度机构*///

void Angle_Up()
{
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 25000) {
    Counter++;
    motor5.Set_Control_Method(Control_Method_ANGLE);
    motor5.Set_Target_Angle(MOTOR_ANGLE);
    motor5.TIM_PID_PeriodElapsedCallback();
    motor6.Set_Control_Method(Control_Method_ANGLE);
    motor6.Set_Target_Angle(-MOTOR_ANGLE);
    motor6.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    HAL_Delay(0);
  }
}
/*M3508电机带动蜗杆，角度增大*/

void Angle_Down()
{
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 2500) {
    Counter++;
    motor5.Set_Control_Method(Control_Method_ANGLE);
    motor5.Set_Target_Angle(0);
    motor5.TIM_PID_PeriodElapsedCallback();
    motor6.Set_Control_Method(Control_Method_ANGLE);
    motor6.Set_Target_Angle(0);
    motor6.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Angle_up_manual()
{
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor5.Set_Control_Method(Control_Method_OMEGA);
    motor5.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor5.TIM_PID_PeriodElapsedCallback();
    motor6.Set_Control_Method(Control_Method_OMEGA);
    motor6.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor6.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Angle_down_manual()
{
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor5.Set_Control_Method(Control_Method_OMEGA);
    motor5.Set_Target_Omega(-MOTOR_BASE_SPEED);
    motor5.TIM_PID_PeriodElapsedCallback();
    motor6.Set_Control_Method(Control_Method_OMEGA);
    motor6.Set_Target_Omega(MOTOR_BASE_SPEED);
    motor6.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Pitch_Stop()
{
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor5.Set_Control_Method(Control_Method_OMEGA);
    motor5.Set_Target_Omega(0);
    motor5.TIM_PID_PeriodElapsedCallback();
    motor6.Set_Control_Method(Control_Method_OMEGA);
    motor6.Set_Target_Omega(0);
    motor6.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Left_Rotation_Vision()
{
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(MOTOR_SPEED_VISION);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(MOTOR_SPEED_VISION);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(MOTOR_SPEED_VISION);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(MOTOR_SPEED_VISION);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}

void Motor_Right_Rotation_Vision()
{
  //将转速设置为宏定义存放在motion.h中，可在头文件中直接调整
  Counter = 0;
  while (Counter <= 50) {
    Counter++;
    motor1.Set_Target_Omega(-MOTOR_SPEED_VISION);
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.Set_Target_Omega(-MOTOR_SPEED_VISION);
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.Set_Target_Omega(-MOTOR_SPEED_VISION);
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.Set_Target_Omega(-MOTOR_SPEED_VISION);
    motor4.TIM_PID_PeriodElapsedCallback();

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    HAL_Delay(0);
  }
}
void Servo_SetAngle(TIM_HandleTypeDef * htim, uint32_t Channel, float angle)
{
  // angle范围：0~180°
  uint16_t pulse = 500 + (angle / 180.0f) * 2000;  // 脉宽范围约0.5~2.5ms
  __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}
