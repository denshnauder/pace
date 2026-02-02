#ifndef DRV_MOTION_H
#define DRV_MOTION_H

#include "alg_pid.h"
#include "drv_can.h"
#include "dvc_motor.h"
#include "stm32f4xx_hal.h"

// 宏定义
#define MOTOR_BASE_SPEED 30.0f * PI
#define MOTOR_HIGH_SPEED 60.0f * PI
#define FRICTION_WHEEL_SPEED 180.0f * PI
#define MOTOR_SPEED_VISION 3.0f * PI
#define MOTOR_BASE_Angle 0.5 * PI

#define STATE_BRAKE 0x8000
#define STATE_FORWARD 0x0001
#define STATE_BACK 0x0002
#define STATE_LEFT_ROT 0x0004
#define STATE_RIGHT_ROT 0x0008
#define STATE_LEFT_MOVE 0x0010
#define STATE_RIGHT_MOVE 0x0020
#define STATE_FORWARD_HIGH 0x0040
#define STATE_BACK_HIGH 0x0080
#define STATE_LEFT_ROT_HIGH 0x0100
#define STATE_RIGHT_ROT_HIGH 0x0200
#define STATE_LEFT_MOVE_HIGH 0x0400
#define STATE_RIGHT_MOVE_HIGH 0x0800
#define STATE_UP 0x1000
#define STATE_DOWN 0x2000

// ==========================================================
// C 链接区域 - 这里的变量和函数可以被 main.c (纯C) 访问
// ==========================================================
#ifdef __cplusplus
extern "C" {
#endif

// 🔴 修正：只加 extern 声明，不要赋值，不要放电机对象
extern uint16_t robot_state;

// C 函数声明
void Motor_Forward();
void Motor_Back();
void Motor_Left_Rotation();
void High_Motor_Right_Rotation();
void High_Motor_Left_Rotation();
void Motor_Right_Rotation();
void Motor_Left_Move();
void Motor_Right_Move();
void High_Motor_Forward();
void High_Motor_Back();
void High_Motor_Left_Move();
void High_Motor_Right_Move();
void Motor_Stop();
void Motor_Brake();
void Angle_Up();
void Angle_Down();
void Angle_up_manual();
void Angle_down_manual();
void Pitch_Stop();
void Motor_Left_Rotation_Vision();
void Motor_Right_Rotation_Vision();
void Servo_SetAngle(TIM_HandleTypeDef * htim, uint32_t Channel, float angle);

#ifdef __cplusplus
}
#endif

// ==========================================================
// C++ 链接区域 - 放置 C++ 类对象和变量的声明
// ==========================================================

// 🔴 修正：必须放在 extern "C" 外面
extern Class_Motor_C620 motor1;
extern Class_Motor_C620 motor2;
extern Class_Motor_C620 motor3;
extern Class_Motor_C620 motor4;
extern Class_Motor_C620 motor5;
extern Class_Motor_C620 motor6;
extern Class_Motor_C620 motor7;
extern Class_Motor_C620 motor8;

// 🔴 修正：只声明，不要赋值
extern uint16_t MOTOR_ANGLE;

void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer * Rx_Buffer);

#endif