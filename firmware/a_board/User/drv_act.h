#ifndef DRV_ACT_H
#define DRV_ACT_H


#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "alg_pid.h"
#include "drv_can.h"
#include "dvc_motor.h"

#define STEPPER_PULSE_DELAY_US    30

extern Class_Motor_C620 motor1;
extern Class_Motor_C620 motor2;
extern Class_Motor_C620 motor3;
extern Class_Motor_C620 motor4;
extern Class_Motor_C620 motor5;
extern Class_Motor_C620 motor6;
extern Class_Motor_C620 motor7;
extern Class_Motor_C620 motor8;




void user_delay_us(uint16_t us);
void user_delay_ms(uint16_t ms);
void Step_Motor_Volt_Init();
void Cylinder_Set();
void paw_rise();
void paw_decline();
void paw_out();
void paw_in();
void paw_dierction_out();
void paw_dierction_in();

void ball_motor_rise();
void ball_motor_decline();

void ball_sweep_on();
void ball_sweep_off();

void ball_suction_on();
void ball_suction_off();

void ball_shoot_on();
void ball_shoot_off();

void clamp_up();
void clamp_down();

void Lift_Up();
void Lift_Down();



#endif
