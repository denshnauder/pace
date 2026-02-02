#ifndef DRV_COMMUNICATION_H
#define DRV_COMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "drv_uart.h"
#include "drv_act.h"
#include "drv_motion.h"

void MoveOrder_Call_Back(uint8_t *Buffer, uint16_t Length);
void Parse_Chassis_Command(uint8_t *cmd);
void Parse_Motion(uint8_t *cmd, uint8_t *NUC);
//extern uint8_t vision_control_flag;
void NUC_Control_Call_Back(uint8_t *rx_buffer, uint16_t Length);
extern uint8_t NUC_rx_buffer[8]; 
#endif
