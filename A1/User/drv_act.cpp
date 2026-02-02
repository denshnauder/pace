#include "drv_act.h"

uint16_t Counter_act = 0;

void user_delay_us(uint16_t us) 
{ 
	for(; us > 0; us--) 
	{ 
		for(uint8_t i = 50; i > 0; i--) 
		{ 
				; 
		} 
	} 
} 
/*微秒延时函数 */


void user_delay_ms(uint16_t ms) 
{ 
    for(; ms > 0; ms--) 
    { 
        user_delay_us(1000); 
    } 
} 

/*毫秒延时函数 */



////////////////////////*射球机构 *////////////////////////

///*射球抬升机构*///


void Step_Motor_Volt_Init()
{
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_0,GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_SET); 

}
/*打开步进电机驱动使能电压*/


void Lift_Up()
{
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET); 
	/*步进电机方向设置*/
	
	for (int paw_i = 0 ; paw_i<22500 ; paw_i++)
	{
		
		GPIOH->ODR ^= GPIO_PIN_12;
		user_delay_us(500);
       
		GPIOH->ODR ^= GPIO_PIN_12;       
		user_delay_us(500);
		
	}	
	/*步进电机脉冲*/	
	
	/*将步进速度设置为宏定义存放在act.h中，可在头文件中直接调整
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET); 
	
	for (int paw_i = 0 ; paw_i<200 ; paw_i++)
	{
		
		GPIOH->ODR ^= GPIO_PIN_12;
		user_delay_us(STEPPER_PULSE_DELAY_US);
        
		GPIOH->ODR ^= GPIO_PIN_12;       
		user_delay_us(STEPPER_PULSE_DELAY_US);
		
	}	
	*/
	
}
/*射球机构步进电机正转，抬升机构上升*/


void Lift_Down()
{
	
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET); 
	/*步进电机方向设置*/
	
	for (int paw_i = 0 ; paw_i<1000 ; paw_i++)
	{
		
		GPIOH->ODR ^= GPIO_PIN_12;
		user_delay_us(500);
        
		GPIOH->ODR ^= GPIO_PIN_12;       
		user_delay_us(500);
		
	}	
	/*步进电机脉冲*/	
	
	/*将步进速度设置为宏定义存放在act.h中，可在头文件中直接调整
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET); 
	
	for (int paw_i = 0 ; paw_i<200 ; paw_i++)
	{
		
		GPIOH->ODR ^= GPIO_PIN_12;
		user_delay_us(STEPPER_PULSE_DELAY_US);
        
		GPIOH->ODR ^= GPIO_PIN_12;       
		user_delay_us(STEPPER_PULSE_DELAY_US);
		
	}	
	*/
	
}
/*射球机构步进电机反转，抬升机构下降*/





////////////////////////*射球机构 *////////////////////////


///*运球抬升机构*///


///*此处缺少一个3508代码*///


///*运球转向机构*///


///*此处缺少一个2006代码*///


///*运球夹取机构*///


void Cylinder_Set()
{
	GPIOH->ODR ^= GPIO_PIN_10;
	//user_delay_ms(2000);
	/*放气，打开气爪*/
	/*充气，闭合气爪*/	
	
}
/*使能电磁阀，打开/闭合气爪*/


///*运球暂存机构*///


void Ball_Store()
{
	
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,2000);
	/*扫球*/

}
/*扫球进筐*/


void Ball_Drop()
{

	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,500);
	/*归位*/


}
/*扫球器归位*/





