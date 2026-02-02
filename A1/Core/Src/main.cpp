/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "alg_pid.h"
#include "drv_act.h"
#include "drv_bsp.h"
#include "drv_can.h"
#include "drv_communication.h"
#include "drv_motion.h"
#include "drv_uart.h"
#include "dvc_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_CAN2_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  Uart_Init(&huart2, rx_buffer, 8, MoveOrder_Call_Back);
  Uart_Init(&huart6, NUC_rx_buffer, 8, NUC_Control_Call_Back);
  BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
  CAN_Init(&hcan1, CAN_Motor_Call_Back);
  CAN_Init(&hcan2, CAN_Motor_Call_Back);
  CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
  CAN_Filter_Mask_Config(&hcan2, CAN_FILTER(14) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);

  motor1.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
  motor1.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor1.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_OMEGA, 1.0f);

  motor2.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
  motor2.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor2.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_OMEGA, 1.0f);

  motor3.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 15.0f * PI);
  motor3.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor3.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA, 1.0f);

  motor4.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 20.0f * PI);
  motor4.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor4.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_OMEGA, 1.0f);

  motor5.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 30.0f * PI);
  motor5.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor5.Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_OMEGA, 1.0f);

  motor6.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 30.0f * PI);
  motor6.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor6.Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_OMEGA, 1.0f);

  motor7.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 20.0f * PI);
  motor7.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor7.Init(&hcan1, CAN_Motor_ID_0x207, Control_Method_OMEGA, 1.0f);

  motor8.PID_Angle.Init(100.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 20.0f * PI);
  motor8.PID_Omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 12500.0f);
  motor8.Init(&hcan1, CAN_Motor_ID_0x208, Control_Method_ANGLE, 1.0f);

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);
  // uint16_t current= 1000;
  // CAN1_0x200_Tx_Data[6] = (uint8_t)(current >> 8); // 高 8 位
  // CAN1_0x200_Tx_Data[7] = (uint8_t)(current);      // 低 8 位

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    Parse_Motion(rx_buffer, NUC_rx_buffer);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
