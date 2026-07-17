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
#include "dma.h"
#include "fdcan.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_dwt.h"
#include "ICM42688_driver.h"
#include "bsp_imu.h"
#include "bsp_flash.h"
#include "string.h"
#include "can_comm.h"
#include "bsp_PWM.h"
#include "pid.h"
#include "can_comm.h"
#include "Mahony.h"
#include "ekf_attitude.h"
#include "data_processing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//          #define Calibrate //操作此宏定义决定是否校准
			#define Fdcan //操作此宏定义决定是否使用fdcan

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t dwt_count;
float dt_can;
float TempWheninit = 43.0f;
static uint32_t led_count;

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
#ifndef Fdcan
  MY_FDCAN1_Init();
#endif


  DWT_Init(170);
	
  while(init_flag)
	ICM42688_init();

  ICM42688_init();
  HAL_Delay(100);
  
#ifndef Fdcan
  can_comm_init();
#else
  fdcan_comm_init();
#endif
  
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
  HAL_Delay(100);
  PID_struct_init(&pid_temperature,POSITION_PID,2000, 300,1000, 20,0);	
	

#ifdef Calibrate
			
	Calibrate_MPU_Offset(&IMU_Data);
	
	imu_flash_write(IMU_Data.Calidata,5);
		
#else	
	float Bias[5];
	imu_flash_read(Bias,5);

  memcpy(IMU_Data.GyroOffset,Bias,3*sizeof(float));
	IMU_Data.AccelScale = Bias[3];
	IMU_Data.TempWhenCali = Bias[4]; 
	califlag = 1;
	bias_gyro_mode = Calibration_successful_mode;
#endif		


	 HAL_Delay(100);
	
    /*使能定时器1中断*/
   HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
      if(init_flag)
      IMU_AHRS_Calcu_task();
			
			
		if(bias_gyro_mode == Calibration_error_mode)
			led_count +=10;
		else
			led_count +=1;
		
		if(	led_count % 1000 == 0)
			 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);							
				
		
		
			DWT_Delay(0.00065f); 
						
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static uint32_t send_dnt = 0;
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	
	if(htim->Instance==TIM3)
	{
		dt_can = DWT_GetDeltaT(&dwt_count);
#ifndef Fdcan
	if(send_dnt++ % 2 == 0){
			can_std_transmit(&hfdcan1,0x011,imu_msg_send.pit_msg.array);
			can_std_transmit(&hfdcan1,0x012,imu_msg_send.yaw_msg.array);
		}
		else{
			can_std_transmit(&hfdcan1,0x013,imu_msg_send.rol_msg.array);
			can_std_transmit(&hfdcan1,0x014,imu_msg_send.cha_angle_msg.array);
		}
#else
		if(send_dnt++ % 2 == 0)
			can_std_transmit(&hfdcan1,0x016,imu_msg_send.all_angle_msg.array);
#endif
//		
		
		

//	can_std_transmit(&hfdcan1,0x011,imu_msg_send.pit_msg.array);
//	can_std_transmit(&hfdcan1,0x012,imu_msg_send.yaw_msg.array);

//	can_std_transmit(&hfdcan1,0x013,imu_msg_send.rol_msg.array);
//	can_std_transmit(&hfdcan1,0x014,imu_msg_send.cha_angle_msg.array);
		
	
//		
	
	//sentry
//	can_std_transmit(&hfdcan1,0x001,imu_msg_send.gim_w_msg.array);
//	can_std_transmit(&hfdcan1,0x002,imu_msg_send.gim_angle_msg.array);		

	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
