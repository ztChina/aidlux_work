/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "motor_control.h"
#include <string.h>
#include <stdlib.h>
#include "serial_command.h"
#include "../../dht11.h"
#include "motor_control.h"
extern TIM_HandleTypeDef htim1;  
void delay_us_TIM1(uint16_t us);
void delay_ms_TIM1(uint16_t ms);


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint8_t rx_data;  // 声明外部变量
extern uint8_t ble_rx_data;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "../../delay.h"  // 新�?�：引用延时函数头文�?
#include "../../ble.h"  // 新�?�：引用蓝牙头文�?
#include "../../DFRobot.h"  // 新�?�：引用蓝牙头文�?
#include "../../bsp_hx711.h"  // 新�?�：引用压力传感器头文件
#include "../../ds18b20.h"
#include <string.h>

/**
  * @brief  获取土壤湿度百分比（0~100%�?
  * @note   ADC值与湿度成反比，需反向计算
  */
float Get_Soil_Humidity(uint32_t adc_value) {
    // 约束ADC值在有效范围�?
    if (adc_value >= AIR_VALUE) return 0.0f;
    if (adc_value <= WATER_VALUE) return 100.0f;
    
    // 反比计算�?�?
    return 100.0f * (AIR_VALUE - adc_value) / (AIR_VALUE - WATER_VALUE);
}

/**
  * @brief  根据湿度百分比返回状�?
  */
const char* Get_Humidity_Status(float humidity) {
    if (humidity > 66.0f) return "Very Wet";
    else if (humidity > 33.0f) return "Wet";
    else return "Dry";
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
		

	
		/*PlayAnimation_Namal();
		PlayAnimation_Sad();
		PlayAnimation_Adore(); */
	
	
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
	SerialCommand_Init();




    
    // ³õʼ»¯HX711
    Init_HX711pin();
    
    // 获取毛皮重量（去皮）
    Get_Maopi();
//		
//			 		DS18B20_Init();             // ?????DS18B20
//   DS18B20_Start();               // ?????????
//		 if (DS18B20_Init()) {
//        printf("DS18B20 Not Found!\r\n");
//    }
//    printf("DS18B20 Ready on PB15!\r\n");
//	
//    printf("System Initialized\r\n");
//    printf("HX711 Weight Measurement Demo\r\n");

//		int16_t temp_1 = DS18B20_GetTemp();
//    printf("Temperature: %.1f 度 \r\n", temp_1 / 10.0);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   // 初始化OLED
   OLED_Init();
   OLED_Clear(); // 清屏
   
   /* 第一屏显示 */
   // 显示第一行：记录
	 

   
   // 显示第二行：2025年7月23日
//   OLED_ShowNum(0, 2, 2025, 4, 16);  // 年份
//   OLED_ShowCHinese(32, 2, 2);       // "年"
//   OLED_ShowNum(48, 2, 7, 2, 16);    // 月份
//   OLED_ShowCHinese(64, 2, 3);       // "月"
//   OLED_ShowNum(80, 2, 23, 2, 16);   // 日期
//   OLED_ShowCHinese(96, 2, 4);       // "日"
//   
//   // 显示第三行：Hello World!
//   OLED_ShowString(0, 6, "Hello World!", 12);
//   
//   // 保持显示1秒
//   HAL_Delay(1000);
//   
//   /* 第二屏显示 */
//   OLED_Clear(); // 清屏  
//   OLED_ShowCHinese(0,  0, 5); // "团" 
//   OLED_ShowCHinese(16, 0, 6); // "对"
//   OLED_ShowCHinese(32, 0, 7); // "成"
//   OLED_ShowCHinese(48, 0, 8); // "员"
//	 OLED_ShowCHinese(32,4, 9); // "团" 
//   OLED_ShowCHinese(48,4, 10); // "对"
//   OLED_ShowCHinese(64,4, 11); // "成"
//	    // 保持显示1秒
//   HAL_Delay(1000);
//   OLED_Clear(); // 清屏
//	 OLED_ShowCHinese(0,  0, 5); // "团" 
//   OLED_ShowCHinese(16, 0, 6); // "对"
//   OLED_ShowCHinese(32, 0, 7); // "成"
//   OLED_ShowCHinese(48, 0, 8); // "员"
//	 OLED_ShowCHinese(32,4, 12); // "团" 
//   OLED_ShowCHinese(48,4, 13); // "对"
//   OLED_ShowCHinese(64,4, 14); // "成"
//	    // 保持显示1秒
//   HAL_Delay(1000);
//	 OLED_Clear(); // 清屏
//	 OLED_ShowCHinese(0,  0, 5); // "团" 
//   OLED_ShowCHinese(16, 0, 6); // "对"
//   OLED_ShowCHinese(32, 0, 7); // "成"
//   OLED_ShowCHinese(48, 0, 8); // "员"
//	 OLED_ShowCHinese(32,4, 15); // "团" 
//   OLED_ShowCHinese(48,4, 16); // "对"
//   OLED_ShowCHinese(64,4, 17); // "成"
//	 
//	    HAL_Delay(1000);
//	 OLED_Clear(); // 清屏
//	 OLED_ShowCHinese(0,  0, 5); // "团" 
//   OLED_ShowCHinese(16, 0, 6); // "对"
//   OLED_ShowCHinese(32, 0, 7); // "成"
//   OLED_ShowCHinese(48, 0, 8); // "员"
//	 OLED_ShowCHinese(48,4, 18); // "团" 
//   OLED_ShowCHinese(64,4, 19); // "对"

//	    HAL_Delay(1000);
//	 OLED_Clear(); // 清屏
//	 OLED_ShowCHinese(0,  0, 5); // "团" 
//   OLED_ShowCHinese(16, 0, 6); // "对"
//   OLED_ShowCHinese(32, 0, 7); // "成"
//   OLED_ShowCHinese(48, 0, 8); // "员"
//	 OLED_ShowCHinese(32,4, 20); // "团" 
//   OLED_ShowCHinese(48,4, 21); // "对"
//   OLED_ShowCHinese(64,4, 22); // "成"
//	    HAL_Delay(1000);
//	 OLED_Clear(); // 清屏
//	 OLED_ShowCHinese(0,  0, 5); // "团" 
//   OLED_ShowCHinese(16, 0, 6); // "对"
//   OLED_ShowCHinese(32, 0, 7); // "成"
//   OLED_ShowCHinese(48, 0, 8); // "员"
//	 OLED_ShowCHinese(48,4, 23); // "团" 
//   OLED_ShowCHinese(64,4, 24); // "对"

//	    HAL_Delay(1000);
//	 OLED_Clear(); // 清屏
//	 OLED_ShowCHinese(0,  0, 5); // "团" 
//   OLED_ShowCHinese(16, 0, 6); // "对"
//   OLED_ShowCHinese(32, 0, 7); // "成"
//   OLED_ShowCHinese(48, 0, 8); // "员"
//	 OLED_ShowCHinese(32,4, 25); // "团" 
//   OLED_ShowCHinese(48,4, 26); // "对"
//   OLED_ShowCHinese(64,4, 27); // "成"
//	 

  while (1)
  {
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);  // 切换PC6引脚电平
//		    HAL_Delay(1000);
//				int16_t temp_1 = DS18B20_GetTemp();
//        printf("Temperature: %.1f度 \r\n", temp_1 / 10.0);
	      // 正转
//        Motor_Set(600, 600, 1, 1);
//        HAL_Delay(5);
//        Motor_UpdateSpeed();
//        // 反转
//        Motor_Set(600, 600, 0, 0);
//        HAL_Delay(5);
//        Motor_UpdateSpeed();
//        // 停�??
//        Motor_Set(0, 0, 0, 0);
//        HAL_Delay(5);
//        Motor_UpdateSpeed();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//				Get_Weight();        
//        // 使用Weight_Shiwu获取重量值
//        printf("Current Weight: %d g\r\n", Weight_Shiwu);        
//        HAL_Delay(500);							
//		    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10);
//        printf("DHT11 Test Start\r\n");
//        uint8_t temp, humi;				
//				HAL_UART_Receive_IT(&huart2, &ble_rx_data, 1);  
//				printf("System Ready. Waiting for Bluetooth Data...\r\n");		
//					
//				uint32_t moisture = Read_Soil_Moisture();
//        printf("ADC Value: %lu \r\n", moisture);  // 打印原�?�ADC�?
//        Check_Moisture_Level(moisture);        // 判断湿度等级
//        uint32_t adc_value = Read_Soil_Moisture();  // 读取ADC�?
//        float humidity = Get_Soil_Humidity(adc_value); // 计算湿度百分�?
//        const char* status = Get_Humidity_Status(humidity);
//        printf("ADC: %lu, Humidity: %.1f%%, Status: %s \r\n", 
//               adc_value, humidity, status);							 
//        HAL_Delay(500);	
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART3) {
//        SerialCommand_ProcessChar(rx_data);
//        HAL_UART_Receive_IT(&huart3, &rx_data, 1);  // 继续接收
//    } else if (huart->Instance == USART2) {
//        BLE_UART_ReceiveCallback(ble_rx_data);
//        HAL_UART_Receive_IT(&huart2, &ble_rx_data, 1);  // 继续接收
//    }
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
