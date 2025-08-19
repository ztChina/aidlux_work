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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


uint32_t encoderCount_TIM3 = 0; // TIM3 ������������
uint32_t encoderCount_TIM4 = 0; // TIM4 ������������
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR1_DIR1_PIN GPIO_PIN_15
#define MOTOR1_DIR1_PORT GPIOB
#define MOTOR1_DIR2_PIN GPIO_PIN_0
#define MOTOR1_DIR2_PORT GPIOF

#define MOTOR2_DIR1_PIN GPIO_PIN_1
#define MOTOR2_DIR1_PORT GPIOF
#define MOTOR2_DIR2_PIN GPIO_PIN_2
#define MOTOR2_DIR2_PORT GPIOF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 全局变量存储速度
int32_t motor1_speed = 0;
int32_t motor2_speed = 0;

//// 获取编码器速度函数
//int32_t GetEncoderSpeed(TIM_HandleTypeDef *htim) {
//    static uint32_t last_count_tim3 = 0;
//    static uint32_t last_count_tim4 = 0;
//    uint32_t current_count;
//    int32_t speed = 0;
//    
//    if(htim->Instance == TIM3) {
//        current_count = __HAL_TIM_GET_COUNTER(htim);
//        speed = (int32_t)(current_count - last_count_tim3);
//        last_count_tim3 = current_count;
//    }
//    else if(htim->Instance == TIM4) {
//        current_count = __HAL_TIM_GET_COUNTER(htim);
//        speed = (int32_t)(current_count - last_count_tim4);
//        last_count_tim4 = current_count;
//    }
//    
//    // 处理计数器溢出（16位计数器）
//    if(speed > 32767) speed -= 65536;
//    if(speed < -32767) speed += 65536;
//    
//    return speed;
//}

//// 获取绝对位置计数
//uint32_t GetEncoderPosition(TIM_HandleTypeDef *htim) {
//    return __HAL_TIM_GET_COUNTER(htim);
//}

//// 测速主函数 - 每100ms调用一次
//void Encoder_Speed_Measurement(void) {
//    static uint32_t last_time = 0;
//    uint32_t current_time = HAL_GetTick();
//    
//    // 每100ms测量一次速度
//    if(current_time - last_time >= 100) {
//        last_time = current_time;
//        
//        // 读取编码器速度
//        motor1_speed = GetEncoderSpeed(&htim3);
//        motor2_speed = GetEncoderSpeed(&htim4);
//        
//        // 读取编码器位置
//        uint32_t position1 = GetEncoderPosition(&htim3);
//        uint32_t position2 = GetEncoderPosition(&htim4);
//        
//        // 打印速度和位置信息
//        printf("Motor1: Speed=%ld, Position=%lu\n", motor1_speed, position1);
//        printf("Motor2: Speed=%ld, Position=%lu\n", motor2_speed, position2);
//        printf("--------------------------------\n");
//    }
//}

//// 连续测速函数 - 可以在主循环中持续调用
//void Continuous_Speed_Measurement(void) {
//    static uint32_t last_time = 0;
//    uint32_t current_time = HAL_GetTick();
//    
//    // 每50ms更新一次
//    if(current_time - last_time >= 50) {
//        last_time = current_time;
//        
//        motor1_speed = GetEncoderSpeed(&htim3);
//        motor2_speed = GetEncoderSpeed(&htim4);
//        
//        // 实时打印速度
//        printf("Speed1: %6ld, Speed2: %6ld\n", motor1_speed, motor2_speed);
//    }
//}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// ����2����׼���ض���������MicroLIB��Full printf��
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetMotorSpeed(uint16_t duty) {
    if(duty > 1000) duty = 1000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
	// 启动编码器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	SetMotorSpeed(300);
	 HAL_GPIO_WritePin(MOTOR1_DIR1_PORT, MOTOR1_DIR1_PIN, GPIO_PIN_RESET); // DIR1 = HIGH
   HAL_GPIO_WritePin(MOTOR1_DIR2_PORT, MOTOR1_DIR2_PIN, GPIO_PIN_SET); // DIR2 = LOW
	 HAL_GPIO_WritePin(MOTOR2_DIR1_PORT, MOTOR2_DIR1_PIN, GPIO_PIN_RESET); // DIR1 = HIGH
   HAL_GPIO_WritePin(MOTOR2_DIR2_PORT, MOTOR2_DIR2_PIN, GPIO_PIN_SET); // DIR2 = LOW
	printf("hello");
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		// ��ȡ TIM3 �� TIM4 �ı�����������ֵ
    encoderCount_TIM3 = __HAL_TIM_GET_COUNTER(&htim3);
    encoderCount_TIM4 = __HAL_TIM_GET_COUNTER(&htim4);

    // �����ٶȣ�����ÿת��Ӧ��������Ϊ 4096��
    float speed_TIM3 = (float)encoderCount_TIM3 / 4096.0 * 60.0; // ת/����
    float speed_TIM4 = (float)encoderCount_TIM4 / 4096.0 * 60.0; // ת/����

    // ����������������Ҫʵʱ�ٶȣ�
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // ��ӡ�ٶ���Ϣ
    printf("Motor Speed (TIM3): %.2f RPM\n", speed_TIM3);
    printf("Motor Speed (TIM4): %.2f RPM\n", speed_TIM4);

    // ��ʱһ��ʱ���ٶ�ȡ��һ���ٶ�
    HAL_Delay(1000); // ÿ���ӡһ��
		
		
		
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
