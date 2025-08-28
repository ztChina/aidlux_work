/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../delay.h"  // ��???��������ʱ����ͷ��??
#include "../../ble.h"  // ��???����������ͷ��??
#include "../../DFRobot.h"  // ��???����������ͷ��??
#include "../../bsp_hx711.h"  // ��???������ѹ��������ͷ�ļ�
#include "../../ds18b20.h"
#include <string.h>
#include <stdio.h>
#include "motor_control.h"
#include <string.h>
#include <stdlib.h>
#include "serial_command.h"
#include "../../dht11.h"
#include "motor_control.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask_LED */
osThreadId_t defaultTask_LEDHandle;
const osThreadAttr_t defaultTask_LED_attributes = {
  .name = "defaultTask_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DS18B20 */
osThreadId_t DS18B20Handle;
const osThreadAttr_t DS18B20_attributes = {
  .name = "DS18B20",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DFROBOT */
osThreadId_t DFROBOTHandle;
const osThreadAttr_t DFROBOT_attributes = {
  .name = "DFROBOT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BLE */
osThreadId_t BLEHandle;
const osThreadAttr_t BLE_attributes = {
  .name = "BLE",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_TASK */
osThreadId_t UART_TASKHandle;
const osThreadAttr_t UART_TASK_attributes = {
  .name = "UART_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask08 */
osThreadId_t myTask08Handle;
const osThreadAttr_t myTask08_attributes = {
  .name = "myTask08",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motor */
osThreadId_t motorHandle;
const osThreadAttr_t motor_attributes = {
  .name = "motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t rx_data;  // �����ⲿ����



int32_t encoderCount_TIM3 = 0; // TIM3 ������������
int32_t encoderCount_TIM4 = 0; // TIM4 ������������


int32_t motor1_speed = 0;
int32_t motor2_speed = 0;



#define MOTOR1_DIR1_PIN GPIO_PIN_1
#define MOTOR1_DIR1_PORT GPIOD
#define MOTOR1_DIR2_PIN GPIO_PIN_2
#define MOTOR1_DIR2_PORT GPIOD

#define MOTOR2_DIR1_PIN GPIO_PIN_3
#define MOTOR2_DIR1_PORT GPIOD
#define MOTOR2_DIR2_PIN GPIO_PIN_4
#define MOTOR2_DIR2_PORT GPIOD




void SetMotorSpeed(uint16_t duty) {
    if(duty > 1000) duty = 1000;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);
void StartTask09(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
//  /* creation of defaultTask_LED */
//  defaultTask_LEDHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_LED_attributes);

//  /* creation of OLED */
//  OLEDHandle = osThreadNew(StartTask03, NULL, &OLED_attributes);

//  /* creation of DS18B20 */
//  DS18B20Handle = osThreadNew(StartTask04, NULL, &DS18B20_attributes);

//  /* creation of DFROBOT */
//  DFROBOTHandle = osThreadNew(StartTask04, NULL, &DFROBOT_attributes);

//  /* creation of BLE */
//  BLEHandle = osThreadNew(StartTask05, NULL, &BLE_attributes);

//  /* creation of UART_TASK */
//  UART_TASKHandle = osThreadNew(StartTask06, NULL, &UART_TASK_attributes);

//  /* creation of myTask07 */
//  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

//  /* creation of myTask08 */
//  myTask08Handle = osThreadNew(StartTask08, NULL, &myTask08_attributes);

//  /* creation of motor */
  motorHandle = osThreadNew(StartTask09, NULL, &motor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask_LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);  
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);  
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
//   OLED_Clear();
//	 OLED_ShowNum(0, 2, 2025, 4, 16);  // ���
//   OLED_ShowCHinese(32, 2, 2);       // "��"
//   OLED_ShowNum(48, 2, 7, 2, 16);    // �·�
//   OLED_ShowCHinese(64, 2, 3);       // "��"
//   OLED_ShowNum(80, 2, 23, 2, 16);   // ����
//   OLED_ShowCHinese(96, 2, 4);       // "��"
//   
//   // ��ʾ�����У�Hello World!
//   OLED_ShowString(0, 6, "Hello World!", 12);
//   
//   // ������ʾ1��
//   HAL_Delay(1000);
//   
//   /* �ڶ�����ʾ */
//   OLED_Clear(); // ����  
//   OLED_ShowCHinese(0,  0, 5); // "��" 
//   OLED_ShowCHinese(16, 0, 6); // "��"
//   OLED_ShowCHinese(32, 0, 7); // "��"
//   OLED_ShowCHinese(48, 0, 8); // "Ա"
//	 OLED_ShowCHinese(32,4, 9); // "��" 
//   OLED_ShowCHinese(48,4, 10); // "��"
//   OLED_ShowCHinese(64,4, 11); // "��"
//	    // ������ʾ1��
//   HAL_Delay(1000);
//   OLED_Clear(); // ����
//	 OLED_ShowCHinese(0,  0, 5); // "��" 
//   OLED_ShowCHinese(16, 0, 6); // "��"
//   OLED_ShowCHinese(32, 0, 7); // "��"
//   OLED_ShowCHinese(48, 0, 8); // "Ա"
//	 OLED_ShowCHinese(32,4, 12); // "��" 
//   OLED_ShowCHinese(48,4, 13); // "��"
//   OLED_ShowCHinese(64,4, 14); // "��"
//	    // ������ʾ1��
//   HAL_Delay(1000);
//	 OLED_Clear(); // ����
//	 OLED_ShowCHinese(0,  0, 5); // "��" 
//   OLED_ShowCHinese(16, 0, 6); // "��"
//   OLED_ShowCHinese(32, 0, 7); // "��"
//   OLED_ShowCHinese(48, 0, 8); // "Ա"
//	 OLED_ShowCHinese(32,4, 15); // "��" 
//   OLED_ShowCHinese(48,4, 16); // "��"
//   OLED_ShowCHinese(64,4, 17); // "��"
//	 
//	    HAL_Delay(1000);
//	 OLED_Clear(); // ����
//	 OLED_ShowCHinese(0,  0, 5); // "��" 
//   OLED_ShowCHinese(16, 0, 6); // "��"
//   OLED_ShowCHinese(32, 0, 7); // "��"
//   OLED_ShowCHinese(48, 0, 8); // "Ա"
//	 OLED_ShowCHinese(48,4, 18); // "��" 
//   OLED_ShowCHinese(64,4, 19); // "��"

//	    HAL_Delay(1000);
//	 OLED_Clear(); // ����
//	 OLED_ShowCHinese(0,  0, 5); // "��" 
//   OLED_ShowCHinese(16, 0, 6); // "��"
//   OLED_ShowCHinese(32, 0, 7); // "��"
//   OLED_ShowCHinese(48, 0, 8); // "Ա"
//	 OLED_ShowCHinese(32,4, 20); // "��" 
//   OLED_ShowCHinese(48,4, 21); // "��"
//   OLED_ShowCHinese(64,4, 22); // "��"
//	    HAL_Delay(1000);
//	 OLED_Clear(); // ����
//	 OLED_ShowCHinese(0,  0, 5); // "��" 
//   OLED_ShowCHinese(16, 0, 6); // "��"
//   OLED_ShowCHinese(32, 0, 7); // "��"
//   OLED_ShowCHinese(48, 0, 8); // "Ա"
//	 OLED_ShowCHinese(48,4, 23); // "��" 
//   OLED_ShowCHinese(64,4, 24); // "��"

//	  HAL_Delay(1000);
//	 OLED_Clear(); // ����
//	 OLED_ShowCHinese(0,  0, 5); // "��" 
//   OLED_ShowCHinese(16, 0, 6); // "��"
//   OLED_ShowCHinese(32, 0, 7); // "��"
//   OLED_ShowCHinese(48, 0, 8); // "Ա"
//	 OLED_ShowCHinese(32,4, 25); // "��" 
//   OLED_ShowCHinese(48,4, 26); // "��"
//   OLED_ShowCHinese(64,4, 27); // "��"	
    osDelay(10);
//		OLED_Clear(); // ����
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the DS18B20 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);  // �л�PC6���ŵ�ƽ
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

//   osDelay(1000);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // PC6=0V
    osDelay(10);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the BLE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
//				HAL_UART_Receive_IT(&huart2, &ble_rx_data, 1);  
//				printf("System Ready. Waiting for Bluetooth Data...\r\n");		
//				
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the UART_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
		DS18B20_Init() ;
   
  /* Infinite loop */
  for(;;)
  {
		taskENTER_CRITICAL();
    DS18B20_Start();
    taskEXIT_CRITICAL();
    
    // 非阻塞等待（允许其他任务运行）
    vTaskDelay(pdMS_TO_TICKS(800));
    
    // 读取温度
    int16_t temp;
    taskENTER_CRITICAL();
    temp = DS18B20_GetTemp();
    taskEXIT_CRITICAL();
    
    if(temp > -550 && temp < 1250) {
      printf("Temp: %.1fC\r\n", temp/10.0f);
    }

    osDelay(20);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
//				uint32_t moisture = Read_Soil_Moisture();
//        printf("ADC Value: %lu \r\n", moisture);  // 打印原�?�ADC�?
//        Check_Moisture_Level(moisture);        // 判断湿度等级
//        uint32_t adc_value = Read_Soil_Moisture();  // 读取ADC�?
//        float humidity = Get_Soil_Humidity(adc_value); // 计算湿度百分�?
//        const char* status = Get_Humidity_Status(humidity);
//        printf("ADC: %lu, Humidity: %.1f%%, Status: %s \r\n", 
//               adc_value, humidity, status);	
//	  HAL_Delay(1000);
//    osDelay(10);
  } 
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Infinite loop */
  for(;;)
  {
		      Get_Weight();        

        printf("Current Weight: %d g\r\n", Weight_Shiwu);        
        HAL_Delay(1000);	
    osDelay(100);
  }
  /* USER CODE END StartTask08 */
}



//static int32_t ext_encoderCount_TIM3 = 0;
//static int32_t ext_encoderCount_TIM4 = 0;
//static uint16_t last_counter_TIM3 = 0;
//static uint16_t last_counter_TIM4 = 0;
///* USER CODE BEGIN Header_StartTask09 */
///**
//* @brief Function implementing the motor thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_StartTask09 */
//void StartTask09(void *argument)
//{
//  /* USER CODE BEGIN StartTask09 */
//	
//	 HAL_GPIO_WritePin(MOTOR1_DIR1_PORT, MOTOR1_DIR1_PIN, GPIO_PIN_SET); // DIR1 = HIGH
//   HAL_GPIO_WritePin(MOTOR1_DIR2_PORT, MOTOR1_DIR2_PIN, GPIO_PIN_RESET); // DIR2 = LOW
//	 HAL_GPIO_WritePin(MOTOR2_DIR1_PORT, MOTOR2_DIR1_PIN, GPIO_PIN_SET); // DIR1 = HIGH
//   HAL_GPIO_WritePin(MOTOR2_DIR2_PORT, MOTOR2_DIR2_PIN, GPIO_PIN_RESET); // DIR2 = LOW
//	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
//	
//	// 启动编码器
//    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
//    SetMotorSpeed(300);
//	
//	
//  /* Infinite loop */
//  for(;;)
//  {

//		
////		encoderCount_TIM3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
////    encoderCount_TIM4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

////   // 计算 RPM（注意：负值表示反转）
////    float speed_TIM3 = (float)encoderCount_TIM3 / 4096.0f * 60.0f;
////    float speed_TIM4 = (float)encoderCount_TIM4 / 4096.0f * 60.0f;

////    // ����������������Ҫʵʱ�ٶȣ�


////printf("Motor Speed (TIM3): %.2f RPM\n", speed_TIM3);
////printf("Motor Speed (TIM4): %.2f RPM\n", speed_TIM4);
////		    __HAL_TIM_SET_COUNTER(&htim3, 0);
////    __HAL_TIM_SET_COUNTER(&htim4, 0);
//    // ��ʱһ��ʱ���ٶ�ȡ��һ���ٶ�
//		
//		
////		int32_t count3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
////int32_t count4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

////printf("Signed CNT3: %ld, CNT4: %ld\n", count3, count4);

////__HAL_TIM_SET_COUNTER(&htim3, 0);
////__HAL_TIM_SET_COUNTER(&htim4, 0);

//    // 读取当前16位计数值
//    uint16_t current_TIM3 = __HAL_TIM_GET_COUNTER(&htim3);
//    uint16_t current_TIM4 = __HAL_TIM_GET_COUNTER(&htim4);

//    // 计算与上次的差值
//    int32_t delta_TIM3 = (int32_t)current_TIM3 - (int32_t)last_counter_TIM3;
//    int32_t delta_TIM4 = (int32_t)current_TIM4 - (int32_t)last_counter_TIM4;

//    // 处理16位溢出/下溢（关键！）
//    if (delta_TIM3 > 32767)  delta_TIM3 -= 65536;  // 下溢（反转太多）
//    if (delta_TIM3 < -32768) delta_TIM3 += 65536;  // 上溢（正转太多）

//    if (delta_TIM4 > 32767)  delta_TIM4 -= 65536;
//    if (delta_TIM4 < -32768) delta_TIM4 += 65536;

//    // 累加到扩展计数器
//    ext_encoderCount_TIM3 += delta_TIM3;
//    ext_encoderCount_TIM4 += delta_TIM4;

//    // 更新上次值
//    last_counter_TIM3 = current_TIM3;
//    last_counter_TIM4 = current_TIM4;

//    // === 现在可以安全使用 ext_encoderCount 计算速度 ===
//    float speed_TIM3 = (float)ext_encoderCount_TIM3 / 4096.0f * 60.0f;  // RPM
//    float speed_TIM4 = (float)ext_encoderCount_TIM4 / 4096.0f * 60.0f;

//    // 打印
//    printf("Motor Speed (TIM3): %.2f RPM\n", speed_TIM3);
//    printf("Motor Speed (TIM4): %.2f RPM\n", speed_TIM4);

//    // 清零扩展计数器，准备下一周期
//    ext_encoderCount_TIM3 = 0;
//    ext_encoderCount_TIM4 = 0;

//    // 延时1秒
//    osDelay(1000);

//  }
//  /* USER CODE END StartTask09 */
//}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartTask09(void *argument)
{
	SerialCommand_ProcessChar(rx_data);
	HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	osDelay(1000);
}

/* USER CODE END Application */

