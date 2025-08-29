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
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float v_left;     // 左轮线速度 (m/s)
    float v_right;    // 右轮线速度 (m/s)
    float temp;       // 温度
    float humidity;   // 湿度
    float water_level;// 水位
} RobotStatus_t;

RobotStatus_t robot_status = {0};
osMutexId_t robotStatusMutexHandle;
osMutexId_t tempMutexHandle;
//typedef struct {
//    float v_left;
//    float v_right;
//} MotorSpeed_t;

extern MotorSpeed_t current_motor_speed; //用于反馈

volatile uint8_t new_cmd_received = 0;
float latest_temp = 0.0f;
//float g_soil_humidity = 100.0;
uint32_t latest_weight = 0;
extern float get_lv;
extern float get_rv;

//extern uint8_t rx_frame_buffer;
//extern uint8_t receiving_header;

//typedef struct {
//    float v_x;
//    float v_z;
//} TargetSpeed_t;

//TargetSpeed_t target_speed = {0};//用于控制



void Send_Status_Frame(RobotStatus_t *status)
{
  uint8_t buf[32];
  uint8_t index = 0;

  buf[index++] = 0xAA;
  buf[index++] = 0x55;
  buf[index++] = 20;
  buf[index++] = 0x01;

  memcpy(&buf[index], &status->v_left, 4); index += 4;
  memcpy(&buf[index], &status->v_right, 4); index += 4;
  memcpy(&buf[index], &status->temp, 4); index += 4;
  memcpy(&buf[index], &status->humidity, 4); index += 4;
  memcpy(&buf[index], &status->water_level, 4); index += 4;

  uint8_t checksum = 0;
  for(int i = 0; i < index; i++) checksum += buf[i];
  buf[index++] = checksum;

  HAL_UART_Transmit(&huart1, buf, index, HAL_MAX_DELAY);
}




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void Serial_SendSensorPacket(const RobotStatus_t* status)
{
    uint8_t frame[2 + 1 + 1 + 4*5 + 1]; 
    frame[0] = 0xAA;
    frame[1] = 0x55;

    uint8_t data_count = 5;
    uint8_t length = 1 + 4 * data_count + 1;
    frame[2] = length;

    frame[3] = 0x01; // 功能码：发送机器人状态

    float data[5] = {
        status->v_left,
        status->v_right,
        status->temp,
        status->humidity,
        status->water_level
    };

    memcpy(&frame[4], &data[0], 4);
    memcpy(&frame[8], &data[1], 4);
    memcpy(&frame[12], &data[2], 4);
    memcpy(&frame[16], &data[3], 4);
    memcpy(&frame[20], &data[4], 4);

    // 校验
    uint8_t checksum = 0;
    for(int i=2; i < 4 + 4*data_count; i++) {
        checksum ^= frame[i];
    }
    frame[24] = checksum;

    for (int i=0; i<sizeof(frame); i++) {
				Serial_SendByte(frame[i]);
    }
}
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
  .priority = (osPriority_t) osPriorityLow1,
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
osThreadId_t WeightHandle;
const osThreadAttr_t myTask08_attributes = {
  .name = "Weight",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motor */
osThreadId_t motorHandle;
const osThreadAttr_t motor_attributes = {
  .name = "motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for collect */
osThreadId_t collectHandle;
const osThreadAttr_t collect_attributes = {
  .name = "collect",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};

/* Definitions for collect */
osThreadId_t comHandle;
const osThreadAttr_t com_attributes = {
  .name = "communication",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for oled */
osThreadId_t oledHandle;
const osThreadAttr_t oled_attributes = {
  .name = "oled_debug",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

osThreadId_t tempTaskHandle;
const osThreadAttr_t tempTask_attributes = {
  .name = "tempTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};


osThreadId_t pumpHandle;
const osThreadAttr_t pump_attributes = {
  .name = "pump",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

//osThreadId_t pidHandle;
//const osThreadAttr_t pid_attributes = {
//  .name = "pid",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow2,
//};

//osMessageQueueId_t pumpQueueHandle;
//const osMessageQueueAttr_t pumpQueue_attributes = {
//  .name = "pumpQueue"
//};




/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t rx_data;  // �����ⲿ����



//int32_t encoderCount_TIM3 = 0; // TIM3 ������������
//int32_t encoderCount_TIM4 = 0; // TIM4 ������������


//int32_t motor1_speed = 0;
//int32_t motor2_speed = 0;



//#define MOTOR1_DIR1_PIN GPIO_PIN_1
//#define MOTOR1_DIR1_PORT GPIOD
//#define MOTOR1_DIR2_PIN GPIO_PIN_2
//#define MOTOR1_DIR2_PORT GPIOD

//#define MOTOR2_DIR1_PIN GPIO_PIN_3
//#define MOTOR2_DIR1_PORT GPIOD
//#define MOTOR2_DIR2_PIN GPIO_PIN_4
//#define MOTOR2_DIR2_PORT GPIOD




//void SetMotorSpeed(uint16_t duty) {
//    if(duty > 1000) duty = 1000;
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty);
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
//}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask03(void *argument);//oled
void StartTask04(void *argument);//？？
void StartTask05(void *argument); //蓝牙接收
void StartTask06(void *argument);//温度
void StartTask07(void *argument);//湿度
void StartTask08(void *argument);//重量
void StartTask09(void *argument);//uart3串口通信
void start_collect_task(void *argument);//采集线程
void start_com_task(void *argument);//通信线程
void start_motor_task(void *argument);//控制线程
void start_oled_debug_task(void *argument);//oled调试
void start_temp_task(void *argument); //温度和重量采集线程，还是单独拿出来采
void start_pump_task(void *argument);
//void start_pid_task(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
		const osMutexAttr_t robotStatusMutex_attributes = {
		.name = "robotStatusMutex"
	};
		const osMutexAttr_t tempMutex_attributes = {
		.name = "tempMutex"
	};
	robotStatusMutexHandle = osMutexNew(&robotStatusMutex_attributes);
	tempMutexHandle = osMutexNew(&tempMutex_attributes);
	// 通过闪灯，确保进入FreeRTOS控制
//	defaultTask_LEDHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_LED_attributes);
	// 启动电机控制
//  motorHandle = osThreadNew(start_motor_task, NULL, &motor_attributes);
//  /* USER CODE END RTOS_THREADS */
	
//	  /* USER CODE BEGIN RTOS_THREADS */
	// 读取温度，土壤湿度，水位线，电机速度等
// collectHandle = osThreadNew(start_collect_task, NULL, &collect_attributes);
//  /* USER CODE END RTOS_THREADS */
	
//	  /* USER CODE BEGIN RTOS_THREADS */
//  comHandle = osThreadNew(start_com_task, NULL, &com_attributes);
//  /* USER CODE END RTOS_THREADS */
//	
//	/* USER CODE BEGIN RTOS_THREADS */
	// 读取温度任务
//	tempTaskHandle = osThreadNew(start_temp_task, NULL, &tempTask_attributes);
///* USER CODE END RTOS_THREADS */
//  /* USER CODE BEGIN RTOS_EVENTS */
	// OLED测试
//	oledHandle = osThreadNew(start_oled_debug_task, NULL, &oled_attributes);
//	// 泵水任务
//	pumpHandle = osThreadNew(start_pump_task, NULL, &pump_attributes);
//	pidHandle = osThreadNew(start_pid_task, NULL, &pid_attributes);
  /* add events, ... */
//	// 蓝牙测试
//  BLEHandle = osThreadNew(StartTask05, NULL, &BLE_attributes);
	// 压力传感器测试
//  WeightHandle = osThreadNew(StartTask08, NULL, &myTask08_attributes);
  /* USER CODE END RTOS_EVENTS */
}


void start_motor_task(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	uint32_t start;
  /* Infinite loop */
  for(;;)
  {
//	 start = HAL_GetTick();
   Motor_UpdateSpeed();

    float vL = Motor_GetLinearSpeedLeft();
    float vR = Motor_GetLinearSpeedRight();

//    osMutexAcquire(robotStatusMutexHandle, osWaitForever);
    current_motor_speed.v_left = vL;
    current_motor_speed.v_right = vR;
		
    Motor_pid_Set();
    osDelay(50); // 20ms
  }
  /* USER CODE END StartTask05 */
}

void start_collect_task(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	Init_HX711pin();
  /* Infinite loop */
   for(;;)
  {
		uint32_t tick = HAL_GetTick();  // 起始 tick
    // 1. 读取温度（单位：0.1°C）
    int16_t raw_temp = DS18B20_GetTemp();
    float temp = 0.0f;
    if(raw_temp > -550 && raw_temp < 1250) {
        temp = raw_temp / 10.0f;
//        printf("Temp: %.1fC\r\n", temp);
//    }
		float temp = 0.0f;
		osMutexAcquire(tempMutexHandle, osWaitForever);
		temp = latest_temp;
		osMutexRelease(tempMutexHandle);
//		uint32_t temp_time = HAL_GetTick() - tick;
//    tick += temp_time;
    // 2. 读取土壤湿度
    uint32_t adc_value = Read_Soil_Moisture();
//		printf("adc_value: %d \r\n", adc_value);
    float humidity = Get_Soil_Humidity(adc_value); // 返回百分比
		uint32_t hx711_raw = latest_weight;
    float water_level = 0.0f;
    if(hx711_raw > Weight_error) {
        int32_t net_weight = (int32_t)(hx711_raw - Weight_error );
        water_level = (float)net_weight / GapValue -Weight_Maopi;
//        printf("Water Level: %.2f g\r\n", water_level);
    }
		// 4. 读取电机速度（可加锁）
    float vL = current_motor_speed.v_left;
    float vR = current_motor_speed.v_right;
//		 uint32_t speed_time = HAL_GetTick() - tick;
//    tick += speed_time;

    // 5. 更新 robot_status（带互斥锁）
    osMutexAcquire(robotStatusMutexHandle, osWaitForever);
	  robot_status.v_left = vL;
    robot_status.v_right = vR;
    robot_status.temp = temp;
    robot_status.humidity = humidity;
    robot_status.water_level = water_level;
    osMutexRelease(robotStatusMutexHandle);

		
    osDelay(50);  // 延迟 100ms
  }
  /* USER CODE END StartTask05 */
}
  }

void start_com_task(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
	RobotStatus_t local;
	uint32_t start, t1, t2, t3;
  for(;;)
  {
//		start = HAL_GetTick();
//		t1 = HAL_GetTick();
    osMutexAcquire(robotStatusMutexHandle, osWaitForever);
    memcpy(&local, &robot_status, sizeof(RobotStatus_t));
    osMutexRelease(robotStatusMutexHandle);
//		t2 = HAL_GetTick();
		// 2. 发送数据帧
		Serial_SendSensorPacket(&local);
		// 3. 检查是否接收到完整帧（解析过程在接收中断中实现）
//		 printf("Water Level: %.2f g\r\n", robot_status.water_level);
//		 printf("humiditY: %.2f \r\n", robot_status.humidity);
		printf("temp: %.2f \r\n", robot_status.temp);
//		t3 = HAL_GetTick();
    osDelay(100);
//		uint32_t end = HAL_GetTick();
//		printf("COM cycle: total=%lu ms | mutex=%lu ms | send=%lu ms\n",
//               end - start, t2 - t1, t3 - t2);
  }
  /* USER CODE END StartTask05 */
}

void start_oled_debug_task(void *argument)//oled调试
{
//  /* USER CODE BEGIN StartTask05 */
   OLED_Init();
   OLED_Clear(); // 清屏
//	float last_v_left = -999.0f, last_v_right = -999.0f;
//	float last_get_vl = -999.0f, last_get_vr = -999.0f;
// char buf[20];
//  /* Infinite loop */
//  for (;;)
//    {
//        // 左轮实际速度显示（robot_status.v_left）
//        if (robot_status.v_left != last_v_left)
//        {
//            last_v_left = robot_status.v_left;

//            OLED_ClearLine(0);  // 清除第0行
//            snprintf(buf, sizeof(buf), "L_Get: %.2f m/s", last_v_left);
//            OLED_ShowString(0, 0, buf, 12);
//        }

//        // 右轮实际速度显示
//        if (robot_status.v_right != last_v_right)
//        {
//            last_v_right = robot_status.v_right;

//            OLED_ClearLine(1);  // 清除第1行
//            snprintf(buf, sizeof(buf), "R_Get: %.2f m/s", last_v_right);
//            OLED_ShowString(0, 1, buf, 12);
//        }

//        // 左轮期望速度显示（get_lv）
//        if (get_lv != last_get_vl)
//        {
//            last_get_vl = get_lv;

//            OLED_ClearLine(2);  // 清除第2行
//            snprintf(buf, sizeof(buf), "L_Set: %.2f m/s", get_lv);
//            OLED_ShowString(0, 2, buf, 12);
//        }

//        // 右轮期望速度显示（get_rv）
//        if (get_rv != last_get_vr)
//        {
//            last_get_vr = get_rv;

//            OLED_ClearLine(3);  // 清除第3行
//            snprintf(buf, sizeof(buf), "R_Set: %.2f m/s", get_rv);
//            OLED_ShowString(0, 3, buf, 12);
//        }

//        osDelay(500);  // 延时刷新
//    }
//  /* USER CODE END StartTask05 */
for(;;)
  {
   OLED_Clear();
	 OLED_ShowNum(0, 2, 2025, 4, 16);  // ���
   OLED_ShowCHinese(32, 2, 2);       // "��"
   OLED_ShowNum(48, 2, 7, 2, 16);    // �·�
   OLED_ShowCHinese(64, 2, 3);       // "��"
   OLED_ShowNum(80, 2, 23, 2, 16);   // ����
   OLED_ShowCHinese(96, 2, 4);       // "��"
   
   // ��ʾ�����У�Hello World!
//   OLED_ShowString(0, 6, "Hello World!", 12);
   
   // ������ʾ1��
   HAL_Delay(1000);
   
   /* �ڶ�����ʾ */
   OLED_Clear(); // 清屏
   OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 9); //  "邱" 
   OLED_ShowCHinese(48,4, 10); // "耀"
   OLED_ShowCHinese(64,4, 11); // "洁"
   HAL_Delay(1000);

   OLED_Clear(); //
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 12); // "郭" 
   OLED_ShowCHinese(48,4, 13); // "昊"
   OLED_ShowCHinese(64,4, 14); // "楠"
   HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 15); // "李" 
   OLED_ShowCHinese(48,4, 16); // "含"
   OLED_ShowCHinese(64,4, 17); // "章"
         
         HAL_Delay(1000);
         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(48,4, 18); // "李" 
   OLED_ShowCHinese(64,4, 19); // "阳"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 20); // "陈" 
   OLED_ShowCHinese(48,4, 21); // "晋"
   OLED_ShowCHinese(64,4, 22); // "川"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(48,4, 23); // "闫" 
   OLED_ShowCHinese(64,4, 24); // "瑞"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 25); // "牟" 
   OLED_ShowCHinese(48,4, 26); // "征"
   OLED_ShowCHinese(64,4, 27); // "宇"        
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 28); // "章" 
   OLED_ShowCHinese(48,4, 29); // "涛"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 30); // "文" 
   OLED_ShowCHinese(48,4, 31); // "煜"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 32); // "段" 
   OLED_ShowCHinese(48,4, 33); // "沛"
         OLED_ShowCHinese(64,4, 34); // "伶"
         HAL_Delay(1000);
    osDelay(10);
		OLED_Clear(); // ����
  }
}


void start_temp_task(void *argument)
{
  DS18B20_Init();
  for (;;)
  {
    int16_t raw_temp = DS18B20_GetTemp();
		latest_weight = HX711_Read();
    float temp = 0.0f;
    if (raw_temp > -550 && raw_temp < 1250) {
      temp = raw_temp / 10.0f;
    }

    osMutexAcquire(tempMutexHandle, osWaitForever);
    latest_temp = temp;
    osMutexRelease(tempMutexHandle);
//		printf("temp_done!\n");

    osDelay(1000);  // DS18B20 建议至少每秒更新一次
  }
}


void start_pump_task(void *argument)
{
		float humidity = 0.0f;
		float water_level = 0.0f;
//		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
    for (;;)
    {
        // Step 1: 获取当前湿度
        
//        xSemaphoreTake(xHumidityMutex, portMAX_DELAY);
				osMutexAcquire(robotStatusMutexHandle, osWaitForever);
				humidity = robot_status.humidity;
				water_level = robot_status.water_level ;
				osMutexRelease(robotStatusMutexHandle);
//        xSemaphoreGive(xHumidityMutex);
//				printf("humidity: %.2f \r\n",  humidity);
        // Step 2: 检查是否需要泵水
        if (humidity <= 30 && water_level>100)
        {
            // 泵水一次
//            Turn_Pump_On();
//					printf("pump!\n");
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);
            osDelay(10000); //泵10秒
//            Turn_Pump_Off();
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);

            // 额外等待一段时间后重新评估
            osDelay(5000);
        }
            osDelay(5000);
    }
}

void start_pid_task(void *argument)
{

 for (;;)
    {
		osDelay(5000);
		}

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
//		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);  
    osDelay(500);
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
   OLED_Clear();
	 OLED_ShowNum(0, 2, 2025, 4, 16);  // ���
   OLED_ShowCHinese(32, 2, 2);       // "��"
   OLED_ShowNum(48, 2, 7, 2, 16);    // �·�
   OLED_ShowCHinese(64, 2, 3);       // "��"
   OLED_ShowNum(80, 2, 23, 2, 16);   // ����
   OLED_ShowCHinese(96, 2, 4);       // "��"
   
   // ��ʾ�����У�Hello World!
//   OLED_ShowString(0, 6, "Hello World!", 12);
   
   // ������ʾ1��
   HAL_Delay(1000);
   
   /* �ڶ�����ʾ */
   OLED_Clear(); // 清屏
   OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 9); //  "邱" 
   OLED_ShowCHinese(48,4, 10); // "耀"
   OLED_ShowCHinese(64,4, 11); // "洁"
   HAL_Delay(1000);

   OLED_Clear(); //
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 12); // "郭" 
   OLED_ShowCHinese(48,4, 13); // "昊"
   OLED_ShowCHinese(64,4, 14); // "楠"
   HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 15); // "李" 
   OLED_ShowCHinese(48,4, 16); // "含"
   OLED_ShowCHinese(64,4, 17); // "章"
         
         HAL_Delay(1000);
         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(48,4, 18); // "李" 
   OLED_ShowCHinese(64,4, 19); // "阳"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 20); // "陈" 
   OLED_ShowCHinese(48,4, 21); // "晋"
   OLED_ShowCHinese(64,4, 22); // "川"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(48,4, 23); // "闫" 
   OLED_ShowCHinese(64,4, 24); // "瑞"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 25); // "牟" 
   OLED_ShowCHinese(48,4, 26); // "征"
   OLED_ShowCHinese(64,4, 27); // "宇"        
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 28); // "章" 
   OLED_ShowCHinese(48,4, 29); // "涛"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 30); // "文" 
   OLED_ShowCHinese(48,4, 31); // "煜"
         HAL_Delay(1000);

         OLED_Clear(); //     
         OLED_ShowCHinese(0,  0, 5); // "团" 
   OLED_ShowCHinese(16, 0, 6); // "队"
   OLED_ShowCHinese(32, 0, 7); // "成"
   OLED_ShowCHinese(48, 0, 8); // "员"
         OLED_ShowCHinese(32,4, 32); // "段" 
   OLED_ShowCHinese(48,4, 33); // "沛"
         OLED_ShowCHinese(64,4, 34); // "伶"
         HAL_Delay(1000);
    osDelay(10);
		OLED_Clear(); // ����
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
				HAL_UART_Receive_IT(&huart2, &ble_rx_data, 1);  
				printf("System Ready. Waiting for Bluetooth Data...\r\n");		
				
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
	osDelay(100);
}

/* USER CODE END Application */

