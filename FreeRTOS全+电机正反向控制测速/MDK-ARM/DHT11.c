#include "dht11.h"
#include "tim.h"  // 使用TIM1实现us级延时

/* 私有函数 */
static void dht11_reset(void);
static uint8_t dht11_check(void);
static uint8_t dht11_read_bit(void);
static uint8_t dht11_read_byte(void);

/* 微秒延时函数（基于TIM1） */
static void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start(&htim1);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
    HAL_TIM_Base_Stop(&htim1);
}

/* 初始化DHT11 */
uint8_t dht11_init(void) {
    GPIO_InitTypeDef gpio_init;
    DHT11_GPIO_CLK_ENABLE();
    
    gpio_init.Pin = DHT11_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &gpio_init);
    
    dht11_reset();
    return dht11_check();
}

/* 复位DHT11 */
static void dht11_reset(void) {
    DHT11_DQ_OUT(0);    // 拉低至少18ms
    HAL_Delay(20);      // 实际延时20ms（留余量）
    DHT11_DQ_OUT(1);    // 释放总线
    delay_us(30);       // 主机拉高20-40us
}

/* 检查DHT11响应 */
static uint8_t dht11_check(void) {
    uint8_t retry = 0;
    while (DHT11_DQ_IN && retry < 100) {  // 等待DHT11拉低80us
        retry++;
        delay_us(1);
    }
    if (retry >= 100) return 1;
    
    retry = 0;
    while (!DHT11_DQ_IN && retry < 100) { // 等待DHT11拉高80us
        retry++;
        delay_us(1);
    }
    return (retry >= 100) ? 1 : 0;
}

/* 读取1位数据 */
static uint8_t dht11_read_bit(void) {
    uint8_t retry = 0;
    while (DHT11_DQ_IN && retry < 100) {  // 等待低电平
        retry++;
        delay_us(1);
    }
    retry = 0;
    while (!DHT11_DQ_IN && retry < 100) { // 等待高电平
        retry++;
        delay_us(1);
    }
    delay_us(40);  // 延时40us后判断电平
    return DHT11_DQ_IN ? 1 : 0;
}

/* 读取1字节数据 */
static uint8_t dht11_read_byte(void) {
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++) {
        data <<= 1;
        data |= dht11_read_bit();
    }
    return data;
}

/* 读取温湿度数据 */
uint8_t dht11_read_data(uint8_t *temp, uint8_t *humi) {
    uint8_t buf[5];
    uint8_t i;
    
    dht11_reset();
    if (dht11_check() == 0) {
        for (i = 0; i < 5; i++) {
            buf[i] = dht11_read_byte();
        }
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {  // 校验和
            *humi = buf[0];
            *temp = buf[2];
            return 0;
        }
    }
    return 1;  // 读取失败
}
