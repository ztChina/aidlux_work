## 本项目嵌入式部分完成了串口的标准库重定向以及Micro库的重定向
可以参考这篇文档https://blog.csdn.net/struggle_success/article/details/134621868

<img width="618" height="268" alt="image" src="https://github.com/user-attachments/assets/18d9f861-0789-4b54-a522-f60bbdaf69c0" />

## 本项目全部采用STM32CubeMX现代化HAL库开发
要启动系统时钟和总线，对时钟进行分频，讲.h和.c文件分开存放，便于代码的维护。

## STM32F407ZGT6实现HX711裸机移植
本设计中需要在运行时连接传感器，不然会阻塞运行。

## STM32F407ZGT6实现DHT11移植
注意使用定时器实现us级延时

## STM32F407ZGT6实现LCD V3屏幕移植
开启FSMC使用，注意使用对应的接口。该屏幕只支持FSMC配置。

## motor文件夹实现STM32F407ZGT6裸机编码器测速电机PWM驱动
体会理论与实际的差距，PWM输出理论控制，编码器测得实际值。

## sensor_test_final实现所有传感器的整合以及驱动
体会多传感器耦合协作设计

## PCB原理图

<img width="1328" height="939" alt="image" src="https://github.com/user-attachments/assets/94e6f61a-813c-41fe-bade-7edd490483d2" />

## PCB图

<img width="847" height="835" alt="image" src="https://github.com/user-attachments/assets/4e471403-66f1-4b49-ac8e-59cad7654f21" />

## PCB实物

![d81dec2f7c321c4d4febcafe73a2cb19_720](https://github.com/user-attachments/assets/758be01b-fac0-444f-876d-1b4b93faba32)

## PCB三维图

<img width="1001" height="991" alt="image" src="https://github.com/user-attachments/assets/77fe679d-e0b2-4de5-a65a-fc98953f47e0" />

