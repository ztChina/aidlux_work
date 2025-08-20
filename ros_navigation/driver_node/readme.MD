# 驱动节点包 #

## 文件介绍 ##

driver_node与下位机通信，fake_stm32模拟串口通信，使用虚拟串口测试
创建虚拟串口对
driver_node使用/dev/pts/X ，另外的fake_stm32 使用/dev/pts/Y
sudo apt install socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0

接收'cmd_vel'，向下位机发布速度控制命令

