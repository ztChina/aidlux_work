#### 查看设备信息

```shell
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)
```



#### 创建lidar设备规则

```shell
root@aidlux:/home/aidlux# vim /etc/udev/rules.d/99-usb-lidar-name.rules
```

内容

```shell
KERNEL=="ttyUSB*",ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="7523",ATTRS{bcdDevice}=="8134", SYMLINK+="my_lidar", MODE="0666"
```



#### 创建串口设备规则

```shell
vim /etc/udev/rules.d/99-usb-usart-name.rules
```

内容

```shell
ATTRS{bcdDevice}=="0264",SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="7523",SYMLINK+="motor", MODE="0666"
```



#### 重置规则

```shell
udevadm control --reload-rules
```

