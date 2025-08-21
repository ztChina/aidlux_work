# 智能小车

## 目录
* [项目介绍](#项目介绍)
* [系统架构](#系统架构)
* [环境及依赖配置](#环境及依赖配置)
* [硬件配置](#硬件配置)
* [运行](#运行)

## 项目介绍
- 本项目基于思岚 RPLIDAR A1 激光雷达，使用 Gmapping 进行 2D SLAM 建图，并通过 ROS Navigation Stack 实现全局/局部规划与 AMCL 粒子滤波定位的自主导航。项目包含：
    - 雷达驱动与 TF 配置
    - Gmapping 参数优化与地图保存
    - AMCL 定位与导航（全局/局部路径规划）
    - YOLO 人体识别（融合视觉与雷达/里程计，支持跟随/到人）
    - 语音交互导航（语音设定目标点、启停、模式切换与状态播报）
    - 可即用的启动脚本与参数模板，帮助快速部署到实际机器人


## 系统架构
![image-20250821155459333](https://raw.githubusercontent.com/i37532/PicGo_PictureHome/main/Windows/image-20250821155459333.png)

## 环境及依赖配置
- ROS相关环境配置及依赖安装见[相关文档](https://github.com/ztChina/aidlux_work/blob/main/ros_navigation/README.md)
- YOLO相关环境配置及依赖安装见[相关文档](https://github.com/ztChina/aidlux_work/blob/main/vision_part/README.md)
- 语音1交互相关环境配置及依赖安装见[相关文档](https://github.com/ztChina/aidlux_work/blob/main/vision_part/README.md)
- 语音2交互功能相关环境配置及依赖安装见[相关文档](https://github.com/ztChina/aidlux_work/tree/main/Dialog-models-and-plugins)
- STM32相关环境配置与原理图见[相关文档](https://github.com/ztChina/aidlux_work/blob/main/%E5%B5%8C%E5%85%A5%E5%BC%8F%E9%83%A8%E5%88%86/readme.md)

## 硬件配置
```python
# 硬件配置内容
```

## 运行
### [ROS部分运行示例](https://github.com/ztChina/aidlux_work/blob/main/ros_navigation/README.md#%E5%90%AF%E5%8A%A8%E7%A4%BA%E4%BE%8B)
### [语音交互部分](https://github.com/ztChina/aidlux_work/blob/main/Dialog-models-and-plugins/README.md#wukong%E8%AF%AD%E9%9F%B3%E4%BA%A4%E4%BA%92%E8%B0%83%E8%AF%95%E6%96%87%E6%A1%A3)



