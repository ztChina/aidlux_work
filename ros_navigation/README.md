# 使用`nav+gmap`实现两轮差速`ROS`小车的建图与导航并实现人体识别和语音交互导航
本项目基于思岚-A1激光雷达，使用 `Gmapping` 进行`SLAM`建图，并通过 `ROS Navigation Stack` 实现自主导航。包含雷达驱动配置、`Gmapping`参数优化、地图保存及导航（全局/局部路径规划、`AMCL`定位）的全部内容。适用于机器人自主探索、室内导航等场景，提供详细的参数配置指南和启动脚本，帮助快速部署。
## 主要功能
- `Gmapping`建图
- `Navigation`路径规划与避障

## 依赖安装
- `Joint_state_publisher`和`robot_state_publisher`的安装
- 这两个包是 `ROS` 中用于机器人模型（`URDF`）状态发布和 `TF` 变换的重要工具，通常用于导航、仿真和可视化（如 `RViz`）。若未安装以上依赖包直接运行程序则机器人模型将无法加载到`RVIZ`中并且建图功能也会无法使用
- 使用`apt`进行安装
```python
#更新软件包列表
sudo apt update

#安装 joint_state_publisher（用于发布关节状态）
sudo apt install ros-<distro>-joint-state-publisher

#安装 robot_state_publisher
sudo apt install ros-<distro>-robot-state-publisher

#注意：将 <distro> 替换为你的 ROS 版本（如 humble 或 melodic）

#安装 GUI 版本（可选，提供关节控制滑块,可在rviz中控制关节旋转）
sudo apt install ros-<distro>-joint-state-publisher-gui

```
-通过以下命令验证是否安装成功：
```python
# 检查 robot_state_publisher
rosrun robot_state_publisher robot_state_publisher --help

# 检查 joint_state_publisher
rosrun joint_state_publisher joint_state_publisher --help
```
如果没有报错，说明安装成功。

- `Navigation`的安装
- `Navigation`是一个开源的机器人导航框架，他的功能是让机器人安全地从`A`点移动到`B`点。路径规划、避障和自主脱困等动能是`Navigation`所具备的基本能力
-使用`apt`安装
```python
# 更新软件包列表
sudo apt update

# 完整安装 Navigation Stack
sudo apt install ros-<distro>-navigation

# 或最小化安装核心组件
sudo apt install ros-<distro>-move-base ros-<distro>-amcl ros-<distro>-map-server

#注意：将 <distro> 替换为你的 ROS 版本（如 humble 或 melodic）。
```
验证安装：
```python
# 检查 move_base
rosrun move_base move_base --help

# 检查 AMCL
rosrun amcl amcl --help

# 检查 map_server
rosrun map_server map_server --help

```
- `Gmapping`建图依赖包及雷达驱动都已包含在项目中。

## 启动示例
### 启动建图功能
- 首先启动`driver_node`下的`driver_node.py`文件,具体命令如下：
```python
# 在chapt目录下完成编译工作
colcon build
source install/setup.bash

# 启动driver_node.py
ros2 run driver_node driver_node.py
```
`driver_node.py`文件内包括对`stm32`端发送的底盘速度信息，通过计算将速度信息转化为里程计数据并发布到`odom`话题并将`odom`与`base_link`的节点连接起来，并、启动`urdf`文件发布机器人从`base_link`到`laser`的`TF`树、启动`stm32`端接收`ROS`端发送的速度命令功能。
- 其次启动`rplidar_ros`下的`test_gmapping.launch.py`文件，并在终端中启动`rviz`
```python
#启动test_gmapping.launch.py
ros2 launch rplidar_ros test_gmapping.launch .py

#新开一个终端，启动rviz
rviz2
```
`gampping`产生的地图会通过`/map`话题进行发布，所以可以通过`rviz`订阅话题进行显示，在`rviz`中将`Fixed Frame`修改为`map`，接着点击`add`在话题中添加`/map`话题

再重开一个终端，启动键盘控制节点，控制小车移动完成完整地图建立
```python
#键盘控制节点
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
在建图操作完成后，输入以下命令保存地图，或者在启动`test_gampping.launch.py`文件前解开该文件内底部的`map_save_launch`代码注释，这将启动地图自动保存功能，时间间隔是30s（使用前确保该代码内的地图保存路径正确）
```python
#安装地图服务
sudo apt install ros-$ROS_DISTRO-nav2-map-server 
#保存地图（默认保存在home目录下）
ros2 run nav_map_server map_saver_cli -f room
```
注意，在进行建图时，请确保整个机器人的`tf`树与`map``odom`的正确连接
### 启动导航功能
- 首先和启动建图第一步一样，启动`driver_node.py`文件，启动小车的底盘控制
- 完成以上操作后，启动`rplidar_ros`文件夹下的`prepare_for_nav.launch.py`文件，启动导航所需要的依赖项
```python
ros2 launch rplidar_ros prepare_for_nav.launch.py
```
- 最后，启动`daohang`文件夹下的`nav.launch.py`文件
```python
ros2 launch daohang nav.launch.py
#完成nav.launch.py文件的启动后，再开一个新的终端，启动Rviz
```
完成以上操作后可以看见`rviz`中已经正确加载我们的地图，但是此时`rviz`中会出现`tf`相关的错误，这是因为此时还没由设定机器人的初始位置，在`rviz`的工具栏中选择`2D Pose Estimate`单击机器人在地图中目前所在的大概位置，拖动鼠标调整机器人朝向，若觉得不够准确可以多次调整，然后使用`Nav2 Goal`给定目标点与朝向，让机器人自主进行导航。

### 启动语音交互导航
- 在完成以上启动导航功能的基础下，启动`duodian`文件下的`yvying_jiaohu_daohang.py`文件用于初始化导航位置，其次启动`ros_listener.py`文件用于监听语音大模型发送的导航命令
```python
#启动yvying_jiaohu_daohang文件
ros2 run duodian yvying_jiaohu_daohang

#启动语音监听节点(启动前请确认语音大模型与机器人处于同一网络下，并修改文件内顶端的ip地址)
ros2 run duodian ros_listener

#可启动shiyan文件用于测试导航节点是否正确（启动前请在文件内解开需要的代码注释或添加需要的测试代码）
ros2 run duodian shiyan
```
完成以上操作后，请根据语音大模型模块的介绍通过语音交互发布导航命令

### 启动YOLO人体识别导航
- 和启动语音交互功能一样，在启动导航功能的基础上，启动`duodian`文件下的`renti_shibi_daohang.py`文件
```python
ros2 run duodian renti_shibi_daohang
```
启动以上文件后，当机器人识别到人后便会自动向特定话题发布信息，`ros`端接收到信息后便会前往对方身边。



# 结语
以上文件只包括了`ros`控制方面的代码未包括语音模块，`YOLO`模块，`stm32`端的底盘控制代码，`stm32`端只需完成底盘速度的发布与接收功能即可。