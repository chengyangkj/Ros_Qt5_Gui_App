## ROS Qt Deskotp GUI App


![GitHub last commit](https://img.shields.io/github/last-commit/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)
![GitHub stars](https://img.shields.io/github/stars/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)
![GitHub forks](https://img.shields.io/github/forks/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)
![GitHub issues](https://img.shields.io/github/issues/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)
![GitHub issues closed](https://img.shields.io/github/issues-closed/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)


### How To Contribute

欢迎提交Issues与bug的pull resquest


***
 简体中文 | [English](./README_en.md)

- 使用qt5实现ros机器人人机界面

- 注意！未经作者的许可，此代码仅用于学习，不能用于其他用途。

- 本仓库以分支的形式，长期维护各种有趣的ROS 可视化项目，持续更新中.....

- 欢迎在issues提交bug
  
[![image.png](https://i.postimg.cc/htDgpxDc/image.png)](https://postimg.cc/N5zW0K2z)

**注意！主分支（master）已不再维护librviz功能，使用Qt自行绘制实现一些图层显示，如需librviz功能请切换分支[rviz_tree](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/rviz_tree)或[CSDN及古月居课程例程版本](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/simple)**

## 一，菜单

- [安装教程](#二，安装教程)
- [分支](#三，分支)
- [使用说明](#四，使用说明)
- [功能介绍](#五，功能介绍)
- [开源协议](#六，开源协议)
- [相关教程及交流群](#七，相关教程及交流群)

## 二，安装教程

#### 1，首先安装ros对qt pkg的支持（非必须）

melodic需要换成自己的ROS版本号

```cpp
sudo apt-get install ros-melodic-qt-create
```

```cpp
sudo apt-get install ros-melodic-qt-build
```
#### 2,安装Qtmultimedia5依赖

程序依赖Qtmultimedia实现音频功能，因此需要安装依赖

```cpp
sudo apt-get install qtmultimedia5-dev
```

#### 3，编译

将软件包放入ros src软件包目录下：

```cpp
catkin_make
```
#### 4,运行
```cpp
rosrun cyrobot_monitor cyrobot_monitor
```
***
#### 5，windows编译

- 借助 ROS windows版本，编译后可在win10平板使用，安装教程[古月学院 如何实现Windows ROS人机交互软件](https://class.guyuehome.com/detail/p_5fc5ab97e4b04db7c091f475/6)

## 三，分支

| 分支名         | 支持平台         | 功能           | 
| ------------- | --------------- |  ------------ | 
| [master](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/master)      | Win10 Ubuntu |ROS + QWidget + QGraphicsview自绘制可视化界面显示    | 
| [qml_hmi](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/qml_hmi)    | Win10 Ubuntu |ROS + QML + C++混合编程，qml自绘制地图，激光雷达可视化显示等demo| 
| [simple](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/simple)      | Win10 Ubuntu |ROS + QWidget + Librviz进行可视化显示，为《ROS人机交互软件开发》系列课程中实现的版本，CSDN博客例程版本|
| [rviz_tree](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/rviz_tree)| Win10 Ubuntu |ROS + QWidget + Librviz原生图层Api实现图层管理，不需手动创建图层|
| [ros_qt_demo](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/ros_qt_demo)| Win10 Ubuntu |cakin_create_qt_pkg 创建的原始包，cmakelist.txt已配置好改为qt5，可以直接编译运行|
| [Flutter App](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/ros_flutter_app)| 基于flutter实现多平台运行|逐步推进.....|


## 四，使用说明

### 1，连接主节点
- 使用前需要在连接界面连接rosore主节点

[![024.png](https://i.postimg.cc/9QrgmF1y/024.png)](https://postimg.cc/Xr6kxWRv)

- 主节点地址：即ROS_MASTER_URI  ROS多机通讯主节点地址，如果只是单机通讯填127.0.0.1即可
- 本机IP：即ROS_IP 软件运行的机器的IP 如果是单机通讯填127.0.0.1即可 注意 此ip填错 会导致只能单向通讯 导致收不到远程小车的话题信息 却有话题列表的现象
- 勾选使用环境变量连接就不使用界面设置的ROS_IP与ROS_MASTER_URI,需要在环境变量文件（~/.bashrc）配置好多机通讯环境变量，否则会导致连接失败，配置多机通讯教程[csdn 博客](https://blog.csdn.net/qq_38441692/article/details/98205852)
- 其他一些话题设置
- 勾选自动连接会在打开软件时进行自动连接
- 点击检测IP会自动检测本机IP并填入ros ip与ros master ip 

### 2,设置

点击连接界面右下角，进行相关必要设置
#### 2.1 话题设置

可视化相关的订阅与发布话题的设置

- [![025.png](https://i.postimg.cc/C1GnbkMk/025.png)](https://postimg.cc/PpqqjLw5)

- 速度控制话题：界面上的速度控制按钮，发布出去的话题名称
- 导航目标点话题：导航时界面上发布导航目标点的话题
- 导航初始点话题：导航时界面上选点发布的导航初始点话题
- 转向灯亮起阈值：主界面速度仪表盘旁的转向灯，收到的角速度大于这个阈值时开始亮
- 里程计话题：订阅里程计话题，以实现速度仪表盘与转向灯功能
- 电池电量话题：电池电量显示的依据，订阅此话题，注意此话题的消息类型为sensor_msgs/BatteryState
- 机器人的坐标话题：订阅此话题，以实现在自绘制地图上动态显示机器人实时坐标
  
#### 2.2 图层设置

在使用librviz与自绘制地图等进行可视化显示时的一些图层属性信息

[![026.png](https://i.postimg.cc/BvMn3pvh/026.png)](https://postimg.cc/4KHgbzVt)

#### 2.3 video设置

界面上进行视频话题进行可视化显示时订阅的话题，其中video0订阅为compressed后的图像话题，以减轻卡顿问题

[![image.png](https://i.postimg.cc/wxJ3yh4B/image.png)](https://postimg.cc/NLsQZrch)

#### 2.4 通用设置

[![image.png](https://i.postimg.cc/rsC8hN7C/image.png)](https://postimg.cc/rRsXsxVK)

- 机器人模型图：主界面左侧显示的机器人图片
- 显示模式：机器人端（只保留主要功能，简化显示） 控制端：所有功能均显示
- 话题订阅线程数：话题订阅时所用的线程数，如遇到话题卡顿，提高此参数
- FrameRate：循环Rate

#### 2.5 坐标系设置

此设置是自绘制地图坐标系能否正常显示的必要设置，通过设置的坐标系Frame进行tf坐标变换：

[![image.png](https://i.postimg.cc/TPTbGyp7/image.png)](https://postimg.cc/Jsd05z3Z)

查看坐标系的Frame方法：

rosrun rqt_tf_tree rqt_tf_tree:

[![image.png](https://i.postimg.cc/59z8BMdm/image.png)](https://postimg.cc/3WKyhVwy)

## 功能介绍

#### 1,速度仪表盘

- 使用前须在连接界面->设置->话题设置中设置odom话题：

[![image.png](https://i.postimg.cc/Kj4Nf01C/image.png)](https://postimg.cc/wR8LTcck)

#### 2, 机器人速度控制

- 使用前需在连接界面->设置->话题设置中设置速度控制话题：

- 控制方式：

  - 键盘热键控制

  - 鼠标点击控制

  - 虚拟摇杆控制

[![002.png](https://i.postimg.cc/yxJmcW5H/002.png)](https://postimg.cc/XBbB0N4H)


#### 3, 电量显示

- 使用前须在在连接界面->设置->话题设置中设置电池电量话题：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405153102508.png) 
#### 4, 地图等信息可视化显示

 使用前需在连接界面->设置->图层设置中进行启用与停用等必要图层设置：
 
 [![026.png](https://i.postimg.cc/BvMn3pvh/026.png)](https://postimg.cc/4KHgbzVt)
#### 4.1 qt自绘制显示

注意使用前需要在连接界面->设置->坐标系设置中进行图层Frame设置：

[![022.png](https://i.postimg.cc/ydnKc8Rv/022.png)](https://postimg.cc/LYqGMRN1)

[![image.png](https://i.postimg.cc/jdJd1sdD/image.png)](https://postimg.cc/nCpJjfmx)

#### 4.2 librviz显示

目前librviz显示待优化，闪退问题严重
## 开源协议
**GNU GPL（GNU General Public License，GNU通用公共许可证）**
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408135643929.png)


- 只要软件中包含了遵循本协议的产品或代码，该软件就必须也遵循 GPL 许可协议，也就是必须开源免费，不能闭源收费，不能作为商用软件。

*GPL 开源协议的主要特点*

- 复制自由 	允许把软件复制到任何人的电脑中，并且不限制复制的数量。

- 传播自由 	允许软件以各种形式进行传播。

- 收费传播 	允许在各种媒介上出售该软件，但必须提前让买家知道这个软件是可以免费获得的；因此，一般来讲，开源软件都是通过为用户提供有偿服务的形式来盈利的。

- 修改自由 	允许开发人员增加或删除软件的功能，但软件修改后必须依然基于GPL许可协议授权。

## 相关教程及交流群

 **本系列教程文章专栏:**

[ROS机器人GUI程序开发](https://blog.csdn.net/qq_38441692/category_9863968.html)

 **本系列课程已上线古月学院，欢迎感兴趣的小伙伴订阅：**

 1. [ROS Qt开发环境搭建以及基础知识介绍](https://class.guyuehome.com/detail/p_5eba414d58533_Uh4XTbPi/6)
 2. [ROS人机交互软件的界面开发](https://class.guyuehome.com/detail/p_5ec490a8d7bd7_b7ucPqUs/6)
 3. [ROS Rviz组件开发方法](https://class.guyuehome.com/detail/p_5edf2d27a1942_foy4nqci/6)
 4. [如何实现ROS windows人机交互软件](https://class.guyuehome.com/detail/p_5fc5ab97e4b04db7c091f475/6)
 
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200612194143186.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)

**开发交流QQ群：** 797497206
