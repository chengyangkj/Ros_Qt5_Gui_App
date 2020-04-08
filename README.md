## ROS Qt Deskotp GUI Apps
Use qt5 to implement the development of the human-machine interface of ros robots
使用qt5实现ros机器人人机界面开发
### 一，Features
### 一，功能介绍
#### 1，Speed dashboard 速度仪表盘

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405102549333.gif)
### 二，Robot speed control 机器人速度控制
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405104454149.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
### 三，Power display 电量显示
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405153102508.png)
### 四，rviz module rviz模块:
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405151916473.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
#### 4.1Subscribe to map topics 订阅map话题
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408122253344.gif)
### 二，安装教程
### 二，Installation tutorial
#### 1，首先安装ros对qt pkg的支持
#### 1，first install ros support for qt pkg
```cpp
sudo apt-get install ros-melodic-qt-create
```

```cpp
sudo apt-get install ros-melodic-qt-build
```
```cpp
sudo apt-get install qtcreator
```
#### 2，Compile
#### 2，编译
Put the package in the ros src package directory：
将软件包放入ros src软件包目录下：
```cpp
catkin_make
```
