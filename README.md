## ROS Qt Deskotp GUI App
***
 简体中文 | [English](./README_en.md)

- 使用qt5实现ros机器人人机界面

- 注意！未经作者的许可，此代码仅用于学习，不能用于其他用途。


- 本仓库以分支的形式，长期维护各种有趣的ROS Qt项目，持续更新中.....

- 欢迎在issues提交bug

## 菜单

- [安装教程](#安装教程)
- [分支](#分支)
- [使用说明] (#使用说明)
- [功能介绍](#功能介绍)
- [开源协议](#开源协议)
- [相关教程及交流群](#相关教程及交流群)

## 安装教程

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

## 分支

#### 1. Qml版本分支（开发中）

- ROS + QML + C++混合编程，使用qml自绘制地图，激光雷达点云等
- [qml_hmi](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/qml_hmi)

#### 3. Lite branch

- 此版本为《ROS人机交互软件开发》系列课程中实现的版本，实现了master分支的基本功能，代码易懂 
- [simple](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/simple)

#### 4,rviz菜单树分支

- 使用rviz自带的菜单树，去实现添加显示图层。master分支所有的图层及菜单均需要手动去写代码实现（并且目前仅支持部分图层显示），此分支调用librviz现成api，所有图层均可以实现,不用去手动创建图层菜单和display

- [rviz_tree](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/rviz_tree)

- [![image.png](https://i.postimg.cc/KY0XyzKD/image.png)](https://postimg.cc/2qL9QC71)


#### 5,ROS Qt Demo
- cakin_create_qt_pkg 创建的原始包，cmakelist.txt已配置好改为qt5，可以直接编译运行

- [ros_qt_demo](https://github.com/chengyangkj/ros_qt_demo)

#### 6，android版本分支 敬请期待

#### 7，web版本分支 敬请期待

## 使用说明

- 使用前需要在菜单->设置 中进行必要设置

![image.png](https://i.postimg.cc/9XYJ7s0m/image.png)

- ROS_MASTER_URI: ROS多机通讯主节点地址，如果只是单机通讯填127.0.0.1即可
- ROS_IP: 软件运行的机器的IP 如果是单机通讯填127.0.0.1即可
- 其他一些话题设置

- 注意！保存设置后需要重启软件生效

## 功能介绍

#### 1,速度仪表盘

- 使用前须在菜单->设置->话题设置中设置odom话题：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200507124144542.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405102549333.gif)

#### 2, 机器人速度控制

![在这里插入图片描述](https://i.postimg.cc/nrjgHkKj/image.png)

#### 3, 电量显示

- 使用前须在菜单->设置->话题设置中设置电量话题(Std_msg/Float32)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405153102508.png) 
#### 4, rviz模块
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405151916473.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
##### 4.1 订阅map话题
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408122253344.gif)

##### 4.2 激光雷达图层显示
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408194648822.gif)

##### 4.3 设置导航初始点
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200411201723417.gif)

##### 4.4 设置导航目标点
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200411201804722.gif)

##### 4.5 定点返航

- 使用前须在菜单->设置->话题设置中设置amcl话题
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200413204212739.gif)

##### 4.6 订阅图像话题

- 提供四个图像显示窗体，可以同时显示四个图像 解决卡顿问题，在video0订阅（image_raw/compressed）即compressed后的图像即可不卡顿，且video0只能订阅压缩后的图像

![加粗样式](https://img-blog.csdnimg.cn/20200507093831130.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)



##### 4.7 快捷指令
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200429204153916.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200429204233788.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)

##### 4.8 显示机器人模型
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200501165154149.gif)

##### 4.9 提供六种rviz工具
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200515184545845.png)

##### 4.10 显示话题列表
[![image.png](https://i.postimg.cc/Z5bGBfgk/image.png)](https://postimg.cc/svL6bJsK)
.
##### 4.11 待完善....


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
