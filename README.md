## ROS Qt Deskotp GUI Apps
Use qt5 to implement the ros robot human-machine interface

使用qt5实现ros机器人人机界面

Attention! This code is for learning purposes only and cannot be used for any other purpose without the author's permission.

注意！未经作者的许可，此代码仅用于学习，不能用于其他用途。

Continuously updating.....

持续更新中.....

### 一，Features
### 一，功能介绍
#### 1，Speed dashboard
#### 1,速度仪表盘
- Before use, you must set the odom topic in the menu-settings-and-topic settings:
- 使用前须在菜单->设置->话题设置中设置odom话题：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200507124144542.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405102549333.gif)
#### 2，Robot speed control
#### 2, 机器人速度控制
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405104454149.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
#### 3，Power display
#### 3, 电量显示
- Before use, you must set the power topic(Std_msg/Float32) in the menu-settings-and-topic settings
- 使用前须在菜单->设置->话题设置中设置电量话题(Std_msg/Float32)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405153102508.png)
#### 4，rviz module 
#### 4, rviz模块
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405151916473.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
##### 4.1 Subscribe to map topics
##### 4.1 订阅map话题
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408122253344.gif)
##### 4.2 Laser Display
##### 4.2 激光雷达图层显示
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408194648822.gif)
##### 4.3 Set the initial point of navigation
##### 4.3 设置导航初始点
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200411201723417.gif)
##### 4.4 Set up navigation target points
##### 4.4 设置导航目标点
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200411201804722.gif)
##### 4.5 Fixed-point return
##### 4.5 定点返航
- Before use, you must set the amcl topic in the menu-settings-and-topic settings
- 使用前须在菜单->设置->话题设置中设置amcl话题
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200413204212739.gif)
##### 4.6 Subscribe to image topics
##### 4.6 订阅图像话题

- Provides four image display forms that can display four images at the same time
- 提供四个图像显示窗体，可以同时显示四个图像
![加粗样式](https://img-blog.csdnimg.cn/20200507093831130.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
- Before use, you need to set the topic and image encoding of the image in the menu->settings->-video (the encoding must be correctly set, otherwise the image will not be displayed correctly). When setting the error, the prompt for coding the wrong is output in info, and the correct encoding is set according to the prompt)

- 使用前需要先在菜单->设置->video 中设置图像的话题和图像编码（编码一定要设置正确，否则会不能正确显示图像。设置错误时在info中会输出编码错误的提示，根据提示设置正确的编码即可）

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200507093731575.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)

##### 4.7 Quick instructions
##### 4.7 快捷指令
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200429204153916.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200429204233788.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)

##### 4.8 Display RobotModel
##### 4.8 显示机器人模型
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200501165154149.gif)

##### 4.9 To be perfected....
##### 4.9 待完善....
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
sudo apt-get install qtmultimedia5-dev
```
#### 2，Compile
#### 2，编译
Put the package in the ros src package directory：
将软件包放入ros src软件包目录下：
```cpp
catkin_make
```
#### 3，run
#### 3,运行
```cpp
rosrun cyrobot_monitor cyrobot_monitor
```
***
### LIENSE
### 开源协议
**GNU GPL（GNU General Public License，GNU通用公共许可证）**
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408135643929.png)

- As long as the software contains products or code that follow this Agreement, the software must also comply with the GPL License Agreement, i.e. it must be open source free, not charged from a closed source, and not commercial software.
- 只要软件中包含了遵循本协议的产品或代码，该软件就必须也遵循 GPL 许可协议，也就是必须开源免费，不能闭源收费，不能作为商用软件。

*Key features of GPL open source protocol*
*GPL 开源协议的主要特点*
- Copy Freedom allows software to be copied to anyone's computer without limiting the number of copies.
- 复制自由 	允许把软件复制到任何人的电脑中，并且不限制复制的数量。
- Freedom of communication Allows software to be disseminated in various forms.
- 传播自由 	允许软件以各种形式进行传播。
- Fee-based communication allows the software to be sold in a variety of media, but buyers must be made aware in advance that the software is free to be obtained;
- 收费传播 	允许在各种媒介上出售该软件，但必须提前让买家知道这个软件是可以免费获得的；因此，一般来讲，开源软件都是通过为用户提供有偿服务的形式来盈利的。
- Fee-based communication allows the software to be sold in a variety of media, but buyers must be made aware in advance that the software is free to be obtained;
Freedom of modification allows developers to add or remove the functionality of the software, but the software must still be licensed on a GPL license agreement after modification.
- 修改自由 	允许开发人员增加或删除软件的功能，但软件修改后必须依然基于GPL许可协议授权。
