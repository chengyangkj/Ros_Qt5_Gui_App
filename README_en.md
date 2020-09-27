## ROS Qt Deskotp GUI App

***

 [简体中文](./README.md) | English

- Use qt5 to implement the ros robot human-machine interface

- Attention! This code is for learning purposes only and cannot be used for any other purpose without the author's permission.

- Continuously updating.....

- Welcome to submit bugs in issues

## Branch
**1.~~Kinetic version branch(Branches are merged)~~**

- ~~[kinetic-devel](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/kinetic-devel "kinetic-devel")~~


**2. Qml version branch(To be perfected)**

- The interface is more beautiful, the function is simple, can be used as a robot on-board display

- [qml_simple](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/qml_simple)


**3. Lite branch**

- This version is the implementation of the 《ROS Human-Computer Interactive Software Development》 series of courses, the basic functions of master branch, the code is easy to understand

- [simple](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/simple)


**4. Windows version branch**
- [windows_devel](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/windows_devel)

**5,rviz menu tree branch**

- Use the menu tree that rviz brings with you to add display layers instead of manually creating layer menus and displays.

- [rviz_tree](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/rviz_tree)

- [![image.png](https://i.postimg.cc/KY0XyzKD/image.png)](https://postimg.cc/2qL9QC71)

***

### 一，Features

#### 1，Speed dashboard

- Before use, you must set the odom topic in the menu-settings-and-topic settings:

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200507124144542.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405102549333.gif)
#### 2，Robot speed control

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405104454149.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
#### 3，Power display

- Before use, you must set the power topic(Std_msg/Float32) in the menu-settings-and-topic settings

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405153102508.png)
#### 4，rviz module 

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405151916473.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
##### 4.1 Subscribe to map topics

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408122253344.gif)

##### 4.2 Laser Display

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408194648822.gif)
##### 4.3 Set the initial point of navigation

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200411201723417.gif)
##### 4.4 Set up navigation target points

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200411201804722.gif)
##### 4.5 Fixed-point return

- Before use, you must set the amcl topic in the menu-settings-and-topic settings
- 使用前须在菜单->设置->话题设置中设置amcl话题
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200413204212739.gif)
##### 4.6 Subscribe to image topics


- Provides four image display forms that can display four images at the same time

![加粗样式](https://img-blog.csdnimg.cn/20200507093831130.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
- Before use, you need to set the topic and image encoding of the image in the menu->settings->-video (the encoding must be correctly set, otherwise the image will not be displayed correctly). When setting the error, the prompt for coding the wrong is output in info, and the correct encoding is set according to the prompt)


##### 4.7 Quick instructions

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200429204153916.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200429204233788.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4NDQxNjky,size_16,color_FFFFFF,t_70)

##### 4.8 Display RobotModel

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200501165154149.gif)
##### 4.9 Available in six rviz tools

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200515184545845.png)
##### 4.10 Show The Topic List

[![image.png](https://i.postimg.cc/Z5bGBfgk/image.png)](https://postimg.cc/svL6bJsK)
##### 4.11 To be perfected....

### 二，Installation tutorial

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

Put the package in the ros src package directory：

```cpp
catkin_make
```
#### 3，run

```cpp
rosrun cyrobot_monitor cyrobot_monitor
```
***
### LIENSE

**GNU GPL**

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200408135643929.png)

- As long as the software contains products or code that follow this Agreement, the software must also comply with the GPL License Agreement, i.e. it must be open source free, not charged from a closed source, and not commercial software.

*Key features of GPL open source protocol*

- Copy Freedom allows software to be copied to anyone's computer without limiting the number of copies.

- Freedom of communication Allows software to be disseminated in various forms.

- Fee-based communication allows the software to be sold in a variety of media, but buyers must be made aware in advance that the software is free to be obtained;

- Fee-based communication allows the software to be sold in a variety of media, but buyers must be made aware in advance that the software is free to be obtained;
Freedom of modification allows developers to add or remove the functionality of the software, but the software must still be licensed on a GPL license agreement after modification.

