## ROS Qt Deskotp GUI App(librviz分支 使用librviz的菜单栏实现图层管理功能)
- Use qt5 to implement the ros robot human-machine interface
- 使用qt5实现ros机器人人机界面

- Attention! This code is for learning purposes only and cannot be used for any other purpose without the author's permission.
- 注意！未经作者的许可，此代码仅用于学习，不能用于其他用途。

- Continuously updating.....
- 持续更新中.....

- Welcome to submit bugs in issues
- 欢迎在issues提交bug
***
## 分支
**rviz menu tree branch rviz  菜单树分支**
- With the rviz menu interface, there is no need to encapsulate the menu yourself.
- 采用rviz菜单接口，无需自己封装菜单
- Through the menu interface, all the Display functions of rviz can be implemented.
- 通过菜单接口，可以实现rviz所有的Display功能
- Supports the import and export of configured Display items for quick and easy.
- 支持导入导出已配置好的Display项，方便快速

[Go back master 回到主分支](https://github.com/chengyangkj/Ros_Qt5_Gui_App)

**警告：此页面上的所有图片都较大，加载较慢**
![rviz_tree-image](http://qghk8ygxs.hn-bkt.clouddn.com/rviz_tree-image.png)

***

### 一，Features
### 一，功能介绍
#### 1，Add Display
#### 1，新增显示
- The new display can provide a series of functions such as drawing, sweeping, etc.
- 新增显示后可以提供建图、扫图等一系列功能。

![Add Display](http://danpe.oss-cn-shanghai.aliyuncs.com/github/add_map.gif)

![slammap](http://danpe.oss-cn-shanghai.aliyuncs.com/github/slammap.gif)


#### 2，Displays import or derive
#### 2,  显示列表的导入导出
![Displays import or derive](http://danpe.oss-cn-shanghai.aliyuncs.com/github/readsavedisplays.gif)

##### 3 To be perfected....
##### 3 待完善....

***

### 二，安装教程
### 二，Installation tutorial
#### 1，首先安装ros对qt pkg的支持
#### 1，first install ros support for qt pkg
``` bash
sudo apt-get install ros-melodic-qt-create
```

``` bash
sudo apt-get install ros-melodic-qt-build
```
``` bash
sudo apt-get install qtmultimedia5-dev
```
#### 2，Git code
#### 2，获取源码
Git code
获取源码
``` bash
git clone https://github.com/chengyangkj/Ros_Qt5_Gui_App
```
Go to the branch
转到分支
``` bash
cd Ros_Qt5_Gui_App
git checkout rviz_tree
```

#### 3，Compile
#### 3，编译
Put the package in the ros src package directory：
将软件包放入ros src软件包目录下：
``` bash
catkin_make
```
#### 4，run
#### 5，运行
``` bash
rosrun cyrobot_rviz_tree cyrobot_rviz_tree
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
