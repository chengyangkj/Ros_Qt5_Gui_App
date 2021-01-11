
## 软件界面（开发中）

![image.png](https://postimg.cc/7CnZr7tm)

## 一，安装教程

开发期间，目前基于turtlebot3仿真环境进行开发，暂未预留其他接口，移植使用的话需要更改代码中的一些话题，tf变换等信息

### 1，安装qtcreator

这里使用的为5.9.9版本（qt5版本的均可）:

[qt5.9.9 download](http://download.qt.io/archive/qt/5.9/5.9.9/qt-opensource-linux-x64-5.9.9.run)

[qt all download](http://download.qt.io/archive/qt/)

qt安装路径需为默认，即为用户的宿主目录，如果安装路径更改了，在下一步配置环境变量中也需要根据自己的实际安装路径进行更改
### 2,配置qt环境变量

```shell
sudo gedit ~/.bashrc
```
添加如下(注意将QT_VERSION改为自己实际版本，我这里版本为5.9.9)：
```shell
#qt
export QT_VERSION=5.9.9
export QTDIR=~/Qt$QT_VERSION/$QT_VERSION/gcc_64
export LD_LIBRARY_PATH=$QTDIR/lib:$LD_LIBRARY_PATH
# export PATH=$PATH:$QTDIR/plugins/platforms
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PAth:$QTDIR/lib/cmake
export QT_QPA_PLATFORM_PLUGIN_PATH=$QTDIR/plugins/platforms/
#export QT_DEBUG_PLUGINS=1
```

### 3，克隆源码
进入工作空间的src目录下：

```shell
git clone https://github.com/chengyangkj/Ros_Qt5_Gui_App
```
切换分支：
```shell
git checkout qml_hmi
```
### 4，编译
进入工作空间根目录
```shell
catkin_make
```
### 5，运行

```shell
rosrun cyrobot_monitor cyrobot_monitor
```
