## 配置说明

第一次运行后，会在可执行程序同级目录生成config.ini

### 1，局部代价地图配置

```
[LocalCostMap]
Topic=/local_costmap/costmap
```

### 2，全局代价地图配置

```
[GlobalCostMap]
Topic=/global_costmap/costmap  ###话题名
```
### 3，全局路径规划配置

```
[GlobalPlan]
Topic=/move_base/DWAPlannerROS/global_plan  ##话题名
```
### 4，激光雷达配置

```
[LaserScan]
Topic=/scan #话题名
```

### 5，机器人局部导航路径配置

```
[LocalPlan]
Topic=/move_base/DWAPlannerROS/local_plan  #话题名
```

### 机器人全局地图

```
[Map]
Topic=/map   #话题名
```

### 6，机器人导航目标点发布配置

```
[NavGoal]
Topic=/move_base_simple/goal #话题名，机器人需要接收此话题
```
### 7，机器人里程计话题

```
[Odometry]
Topic=/odom  #话题名，软件订阅此话题用于获取速度等信息
```
### 8，机器人初始位置话题

```
[Reloc]
Topic=/initialpose  #机器人重定位话题，软件发布此话题，机器人需要订阅此话题用于重定位
```
### 9，机器人速度话题

```
[Speed]
Topic=/cmd_vel  #机器人接收此话题，用于软件下发手动控制速度
```