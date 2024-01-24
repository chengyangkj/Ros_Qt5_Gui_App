## 配置说明

第一次运行后，会在可执行程序同级目录生成config.json

图层相关的配置存在于:
display_config
拓扑地图配置存在于:
topology_map_config

### 1，局部代价地图配置

```
{
      "display_name": "LocalCostMap",
      "topic": "/local_costmap/costmap",
      "enable": true
}
```

### 2，全局代价地图配置

```
{
      "display_name": "GlobalCostMap",
      "topic": "/global_costmap/costmap",
      "enable": true
}
```
### 3，全局路径规划配置

```
{
      "display_name": "GlobalPlan",
      "topic": "/move_base/DWAPlannerROS/global_plan",
      "enable": true
}
```
### 4，激光雷达配置

```
{
      "display_name": "LaserScan",
      "topic": "/scan",
      "enable": true
}
```

### 5，机器人局部导航路径配置

```
{
      "display_name": "LocalPlan",
      "topic": "/move_base/DWAPlannerROS/local_plan",
      "enable": true
}
```

### 机器人全局地图

```
{
      "display_name": "Map,
      "topic": "/map",
      "enable": true
}
```

### 6，机器人导航目标点发布配置

```
{
      "display_name": "NavGoal",
      "topic": "/move_base_simple/goal",
      "enable": true
}
```
### 7，机器人里程计话题

```
{
      "display_name": "Odometry",
      "topic": "/odom ",
      "enable": true
}
```
### 8，机器人初始位置话题

```
{
      "display_name": "Reloc",
      "topic": "/initialpose",
      "enable": true
}
```
### 9，机器人速度话题

```
{
      "display_name": "Speed",
      "topic": "/cmd_vel",
      "enable": true
}
```

### 拓扑地图名

软件初始化时会读取此配置，用于加载/保存拓扑地图

其中拓扑地图的保存路径配置项为：

```
 "topology_map_config": {
    "map_name": "./default_topology_map.json"
  }

```
拓扑地图包含以下信息：
 - 导航目标点