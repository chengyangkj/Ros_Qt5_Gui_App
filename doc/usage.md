# 功能使用指南

## 地图显示与编辑

### 地图显示
软件支持显示全局地图和局部地图，地图数据来自 ROS 话题。

要启用地图显示，请确保 `config.json` 中有以下配置：
```json
{
  "topics": {
    "map": {
      "display_name": "Map",
      "topic": "/map",
      "enable": true
    }
  }
}
```

### 地图编辑
提供以下编辑功能：

![地图编辑](./images/edit_map.png)
![地图编辑工具](./images/edit_map2.png)

#### 拓扑地图
支持拖拽设置机器人导航目标点

拓扑地图 ROS2 消息定义：https://github.com/chengyangkj/topology_msgs

- 保存地图: 拓扑地图编辑完成后，点击保存地图按钮，软件会将地图按照打开的路径进行保存,存为*.topology的json格式
同时发布 /map/topology/update Topic，更新最新的拓扑地图，用户可订阅此话题做保存逻辑。

- 地图加载: 软件订阅 /map/topology 话题，如果话题没有订阅到内容，则会读取相对路径存储的默认地图


![导航点设置](./images/set_nav_goal.gif)

注意：如果导航点发布无响应，请检查以下配置：
```json
{
  "move_base_simple": {
    "display_name": "NavGoal",
    "topic": "/move_base_simple/goal",
    "enable": true
  }
}
```

#### 橡皮擦工具
点击橡皮擦可以擦除地图中的障碍物：

![橡皮擦工具](./images/erase.gif)

#### 画笔工具
使用画笔绘制障碍物：

![画笔工具](./images/pencil.gif)

#### 线段绘制
在地图上绘制直线：

![线段绘制](./images/draw_line.gif)

#### 地图保存
编辑完成后，点击保存按钮保存：
- *.pgm - 图像数据
- *.yaml - 地图描述文件
- *.topology - 拓扑数据（导航点信息）

![保存地图](./images/save_map.png)

## 机器人控制

### 手动控制
使用键盘或界面按钮控制机器人：

![手动控制](./images/manual_control.jpg)

注意：请检查手动控制的配置：
```json
{
  "velocity": {
    "display_name": "Speed",
    "topic": "/cmd_vel",
    "enable": true
  }
}
```

### 机器人重定位
左键按住拖动设置位置，右键旋转方向：

![重定位](./images/reloc.png)
![重定位演示](./images/reloc.gif)

注意：确保正确配置：
```json
{
  "initialpose": {
    "display_name": "Reloc",
    "topic": "/initialpose",
    "enable": true
  }
}
```

### 速度仪表盘
实时显示机器人速度：

![速度仪表盘](./images/speed_dashboard.jpg)

需要的配置：
```json
{
  "odom": {
    "display_name": "Odometry",
    "topic": "/odom",
    "enable": true
  }
}
```

### 电池显示
显示实时电池状态：

![电池状态](./images/battery_state.png)

配置（使用 sensor_msgs::BatteryState）：
```json
{
  "battery": {
    "display_name": "Battery",
    "topic": "/battery",
    "enable": true
  }
}
```

## 导航功能

### 多点导航
设置多个导航点按顺序执行：

![多点导航](./images/multi_nav.png)
![导航演示](./images/main.gif)

使用步骤：
1. 添加导航点
2. 设置执行顺序
3. 点击"开始任务"按钮
4. 监控任务进度

## 相机显示
支持多路相机图像：
- RGB和深度图像
- 压缩传输
- 移植自 rqt_image_view

配置示例：
```json
{
  "images": [
    {
      "location": "front",
      "topic": "/camera/rgb/image_raw",
      "enable": true
    },
    {
      "location": "front/depth",
      "topic": "/camera/depth/image_raw",
      "enable": true
    }
  ]
}
```

## 机器人车身显示
支持多种车身形状：
- 矩形
- 圆形
- 自定义形状

配置示例：
```json
{
  "robot_shape_config": {
    "shaped_points": [
      {"x": 0.5, "y": 0.5},
      {"x": 0.5, "y": -0.5},
      {"x": -0.5, "y": -0.5},
      {"x": -0.5, "y": 0.5}
    ],
    "is_ellipse": false,
    "color": "0x00000FF",
    "opacity": 0.5
  }
}
``` 