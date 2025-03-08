# 配置说明

## 配置文件位置

配置文件 `config.json` 在首次运行软件后会自动生成在可执行程序同级目录下。修改配置后需要重启软件生效。

## 配置项说明

### ROS 话题配置

```json
{
  "topics": {
    "map": {
      "display_name": "Map",
      "topic": "/map",
      "enable": true
    },
    "laser": {
      "display_name": "LaserScan", 
      "topic": "/scan",
      "enable": true
    },
    "odom": {
      "display_name": "Odometry",
      "topic": "/odom", 
      "enable": true
    },
    "velocity": {
      "display_name": "Speed",
      "topic": "/cmd_vel",
      "enable": true
    },
    "initialpose": {
      "display_name": "Reloc",
      "topic": "/initialpose",
      "enable": true
    },
    "move_base_simple": {
      "display_name": "NavGoal",
      "topic": "/move_base_simple/goal",
      "enable": true
    },
    "battery": {
      "display_name": "Battery",
      "topic": "/battery",
      "enable": true
    }
  }
}
```

### 机器人外形配置

```json
{
  "robot_shape_config": {
    "shaped_points": [
      {
        "x": 0.5,
        "y": 0.5
      },
      {
        "x": 0.5,
        "y": -0.5
      },
      {
        "x": -0.5,
        "y": -0.5
      },
      {
        "x": -0.5,
        "y": 0.5
      }
    ],
    "is_ellipse": false,
    "color": "0x00000FF",
    "opacity": 0.5
  }
}
```

### 相机配置

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