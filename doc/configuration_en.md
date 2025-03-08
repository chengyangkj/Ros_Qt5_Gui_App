# Configuration Guide

## Configuration File Location

The configuration file `config.json` will be automatically generated in the same directory as the executable when first running the software. Changes to the configuration require restarting the software to take effect.

## Configuration Items

### ROS Topic Configuration

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

### Robot Shape Configuration

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

### Camera Configuration

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