# Usage Guide

## Map Display and Editing

### Map Display
The software supports displaying global and local maps, with map data coming from ROS topics.

To enable map display, ensure the following configuration in `config.json`:
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

### Map Editing
The following editing features are available:

![Map Editing](./images/edit_map.png)
![Map Editing Tools](./images/edit_map2.png)

#### Topological Map
Support drag-and-drop setting of robot navigation target points:

![Navigation Points](./images/set_nav_goal.gif)

Note: If navigation point publishing has no response, check the following configuration:
```json
{
  "move_base_simple": {
    "display_name": "NavGoal",
    "topic": "/move_base_simple/goal",
    "enable": true
  }
}
```

#### Eraser Tool
Click the eraser to erase obstacles in the map:

![Eraser Tool](./images/erase.gif)

#### Pencil Tool
Use the pencil to draw obstacles:

![Pencil Tool](./images/pencil.gif)

#### Line Drawing
Draw straight lines on the map:

![Line Drawing](./images/draw_line.gif)

#### Map Saving
After editing, click the save button to save:
- *.pgm - Image data
- *.yaml - Map description file
- *.topology - Topological data (navigation point information)

![Save Map](./images/save_map.png)

## Robot Control

### Manual Control
Use keyboard or interface buttons to control the robot:

![Manual Control](./images/manual_control.jpg)

Note: Please check the manual control configuration:
```json
{
  "velocity": {
    "display_name": "Speed",
    "topic": "/cmd_vel",
    "enable": true
  }
}
```

### Robot Relocation
Hold left button to drag position, right button to rotate:

![Relocation](./images/reloc.png)
![Relocation Demo](./images/reloc.gif)

Note: Ensure correct configuration:
```json
{
  "initialpose": {
    "display_name": "Reloc",
    "topic": "/initialpose",
    "enable": true
  }
}
```

### Speed Dashboard
Display real-time robot speed:

![Speed Dashboard](./images/speed_dashboard.jpg)

Required configuration:
```json
{
  "odom": {
    "display_name": "Odometry",
    "topic": "/odom",
    "enable": true
  }
}
```

### Battery Display
Show real-time battery status:

![Battery Status](./images/battery_state.png)

Configuration (using sensor_msgs::BatteryState):
```json
{
  "battery": {
    "display_name": "Battery",
    "topic": "/battery",
    "enable": true
  }
}
```

## Navigation Features

### Multi-point Navigation
Set multiple navigation points to execute in sequence:

![Multi-point Navigation](./images/multi_nav.png)
![Navigation Demo](./images/main.gif)

Usage steps:
1. Add navigation points
2. Set execution order
3. Click "Start Task" button
4. Monitor task progress

## Camera Display
Support multiple camera streams:
- RGB and depth images
- Compressed transmission
- Ported from rqt_image_view

Configuration example:
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

## Robot Shape Display
Support various robot shapes:
- Rectangle
- Circle
- Custom shapes

Configuration example:
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