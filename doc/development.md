# 开发者指南

## 开发环境配置

### IDE 配置

#### Qt Creator
1. 安装 Qt Creator:
```bash
sudo apt-get install qtcreator
```

2. 打开项目:
- 选择 File -> Open File or Project
- 选择项目根目录下的 CMakeLists.txt
- 配置构建目录

#### VSCode
1. 安装必要插件:
- C/C++
- CMake
- CMake Tools

2. 配置 CMake:
- 在 `.vscode/settings.json` 中配置 CMake 路径
- 设置构建目录

## 代码结构

```
src
├── CMakeLists.txt
├── basic
├── channel
│   ├── CMakeLists.txt
│   ├── manager
│   ├── ros1
│   ├── ros2
├── common
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── config
│   │   └── logger
│   └── src
│       ├── config
│       │   ├── config_manager.cc
│       │   └── topologyMap_test.cpp
│       └── logger
│           ├── easylogging++.cc
│           └── logger.cc
├── core
│   ├── CMakeLists.txt
│   ├── main.cpp
│   └── runtime
├── mainwindow
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── display
│   │   ├── mainwindow.h
│   │   └── widgets
│   ├── resource
│   │   ├── background
│   │   ├── images
│   │   ├── images.qrc
│   │   ├── media
│   │   │   ├── refresh_return.wav
│   │   │   └── start_return.wav
│   │   └── media.qrc
│   ├── src
│   │   ├── display
│   │   │   ├── display_cost_map.cpp
│   │   │   ├── display_demo.cpp
│   │   │   ├── display_occ_map.cpp
│   │   │   ├── display_path.cpp
│   │   │   ├── laser_points.cpp
│   │   │   ├── manager
│   │   │   │   ├── display_factory.cpp
│   │   │   │   ├── display_manager.cpp
│   │   │   │   ├── scene_manager.cpp
│   │   │   │   └── view_manager.cpp
│   │   │   ├── point_shape.cpp
│   │   │   ├── robot_shape.cpp
│   │   │   └── virtual_display.cpp
│   │   ├── mainwindow.cpp
│   │   └── widgets
│   │       ├── dashboard.cpp
│   │       ├── joystick.cpp
│   │       ├── nav_goal_table_view.cpp
│   │       ├── nav_goal_widget.cpp
│   │       ├── ratio_layouted_frame.cpp
│   │       └── set_pose_widget.cpp
│   └── ui
│       └── mainwindow.ui
└── plugin
    ├── CMakeLists.txt
    └── task_processor
        ├── task_processor.cpp
        ├── task_processor.h
        └── task_processor_test.cc
```

## 开发规范

### 代码风格
- 使用 clang-format 格式化代码
- 遵循 Google C++ 代码规范

### 提交规范
- feat: 新功能
- fix: 修复 bug
- docs: 文档更新
- style: 代码格式修改
- refactor: 代码重构
- test: 测试用例修改
- chore: 构建过程或辅助工具的变动

