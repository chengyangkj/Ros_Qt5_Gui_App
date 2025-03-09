<!--
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-09-02 07:23:43
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-06 14:03:03
 * @FilePath: /ROS2_Qt5_Gui_App/README.md
-->
<div align="center">

# ROS Qt5 GUI App

*A lightweight ROS1/ROS2 mobile robot human-machine interaction software*

[简体中文](./README.md) | [English](./README_en.md)

[![GitHub last commit](https://img.shields.io/github/last-commit/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)](https://github.com/chengyangkj/Ros_Qt5_Gui_App/commits/master)
[![GitHub stars](https://img.shields.io/github/stars/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)](https://github.com/chengyangkj/Ros_Qt5_Gui_App/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)](https://github.com/chengyangkj/Ros_Qt5_Gui_App/network/members)
[![GitHub issues](https://img.shields.io/github/issues/chengyangkj/Ros_Qt5_Gui_App?style=flat-square)](https://github.com/chengyangkj/Ros_Qt5_Gui_App/issues)
[![QQ Group](https://img.shields.io/badge/QQ%20Group-797497206-purple)](http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&k=mvzoO6tJQtu0ZQYa_itHW7JrT0i4OCdK&authKey=exOT53pUpRG85mwuSMstWKbLlnrme%2FEuJE0Rt%2Fw6ONNvfHqftoWMay03mk1Qi7yv&noverify=0&group_code=797497206)

![humble](https://github.com/chengyangkj/Ros_Qt5_Gui_App/actions/workflows/ros_humble_build.yaml/badge.svg)
![foxy](https://github.com/chengyangkj/Ros_Qt5_Gui_App/actions/workflows/ros_foxy_build.yaml/badge.svg)
![noetic](https://github.com/chengyangkj/Ros_Qt5_Gui_App/actions/workflows/ros_noetic_build.yaml/badge.svg)
![galactic](https://github.com/chengyangkj/Ros_Qt5_Gui_App/actions/workflows/ros_galactic_build.yaml/badge.svg)
![melodic](https://github.com/chengyangkj/Ros_Qt5_Gui_App/actions/workflows/ros_melodic_build.yaml/badge.svg)

</div>

## 📖 Introduction

This project is developed based on Qt5 and built with CMake, enabling the use of a single codebase in both ROS1/ROS2 systems. During compilation, the software automatically detects the ROS1/ROS2 environment variables and builds accordingly, achieving isolation between ROS communication and interface.

All features are self-implemented through custom drawing, making it easy to run on low-performance edge devices. The project has integrated CI to ensure compatibility across multiple ROS versions and system versions.

### ✨ Features

| Feature | Status | Notes |
|---------|--------|--------|
| ROS1/ROS2 Communication | ✅ | |
| Global/Local Map Display | ✅ | |
| Real-time Robot Position Display | ✅ | |
| Robot Speed Dashboard | ✅ | |
| Manual Robot Control | ✅ | |
| Robot Relocation | ✅ | |
| Single/Multi-point Navigation | ✅ | |
| Global/Local Path Display | ✅ | |
| Robot Topological Map | ✅ | |
| Battery Level Display | ✅ | |
| Map Editing | ✅ | |
| Navigation Task Chain | ✅ | Has bugs |
| Map Load/Save | ✅ | |
| Camera Image Display | ✅ | Ported from rqt_image_view |
| Robot Footprint Display | ✅ | Supports custom shapes |
| Rosbridge Communication | 🏷️ | In development |
| 3D Layer Display | 🏷️ | In development |
| Topological Path Planning | 🏷️ | In development |
| Robot History Trail Recording | 🏷️ | In development |

### 🖼️ Interface Preview

![Main Interface](./doc/images/main.png)
![Running Effect](./doc/images/main.gif)
![Mapping Effect](./doc/images/mapping.gif)

## 🚀 Quick Start

### Requirements

- Ubuntu 18.04+
- ROS1/ROS2 environment
- Qt5 basic environment

### Install Dependencies

```bash
sudo apt-get update
sudo apt-get install qtbase5-private-dev libqt5svg5-dev libsdl-image1.2-dev libsdl1.2-dev -y
```

### Build

```bash
mkdir -p ~/qt_ws
cd ~/qt_ws
git clone https://github.com/chengyangkj/Ros_Qt5_Gui_App
cd Ros_Qt5_Gui_App
mkdir build && cd build
cmake ..
make
```

### Run

```bash
./ros_qt5_gui_app
```

## 📚 Documentation

- [Configuration Guide](./doc/configuration_en.md)
- [User Guide](./doc/usage_en.md)
- [Development Guide](./doc/development_en.md)
- [FAQ](./doc/faq_en.md)

## 🤝 Contributing

[Issues](https://github.com/chengyangkj/Ros_Qt5_Gui_App/issues) and [Pull Requests](https://github.com/chengyangkj/Ros_Qt5_Gui_App/pulls) are welcome.

## 📊 Star History

<div align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Qt5_Gui_App&type=Timeline&theme=dark" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Qt5_Gui_App&type=Timeline" />
    <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=chengyangkj/Ros_Qt5_Gui_App&type=Timeline" width="75%" />
  </picture>
</div>

## 📱 Related Projects

A cross-platform mobile robot HMI software based on Flutter for ROS1/ROS2 is now open source:

![Flutter Version](./doc/images/flutter.png)

For details, visit [ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)

## 🔗 Related Links

| Branch | Supported Platforms | Description |
|--------|-------------------|-------------|
| [master](https://github.com/chengyangkj/ROS_Qt5_Gui_App/tree/master) | Win10 Ubuntu | ROS + QWidget + QGraphicsview custom visualization interface display |
| [qml_hmi](https://github.com/chengyangkj/ROS_Qt5_Gui_App/tree/qml_hmi) | Win10 Ubuntu | ROS + QML + C++ hybrid programming, QML self-drawn map, lidar and other visualization demos |
| [simple](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/simple) | Win10 Ubuntu | ROS + QWidget + Librviz visualization display, CSDN blog "ROS Human-Machine Interaction Software Development" course implementation version |
| [rviz_tree](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/rviz_tree) | Win10 Ubuntu | ROS + QWidget + Librviz native layer API management, no need to manually create layers |
| [ros_qt_demo](https://github.com/chengyangkj/ros_qt_demo) | Win10 Ubuntu | Original package created using cakin_create_qt_pkg, cmakelist.txt configured for qt5, can be directly compiled and run |
| [ros2_qt_demo](https://github.com/chengyangkj/ros2_qt_demo) | ROS2 | Qt demo package running on ROS2 platform, cmakelist.txt configured for qt5, can be built using colcon build |
| [ROS2_Qt5_Gui_App](https://github.com/chengyangkj/ROS2_Qt5_Gui_App) | ROS2 | Same as this repository/no longer maintained |
| [Flutter App](https://github.com/chengyangkj/ROS_Flutter_Gui_App) | Multi-platform (Flutter) | Implemented |

## 💬 Discussion Group

QQ Group: 797497206

## 📄 License

This project is licensed under the [MIT](LICENSE) License.

