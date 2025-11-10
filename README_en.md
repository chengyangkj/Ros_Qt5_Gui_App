<!--
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-09-02 07:23:43
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2024-01-15
 * @FilePath: /Ros_Qt5_Gui_App/README_en.md
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
|---------|--------|-------|
| ROS1 Communication Support | ✅ | Basic features implemented, continuously optimized |
| ROS2 Communication Support | ✅ | Stable and long-term support |
| ROSBridge Communication Support | ✅ | Supports WebSocket connection and automatic reconnection |
| Global/Local Map Display | ✅ | Supports OccupancyGrid maps |
| Real-time Robot Position Display | ✅ | Based on TF transforms |
| Robot Speed Dashboard | ✅ | Real-time linear and angular velocity display |
| Manual Robot Control | ✅ | Supports velocity control |
| Robot Relocation | ✅ | Supports 2D Pose Estimate |
| Single/Multi-point Navigation | ✅ | Supports navigation goal setting |
| Global/Local Path Display | ✅ | Real-time planned path display |
| Topological Point Editing | ✅ | Visual editing of topological points |
| Battery Level Display | ✅ | Subscribes to BatteryState topic |
| Map Obstacle Editing | ✅ | Supports map editing |
| Topological Path Editing | ✅ | Visual editing of topological paths |
| Map Load/Save | ✅ | Supports map file management |
| Camera Image Display | ✅ | Supports multiple image streams |
| Robot Footprint Display | ✅ | Subscribes to footprint topic |
| LiDAR Display | ✅ | Supports LaserScan visualization |

### 🖼️ Interface Preview

![Main Interface](./doc/images/main.png)
![Running Effect](./doc/images/main.gif)
![Mapping Effect](./doc/images/mapping.gif)

## 🚀 Quick Start

### Requirements

- **Operating System**: Ubuntu 18.04+ / Windows 10+
- **ROS Environment**: ROS1 (Melodic/Noetic) or ROS2 (Foxy/Galactic/Humble)
- **Qt5**: Qt5.12+ (Qt5 Core, Widgets, SVG)
- **CMake**: 3.16+
- **Compiler**: GCC 7+ / MSVC 2019+

### Install Dependencies

#### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y \
  qtbase5-dev \
  qtbase5-private-dev \
  libqt5svg5-dev \
  qtbase5-dev-tools \
  libeigen3-dev \
  libgtest-dev \
  libsdl-image1.2-dev \
  libsdl1.2-dev
```

#### Windows

Windows platform requires manual installation of Qt5 and environment variable configuration, or use package managers like vcpkg.

### CMake Upgrade

Systems with Ubuntu 20.04 and below come with an outdated CMake version that needs to be upgraded to 3.16+. Ubuntu 22.04 and above can skip this step.

```bash
wget https://cmake.org/files/v3.16/cmake-3.16.4-Linux-x86_64.sh -O cmake-install.sh
chmod +x cmake-install.sh
sudo ./cmake-install.sh --prefix=/usr/local --skip-license
```

### Build

```bash
# Clone repository
git clone https://github.com/chengyangkj/Ros_Qt5_Gui_App.git
cd Ros_Qt5_Gui_App

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make -j$(nproc)  # Linux
# or
cmake --build . --config Release  # Windows
```

### Run

#### Method 1: Using Startup Script (Recommended)

After building, the startup script will be automatically copied to the `build` directory. Simply run:

```bash
cd build
./start.sh  # Linux
# or
start.bat   # Windows
```

The startup script will automatically:
- Set library file paths
- Launch the program

#### Method 2: Manual Run

```bash
cd build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib  # Linux
./ros_qt5_gui_app
```

#### Method 3: Run After Installation

```bash
cd build
make install  # Linux
# or
cmake --install . --config Release  # Windows

cd ../install/bin
./start.sh  # Linux
# or
start.bat   # Windows
```

### Configuration

Before first run, please ensure:

1. **ROS Environment Configured**: Make sure ROS setup.bash/setup.bat has been sourced
2. **Topic Configuration**: Check if topic names in `config.json` match your ROS system
3. **Channel Selection**: Select the correct communication channel (ROS1/ROS2/ROSBridge) in the configuration interface

For detailed configuration instructions, please refer to [Configuration Documentation](./doc/configuration_en.md)

## 📚 Documentation

- [Configuration Guide](./doc/configuration_en.md) - Detailed configuration options
- [User Guide](./doc/usage_en.md) - Feature usage tutorials
- [Development Guide](./doc/development_en.md) - Development environment setup and code structure
- [FAQ](./doc/faq_en.md) - FAQ and troubleshooting

## 🏗️ Project Structure

```
Ros_Qt5_Gui_App/
├── src/                    # Source code directory
│   ├── core/              # Core module (main program entry)
│   ├── mainwindow/        # Main window and interface
│   ├── common/            # Common libraries
│   ├── basic/             # Basic data structures
│   ├── channel/           # Communication channels (ROS1/ROS2/ROSBridge)
│   └── plugin/            # Plugin system
├── install/               # Installation scripts
│   ├── linux/bin/        # Linux startup scripts
│   └── windows/bin/       # Windows startup scripts
├── doc/                   # Documentation directory
├── cmake/                 # CMake modules
└── CMakeLists.txt        # Main CMake configuration file
```

## 🤝 Contributing

Welcome to submit [Issues](https://github.com/chengyangkj/Ros_Qt5_Gui_App/issues) and [Pull Requests](https://github.com/chengyangkj/Ros_Qt5_Gui_App/pulls)!

If you have any ideas or suggestions, feel free to submit them to [🌟 Wishlist/Requirements](https://github.com/chengyangkj/Ros_Qt5_Gui_App/issues/29). They might be implemented someday!

### Contributing Guide

1. Fork this repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📊 Star History

<div align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Qt5_Gui_App&type=Timeline&theme=dark" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Qt5_Gui_App&type=Timeline" />
    <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=chengyangkj/Ros_Qt5_Gui_App&type=Timeline" width="75%" />
  </picture>
</div>

## 📱 Related Projects

A cross-platform mobile robot HMI software based on Flutter for ROS1/ROS2 is now officially open source:

![Flutter Version](./doc/images/flutter.png)

For details, visit [ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)

## 🔗 Related Links

| Branch | Supported Platforms | Description |
|--------|-------------------|-------------|
| [master](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/master) | Win10 Ubuntu | ROS + QWidget + QGraphicsView custom visualization interface display |
| [qml_hmi](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/qml_hmi) | Win10 Ubuntu | ROS + QML + C++ hybrid programming, QML self-drawn map, lidar and other visualization demos |
| [simple](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/simple) | Win10 Ubuntu | ROS + QWidget + Librviz visualization display, CSDN blog "ROS Human-Machine Interaction Software Development" course implementation version |
| [rviz_tree](https://github.com/chengyangkj/Ros_Qt5_Gui_App/tree/rviz_tree) | Win10 Ubuntu | ROS + QWidget + Librviz native layer API management, no need to manually create layers |
| [ros_qt_demo](https://github.com/chengyangkj/ros_qt_demo) | Win10 Ubuntu | Original package created using catkin_create_qt_pkg, CMakeLists.txt configured for Qt5, can be directly compiled and run |
| [ros2_qt_demo](https://github.com/chengyangkj/ros2_qt_demo) | ROS2 | Qt demo package running on ROS2 platform, CMakeLists.txt configured for Qt5, can be built using colcon build |
| [ROS2_Qt5_Gui_App](https://github.com/chengyangkj/ROS2_Qt5_Gui_App) | ROS2 | Same as this repository/no longer maintained |
| [Flutter App](https://github.com/chengyangkj/ROS_Flutter_Gui_App) | Multi-platform (Flutter) | Implemented |

## 💬 Discussion Group

- **QQ Group**: 797497206
- **Issues**: [GitHub Issues](https://github.com/chengyangkj/Ros_Qt5_Gui_App/issues)

## 📄 License

This project is licensed under the [MIT](LICENSE) License.

## 🙏 Acknowledgments

Thanks to all contributors and users for their support!
