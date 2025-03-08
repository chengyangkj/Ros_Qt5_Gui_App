# Developer Guide

## Development Environment Setup

### IDE Configuration

#### Qt Creator
1. Install Qt Creator:
```bash
sudo apt-get install qtcreator
```

2. Open project:
- Select File -> Open File or Project
- Select CMakeLists.txt in project root directory
- Configure build directory

#### VSCode
1. Install required plugins:
- C/C++
- CMake
- CMake Tools

2. Configure CMake:
- Configure CMake path in `.vscode/settings.json`
- Set build directory

## Code Structure

```
.
├── doc/                    # Project documentation
│   ├── images/            # Documentation images
│   ├── configuration.md   # Configuration guide
│   ├── usage.md          # Usage guide
│   ├── development.md    # Development guide
│   └── faq.md           # FAQ
├── include/               # Header files
│   ├── common/           # Common utility headers
│   ├── widgets/          # UI component headers
│   └── ros_bridge/       # ROS communication interface headers
├── src/                   # Source code
│   ├── common/           # Common utility implementation
│   │   ├── configuration.cpp    # Configuration management
│   │   ├── global.cpp          # Global variables
│   │   └── topic_manager.cpp   # Topic management
│   ├── widgets/          # Qt UI components
│   │   ├── main_window.cpp     # Main window
│   │   ├── map_widget.cpp      # Map component
│   │   ├── control_panel.cpp   # Control panel
│   │   └── dashboard.cpp       # Dashboard component
│   ├── ros_bridge/       # ROS communication interface
│   │   ├── ros_bridge.cpp      # ROS bridge layer
│   │   ├── topic_subscriber.cpp # Topic subscription
│   │   └── topic_publisher.cpp  # Topic publishing
│   └── main.cpp          # Program entry
├── resources/            # Resource files
│   ├── images/          # Icons and images
│   ├── qss/            # Style sheets
│   └── translations/   # Language files
├── scripts/             # Helper scripts
├── test/                # Test code
└── CMakeLists.txt      # CMake build file
```

## Development Standards

### Code Style
- Use clang-format for code formatting
- Follow Google C++ Style Guide

### Commit Convention
- feat: New feature
- fix: Bug fix
- docs: Documentation update
- style: Code format changes
- refactor: Code refactoring
- test: Test case changes
- chore: Build process or auxiliary tool changes 