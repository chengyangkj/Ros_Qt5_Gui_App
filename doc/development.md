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
.
├── doc/                    # 项目文档
│   ├── images/            # 文档图片
│   ├── configuration.md   # 配置说明
│   ├── usage.md          # 使用指南
│   ├── development.md    # 开发指南
│   └── faq.md           # 常见问题
├── include/               # 头文件
│   ├── common/           # 通用工具类头文件
│   ├── widgets/          # 界面组件头文件
│   └── ros_bridge/       # ROS通信接口头文件
├── src/                   # 源代码
│   ├── common/           # 通用工具类实现
│   │   ├── configuration.cpp    # 配置管理
│   │   ├── global.cpp          # 全局变量
│   │   └── topic_manager.cpp   # 话题管理
│   ├── widgets/          # Qt界面组件
│   │   ├── main_window.cpp     # 主窗口
│   │   ├── map_widget.cpp      # 地图组件
│   │   ├── control_panel.cpp   # 控制面板
│   │   └── dashboard.cpp       # 仪表盘组件
│   ├── ros_bridge/       # ROS通信接口
│   │   ├── ros_bridge.cpp      # ROS桥接层
│   │   ├── topic_subscriber.cpp # 话题订阅
│   │   └── topic_publisher.cpp  # 话题发布
│   └── main.cpp          # 程序入口
├── resources/            # 资源文件
│   ├── images/          # 图标等图片资源
│   ├── qss/            # 样式表
│   └── translations/   # 多语言翻译文件
├── scripts/             # 辅助脚本
├── test/                # 测试代码
└── CMakeLists.txt      # CMake构建文件
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

