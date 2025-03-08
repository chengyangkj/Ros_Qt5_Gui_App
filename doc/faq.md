# 常见问题

## 编译相关

### Q: 编译时提示缺少 Qt 依赖
A: 执行以下命令安装完整的 Qt 依赖：
```bash
sudo apt-get install qtbase5-dev qt5-qmake qtbase5-dev-tools libqt5svg5-dev qtbase5-private-dev -y
```

### Q: 找不到 ROS 相关头文件
A: 确保已经 source 了 ROS 环境：
```bash
# ROS1
source /opt/ros/<distro>/setup.bash

# ROS2
source /opt/ros/<distro>/setup.bash
```

## 运行相关

### Q: 程序启动后无法显示地图
可能的原因：
1. ROS 话题未正确配置
2. 地图服务未启动
3. 网络连接问题

解决方案：
1. 检查 config.json 中的话题配置
2. 确认地图服务是否正常运行
3. 检查 ROS_MASTER_URI 配置

### Q: 手动控制不响应
可能的原因：
1. 速度话题配置错误
2. 权限问题

解决方案：
1. 检查 cmd_vel 话题配置
2. 确认用户有权限发布速度指令

### Q: 相机图像显示黑屏
可能的原因：
1. 相机驱动未启动
2. 图像话题配置错误
3. 图像格式不兼容

解决方案：
1. 启动相机驱动
2. 检查图像话题配置
3. 确认图像编码格式

## 功能相关

### Q: 如何添加自定义导航点？
A: 在地图编辑模式下：
1. 点击"添加导航点"按钮
2. 在地图上点击需要添加的位置
3. 输入导航点名称
4. 保存修改

### Q: 如何配置异形车身？
A: 修改 config.json 中的 robot_shape_config：
1. 设置 shaped_points 数组
2. 按顺时针顺序添加车身轮廓点
3. 设置 is_ellipse 为 false
4. 调整 color 和 opacity

### Q: 多点导航任务如何使用？
A: 按以下步骤操作：
1. 添加多个导航点
2. 设置导航点顺序
3. 点击"开始任务"按钮
4. 等待任务完成

## 其他问题

### Q: 如何报告 Bug？
A: 请在 GitHub Issues 中提交问题，并提供：
1. 问题描述
2. 复现步骤
3. 系统环境信息
4. 错误日志
5. 截图（如果有）

### Q: 如何参与项目开发？
A: 欢迎提交 Pull Request：
1. Fork 项目
2. 创建特性分支
3. 提交代码修改
4. 创建 Pull Request
5. 等待审核 