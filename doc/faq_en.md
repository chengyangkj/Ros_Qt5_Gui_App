# Frequently Asked Questions

## Compilation Related

### Q: Missing Qt dependencies during compilation
A: Execute the following command to install complete Qt dependencies:
```bash
sudo apt-get install qtbase5-dev qt5-qmake qtbase5-dev-tools libqt5svg5-dev qtbase5-private-dev -y
```

### Q: Cannot find ROS related headers
A: Make sure ROS environment is sourced:
```bash
# ROS1
source /opt/ros/<distro>/setup.bash

# ROS2
source /opt/ros/<distro>/setup.bash
```

## Runtime Related

### Q: Program cannot display map after startup
Possible causes:
1. ROS topics not properly configured
2. Map service not running
3. Network connection issues

Solutions:
1. Check topic configuration in config.json
2. Verify map service is running normally
3. Check ROS_MASTER_URI configuration

### Q: Manual control not responding
Possible causes:
1. Velocity topic configuration error
2. Permission issues

Solutions:
1. Check cmd_vel topic configuration
2. Verify user has permission to publish velocity commands

### Q: Camera image shows black screen
Possible causes:
1. Camera driver not started
2. Image topic configuration error
3. Incompatible image format

Solutions:
1. Start camera driver
2. Check image topic configuration
3. Verify image encoding format

## Feature Related

### Q: How to add custom navigation points?
A: In map editing mode:
1. Click "Add Navigation Point" button
2. Click desired location on map
3. Enter navigation point name
4. Save changes

### Q: How to configure custom robot shape?
A: Modify robot_shape_config in config.json:
1. Set shaped_points array
2. Add outline points clockwise
3. Set is_ellipse to false
4. Adjust color and opacity

### Q: How to use multi-point navigation tasks?
A: Follow these steps:
1. Add multiple navigation points
2. Set navigation point order
3. Click "Start Task" button
4. Wait for task completion

## Other Issues

### Q: How to report bugs?
A: Please submit issues on GitHub Issues with:
1. Problem description
2. Steps to reproduce
3. System environment information
4. Error logs
5. Screenshots (if available)

### Q: How to contribute to the project?
A: Welcome to submit Pull Requests:
1. Fork the project
2. Create feature branch
3. Submit code changes
4. Create Pull Request
5. Wait for review 