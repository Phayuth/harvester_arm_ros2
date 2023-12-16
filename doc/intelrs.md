# Intel RealSense
## Usage
```
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 ordered_pc:=true pointcloud.enable:=true align_depth.enable:=true
ros2 param set /camera/camera pointcloud.ordered_pc True
```

## Reference
- [Realsense ROS Package](https://github.com/IntelRealSense/realsense-ros)
