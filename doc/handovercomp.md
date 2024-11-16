# ICRA 2024 Handover Competition

## Launch camera

### Camera Left
```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 pointcloud.enable:=false camera_namespace:=camera_left camera_name:=camera_left serial_no:=_102422072725
```

### Camera Right
```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 pointcloud.enable:=false camera_namespace:=camera_right camera_name:=camera_right serial_no:=_048322070613
```

## Camera Calibration

```bash
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 8x6 --square 0.024 right:=/camera_right/color/image_raw left:=/camera_left/color/image_raw right_camera:=/camera_right left_camera:=/camera_left
```

Mono
```bash
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 8x6 --square 0.024 image:=/camera/color/image_raw camera:=/camera/color
```