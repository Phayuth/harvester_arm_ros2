# Intel RealSense

## Usage

### ROS2 Foxy

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 ordered_pc:=true pointcloud.enable:=true align_depth.enable:=true

ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x9 rgb_camera.profile:=1280x720x9 ordered_pc:=true pointcloud.enable:=true align_depth.enable:=true

ros2 param set /camera/camera pointcloud.ordered_pc True
```

### ROS Noetic

```bash
roslaunch realsense2_camera rs_aligned_depth.launch filters:=pointcloud ordered_pc:=true
```

## Reference

* [Realsense ROS Package](https://github.com/IntelRealSense/realsense-ros)
* [ROS2 Camera Calibration](https://docs.ros.org/en/ros2_packages/rolling/api/camera_calibration/index.html)



```bash
sudo docker run -it --runtime nvidia --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix stereolabs/zed:4.1-gl-devel-cuda11.4-ubuntu20.04
```
xhost +si:localuser:root # allow connection to xserver
sudo docker images
sudo docker ps -a
sudo docker start -i ab5a59f8c78e

<!-- new bash terminal -->
docker exec -it <container> bash

<!-- copy -->
docker cp <containerId>:/file/path/within/container /host/path/target
docker cp ab5a59f8c78e:/root/Documents/ZED/Meshes/Live_27350348_10-05-2024-16-24/mesh.obj /home/yuth/meshscan
sudo docker cp ab5a59f8c78e:/root/Documents/ZED/Meshes/Live_27350348_10-05-2024-16-17/fused_point_cloud.ply /home/yuth/meshscan


ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2