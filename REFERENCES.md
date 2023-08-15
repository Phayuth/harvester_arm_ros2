# UR5e
---
### Install
1. [UR5 ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy)
2. ros2_control
3. [calibration](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_calibration/README.md)

### Launch Minimal UR5e with Rviz and Joint controller
- Fake Hardware
```ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true```

- Real Hardware
```ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.3 launch_rviz:=true```


# Robotiq 2f Gripper
---
### Install
```
sudo pip3 install -U pymodbus
sudo pip3 install pyserial
```
1. Check usb port with `dmesg | egrep --color 'serial|tty'`
2. Give permission to port with `sudo chmod a+rw /dev/ttyUSB1`

### Usage
- run 
```demo.py```

### Reference
https://github.com/KavrakiLab/robotiq_85_gripper


# onrobot Soft Gripper
---
### Requirement
```
sudo pip3 install -U pymodbus
```

### Installation
```
mkdir -p ws_gripper/src
cd ws_gripper/src
git clone 
cd ..
colcon build
source ~/ws_gripper/install/setup.bash
```

### Usage
- Bring up the gripper. Enter the launch folder
```ros2 launch bringup_gripper.launch.py```

- Control the gripper via rosservice, where desired width is between 110mm and 750 mm
```ros2 service call /gripper_command onrobotsg_interfaces/srv/Sg desiredwidth:\ desired width\```

- Control via client service. Go to client.py file and change the desired width value then call:
```ros2 run onrobotsg onrobotsg_client```


# IntelRealSense
---

[Link](https://github.com/IntelRealSense/realsense-ros)

### Usage
```
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30
ros2 param set /camera/camera pointcloud.ordered_pc True
ros2 launch realsense2_camera rs_launch.py ordered_pc:=true pointcloud.enable:=true align_depth.enable:=true
```