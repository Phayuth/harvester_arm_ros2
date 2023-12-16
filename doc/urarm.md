# UR5e
## Install
1. [UR5 ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy)
2. ros2_control
3. [calibration](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_calibration/README.md)

## Launch Minimal UR5e with Rviz and Joint controller
- Fake Hardware

```
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```

- Real Hardware
```
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.3 launch_rviz:=true
```

# DH download from ros2 ur5e calibration

| Parameter | dh_theta               | dh_a                   | dh_d                | dh_alpha             |
| ---       | ---                    | ---                    | ---                 | ---                  |
| Joint 1   | -4.28707430111253e-08, | -1.99460667323154e-05, | 0.162553510137443,  | 1.57081954464255,    |
| Joint 2   | 1.48230050234003,      | -0.0375694843707429,   | -414.242852801104,  | -0.00102218947890982,|
| Joint 3   | 4.92978966326288,      | -0.389040114197933,    | 425.001713579027,   | 0.00468723169528323, |
| Joint 4   | -0.128903795426445,    | 6.30974349355248e-05,  | -10.625155821775,   | 1.5698846550097,     |
| Joint 5   | -2.07851623169886e-07, | 1.87759283570936e-05,  | 0.0997619369268171, | -1.57071698140961,   |
| Joint 6   | -3.19836189504718e-07  | 0                      | 0.0997305031064922  | 0                    |