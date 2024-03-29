# Onrobot Soft Gripper
## Requirement
```
sudo pip3 install -U pymodbus
```

## Installation
```
mkdir -p ws_gripper/src
cd ws_gripper/src
git clone
cd ..
colcon build
source ~/ws_gripper/install/setup.bash
```

## Usage
- Bring up the gripper. Enter the launch folder
```
ros2 launch bringup_gripper.launch.py
```

- Control the gripper via rosservice, where desired width is between 110mm and 750 mm
```
ros2 service call /gripper_command onrobotsg_interfaces/srv/Sg desiredwidth:\ desired width\
```

- Control via client service. Go to client.py file and change the desired width value then call:
```
ros2 run onrobotsg onrobotsg_client
```