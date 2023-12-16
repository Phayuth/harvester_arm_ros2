# Robotiq 2f Gripper
## Install
```
sudo pip3 install -U pymodbus
sudo pip3 install pyserial
```
1. Check usb port with `dmesg | egrep --color 'serial|tty'`
2. Give permission to port with `sudo chmod a+rw /dev/ttyUSB1`

## Usage
- run 
```demo.py```

## Reference
- [Robotiq 2F Gripper Driver](https://github.com/KavrakiLab/robotiq_85_gripper)