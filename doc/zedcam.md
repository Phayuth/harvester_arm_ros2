# ZedCam

## Usage

```bash
sudo docker run -it --runtime nvidia --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix stereolabs/zed:4.1-gl-devel-cuda11.4-ubuntu20.04

xhost +si:localuser:root # allow connection to xserver
sudo docker images
sudo docker ps -a
sudo docker start -i ab5a59f8c78e
```

## New bash terminal
```bash
docker exec -it containerId bash
```

## Copy
```bash
sudo docker cp containerId:/file/path/within/container /host/path/target
```

## ZED ROS Launch

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```


## Reference

* https://www.stereolabs.com/docs/docker/install-guide-linux
* https://www.stereolabs.com/docs/docker/using-ros
* https://github.com/stereolabs/zed-ros2-wrapper/tree/master