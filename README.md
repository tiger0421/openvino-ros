# openvino-ros
This repository enables you to run semantic-segmentaion on ROS2 (eloquent).
Also, you can use ROS1 because ros-bridgge is automatically run.

## Usage
1. Run the docker image  
Terminal A
```
$ git clone https://github.com/tiger0421/openvino-ros.git
$ cd openvino-ros/docker
$ docker-compose up -d
```

2. Build this repo
```
$ docker exec -it vino /bin/bash
(in the container)
(openvino)$ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
(openvino)$ git clone https://github.com/tiger0421/openvino-ros.git
(openvino)$ cd ~/ros2_ws
(openvino)$ colcon build --symlink-install
```
(You can ignore the message about not found __init__.py)

3. Run segmentation node
```
(openvino)$ . install/setup.bash && . install/local_setup.bash
(openvino)$ ros2 run vino segmentation_image
```

  TerminalB
```
$ docker exec -it roscore /bin/bash
(in the container)
# . /opt/ros/melodic/setup.bash
# rosbag play --clock PATH/TO/.bag
```

Then, you can see the segmentated image by rviz

## Demo
https://youtu.be/4PybRfNfNi8
