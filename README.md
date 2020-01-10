# suii_3d_vision_ros

This repository contains the 3D vision soluction for object detection for RoboCup@work competition that RoboHub Eindhoven participates in.
The system works as follows. Two images are made, one RGB image to detect what objects are in front of the camera with YOLO and one depth image of the environment.
YOLO outputs ROI' s that are used to cut out certain regions in the depth image. Out of these regions the object is segmented and the TF is determined.
After the TF is determined it gets broadcasted with ROS. Diagrams of the software and the Design_Docs file can be found in the [Docs](https://github.com/JeroenBongers96/suii_3d_vision_ros/tree/master/docs) folder.
The Design_Doc file contains a more in depth explanation of the software.

## Test Results

The code has been tested and has been compared to the [2D_Camera_Accuracy_Test](https://github.com/RoboHubEindhoven-User/suii/wiki/Camera_accuracy_test-results) that was done by RoboHub.
All results and conclusions can be found in the [wiki](https://github.com/JeroenBongers96/suii_3d_vision_ros/wiki).

## Getting Started

For this project Ubuntu 18.04 LTS is used. It is optional to use the nVidia CUDA toolkit for faster YOLO inferencing.

### Prerequisites

The package runs on C++, Python2.7, and Python 3.6 and you need all of those to let the system work. It makes use of the following packages:

C++:
* [Intel RealSense SDK2.0](https://github.com/IntelRealSense/librealsense)
* [PCL](http://pointclouds.org/)
* [ROS Melodic Morenia](http://wiki.ros.org/melodic)
* [OpenCV](https://opencv.org/)

Python2:
* [ROS Melodic Morenia](http://wiki.ros.org/melodic)

Python3:
* [OpenCV](https://opencv.org/)
* numpy
* torch >= 1.1.0
* tqdm
* matplotlib
* pycocotools

### Installing

To install the suii_3d_vision_ros package in your catkin workspace, you will need to run the following lines:

```
cd catkin_ws/src
git clone https://github.com/JeroenBongers96/suii_3d_vision_ros.git
catkin_make
```

After installation of the package go to [yolo.py](https://github.com/JeroenBongers96/suii_3d_vision_ros/blob/master/yolo/yolo.py) and change the path of the config, data and weights files.

```
defaults_dict = {
    "cfg": "/home/user/catkin_ws/src/yolo_config/full_yolo.cfg", # Config file path
    "data": "/home/user/catkin_ws/src/yolo_config/full_yolo", # Data file path
    "weights": "/home/user/catkin_ws/src/yolo_config/full_yolo.backup", # Weights file path
    "conf_thres": 0.6, # Confidence threshold (accuracy)
    "nms_thres": 0.5,  # Non-maximum supression threshold (compression, lower value = more compression)
    "size": 416 # DO NOT CHANGE!!!
}
```

## Running the tests

If all installations are done, run the code with:

```
roscore
rosrun suii_3d_vision_ros yolo_main.py
rosrun suii_3d_vision_ros yolo_server.py 
rosrun main
```

## Authors

* **Jeroen Bongers** - *Initial work* - [JeroenBongers96](https://github.com/JeroenBongers96)
* **Mark Geraets** - *Initial work* - [markgrts](https://github.com/markgrts)

In name of [RoboHub Eindhoven](https://robohub-eindhoven.nl/)

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE.md](https://github.com/JeroenBongers96/suii_3d_vision_ros/blob/master/LICENSE) file for details
