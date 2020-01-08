# suii_3d_vision_ros

This repository contains the 3D vision soluction for object detection for RoboCup@work competition that RoboHub Eindhoven participates in.
The system works as follows. Two images are made, one RGB image to detect what objects are in front of the camera with YOLO and one depth image of the table.
YOLO outputs ROI' s that are used to cut out certain regions in the depth image. Out of these regions the object is segmented and the TF is determined.
After the TF is determined it gets broadcasted with ROS.

## Getting Started

For this project Ubuntu 18.04 LTS is used. It is optional to use the nVidia CUDA toolkit for faster YOLO inferencing.

### Prerequisites

The packege runs on C++, Python2.7, and Python 3.6 and you need all of those to let the system work. It makes use of the following packages:

C++:
* [Intel RealSense SDK2.0](https://github.com/IntelRealSense/librealsense)
* [PCL](http://pointclouds.org/)
* [ROS Melodic Morenia](http://wiki.ros.org/melodic)

Python2:
* [ROS Melodic Morenia](http://wiki.ros.org/melodic)

Python3:
* [OpenCV](https://opencv.org/)

### Installing

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

## Authors

* **Jeroen Bongers** - *Initial work* - [JeroenBongers96](https://github.com/JeroenBongers96)
* **Mark Geraets** - *Initial work* - [markgrts](https://github.com/markgrts/++)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc

