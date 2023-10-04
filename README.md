# yolo_ros_plugin
## Just a yolo_ros_plugin
This is a YOLO ROS Plugin through usage of OpenCV, which can support ```sensor_msgs::Image & sensor_msgs::CompressedImage``` datatype. Should be sufficient for the application requirement of AIRO-Lab members. Part of the code might not be very efficient, so suggestions or even pull requests are warmly welcomed :)


## Yolov4
1. Pre-requisites:
```
# ROS Melodic || Noetic
# OpenCV >= 4.4
# Python 3.8

### for GPU usage:
# CUDNN >= 7.0
```

2. Utilization:

 - Define your ros message topic at [here](/yolo_node/launch/CNN_SUBPUB_topics.yaml).
 - Define your config, weight, and name file at [here](/yolo_node//launch/yolov4.launch). When you clone this repo, default should have Yolov4-tiny.
 - In the same launch file, define your input type (under para: input_type obviously - duh). It's your choice whether you need depth/compressed or not.
 - In the same launch file, define whether or not you are using GPU. We plan to write another tutorial on setting up GPU environs -- but not today.
 - Launch! You can do ```rqt_image_view``` and check  topic ```/showoff_this_dope_image```.
 - If you wish to do some further shit, in the [yolo.cpp](/yolo_node/src/yolo.cpp), do something with the object ```obj_vector``` after detection.

## Yolov8
It's the year 2023, and Jesus mother of Christ, we have come all the way through Yolov8 since the first publication of You only look once. So a python version yolo-ros-plugin with v8 is also attached in this repo.

```
pip install ultralytics
```
and run
```
roslaunch yolo_node yolov8.launch
```

We did not draw the bounding boxes for you, neither saving those information in some random object. You might have to do something by yourself based on ```results = self.model(self.cv_image)[0]```.

## Maintainer
[pattylo](https://github.com/pattylo) @ AIRO-LAB @ RCUAS, HKPolyU

