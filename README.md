# Autonomous Vehicle (VisionControl)

## Description
This Autonomous Vehicle package is a ROS-based software package designed for line following with computer vision. 
It includes modules for color filter for streetlights recognition, and line following control.

## Installation

### Dependencies
The following Python libraries are used:
- OpenCV
- CV_Bridge
- Numpy
- Rospy

Additionally the following ROS messages are used:
- std_msgs (String, Float32)
- sensor_msgs (Image)


### Building
```bash
cd ~/catkin_ws/src
git clone https://github.com/OctavioRguez/Autonomous_Vehicle.git
git checkout VisionControl
mv ./Autonomous_Vehicle ./autonomous_vehicle
cd ..
catkin_make
```

## Launch
For running the project, you can use the following command:
```bash
    roslaunch autonomous_vehicle imgProcess.launch
```

## License
This project is licensed under the BSD 3-Clause License. See the LICENSE file for details.
