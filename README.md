# Autonomous Vehicle (Closedloop)

## Description
This Autonomous Vehicle package is a ROS-based software package designed for path navigation. 
It includes modules for Closedloop control, and Path generation.

## Installation

### Dependencies
The following Python libraries are used:
- Numpy
- Rospy

Additionally the following ROS messages are used:
- std_msgs (Bool, Float32)
- geometry_msgs (Twist)
- closedloop_control (path) - Custom message


### Building
```bash
cd ~/catkin_ws/src
git clone https://github.com/OctavioRguez/Autonomous_Vehicle.git
git branch Closedloop
cd ..
mv ./Autonomous_Vehicle ./autonomous_vehicle
catkin_make
```

## Launch
For running the project, you can use the following command:
```bash
    roslaunch autonomous_vehicle control.launch
```

## License
This project is licensed under the BSD 3-Clause License. See the LICENSE file for details.
