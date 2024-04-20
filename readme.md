HRVO Package
================

## Workspace
------------
- Ubuntu 18.04
- ROS Melodic

## Getting dependencies
------------
```
cd ${ROS_WORKSPACE}/src
git clone https://github.com/srl-freiburg/animated_markers.git
```

## Building
------------
```
cd ${ROS_WORKSPACE}
catkin_make
```

## Running
------------
```
roslaunch hrvo animated_walking_man.launch
```

## Issues
------------
- Error1: "ERROR: cannot launch node of type [hrvo/animated_walking_man.py]: Cannot locate node of type [animated_walking_man.py] in package [hrvo]"
When running: ``` roslaunch hrvo animated_walking_man.launch ```

