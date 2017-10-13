# lab_ros_aruco

A simple ROS wrapper for the [ArUco](https://www.uco.es/investiga/grupos/ava/node/26) Library.
It takes the rostopics `/image` and `/camera info` and output any detected markers in `\markers`. Please refer to the launch file for example.

### TODO
- Split the ROS Messages out into a seperate package.
- Output image with detected markers.
