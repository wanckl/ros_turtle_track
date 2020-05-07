# ROS Turtle track
---
### A beginner level example of turtlesim, include ros service call and topic publish and subscrib.

### Requrments: 
1. Ubuntu 16.04 LTS
2. ros-kinetic-desktop-full
3. turtlesim (installed by default)

This demo is mainly a target track founction, use default turtlesim pkg. 
first step is calling inner service of turtlesim node to spawn a new turtle named turtle2, so there is now two turtles in the playgrund. 
then subscrib the posetion of two turtle to calculate where and how long the turtle1 is located from turtle2. 
finally publish a turtle2/cmd_val include abouve info and loop again to have a realtime situation. 

Usage:
1. Start roscore first
2. Start turtlesim node
3. If you compile the pakage and add refresh your terminal source, you can find rosrun <your_pkg_name> service_spawn, enter
4. As same, find rosrun <your_pkg_name> track_path, enter
5. Use "rosrun turtlesim turtle_teleop_key" to move your turtle1 anywhere, have fun!

NOTICE: track_path while calculate only when two turtles are exist and alive.

![](https://github.com/wanckl/ros_turtle_track/blob/master/FlameShot_2020-05-07_16-49-37.png?raw=true)

It is only a learning demo at beginner level, over. 
05-07-2020, Wenthday. 
Email:wanckl@foxmail.com 
