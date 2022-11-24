# About  #
This repo is a platform for testing the Obstacle avoidance alogirthm on the drone. This work uses sjtu_drone as a simulation platform.
sjtu_drone is a quadrotor simulation program forked from ['tum_simulator'] (http://wiki.ros.org/tum_simulator) , which is developed with ROS + Gazebo.

# Requirements #
This package is compatible with ROS Melodic version (Ubuntu 18.04).
Please refer the following for the ['installation'](http://wiki.ros.org/Installation/Ubuntu) of ROS on your computer. Prefreable is ROS melodic with ubuntu 18.04 or ROS Noetic with Ubuntu 20.04.

# Download and Compiling this package #
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Avi241/Obstacle_Avoidance.git
$ cd ~/catkin_ws
$ catkin build
```

# Run
The simplest way is calling after you have built the workspace successfully.

```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch sjtu_drone simple.launch
```
# Running with keyboard
In second terminal:

```
$ rosrun sjtu_drone drone_keyboard
```
# Steps 

Press Z to takeoff.   Now you can control your drone with keyboars as shown in the Gui


# For obstacle avoidance

Go to location ~/catkin_ws/src/Obstacle_Avoidance/sjtu-drone/scripts . Open the file ```obstacle_avoidance.py ``` now you can write your obstacle avoidane algorithms here in the main function of this code.\\
<b>Note</b> : If your are using ROS Noetic change the 1st line of the code 

```
#!/usr/bin/env python
```
 to 
 ```
 #!/usr/bin/env python3
```


# To run obstacle avoidance program

```
rosrun sjtu_drone obstacle_avoidance.py

```