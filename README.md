# Gesture based controller

This package collects ROS nodes that map IMU data to velocity commands for mobile robots.

## Installation

Clone the repository in the source folder of your catkin workspace.

    git clone https://github.com/EmaroLab/gesture-based-controller.git

## Dependencies

In order to succesfully run the code, you should have installed [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Nodes description

<strong>gb_controller</strong> : It performs a linear mapping of accelerations recorded by an IMU sensor to the velocity commands for a mobile robot.

The mapping coefficients depend on the robot to control and they can be set from launch file.


    twist.linear.x = x * linear_coefficient
    twist.angular.z = y * angular_coefficient

## Documentation

[emarolab.github.io/gesture-based-controller/](https://emarolab.github.io/gesture-based-controller/)

## Author

[Alessandro Carfì](https://github.com/ACarfi) e-mail: alessandro.carfi@dibris.unige.it

This work derived from the [original work](https://github.com/enriquecoronadozu/wearable_control_hrp) of [Enrique Coronado](https://github.com/enriquecoronadozu)
