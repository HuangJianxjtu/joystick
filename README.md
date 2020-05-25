# Joystick Remote Control in ROS

This package is for joystick remote control in ROS, it subscribes the joystick information from topic "/joy" and publish the velocity command on the topic "/mobile_base/commands/velocity".  We using logitech joystick and other standard joysticks are also OK. 

## Prerequisite

Hardware: logitech joystick or other standard joysticks

environment: Ubuntu 16.04, ROS Kinetic

>* download this package and copy it to the src directory of your catkin workspace. then `catkin_make` and `source devel/setup.bash`
>* `sudo apt install ros-kinetic-joy`

## How to run?

`rosrun joy joy_node`

`rosrun joystick joystick`

## How to use?

>* High speed, button control mode

press 'Y' button and use orientation button to control robot

>* Low speed, button control mode

press 'A' button and use orientation button to control robot

>* High speed, joystick(摇杆) control mode

press 'X' button and use joystick to control robot

>* Low speed, joystick control mode

press 'B' button and use joystick to control robot

## Tips

>* 检查手柄的连接, `ls /dev/input/`;查看是否有js0,有则说明已经连接上了
>* 测试手柄信号，`sudo jstest /dev/input/js0`
