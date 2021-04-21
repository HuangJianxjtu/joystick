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
注意遥控器的模式(mode)， 否则会导致错乱。

### 开关量控制
>* 飞盘转动：长按'LB'， 再按'X'
>* 飞盘停止：长按'LB'， 再按'B'
>* 球框上升：长按'LB'， 再按'Y'
>* 球框下降：长按'LB'， 再按'A'
>* 球框停止：长按'LB'， 再按'RB'

### 底盘运动控制
>* 高速按键控制模式：长按'Y'，用方向按键来控制机器人
>* 低速按键控制模式：长按'A'，用方向按键来控制机器人
>* 高速摇杆控制模式：长按'X'，用左摇杆来控制机器人
>* 低速摇杆控制模式：长按'B'，用左摇杆来控制机器人

## Tips

>* 检查手柄的连接, `ls /dev/input/`;查看是否有js0,有则说明已经连接上了
>* 测试手柄信号，`sudo jstest /dev/input/js0`
