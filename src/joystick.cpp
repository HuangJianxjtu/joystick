#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>

//以下为串口通讯需要的头文件
#include <string>        
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <serial/serial.h>

using namespace std;
int highSpeed_button_flag = 0;
int lowSpeed_button_flag = 0;
int highSpeed_joystick_flag = 0;
int lowSpeed_joystick_flag = 0;
float forward_back_joystick = 0.0;
float right_left_joystick = 0.0;
float forward_back_button = 0.0;
float right_left_button = 0.0;

serial::Serial raybot_serial3; //声明与下位机3通信的的串口对象

// for RE40 motor
float scale_linear_high = -0.60;
float scale_angle_high = -0.50;
float scale_linear_low = -0.40;
float scale_angle_low = -0.40;

float alpha_low_pass = 0.7;
float speed_linear_past = 0.0;
float speed_turn_past = 0.0;

unsigned char serial3_data[4];   //要发给串口3的数据

float scale_int(int i)
{
    if(i>10)
        return 1.0;
    else if(i < -10)
        return -1.0;
    else
        return 0.0;
}

void callback(const sensor_msgs::Joy::ConstPtr& Joy)
{
    if(Joy->buttons[4] == 1){ //向下位机3发送命令
        serial3_data[0] = 0x4F; //大写字母'O'
        serial3_data[1] = 0x44; //大写字母'D'
        serial3_data[3] = 0x52; //大写字母'R'
        if(Joy->buttons[2] == 1){ // 飞盘转动
            serial3_data[2] = 0x31; // 数字‘1’
        } else if(Joy->buttons[1] == 1){// 飞盘停止
            serial3_data[2] = 0x32; // 数字‘2’
        } else if(Joy->buttons[3] == 1){ //球框上升
            serial3_data[2] = 0x33; // 数字'3'
        } else if(Joy->buttons[0] == 1){ //球框下降
            serial3_data[2] = 0x34; //数字'4'
        } else if(Joy->buttons[5] == 1){  //球框停止运动
            serial3_data[2] = 0x35; //数字'5'
        } else{
            serial3_data[2] = 0x00; //错误指令
        }
        if(serial3_data[2] != 0x00){
            raybot_serial3.write(serial3_data, 4);
        }
    } else{
        highSpeed_button_flag = Joy->buttons[3];
        lowSpeed_button_flag = Joy->buttons[0];
        highSpeed_joystick_flag = Joy->buttons[2];
        lowSpeed_joystick_flag = Joy->buttons[1];
        forward_back_joystick = Joy->axes[1];
        right_left_joystick = Joy->axes[0];
        forward_back_button = Joy->axes[7];
        right_left_button = Joy->axes[6];
    }

}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "joystick");

    ros::NodeHandle n; //实例化节点
    ros::Subscriber sub;
    ros::Publisher pub;

    //初始化与电机通信的串口对象
    try
    {
        //设置串口属性，并打开串口
        raybot_serial3.setPort("/dev/raybot_serial3");
        raybot_serial3.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        raybot_serial3.setTimeout(to);
        raybot_serial3.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open raybot_serial3");
        return -1;
    }
    if(raybot_serial3.isOpen())
        ROS_INFO_STREAM("raybot_serial3 is opened.");
    else
        return -1;

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);//将速度发给/cmd_vel
    sub = n.subscribe<sensor_msgs::Joy>("/joy",1000,callback); //订阅游戏手柄发来的数据(/joy)
    
    ros::Rate loop_rate(20); //20Hz
    geometry_msgs::Twist v_cmd;
    
    while(ros::ok())
    {
        //以20Hz频率发布速度指令
        if(highSpeed_joystick_flag)  // 摇杆控制, high speed mode
        {
            v_cmd.linear.x = forward_back_joystick*scale_linear_high;
            v_cmd.angular.z = right_left_joystick*scale_angle_high;
        }
        else if(lowSpeed_joystick_flag)
        {
            v_cmd.linear.x = forward_back_joystick*scale_linear_low;
            v_cmd.angular.z = right_left_joystick*scale_angle_low;
        }
        else if(highSpeed_button_flag)
        {
            v_cmd.linear.x = forward_back_button*scale_linear_high;
            v_cmd.angular.z = right_left_button*scale_angle_high;
        }
        else if(lowSpeed_button_flag)
        {
            v_cmd.linear.x = forward_back_button*scale_linear_low;
            v_cmd.angular.z = right_left_button*scale_angle_low;
        }
        else
        {
            v_cmd.linear.x = 0.0;
            v_cmd.angular.z = 0.0;
        }
        // low-pass filter, to make robot move more smooth
        v_cmd.linear.x = (1-alpha_low_pass)*v_cmd.linear.x + alpha_low_pass*speed_linear_past;
        v_cmd.angular.z = (1-alpha_low_pass)*v_cmd.angular.z + alpha_low_pass*speed_turn_past;

        pub.publish(v_cmd);

        speed_linear_past = v_cmd.linear.x;
        speed_turn_past = v_cmd.angular.z;
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}