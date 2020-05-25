#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>

using namespace std;
int highSpeed_button_flag = 0;
int lowSpeed_button_flag = 0;
int highSpeed_joystick_flag = 0;
int lowSpeed_joystick_flag = 0;
float forward_back_joystick = 0.0;
float right_left_joystick = 0.0;
float forward_back_button = 0.0;
float right_left_button = 0.0;

float scale_linear_high = 0.4;
float scale_angle_high = 0.8;
float scale_linear_low = 0.2;
float scale_angle_low = 0.4;

float alpha_low_pass = 0.7;
float speed_linear_past = 0.0;
float speed_turn_past = 0.0;

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
    highSpeed_button_flag = Joy->buttons[3];
    lowSpeed_button_flag = Joy->buttons[0];
    highSpeed_joystick_flag = Joy->buttons[2];
    lowSpeed_joystick_flag = Joy->buttons[1];

    forward_back_joystick = Joy->axes[1];
    right_left_joystick = Joy->axes[0];
    forward_back_button = Joy->axes[7];
    right_left_button = Joy->axes[6];
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "joystick");

    ros::NodeHandle n; //实例化节点
    ros::Subscriber sub;
    ros::Publisher pub;

    pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);//将速度发给/cmd_vel
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