#ifndef _NABORIS_MOTOR_CONTROL_H_
#define _NABORIS_MOTOR_CONTROL_H_

#include "ros/ros.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class NaborisMotorControl {
private:
    ros::NodeHandle nh;

    ros::Publisher motor_command_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber joystick_sub;
    std_msgs::Int16MultiArray motor_command_msg;

    string motor_command_topic;
    string cmd_vel_topic;
    string joystick_topic;

    double max_rpm;
    double max_rps;
    double dist_between_wheels_mm;
    double dist_between_axles_mm;
    double wheel_radius_mm;

    int joystick_strafe_axis_x;
    int joystick_strafe_axis_y;
    int joystick_rotate_axis;

    void set_motor_command(int m1, int m2, int m3, int m4);
    void command_motors(double linear_x, double linear_y, double angular_z);

    void cmd_vel_callback(const geometry_msgs::Twist& twist_msg);
    void joystick_callback(const sensor_msgs::Joy& joy_msg);

public:
    NaborisMotorControl(ros::NodeHandle* nodehandle);

    static const string NODE_NAME;
    static const string ROBOT_INFO;
    static const double MAX_JOYSTICK_VALUE;
};

#endif // _NABORIS_MOTOR_CONTROL_H_
