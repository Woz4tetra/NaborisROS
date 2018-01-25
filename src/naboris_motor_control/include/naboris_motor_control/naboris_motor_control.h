#ifndef _NABORIS_MOTOR_CONTROL_H_
#define _NABORIS_MOTOR_CONTROL_H_

#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class NaborisMotorControl {
private:
    ros::NodeHandle nh;

    ros::Publisher motor_command_pub;
    ros::Subscriber cmd_vel_sub;
    std_msgs::Int16MultiArray motor_command_msg;

    string motor_command_topic;
    string cmd_vel_topic;

    double max_rpm;
    double max_rps;
    double dist_between_wheels_mm;
    double dist_between_axles_mm;
    double wheel_radius_mm;

    void set_motor_command(int m1, int m2, int m3, int m4);
    void command_motors(double linear_x, double linear_y, double angular_z);

    void cmd_vel_callback(const geometry_msgs::Twist& twist_msg);

public:
    NaborisMotorControl(ros::NodeHandle* nodehandle);

    static const string NODE_NAME;
    static const string ROBOT_INFO;
};

#endif // _NABORIS_MOTOR_CONTROL_H_
