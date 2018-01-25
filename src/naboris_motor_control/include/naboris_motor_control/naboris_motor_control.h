#ifndef _NABORIS_MOTOR_CONTROL_H_
#define _NABORIS_MOTOR_CONTROL_H_

#include "ros/ros.h"
#include "naboris_odometry/EncoderDelta.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "naboris_odometry/EncoderDelta.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class NaborisMotorControl {
private:
    ros::NodeHandle nh;

    ros::Publisher motor_command_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber odometry_sub;
    std_msgs::Int16MultiArray motor_command_msg;

    string motor_command_topic;
    string odometry_topic;
    string cmd_vel_topic;

    geometry_msgs::Twist twist_set_point;

    void set_motor_command(int m1, int m2, int m3, int m4);
    void command_motors(int speed, int angle, int angular);

    void cmd_vel_callback(const geometry_msgs::Twist& twist_msg);
    void odometry_callback(const nav_msgs::Odometry& odometry_msg);

public:
    NaborisMotorControl(ros::NodeHandle* nodehandle);

    static const string NODE_NAME;
};

#endif // _NABORIS_MOTOR_CONTROL_H_
