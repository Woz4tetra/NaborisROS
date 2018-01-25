#ifndef _NABORIS_ODOMETRY_H_
#define _NABORIS_ODOMETRY_H_

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include <tf/transform_datatypes.h>
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

class NaborisOdometry {
private:
    ros::NodeHandle nh;

    ros::Publisher odom_pub;

    ros::Subscriber imu_sub;
    ros::Subscriber left_encoder_sub;
    ros::Subscriber right_encoder_sub;

    nav_msgs::Odometry odom_msg;

    string imu_topic;
    string right_encoder_topic;
    string left_encoder_topic;

    void imu_callback(const sensor_msgs::Imu& imu_msg);
    void right_enc_callback(const std_msgs::Int64& encoder_msg);
    void left_enc_callback(const std_msgs::Int64& encoder_msg);

    bool right_updated;
    bool left_updated;

    long long left_tick;
    long long right_tick;

    long long prev_left_tick;
    long long prev_right_tick;

    double position_x;
    double position_y;
    double position_z;

    ros::Duration velocity_timeout;
    ros::Time prev_enc_time;

    double counts_per_revolution;
    double gear_ratio;
    double wheel_radius_mm;
    double ticks_to_mm;
    double dist_between_wheels_mm;
    double dist_between_axles_mm;

    void update_odom_message_enc();

public:
    NaborisOdometry(ros::NodeHandle* nodehandle);

    static const string CHILD_FRAME_ID;
    static const string NODE_NAME;
};

struct Error : exception
{
    char text[1000];

    Error(char const* fmt, ...) __attribute__((format(printf,2,3))) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(text, sizeof text, fmt, ap);
        va_end(ap);
    }

    char const* what() const throw() { return text; }
};

#endif // _NABORIS_ODOMETRY_H_
