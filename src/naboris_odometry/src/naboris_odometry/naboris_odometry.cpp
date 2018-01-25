#include <naboris_odometry/naboris_odometry.h>

const string NaborisOdometry::CHILD_FRAME_ID = "base_link";
const string NaborisOdometry::NODE_NAME = "naboris_odometry";

NaborisOdometry::NaborisOdometry(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 5);

    if (!nh.getParam(NODE_NAME + "/imu_topic", imu_topic))
    {
        ROS_INFO_STREAM("IMU topic parameter not found, using default");
        imu_topic = "BNO055";
    }

    if (!nh.getParam(NODE_NAME + "/right_encoder_topic", right_encoder_topic))
    {
        ROS_INFO_STREAM("Right encoder topic parameter not found, using default");
        left_encoder_topic = "right_encoder";
    }

    if (!nh.getParam(NODE_NAME + "/left_encoder_topic", left_encoder_topic))
    {
        ROS_INFO_STREAM("Left encoder topic parameter not found, using default");
        left_encoder_topic = "left_encoder";
    }

    if (!nh.getParam(NODE_NAME + "/counts_per_revolution", counts_per_revolution))
    {
        ROS_INFO_STREAM("Counts per revolution parameter not found, using default (12)");
        counts_per_revolution = 12;
    }

    if (!nh.getParam(NODE_NAME + "/gear_ratio", gear_ratio))
    {
        ROS_INFO_STREAM("Gear ratio parameter not found, using default (150.58)");
        gear_ratio = 150.58;
    }

    if (!nh.getParam(NODE_NAME + "/wheel_radius_mm", wheel_radius_mm))
    {
        ROS_INFO_STREAM("Wheel ratio parameter not found, using default (27.0 mm)");
        gear_ratio = 27.0;
    }

    if (!nh.getParam(NODE_NAME + "/dist_between_wheels_mm", dist_between_wheels_mm))
    {
        ROS_INFO_STREAM("Dist between wheels parameter not found, using default (109.0 mm)");
        dist_between_wheels_mm = 109.0;
    }

    if (!nh.getParam(NODE_NAME + "/dist_between_axles_mm", dist_between_axles_mm))
    {
        ROS_INFO_STREAM("Dist between axles parameter not found, using default (143.0 mm)");
        dist_between_axles_mm = 143.0;
    }

    ticks_to_mm = wheel_radius_mm * 2 * M_PI / (gear_ratio * counts_per_revolution);

    imu_sub = nh.subscribe(imu_topic, 1, &NaborisOdometry::imu_callback, this);
    right_encoder_sub = nh.subscribe(right_encoder_topic, 1, &NaborisOdometry::right_enc_callback, this);
    left_encoder_sub = nh.subscribe(left_encoder_topic, 1, &NaborisOdometry::left_enc_callback, this);

    odom_msg.child_frame_id = CHILD_FRAME_ID;
}

void NaborisOdometry::imu_callback(const sensor_msgs::Imu& imu_msg)
{
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.quaternion = imu_msg.orientation;
    odom_msg.twist.angular = imu_msg.angular_velocity;

    odom_pub.publish(odom_msg);
}

void NaborisOdometry::right_enc_callback(const std_msgs::Int64& imu_msg)
{
    right_updated = true;
    right_tick = imu_msg.data;
    update_odom_message_enc();
}

void NaborisOdometry::left_enc_callback(const std_msgs::Int64& imu_msg)
{
    left_updated = true;
    left_tick = imu_msg.data;
    update_odom_message_enc();
}

void NaborisOdometry::update_odom_message_enc()
{
    if (right_updated && left_updated)
    {
        long long delta_right = right_tick - prev_right_tick;
        long long delta_left = left_tick - prev_left_tick;

        prev_right_tick = right_tick;
        prev_left_tick = left_tick;

        double delta_dist = ticks_to_mm * (delta_right + delta_left) / 2;

        right_updated = false;
        left_updated = false;

        tf::Matrix3x3 rotation_mat(odom_msg.pose.quaternion);
        std_msgs::Vector3 dist_vector(delta_dist, 0.0, 0.0);

        tf::Matrix3x3 rotated_dist = rotation_mat * dist_vector;

        double delta_x = rotated_dist[0][2];
        double delta_y = rotated_dist[1][2];
        double delta_z = rotated_dist[2][2];

        odom_msg.header.stamp = ros::Time::now();
        odom_msg.pose.pose.position.x += delta_x;
        odom_msg.pose.pose.position.y += delta_y;
        odom_msg.pose.pose.position.z += delta_z;

        ros::Time dt = odom_msg.header.stamp - prev_enc_time;
        prev_enc_time = odom_msg.header.stamp;

        std_msgs::Vector3 linear_vel(delta_x / dt.toSec(), delta_y / dt.toSec(), delta_z / dt.toSec());
        odom_msg.twist.linear = linear_vel;

        odom_pub.publish(odom_msg);
    }
}
