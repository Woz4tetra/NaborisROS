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
        right_encoder_topic = "right_encoder";
    }

    if (!nh.getParam(NODE_NAME + "/left_encoder_topic", left_encoder_topic))
    {
        ROS_INFO_STREAM("Left encoder topic parameter not found, using default");
        left_encoder_topic = "left_encoder";
    }

    if (!nh.getParam(NODE_NAME + "/counts_per_revolution", counts_per_revolution))
    {
        ROS_INFO_STREAM("Counts per revolution parameter not found, using default (12)");
        counts_per_revolution = 12.0;
    }

    if (!nh.getParam(NODE_NAME + "/gear_ratio", gear_ratio))
    {
        ROS_INFO_STREAM("Gear ratio parameter not found, using default (150.58)");
        gear_ratio = 150.58;
    }

    if (!nh.getParam(NODE_NAME + "/wheel_radius_mm", wheel_radius_mm))
    {
        ROS_INFO_STREAM("Wheel ratio parameter not found, using default (27.0 mm)");
        wheel_radius_mm = 27.0;
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

    ticks_to_mm = wheel_radius_mm * 2.0 * M_PI / (gear_ratio * counts_per_revolution);
    ROS_INFO("ticks to mm: %f", ticks_to_mm);

    imu_sub = nh.subscribe(imu_topic, 1, &NaborisOdometry::imu_callback, this);
    right_encoder_sub = nh.subscribe(right_encoder_topic, 1, &NaborisOdometry::right_enc_callback, this);
    left_encoder_sub = nh.subscribe(left_encoder_topic, 1, &NaborisOdometry::left_enc_callback, this);

    odom_msg.child_frame_id = CHILD_FRAME_ID;

    right_updated = false;
    left_updated = false;
    left_tick = 0;
    right_tick = 0;
    prev_left_tick = 0;
    prev_right_tick = 0;

    position_x = 0.0;
    position_y = 0.0;
    position_z = 0.0;

    velocity_timeout = ros::Duration(0.15);
}

void NaborisOdometry::imu_callback(const sensor_msgs::Imu& imu_msg)
{
    ros::Duration dt = odom_msg.header.stamp - prev_enc_time;

    if (dt > velocity_timeout) {
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
    }
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.orientation = imu_msg.orientation;
    odom_msg.twist.twist.angular = imu_msg.angular_velocity;

    odom_pub.publish(odom_msg);
}

void NaborisOdometry::right_enc_callback(const std_msgs::Int64& encoder_msg)
{
    right_updated = true;
    right_tick = encoder_msg.data;
    update_odom_message_enc();
}

void NaborisOdometry::left_enc_callback(const std_msgs::Int64& encoder_msg)
{
    left_updated = true;
    left_tick = encoder_msg.data;
    update_odom_message_enc();
}

void NaborisOdometry::update_odom_message_enc()
{
    if (right_updated && left_updated)
    {
        right_updated = false;
        left_updated = false;

        long long delta_right = right_tick - prev_right_tick;
        long long delta_left = left_tick - prev_left_tick;

        ROS_DEBUG("-----------");
        ROS_DEBUG("d right: %lli, d left: %lli", delta_right, delta_left);

        prev_right_tick = right_tick;
        prev_left_tick = left_tick;

        double delta_dist = ticks_to_mm * (double)(delta_right + delta_left) / 2;

        ROS_DEBUG("delta_dist: %f", delta_dist);

        if (delta_dist == 0.0)
        {
            ROS_DEBUG("delta_dist too small");
            odom_msg.pose.pose.position.x = position_x;
            odom_msg.pose.pose.position.y = position_y;
            odom_msg.pose.pose.position.z = position_z;

            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
        }
        else
        {
            const tf::Quaternion q(
                odom_msg.pose.pose.orientation.w,
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z
            );
            const tf::Matrix3x3 rotation_mat(q);
            const tf::Vector3 dist_vector(delta_dist, 0.0, 0.0);

            tfScalar yaw, pitch, roll;
            rotation_mat.getEulerYPR(yaw, pitch, roll, 1);
            ROS_DEBUG("y: %f, p: %f, r: %f", yaw, pitch, roll);

            tf::Vector3 rotated_dist = rotation_mat * dist_vector;

            double delta_x = rotated_dist[0];
            double delta_y = rotated_dist[1];
            double delta_z = rotated_dist[2];

            ROS_DEBUG("dx: %f, dy: %f, dz: %f", delta_x, delta_y, delta_z);
            position_x += delta_x;
            position_y += delta_y;
            position_z += delta_z;
            ROS_DEBUG("x: %f, y: %f, z: %f", position_x, position_y, position_z);
            odom_msg.pose.pose.position.x = position_x;
            odom_msg.pose.pose.position.y = position_y;
            odom_msg.pose.pose.position.z = position_z;

            odom_msg.header.stamp = ros::Time::now();
            ros::Duration dt = odom_msg.header.stamp - prev_enc_time;
            prev_enc_time = odom_msg.header.stamp;

            odom_msg.twist.twist.linear.x = delta_x / dt.toSec();
            odom_msg.twist.twist.linear.y = delta_y / dt.toSec();
            odom_msg.twist.twist.linear.z = delta_z / dt.toSec();
        }


        odom_pub.publish(odom_msg);
    }
}
