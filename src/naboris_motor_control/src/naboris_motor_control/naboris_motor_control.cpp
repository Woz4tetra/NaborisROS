#include <naboris_motor_control/naboris_motor_control.h>

const string NaborisMotorControl::NODE_NAME = "naboris_motor_control";

NaborisMotorControl::NaborisMotorControl(ros::NodeHandle* nodehandle):nh(*nodehandle)
{

    if (!nh.getParam(NODE_NAME + "/motor_command_topic", motor_command_topic))
    {
        ROS_INFO_STREAM("Motor command topic parameter not found, using default");
        motor_command_topic = "motor_commands";
    }

    if (!nh.getParam(NODE_NAME + "/cmd_vel_topic", cmd_vel_topic))
    {
        ROS_INFO_STREAM("Cmd vel topic parameter not found, using default");
        cmd_vel_topic = "cmd_vel";
    }

    if (!nh.getParam(NODE_NAME + "/odometry_topic", odometry_topic))
    {
        ROS_INFO_STREAM("Encoder delta topic parameter not found, using default");
        odometry_topic = "odometry";
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

    cmd_vel_sub = nh.subscribe(cmd_vel_topic, 1, &NaborisMotorControl::cmd_vel_callback, this);
    odometry_sub = nh.subscribe(odometry_topic, 1, &NaborisMotorControl::odometry_callback, this);
    motor_command_pub = nh.advertise<std_msgs::Int16MultiArray>(motor_command_topic, 5);

    motor_command_msg.layout.dim[0].size = 1;
    motor_command_msg.layout.dim[0].stride = 4;

    for (int i = 0; i < motor_command_msg.layout.dim[0].stride; i++) {
		motor_command_msg.data.push_back(0);
	}
}

void NaborisMotorControl::cmd_vel_callback(const geometry_msgs::Twist& twist_msg)
{
    twist_set_point = geometry_msgs::Twist(twist_msg);

    // broadcast set point to pid
}

// callback for PID to call command_motors

void NaborisMotorControl::command_motors(double linear_x, double linear_y, double angular_z)  // arbitrary units determined by the pid controller
{
    double tangential_vel = angular_z * (dist_between_wheels_mm / 2);

    double x_speed = linear_x / wheel_radius_mm;
    double y_speed = linear_y / wheel_radius_mm;
    double tan_speed = tangential_vel / wheel_radius_mm;

    set_motor_command(
        x_speed - y_speed - tan_speed,
        x_speed + y_speed + tan_speed,
        x_speed + y_speed - tan_speed,
        x_speed - y_speed + tan_speed
    );

    motor_command_pub.publish(motor_command_msg);
}
/*
void NaborisMotorControl::command_motors(int speed, int angle, int angular)
{
    int fraction_speed = 0;
    if (0 <= angle && angle < 90)
    {
        fraction_speed = -2 * speed / 90 * angle + speed;
        set_motor_command(speed + angular, fraction_speed - angular, fraction_speed + angular, speed - angular);
    }

    else if (90 <= angle && angle < 180)
    {
        fraction_speed = -2 * speed / 90 * (angle - 90) + speed;
        set_motor_command(fraction_speed + angular, -speed - angular, -speed + angular, fraction_speed - angular);
    }

    else if (180 <= angle && angle < 270)
    {
        fraction_speed = 2 * speed / 90 * (angle - 180) - speed;
        set_motor_command(-speed + angular, fraction_speed - angular, fraction_speed + angular, -speed - angular);
    }

    else if (270 <= angle && angle < 360)
    {
        fraction_speed = 2 * speed / 90 * (angle - 270) - speed;
        set_motor_command(fraction_speed + angular, speed - angular, speed + angular, fraction_speed - angular);
    }

    motor_command_pub.publish(motor_command_msg);
}
*/

void NaborisMotorControl::set_motor_command(int m1, int m2, int m3, int m4)
{
    motor_command_msg.data[0] = m1 % 256;
    motor_command_msg.data[1] = m2 % 256;
    motor_command_msg.data[2] = m3 % 256;
    motor_command_msg.data[3] = m4 % 256;
}

void NaborisMotorControl::odometry_callback(const nav_msgs::Odometry& odometry_msg)
{
    // broadcast sensor measurement to PID
}
