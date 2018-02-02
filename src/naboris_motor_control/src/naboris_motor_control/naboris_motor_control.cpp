#include <naboris_motor_control/naboris_motor_control.h>

const string NaborisMotorControl::NODE_NAME = "naboris_motor_control";
const string NaborisMotorControl::ROBOT_INFO = "naboris_info";
const double NaborisMotorControl::MAX_JOYSTICK_VALUE = 32767.0;

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

NaborisMotorControl::NaborisMotorControl(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    nh.param<string>("motor_command_topic", motor_command_topic, "/motor_commands");
    nh.param<string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    nh.param<string>("joystick_topic", joystick_topic, "/joy");

    nh.param<int>("joystick_strafe_axis_x", joystick_strafe_axis_x, 0);
    nh.param<int>("joystick_strafe_axis_y", joystick_strafe_axis_y, 1);
    nh.param<int>("joystick_rotate_axis", joystick_rotate_axis, 2);

    nh.param<double>("/max_rpm", max_rpm, 200.0);
    nh.param<double>("/dist_between_wheels_mm", dist_between_wheels_mm, 109.0);
    nh.param<double>("/dist_between_axles_mm", dist_between_axles_mm, 143.0);
    nh.param<double>("/wheel_radius_mm", wheel_radius_mm, 27.0);

    ROS_INFO("motor_command_topic: %s", motor_command_topic.c_str());
    ROS_INFO("cmd_vel_topic: %s", cmd_vel_topic.c_str());
    ROS_INFO("joystick_topic: %s", joystick_topic.c_str());

    ROS_INFO("max_rpm: %f", max_rpm);
    ROS_INFO("dist_between_wheels_mm: %f", dist_between_wheels_mm);
    ROS_INFO("dist_between_axles_mm: %f", dist_between_axles_mm);
    ROS_INFO("wheel_radius_mm: %f", wheel_radius_mm);

    max_rps = max_rpm / 60;

    cmd_vel_sub = nh.subscribe(cmd_vel_topic, 1, &NaborisMotorControl::cmd_vel_callback, this);
    joystick_sub = nh.subscribe(joystick_topic, 1, &NaborisMotorControl::joystick_callback, this);
    motor_command_pub = nh.advertise<std_msgs::Int16MultiArray>(motor_command_topic, 5);

    motor_command_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    motor_command_msg.layout.dim[0].size = 1;
    motor_command_msg.layout.dim[0].stride = 4;

    for (int i = 0; i < motor_command_msg.layout.dim[0].stride; i++) {
        motor_command_msg.data.push_back(0);
	}
}

void NaborisMotorControl::joystick_callback(const sensor_msgs::Joy& joy_msg)
{
    double linear_x = joy_msg.axes[joystick_strafe_axis_x] * max_rps;
    double linear_y = joy_msg.axes[joystick_strafe_axis_y] * max_rps;
    double angular_z = joy_msg.axes[joystick_rotate_axis] * max_rps;

    command_motors(linear_x, linear_y, angular_z);
}

void NaborisMotorControl::cmd_vel_callback(const geometry_msgs::Twist& twist_msg) {
    command_motors(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
}

// callback for PID to call command_motors

void NaborisMotorControl::command_motors(double linear_x, double linear_y, double angular_z)  // x = rps, y = rps, z = rps
{
    // double tangential_vel = angular_z * (dist_between_wheels_mm / 2);
    //
    // // convert to rotations per second
    // double x_rps = linear_x / wheel_radius_mm;  // arc (mm/s) / radius (mm) = rotations per second
    // double y_rps = linear_y / wheel_radius_mm;
    // double tan_rps = tangential_vel / wheel_radius_mm;

    // somewhat arbitrarily convert desired rps to bytes. This is an open loop segment of a closed loop system.
    int y_speed = (int)(linear_x * 255.0 / max_rps);
    int x_speed = (int)(linear_y * 255.0 / max_rps);
    int tan_speed = (int)(-angular_z * 255.0 / max_rps);

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
    motor_command_msg.data[0] = clip(-m1, -255, 255);
    motor_command_msg.data[1] = clip(-m2, -255, 255);
    motor_command_msg.data[2] = clip(-m3, -255, 255);
    motor_command_msg.data[3] = clip(-m4, -255, 255);
}
