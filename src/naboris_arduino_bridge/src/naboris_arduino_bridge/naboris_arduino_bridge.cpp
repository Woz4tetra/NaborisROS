#include <naboris_arduino_bridge/naboris_arduino_bridge.h>

#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string)  string_to_int64(string)

const string NaborisArduinoBridge::IMU_FRAME_ID = "Imu";
const string NaborisArduinoBridge::CHILD_FRAME_ID = "base_link";

const string NaborisArduinoBridge::NODE_NAME = "naboris_arduino_bridge";
const string NaborisArduinoBridge::PACKET_END = "\n";

const string NaborisArduinoBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string NaborisArduinoBridge::READY_MESSAGE = "ready!" + PACKET_END;
const string NaborisArduinoBridge::START_COMMAND = "g" + PACKET_END;
const string NaborisArduinoBridge::STOP_COMMAND = "s" + PACKET_END;
const string NaborisArduinoBridge::IMU_MESSAGE_HEADER = "imu";
const string NaborisArduinoBridge::MESSAGE_DELIMITER = "\t";

const string NaborisArduinoBridge::ENCODER_MESSAGE_HEADER = "e";

const size_t NaborisArduinoBridge::MOTOR_COMMAND_MESSAGE_LEN = 17;

long long string_to_int64(string s) {
    stringstream ss(s);
    long long integer = 0;
    ss >> integer;
    return integer;
}

NaborisArduinoBridge::NaborisArduinoBridge(ros::NodeHandle* nodehandle):
    nh(*nodehandle)
{
    ROS_INFO("Naboris Arduino bridge starting...");

    nh.param<string>("imu_pub_topic", imu_pub_topic, "/BNO055");
    nh.param<string>("right_enc_pub_topic", right_enc_pub_topic, "/right_encoder");
    nh.param<string>("left_enc_pub_topic", left_enc_pub_topic, "/left_encoder");
    nh.param<string>("motor_sub_topic", motor_sub_topic, "/motor_commands");
    nh.param<string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param<int>("serial_baud", serial_baud, 115200);

    ROS_INFO("imu_pub_topic: %s", imu_pub_topic.c_str());
    ROS_INFO("right_enc_pub_topic: %s", right_enc_pub_topic.c_str());
    ROS_INFO("left_enc_pub_topic: %s", left_enc_pub_topic.c_str());
    ROS_INFO("motor_sub_topic: %s", motor_sub_topic.c_str());
    ROS_INFO("serial_port: %s", serial_port.c_str());

    imu_pub = nh.advertise<sensor_msgs::Imu>(imu_pub_topic, 5);
    right_encoder_pub = nh.advertise<std_msgs::Int64>(right_enc_pub_topic, 5);
    left_encoder_pub = nh.advertise<std_msgs::Int64>(left_enc_pub_topic, 5);

    motor_command_sub = nh.subscribe(motor_sub_topic, 1, &NaborisArduinoBridge::motor_command_callback, this);

    euler_roll = 0.0;
    euler_pitch = 0.0;
    euler_yaw = 0.0;

    ROS_INFO("Naboris Arduino bridge init done");
}


bool NaborisArduinoBridge::waitForPacket(const string packet)
{
    ros::Time begin = ros::Time::now();
    ros::Duration timeout = ros::Duration(5.0);

    while ((ros::Time::now() - begin) < timeout)
    {
        if (serial_ref.available()) {
            serial_buffer = serial_ref.readline();
            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            if (serial_buffer.compare(packet) == 0) {
                ROS_INFO("Naboris sent %s!", packet.c_str());
                return true;
            }
        }
    }

    ROS_ERROR("Timeout reached. Serial buffer didn't contain '%s', buffer: %s", packet.c_str(), serial_buffer.c_str());
    return false;
}

void NaborisArduinoBridge::eulerToQuat(sensor_msgs::Imu &imu_msg, float roll, float pitch, float yaw)
{
    double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	imu_msg.orientation.w = cy * cr * cp + sy * sr * sp;
	imu_msg.orientation.x = cy * sr * cp - sy * cr * sp;
	imu_msg.orientation.y = cy * cr * sp + sy * sr * cp;
	imu_msg.orientation.z = sy * cr * cp - cy * sr * sp;
}

int NaborisArduinoBridge::run()
{
    try
    {
        serial_ref.setPort(serial_port);
        serial_ref.setBaudrate(serial_baud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_ref.setTimeout(timeout);
        serial_ref.open();
    }
    catch (serial::IOException e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        ROS_ERROR_STREAM(serial_port);
        return -1;
    }

    if (!waitForPacket(HELLO_MESSAGE)) {
        return 1;
    }
    if (!waitForPacket(READY_MESSAGE)) {
        return 1;
    }

    serial_ref.write(START_COMMAND);

    ros::Rate clock_rate(60);  // 60 Hz

    while (ros::ok())
    {
        ros::spinOnce();
        clock_rate.sleep();

        if (serial_ref.available())
        {
            serial_buffer = serial_ref.readline();

            if (serial_buffer.at(0) == '-') {
                ROS_WARN("message: %s", serial_buffer.substr(1).c_str());
                continue;
            }

            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            // Parse IMU segment
            if (serial_buffer.length() > IMU_MESSAGE_HEADER.size() &&
                serial_buffer.compare(0, IMU_MESSAGE_HEADER.size(), IMU_MESSAGE_HEADER) == 0) {
                parseImuMessage();
            }
            else if (serial_buffer.length() > ENCODER_MESSAGE_HEADER.size() &&
                serial_buffer.compare(0, ENCODER_MESSAGE_HEADER.size(), ENCODER_MESSAGE_HEADER) == 0) {
                parseEncoderMessage();
            }
        }
    }

    serial_ref.write(STOP_COMMAND);

    return 0;
}

void NaborisArduinoBridge::parseImuMessage()
{
    // strip off header and the trailing newline character
    serial_buffer = serial_buffer.substr(IMU_MESSAGE_HEADER.size() + 1, serial_buffer.size()-1);

    imu_msg.header.frame_id = IMU_FRAME_ID;
    imu_msg.header.stamp = ros::Time::now();

    size_t pos = 0;
    string token;
    while ((pos = serial_buffer.find(MESSAGE_DELIMITER)) != string::npos)
    {
        token = serial_buffer.substr(0, pos);
        if (token.size() == 0) {
            break;
        }

        switch (token.at(0)) {
            case 't': ROS_DEBUG("imu arduino time: %s", token.substr(1).c_str()); break;
            case 'e':
                switch (token.at(1)) {
                    case 'z': euler_roll = M_PI / 180.0 * STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': euler_pitch = M_PI / 180.0 * STR_TO_FLOAT(token.substr(2)); break;
                    case 'x': euler_yaw = M_PI / 180.0 * STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            // case 'a':
            //
            //     break;
            case 'g':
                switch (token.at(1)) {
                    case 'x': imu_msg.angular_velocity.x = STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': imu_msg.angular_velocity.y = STR_TO_FLOAT(token.substr(2)); break;
                    case 'z': imu_msg.angular_velocity.z = STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            // case 'm':
            //
            //     break;
            case 'l':
                switch (token.at(1)) {
                    case 'x': imu_msg.linear_acceleration.x = STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': imu_msg.linear_acceleration.y = STR_TO_FLOAT(token.substr(2)); break;
                    case 'z': imu_msg.linear_acceleration.z = STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            case 'q':
                switch (token.at(1)) {
                    case 'w': imu_msg.orientation.w = STR_TO_FLOAT(token.substr(2)); break;
                    case 'x': imu_msg.orientation.x = STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': imu_msg.orientation.y = STR_TO_FLOAT(token.substr(2)); break;
                    case 'z': imu_msg.orientation.z = STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            case 's':
                switch (token.at(1)) {
                    case 's': ROS_DEBUG("system status: %s", token.substr(2).c_str()); break;
                    case 'g': ROS_DEBUG("gyro status: %s", token.substr(2).c_str()); break;
                    case 'a': ROS_DEBUG("accel status: %s", token.substr(2).c_str()); break;
                    case 'm': ROS_DEBUG("mag status: %s", token.substr(2).c_str()); break;
                }
                break;
            default:
                ROS_WARN("Invalid segment type! Segment: '%s', packet: '%s'", token.c_str(), serial_buffer.c_str());
                break;
        }

        serial_buffer.erase(0, pos + MESSAGE_DELIMITER.length());
    }

    eulerToQuat(imu_msg, euler_roll, euler_pitch, euler_yaw);

    imu_pub.publish(imu_msg);
}

void NaborisArduinoBridge::parseEncoderMessage()
{
    serial_buffer = serial_buffer.substr(ENCODER_MESSAGE_HEADER.size(), serial_buffer.size() - 1);
    ROS_DEBUG("buffer: %s", serial_buffer.c_str());

    size_t time_index = serial_buffer.find(MESSAGE_DELIMITER);
    ROS_DEBUG("encoder arduino time: %s", serial_buffer.substr(1, time_index).c_str());

    switch (serial_buffer.at(0)) {
        case 'r':
            right_encoder_msg.data = -STR_TO_INT(serial_buffer.substr(time_index + 1));
            ROS_DEBUG("right_encoder_msg: %lli", right_encoder_msg.data);

            right_encoder_pub.publish(right_encoder_msg);
            break;
        case 'l':
            left_encoder_msg.data = STR_TO_INT(serial_buffer.substr(time_index + 1));
            ROS_DEBUG("left_encoder_msg: %lli", left_encoder_msg.data);

            left_encoder_pub.publish(left_encoder_msg);
            break;
        default:
            ROS_WARN("Invalid segment type! Segment: '%c', packet: '%s'", serial_buffer.at(0), serial_buffer.c_str());
            break;
    }
}


void NaborisArduinoBridge::motor_command_callback(const std_msgs::Int16MultiArray& motor_commands)
{
    if (motor_commands.layout.dim[0].stride == 4)
    {
        char buffer[MOTOR_COMMAND_MESSAGE_LEN];
        sprintf(buffer, "d%04d%04d%04d%04d\n",
            motor_commands.data[0], motor_commands.data[1], motor_commands.data[2], motor_commands.data[3]
        );

        ROS_DEBUG("writing %s", buffer);
        serial_ref.write(buffer);
    }
    else {
        ROS_WARN("Invalid array size");
    }
}
