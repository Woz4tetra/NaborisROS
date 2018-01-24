#include <naboris_arduino_bridge/naboris_arduino_bridge.h>

#define STR_TO_FLOAT(string) strtof((string).c_str(), 0)


const std::string NaborisArduinoBridge::NODE_NAME = "naboris_arduino_bridge";
const std::string NaborisArduinoBridge::PACKET_END = "\n";

const std::string NaborisArduinoBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const std::string NaborisArduinoBridge::READY_MESSAGE = "ready!" + PACKET_END;
const std::string NaborisArduinoBridge::START_COMMAND = "g" + PACKET_END;
const std::string NaborisArduinoBridge::STOP_COMMAND = "s" + PACKET_END;
const std::string NaborisArduinoBridge::IMU_MESSAGE_HEADER = "imu";
const std::string NaborisArduinoBridge::IMU_MESSAGE_DELIMITER = "\t";


NaborisArduinoBridge::NaborisArduinoBridge()
{
    imu_pub = nh.advertise<sensor_msgs::Imu>("BNO055", 5);

    if (!nh.getParam(NODE_NAME + "/serial_port", serial_port))
    {
        ROS_INFO_STREAM("Serial Port Parameter not found, using default");
        serial_port = "/dev/ttyUSB0";
    }
    if (!nh.getParam(NODE_NAME + "/serial_baud", serial_baud))
    {
        ROS_INFO_STREAM("Serial Baud parameter not found, using default");
        serial_baud = 115200;
    }
}


void NaborisArduinoBridge::waitForPacket(const std::string packet)
{
    ros::Time begin = ros::Time::now();
    ros::Duration timeout = ros::Duration(5.0);

    while ((ros::Time::now() - begin) < timeout)
    {
        if (serial_ref.available()) {
            serial_buffer = serial_ref.readline();
            ROS_INFO("buffer: %s", serial_buffer.c_str());

            if (serial_buffer.compare(packet) == 0) {
                ROS_INFO("Naboris sent %s!", packet.c_str());
                return;
            }
        }
    }

    throw Error("Timeout reached. Serial buffer didn't contain '%s', buffer: %s", packet.c_str(), serial_buffer.c_str());
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

    waitForPacket(HELLO_MESSAGE);
    waitForPacket(READY_MESSAGE);

    serial_ref.write(START_COMMAND);

    sensor_msgs::Imu imu_msg;

    while (ros::ok())
    {
        if (serial_ref.available())
        {
            serial_buffer = serial_ref.readline();
            // serial_buffer += serial_ref.read(serial_ref.available());
            if (serial_buffer.at(0) == '-') {
                ROS_WARN("message: %s", serial_buffer.substr(1).c_str());
                continue;
            }

            ROS_INFO("buffer: %s", serial_buffer.c_str());
            if (serial_buffer.length() > 3 && serial_buffer.compare(0, 3, IMU_MESSAGE_HEADER) == 0)
            {
                serial_buffer = serial_buffer.substr(IMU_MESSAGE_HEADER.size() + 1, serial_buffer.size()-1);

                imu_msg.header.frame_id = "Imu";
                imu_msg.header.stamp = ros::Time::now();

                size_t pos = 0;
                std::string token;
                while ((pos = serial_buffer.find(IMU_MESSAGE_DELIMITER)) != std::string::npos) {
                    token = serial_buffer.substr(0, pos);
                    if (token.size() == 0) {
                        break;
                    }

                    switch (token.at(0)) {
                        case 't': ROS_DEBUG("arduino time: %s", token.substr(1).c_str()); break;
                        // case 'e':
                        //     break;
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

                    serial_buffer.erase(0, pos + IMU_MESSAGE_DELIMITER.length());
                }

                imu_pub.publish(imu_msg);
            }
        }

        // send commands on the queue
    }

    serial_ref.write(STOP_COMMAND);

    return 0;
}
