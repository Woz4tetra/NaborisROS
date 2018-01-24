#ifndef _NABORIS_ARDUINO_BRIDGE_H_
#define _NABORIS_ARDUINO_BRIDGE_H_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"

#include <sstream>

class NaborisArduinoBridge {
private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;

    std::string serial_port;
    int serial_baud;
    std::string serial_buffer;
    serial::Serial serial_ref;

    void waitForPacket(const std::string);

public:
    NaborisArduinoBridge();

    static const std::string NODE_NAME;
    static const std::string PACKET_END;
    static const std::string HELLO_MESSAGE;
    static const std::string READY_MESSAGE;
    static const std::string START_COMMAND;
    static const std::string STOP_COMMAND;

    static const std::string IMU_MESSAGE_HEADER;
    static const std::string IMU_MESSAGE_DELIMITER;

    int run();
};

struct Error : std::exception
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

#endif // _NABORIS_ARDUINO_BRIDGE_H_
