
#include "naboris_motor_control/naboris_motor_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NaborisMotorControl::NODE_NAME);
    ros::NodeHandle nh("~");

    NaborisMotorControl broadcaster(&nh);
    ros::spin();
    return 0;
}
