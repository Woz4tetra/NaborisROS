
#include "naboris_odometry/naboris_odometry.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NaborisOdometry::NODE_NAME);
    ros::NodeHandle nh("~");

    NaborisOdometry broadcaster(&nh);
    ros::spin();
    return 0;
}
