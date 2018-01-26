
#include "naboris_ip_cam/naboris_ip_cam.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NaborisIpCam::NODE_NAME);
    ros::NodeHandle nh;

    NaborisIpCam broadcaster(&nh);
    ros::spin();
    return 0;
}
