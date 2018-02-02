
#include "naboris_stereo/naboris_stereo.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NaborisStereo::NODE_NAME);
    ros::NodeHandle nh("~");
    NaborisStereo broadcaster(&nh);
    ros::spin();

    return 0;
}
