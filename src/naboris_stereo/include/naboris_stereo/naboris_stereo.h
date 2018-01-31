
#ifndef _NABORIS_STEREO_H_
#define _NABORIS_STEREO_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class NaborisStereo {
private:
    ros::NodeHandle nh;

    string right_cam_sub_topic;
    string left_cam_sub_topic;
    string right_cam_pub_topic;
    string left_cam_pub_topic;

    image_transport::ImageTransport img_transport;

    image_transport::Subscriber right_image_sub;
    image_transport::Publisher right_image_pub;
    void right_image_callback(const sensor_msgs::ImageConstPtr& right_image_msg);

    image_transport::Subscriber left_image_sub;
    image_transport::Publisher left_image_pub;
    void left_image_callback(const sensor_msgs::ImageConstPtr& left_image_msg);

    bool right_img_ready;
    bool left_img_ready;
    
public:
    NaborisStereo(ros::NodeHandle* nodehandle);

    static const string NODE_NAME;
};


#endif  // _NABORIS_STEREO_H_
