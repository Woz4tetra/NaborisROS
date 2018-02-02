
#ifndef _NABORIS_STEREO_H_
#define _NABORIS_STEREO_H_

#include <ros/ros.h>
#include <vector>
#include <limits>

#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace message_filters;
using namespace sensor_msgs;
using namespace std;

#define ERASE_UPTO_MIN_INDEX(image_vector) image_vector.erase( \
    image_vector.begin(), \
    image_vector.begin() + (min_time_diff_index - 1) \
)

class NaborisStereo {
private:
    ros::NodeHandle nh;

    string right_cam_sub_topic;
    string left_cam_sub_topic;

    string right_cam_pub_topic;
    string left_cam_pub_topic;

    image_transport::CameraSubscriber right_image_sub;
    image_transport::CameraSubscriber left_image_sub;

    image_transport::CameraPublisher right_image_pub;
    image_transport::CameraPublisher left_image_pub;

    image_transport::ImageTransport img_transport;

    void right_image_callback(
        const sensor_msgs::ImageConstPtr& right_image_msg,
        const sensor_msgs::CameraInfoConstPtr& right_info_msg
    );
    void left_image_callback(
        const sensor_msgs::ImageConstPtr& left_image_msg,
        const sensor_msgs::CameraInfoConstPtr& left_info_msg
    );

    bool left_cam_ready;
    bool right_cam_in_sync;
    bool right_cam_in_sync_prev;
    double time_diff_sum;
    unsigned int time_diff_count;

    sensor_msgs::ImagePtr right_saved_image;
    double right_saved_timestamp;
    sensor_msgs::CameraInfoPtr right_saved_info;

    std::vector<sensor_msgs::ImagePtr> left_image_vector;
    std::vector<sensor_msgs::CameraInfoPtr> left_info_vector;
    std::vector<double> left_stamp_vector;


public:
    NaborisStereo(ros::NodeHandle* nodehandle);

    static const string NODE_NAME;
    static const double MIN_ALLOWED_TIME_DIFF;
};


#endif  // _NABORIS_STEREO_H_
