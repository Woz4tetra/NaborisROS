
#include "naboris_stereo/naboris_stereo.h"

const string NaborisStereo::NODE_NAME = "naboris_stereo";

NaborisStereo::NaborisStereo(ros::NodeHandle* nodehandle):
    nh(*nodehandle),
    img_transport(nh)
{
    std::string camera_info_url;
    std::string camera_name;

    ros::param::param<std::string>("right_cam_sub_topic", right_cam_sub_topic, "/naboris_ip_cam/image_raw");
    ros::param::param<std::string>("left_cam_sub_topic", left_cam_sub_topic, "/raspicam_node/image");

    ROS_INFO("right_cam_sub_topic: %s", right_cam_sub_topic.c_str());
    ROS_INFO("left_cam_sub_topic: %s", left_cam_sub_topic.c_str());

    if (!nh.getParam("/" + NODE_NAME + "/right_cam_pub_topic", right_cam_pub_topic))
    {
        ROS_INFO_STREAM("Right camera publish topic parameter not found, using default");
        right_cam_pub_topic = "/naboris_stereo/right/image_raw";
    }

    if (!nh.getParam("/" + NODE_NAME + "/left_cam_pub_topic", left_cam_pub_topic))
    {
        ROS_INFO_STREAM("Left camera publish topic parameter not found, using default");
        left_cam_pub_topic = "/naboris_stereo/left/image_raw";
    }

    right_img_ready = false;
    left_img_ready = false;

    right_image_sub = img_transport.subscribe(right_cam_sub_topic, 1, &NaborisStereo::right_image_callback, this);
    right_image_pub = img_transport.advertise(right_cam_pub_topic, 1);

    left_image_sub = img_transport.subscribe(
        left_cam_sub_topic, 1, &NaborisStereo::left_image_callback,
        this, image_transport::TransportHints("compressed")
    );
    left_image_pub = img_transport.advertise(left_cam_pub_topic, 1);
}

void NaborisStereo::right_image_callback(const sensor_msgs::ImageConstPtr& right_image_msg)
{
    right_img_ready = true;
    ROS_INFO("received right image");
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // cv::Mat converted_image;
    // cv::cvtColor(cv_ptr->image, converted_image, cv::COLOR_BGR2GRAY);

    sensor_msgs::ImagePtr right_image_msg_pub = cv_bridge::CvImage(
        right_image_msg->header, sensor_msgs::image_encodings::BGR8, cv_ptr->image //converted_image
    ).toImageMsg();

    if (right_img_ready && left_img_ready)
    {
        ROS_INFO("right_image_msg->header.stamp: %f, %d", right_image_msg->header.stamp.toSec(), right_image_msg->header.seq);
        right_image_pub.publish(right_image_msg_pub);
    }
}


void NaborisStereo::left_image_callback(const sensor_msgs::ImageConstPtr& left_image_msg)
{
    left_img_ready = true;
    ROS_INFO("received left image");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(left_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // cv::Mat converted_image = cv::imdecode(cv::Mat(cv_ptr->image), 1);
    cv::Mat converted_image = cv::Mat(cv_ptr->image);
    // cv::cvtColor(converted_image, converted_image, cv::COLOR_BGR2GRAY);

    sensor_msgs::ImagePtr left_image_msg_pub = cv_bridge::CvImage(
        left_image_msg->header, sensor_msgs::image_encodings::BGR8, converted_image
    ).toImageMsg();

    if (right_img_ready && left_img_ready)
    {
        ROS_INFO("left_image_msg->header.stamp: %f, %d", left_image_msg->header.stamp.toSec(), left_image_msg->header.seq);
        left_image_pub.publish(left_image_msg_pub);
    }
}
