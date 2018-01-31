#ifndef _NABORIS_IP_CAM_H_
#define _NABORIS_IP_CAM_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class NaborisIpCam {
private:
    ros::NodeHandle nh;

    bool has_connection;
    double fps;
    cv::VideoCapture capture;

    boost::shared_ptr<image_transport::ImageTransport> image_transport;
    image_transport::CameraPublisher image_pub;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
    cv_bridge::CvImage cv_img;
    ros::Timer timer;

    void connectToCamera();

    void timerCallback(const ros::TimerEvent &event);

public:
    NaborisIpCam(ros::NodeHandle* nodehandle);

    static const string NODE_NAME;

    static const string USERNAME;
    static const string PASSWORD;
    static const string IP_ADDRESS;
    static const string PORT;
    static const string URL_SUFFIX;
    static const string IMAGE_PUB_TOPIC;
};

#endif  // _NABORIS_IP_CAM_H_
