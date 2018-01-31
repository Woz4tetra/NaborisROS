
#include "naboris_ip_cam/naboris_ip_cam.h"

const string NaborisIpCam::USERNAME = "robot";
const string NaborisIpCam::PASSWORD = "naboris";
const string NaborisIpCam::IP_ADDRESS = "10.76.76.1";
const string NaborisIpCam::PORT = "80";
const string NaborisIpCam::URL_SUFFIX = "/api/robot/rightcam";

const string NaborisIpCam::IMAGE_PUB_TOPIC = "raspicam_node_right/image_raw";

const string NaborisIpCam::NODE_NAME = "naboris_ip_cam";

NaborisIpCam::NaborisIpCam(ros::NodeHandle* nodehandle):
    nh(*nodehandle),
    image_transport(new image_transport::ImageTransport(nh)),
    image_pub(image_transport->advertiseCamera(IMAGE_PUB_TOPIC, 1))
{
    std::string camera_info_url;
    std::string camera_name;

    if (!nh.getParam(NODE_NAME + "/camera_info_url", camera_info_url))
    {
        ROS_INFO_STREAM("Camera info url parameter not found, using default");
        camera_info_url = "package://naboris_ip_cam/camera_info/naboris_ip_cam_410x308_30fps.yaml";
    }

    if (!nh.getParam(NODE_NAME + "/camera_name", camera_name))
    {
        ROS_INFO_STREAM("Camera name parameter not found, using default");
        camera_name = "right_cam";
    }

    if (!nh.getParam(NODE_NAME + "/fps", fps))
    {
        ROS_INFO_STREAM("FPS parameter not found, using default (30.0)");
        fps = 30.0;
    }

    camera_info_manager.reset(new camera_info_manager::CameraInfoManager(nh, camera_name, camera_info_url));
    timer = nh.createTimer(ros::Duration(1.0 / fps), &NaborisIpCam::timerCallback, this);

    connectToCamera();
}


void NaborisIpCam::connectToCamera()
{
    const std::string video_stream_address =
      "http://" + USERNAME + ":" + PASSWORD + "@" + IP_ADDRESS + ":" + PORT + "/" + URL_SUFFIX;

    has_connection = true;
    if (!capture.open(video_stream_address))
    {
        has_connection = false;
        ROS_ERROR("Error opening video stream or file");
    }
}

void NaborisIpCam::timerCallback(const ros::TimerEvent &event)
{
    if (!has_connection) {
        return;
    }

    cv::Mat image;
    if (!capture.read(image))
    {
        ROS_WARN("No frame");
        cv::waitKey();
    }

    cv_img.header.stamp = ros::Time::now();
    cv_img.header.frame_id = "naboris_right_cam";
    cv_img.encoding = "bgr8";
    cv_img.image = image;

    sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo(camera_info_manager->getCameraInfo()));
    camera_info->height = image.rows;
    camera_info->width = image.cols;
    camera_info->header = cv_img.header;

    image_pub.publish(cv_img.toImageMsg(), camera_info);
}
