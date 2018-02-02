
#include "naboris_stereo/naboris_stereo.h"

const string NaborisStereo::NODE_NAME = "naboris_stereo";
const double NaborisStereo::MIN_ALLOWED_TIME_DIFF = 0.3;

NaborisStereo::NaborisStereo(ros::NodeHandle* nodehandle):
    nh(*nodehandle),
    img_transport(nh)
{
    ROS_INFO("Stereo node starting...");

    nh.param<string>("right_cam_sub_topic", right_cam_sub_topic, "/naboris_ip_cam/image_ra");
    nh.param<string>("left_cam_sub_topic", left_cam_sub_topic, "/raspicam_node/imag");

    // nh.param<string>("right_cam_pub_topic", right_cam_pub_topic, "/naboris_ip_cam/image_ra");
    // nh.param<string>("left_cam_pub_topic", left_cam_pub_topic, "/raspicam_node/imag");

    right_image_sub = img_transport.subscribeCamera(
        "/naboris_ip_cam/image_raw",
        1,
        boost::bind(
            &NaborisStereo::right_image_callback,
            this,
            _1, _2
        )
    );
    left_image_sub = img_transport.subscribeCamera(
        "/raspicam_node/image",
        1,
        boost::bind(
            &NaborisStereo::left_image_callback,
            this,
            _1, _2
        ),
        ros::VoidPtr(),
        image_transport::TransportHints("compressed")
    );

    right_image_pub = img_transport.advertiseCamera("/naboris_stereo/right/image_raw", 1);
    left_image_pub = img_transport.advertiseCamera("/naboris_stereo/left/image_raw", 1);

    right_cam_in_sync = false;
    right_cam_in_sync_prev = false;
    time_diff_sum = 0.0;
    time_diff_count = 0;

    ROS_INFO("Stereo node init done");
}

// void NaborisStereo::callback(const CompressedImageConstPtr& right_image_msg, const ImageConstPtr& left_image_msg)
// void NaborisStereo::callback(const ImageConstPtr& right_image_msg, const ImageConstPtr& left_image_msg)
// {
//     ROS_INFO("stereo node received images");
//     right_image_pub.publish(right_image_msg);
//     left_image_pub.publish(left_image_msg);
// }

void NaborisStereo::right_image_callback(const ImageConstPtr& right_image_msg, const sensor_msgs::CameraInfoConstPtr& right_info_msg) {
    right_saved_image.reset(new sensor_msgs::Image(*right_image_msg));
    // right_saved_image = extractMat(right_image_msg);
    right_saved_info.reset(new sensor_msgs::CameraInfo(*right_info_msg));
    right_saved_timestamp = right_image_msg->header.stamp.toSec();
    // if (!right_saved_image.empty()) {
        // ROS_INFO("Received right image: %f", right_saved_timestamp);
    // }
}

void NaborisStereo::left_image_callback(const ImageConstPtr& left_image_msg, const sensor_msgs::CameraInfoConstPtr& left_info_msg)
{
    // cv::Mat left_image = extractMat(left_image_msg);
    // if (left_image.empty()) {
    //     return;
    // }

    sensor_msgs::ImagePtr left_saved_msg(new sensor_msgs::Image(*left_image_msg));
    left_image_vector.push_back(left_saved_msg);

    sensor_msgs::CameraInfoPtr left_saved_info(new sensor_msgs::CameraInfo(*left_info_msg));
    left_info_vector.push_back(left_saved_info);

    left_stamp_vector.push_back(left_image_msg->header.stamp.toSec());
    // ROS_INFO("Received left image: %f", left_stamp_vector.back());

    double min_time_diff = std::numeric_limits<double>::infinity();
    size_t min_time_diff_index = 0;

    for (size_t index = 0; index < left_stamp_vector.size(); index++) {
        double time_diff = abs(left_stamp_vector.at(index) - right_saved_timestamp);
        if (time_diff < min_time_diff) {
            min_time_diff = time_diff;
            min_time_diff_index = index;
        }
    }

    right_cam_in_sync_prev = right_cam_in_sync;
    if (min_time_diff > MIN_ALLOWED_TIME_DIFF) {
        right_cam_in_sync = false;
        if (right_cam_in_sync != right_cam_in_sync_prev) {
            ROS_INFO("Right image out of sync! Will keep checking for images.");
        }
        return;
    }

    right_cam_in_sync = true;
    if (right_cam_in_sync != right_cam_in_sync_prev) {
        ROS_INFO("Right image in sync.");
    }

    time_diff_sum += min_time_diff;
    time_diff_count++;
    if (time_diff_count % 100 == 0) {
        ROS_INFO("Average stereo image time diff: %f. Num synced images: %d", time_diff_sum / (double)(time_diff_count), time_diff_count);
    }

    if (min_time_diff_index > 0) {
        ERASE_UPTO_MIN_INDEX(left_image_vector);
        ERASE_UPTO_MIN_INDEX(left_info_vector);
        ERASE_UPTO_MIN_INDEX(left_stamp_vector);
    }

    std_msgs::Header left_header;
    left_header.stamp = ros::Time::now();
    left_header.frame_id = "left_stereo";

    std_msgs::Header right_header;
    right_header.stamp = ros::Time::now();
    right_header.frame_id = "right_stereo";

    left_image_vector.at(0)->header = left_header;
    left_info_vector.at(0)->header = left_header;
    right_saved_image->header = right_header;
    right_saved_info->header = right_header;

    // left_image_pub.publish(matToMsg(left_image_vector.at(0), header), left_info_vector.at(0));
    left_image_pub.publish(left_image_vector.at(0), left_info_vector.at(0));
    right_image_pub.publish(right_saved_image, right_saved_info);
}

/*
ImagePtr NaborisStereo::matToMsg(cv::Mat image, std_msgs::Header header) {
    return cv_bridge::CvImage(
        header, image_encodings::BGR8, image
    ).toImageMsg();
}

cv::Mat NaborisStereo::extractMat(const ImageConstPtr& image_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    return cv_ptr->image;
}


void NaborisStereo::right_image_callback(const ImageConstPtr& right_image_msg)
{
    right_img_ready = true;
    ROS_INFO("received right image");
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_image_msg, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // cv::Mat converted_image;
    // cv::cvtColor(cv_ptr->image, converted_image, cv::COLOR_BGR2GRAY);

    ImagePtr right_image_msg_pub = cv_bridge::CvImage(
        right_image_msg->header, image_encodings::BGR8, cv_ptr->image //converted_image
    ).toImageMsg();

    if (right_img_ready && left_img_ready)
    {
        ROS_INFO("right_image_msg->header.stamp: %f, %d", right_image_msg->header.stamp.toSec(), right_image_msg->header.seq);
        right_image_pub.publish(right_image_msg_pub);
    }
}


void NaborisStereo::left_image_callback(const ImageConstPtr& left_image_msg)
{
    left_img_ready = true;
    ROS_INFO("received left image");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(left_image_msg, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // cv::Mat converted_image = cv::imdecode(cv::Mat(cv_ptr->image), 1);
    cv::Mat converted_image = cv::Mat(cv_ptr->image);
    // cv::cvtColor(converted_image, converted_image, cv::COLOR_BGR2GRAY);

    left_image_msg_pub = cv_bridge::CvImage(
        left_image_msg->header, image_encodings::BGR8, converted_image
    ).toImageMsg();

    if (right_img_ready && left_img_ready)
    {
        ROS_INFO("left_image_msg->header.stamp: %f, %d", left_image_msg->header.stamp.toSec(), left_image_msg->header.seq);
        left_image_pub.publish(left_image_msg_pub);
    }
}*/
