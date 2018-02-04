
#include "naboris_stereo/naboris_stereo.h"

const string NaborisStereo::NODE_NAME = "naboris_stereo";
const double NaborisStereo::MIN_ALLOWED_TIME_DIFF = 0.3;

NaborisStereo::NaborisStereo(ros::NodeHandle* nodehandle):
    nh(*nodehandle),
    img_transport(nh)
{
    ROS_INFO("Stereo node starting...");

    nh.param<string>("right_cam_sub_topic", right_cam_sub_topic, "/naboris_ip_cam/image_raw");
    nh.param<string>("left_cam_sub_topic", left_cam_sub_topic, "/raspicam_node/image");
    nh.param<double>("fps", fps, 5.0);

    ROS_INFO("right_cam_sub_topic: %s", right_cam_sub_topic.c_str());
    ROS_INFO("left_cam_sub_topic: %s", left_cam_sub_topic.c_str());
    ROS_INFO("output fps: %f", fps);

    nh.param<string>("right_cam_pub_topic", right_cam_pub_topic, "/naboris_stereo/right/image_raw");
    nh.param<string>("left_cam_pub_topic", left_cam_pub_topic, "/naboris_stereo/left/image_raw");

    ROS_INFO("right_cam_pub_topic: %s", right_cam_pub_topic.c_str());
    ROS_INFO("left_cam_pub_topic: %s", left_cam_pub_topic.c_str());

    right_image_sub = img_transport.subscribeCamera(
        right_cam_sub_topic,
        1,
        boost::bind(
            &NaborisStereo::right_image_callback,
            this,
            _1, _2
        )
    );
    left_image_sub = img_transport.subscribeCamera(
        left_cam_sub_topic,
        1,
        boost::bind(
            &NaborisStereo::left_image_callback,
            this,
            _1, _2
        ),
        ros::VoidPtr(),
        image_transport::TransportHints("compressed")
    );

    right_image_pub = img_transport.advertiseCamera(right_cam_pub_topic, 1);
    left_image_pub = img_transport.advertiseCamera(left_cam_pub_topic, 1);

    bool left_cam_ready = false;
    right_cam_in_sync = false;
    right_cam_in_sync_prev = false;
    time_diff_sum = 0.0;
    time_diff_count = 0;

    prev_pub_time = ros::Time::now();
    pub_duration = ros::Duration(1 / fps);

    ROS_INFO("Stereo node init done");
}

void NaborisStereo::right_image_callback(const ImageConstPtr& right_image_msg, const sensor_msgs::CameraInfoConstPtr& right_info_msg)
{
    if (!left_cam_ready) {
        left_cam_ready = true;
        ROS_INFO("Left image ready");
    }
    right_saved_image.reset(new sensor_msgs::Image(*right_image_msg));
    right_saved_info.reset(new sensor_msgs::CameraInfo(*right_info_msg));
    right_saved_timestamp = right_image_msg->header.stamp.toSec();
}

void NaborisStereo::left_image_callback(const ImageConstPtr& left_image_msg, const sensor_msgs::CameraInfoConstPtr& left_info_msg)
{
    ros::Time current_time = ros::Time::now();
    if ((current_time - prev_pub_time) < pub_duration) {
        return;
    }
    prev_pub_time = current_time;

    sensor_msgs::ImagePtr left_saved_msg(new sensor_msgs::Image(*left_image_msg));
    left_image_vector.push_back(left_saved_msg);

    sensor_msgs::CameraInfoPtr left_saved_info(new sensor_msgs::CameraInfo(*left_info_msg));
    left_info_vector.push_back(left_saved_info);

    left_stamp_vector.push_back(left_image_msg->header.stamp.toSec());

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

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "stereo";

    left_image_vector.at(0)->header = header;
    left_info_vector.at(0)->header = header;
    right_saved_image->header = header;
    right_saved_info->header = header;

    left_image_pub.publish(left_image_vector.at(0), left_info_vector.at(0));
    right_image_pub.publish(right_saved_image, right_saved_info);
}
