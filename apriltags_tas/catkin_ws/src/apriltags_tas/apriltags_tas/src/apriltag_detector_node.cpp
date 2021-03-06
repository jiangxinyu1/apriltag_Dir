/*
 * @Author: your name
 * @Date: 2021-11-30 19:08:57
 * @LastEditTime: 2021-12-01 17:17:08
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_tas/src/apriltag_detector_node.cpp
 */
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <apriltags_msgs/AprilTagDetections.h>
#include <apriltag_ros/common_functions.h>
#include <apriltags_tas/AprilTagDetectorConfig.h>
#include <apriltags_tas/apriltag_detector.h>

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "apriltag_detector");
    ros::NodeHandle nh("~");

    std::string camera_info_topic ;
    nh.getParam("camera_info_topic", camera_info_topic);

    bool use_test_input_image ;
    nh.getParam("use_test_input_image", use_test_input_image);

    boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info;

    

    if (!use_test_input_image)
    {
        ROS_INFO_STREAM("Waiting for camera info on topic \"" << camera_info_topic << "\"");
        camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic);
        if (camera_info == NULL)
        {
            ROS_ERROR_STREAM("Got no camera info on topic \"" << camera_info_topic << "\"");
        }
    }

    apriltag_ros::TagDetector tag_config(nh);

    AprilTagDetector apriltag_detector(camera_info, tag_config);

    dynamic_reconfigure::Server<apriltags_tas::AprilTagDetectorConfig> reconfigure_server;
    reconfigure_server.setCallback(boost::bind(&AprilTagDetector::reconfigure, &apriltag_detector, _1, _2));

    image_transport::ImageTransport it(nh);

    apriltag_detector.detections_pub_ = nh.advertise<apriltags_msgs::AprilTagDetections>("detections", 1, false);
    apriltag_detector.image_pub_ = it.advertise("image", 1);

    std::unique_ptr<image_transport::Subscriber> sub;

    if (!use_test_input_image)
    {
        std::string image_topic;
        nh.getParam("image_topic", image_topic);

        sub = std::make_unique<image_transport::Subscriber>(
            it.subscribe(image_topic, 1, &AprilTagDetector::imageCallback, &apriltag_detector));
    }
    else
    {
        std::string test_input_image_path;
        nh.getParam("test_input_image_path", test_input_image_path);

        cv::Mat img = cv::imread(test_input_image_path);

        // cv::imshow("img",img);
        // cv::waitKey(0);

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        apriltag_detector.imageCallback(image_msg);
    }

    ros::spin();

    return 0;
}