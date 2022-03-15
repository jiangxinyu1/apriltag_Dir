/*
 * @Author: your name
 * @Date: 2021-11-30 19:08:57
 * @LastEditTime: 2021-12-02 16:52:58
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /apriltags_tas/catkin_ws/src/apriltags_tas/apriltags_tas/include/apriltags_tas/apriltag_detector.h
 */
#pragma once

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <apriltags/TagFamily.h>

#include <apriltags/Tag16h5.h>
#include <apriltags/Tag25h7.h>
#include <apriltags/Tag25h9.h>
#include <apriltags/Tag36h11.h>
#include <apriltags/Tag36h9.h>
#include <apriltags/TagDetector.h>

#include <apriltag_ros/common_functions.h>

#include <apriltags_tas/AprilTagDetectorConfig.h>

class AprilTagDetector
{
  public:
    AprilTagDetector(const sensor_msgs::CameraInfo::ConstPtr& camera_info, apriltag_ros::TagDetector& tag_config);

    void reconfigure(apriltags_tas::AprilTagDetectorConfig& config, uint32_t level);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::Publisher detections_pub_;
    image_transport::Publisher image_pub_;

  private:
    void process(const cv::Mat& image);

    std::vector<AprilTags::TagDetection> detectAprilTags(cv::Mat& img) noexcept;
    void refineCornerPointsByDirectEdgeOptimization(cv::Mat& img,
                                                    std::vector<AprilTags::TagDetection>& tag_detections) noexcept;
    void filterCrossCorners(cv::Mat& img, std::vector<AprilTags::TagDetection>& tag_detections) noexcept;

    void filterUnknownTags(std::vector<AprilTags::TagDetection>& tag_detections) noexcept;
    void removeBadTags(std::vector<AprilTags::TagDetection>& tag_detections) noexcept;

    bool getPose(AprilTags::TagDetection& tag, geometry_msgs::Pose& pose) noexcept;

    void publishTagDetections(std::vector<AprilTags::TagDetection>& tag_detections, std_msgs::Header header) noexcept;
    void publishTfTransform(std::vector<AprilTags::TagDetection>& tag_detections, std_msgs::Header header) noexcept;
    void drawTagDetections(cv::Mat img, std::vector<AprilTags::TagDetection>& tag_detections) noexcept;

    void myPoseCheck(cv::Mat img, Eigen::Matrix4d &T1, Eigen::Matrix4d &T2) noexcept;
    void myGetPose(std::vector<AprilTags::TagDetection>& tag_detections, Eigen::Matrix4d &T1,Eigen::Matrix4d &T2, bool &blBothReady) noexcept;

    std::shared_ptr<AprilTags::TagDetector> apriltag_cpp_detector_;

    image_geometry::PinholeCameraModel camera_model_;
    
    cv::Mat image_;
    std_msgs::Header img_header_;

    apriltag_ros::TagDetector tag_config_;

    apriltags_tas::AprilTagDetectorConfig config_;
};

bool SaveImage(const cv::Mat& img, std::string& absolutePath);