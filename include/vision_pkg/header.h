#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <dynamic_reconfigure/server.h>
#include "vision_pkg/dynamicConfig.h"


class Vision
{
public:
    void onInit();
    void receiveCam(const sensor_msgs::ImageConstPtr &image);
    void imgProcess(cv::Mat &image);
    void dynamicCallback(vision_pkg::dynamicConfig& config);
    int getDistance(const cv::Point2f &p1 , const cv::Point2f &p2);
    dynamic_reconfigure::Server<vision_pkg::dynamicConfig> server_;
    dynamic_reconfigure::Server<vision_pkg::dynamicConfig>::CallbackType callback_;

    int lower_hsv_h_;
    int lower_hsv_s_;
    int lower_hsv_v_;
    int upper_hsv_h_;
    int upper_hsv_s_;
    int upper_hsv_v_;
    int morph_type_;
    int morph_iterations_;
    int morph_size_;
    int tl_x_;
    int tl_y_;
    int rect_width_;
    int rect_height_;
    int distance_thresh_;
    bool init_;
    cv::Point2f prev_pt_;
    cv::Point2f predict_pt_;
    cv::Mat measurement_;
    cv::KalmanFilter kf_;
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher binary_pub_;
    ros::Publisher segmentation_pub_;
};