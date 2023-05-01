#pragma once
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <dynamic_reconfigure/server.h>
#include "vision_pkg/dynamicConfig.h"
#include <std_msgs/Int16.h>

class Vision
{
public:
    void onInit();
    void receiveCam(const sensor_msgs::ImageConstPtr &image);
    void imgProcess(cv::Mat &image);
    void dynamicCallback(vision_pkg::dynamicConfig& config);
    bool priorProcess(const cv::Mat &mor_img);
    int escapeProcess(const cv::Mat &mor_img);
    int getDistance(const cv::Point2f &p1 , const cv::Point2f &p2);
    dynamic_reconfigure::Server<vision_pkg::dynamicConfig> server_;
    dynamic_reconfigure::Server<vision_pkg::dynamicConfig>::CallbackType callback_;

    int lower_hsv_h_;
    int middle_x_;
    int middle_y_;
    int judge_range1_;
    int judge_range2_;
    int judge_range3_;
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
    int area_threhsold_;
    
    int left1_;
    int left2_;
    int right1_;
    int right2_;
    int min_escape_distance_;
    int prior_y_threshold_;
    int prior_point_num_threshold_;


    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher binary_pub_;
    ros::Publisher segmentation_pub_;
    ros::Publisher direction_pub_;
};