#include "../include/vision_pkg/header.h"

void Vision::onInit()
{
   image_sub_ = nh_.subscribe("/usb_cam/image_raw",1,&Vision::receiveCam,this);
   binary_pub_ = nh_.advertise<sensor_msgs::Image>("/vision/binary_publihser",1);
   segmentation_pub_ = nh_.advertise<sensor_msgs::Image>("/vision/segmentation_publihser",1);
   direction_pub_ = nh_.advertise<std_msgs::Int16>("/vision/direction",1);
   callback_ = boost::bind(&Vision::dynamicCallback, this, _1);
   middle_x_ = 160;
   middle_y_ = 156;
   server_.setCallback(callback_);
}

void Vision::dynamicCallback(vision_pkg::dynamicConfig &config)
{
    morph_type_ = config.morph_type;
    morph_iterations_ = config.morph_iterations;
    lower_hsv_h_=config.lower_hsv_h;
    lower_hsv_s_=config.lower_hsv_s;
    lower_hsv_v_=config.lower_hsv_v;
    upper_hsv_h_=config.upper_hsv_h;
    upper_hsv_s_=config.upper_hsv_s;
    upper_hsv_v_=config.upper_hsv_v;
    morph_size_=config.morph_size;
    tl_x_=config.tl_x;
    tl_y_=config.tl_y;
    rect_width_ = config.rect_width;
    rect_height_ = config.rect_height;
    judge_range1_=config.judge_range1;
    judge_range2_=config.judge_range2;
    judge_range3_=config.judge_range3;
    area_threhsold_=config.area_threshold;
    left1_ = config.left1;
    left2_ = config.left2;
    right1_ = config.right1;
    right2_ = config.right2;
    min_escape_distance_ = config.min_escape_distance;
    prior_y_threshold_ = config.prior_y_threshold;
    prior_point_num_threshold_ = config.prior_point_num_threshold;
}
//int g_num = 0;
void Vision::receiveCam(const sensor_msgs::ImageConstPtr &image)
{
    auto cv_image_ptr = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    imgProcess(cv_image_ptr->image);
//    cv::imwrite("/home/yamabuki/10GDisk/image/"+std::to_string(g_num)+".jpg",cv_image_ptr->image);
//    g_num++;
}

inline int Vision::getDistance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
}

int Vision::escapeProcess(const cv::Mat &mor_img)
{
    int max_y = 240;
    for (int i = 0; i < 320; i++)
    {
        for (int j = 239; j >= tl_y_; j--)
        {
            if (mor_img.at<uchar>(j,i) == 0)
            {
                max_y = j+1 < max_y? j+1 : max_y;
                break;
            }
        }
    }
    return max_y;
}

bool Vision::priorProcess(const cv::Mat &mor_img)
{
    int left_edge_counter = 0;
    int right_edge_counter = 0;
    for (int j =tl_y_; j < tl_y_+prior_y_threshold_; j++)
    {
        if (mor_img.at<uchar>(j,0) == 255)
            left_edge_counter++;
        if (mor_img.at<uchar>(j,319) == 255)
            right_edge_counter++;
    }

    if (prior_y_threshold_ - left_edge_counter < prior_point_num_threshold_ && prior_y_threshold_ - right_edge_counter > prior_point_num_threshold_) // left full
        return true;
    else
        return false;
}

void Vision::imgProcess(cv::Mat &image)
{
    auto * hsv_ptr=new cv::Mat();
    auto * binary_ptr=new cv::Mat();
    cv::Mat mor_img;
    
    cv::cvtColor(image,*hsv_ptr,cv::COLOR_BGR2HSV);
//    cv::cvtColor(image,*hsv_ptr,cv::COLOR_BGR2GRAY);
//    cv::threshold(*hsv_ptr,*binary_ptr,0,255,CV_THRESH_BINARY+CV_THRESH_OTSU);
    cv::inRange(*hsv_ptr,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),*binary_ptr);
    delete hsv_ptr;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1+2*morph_size_, 1+2*morph_size_), cv::Point(-1, -1));
    cv::morphologyEx(*binary_ptr,mor_img,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
    delete binary_ptr;

    if (tl_x_+rect_width_>320) rect_width_=320-tl_x_;
    if (tl_y_+rect_height_>240) rect_height_=240-tl_y_;

    cv::Rect rect(tl_x_,tl_y_,rect_width_,rect_height_);
    cv::Mat mask = cv::Mat::zeros(cv::Size(320,240),CV_8UC1);
    mask(rect).setTo(255);
    cv::bitwise_and(mask,mor_img,mor_img);
//    cv::Mat label,centriod,status;
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(),contours.end(),[](const auto &v1, const auto &v2){return cv::contourArea(v1) > cv::contourArea(v2);});
    if (contours.empty() || cv::contourArea(contours[0]) < area_threhsold_)
    {
        std_msgs::Int16 data;
        std::string text = "go straight";
        data.data = 0;
        cv::putText(image,text,cv::Point(10,10),1,1,cv::Scalar(0,255,255),2);
        direction_pub_.publish(data);
        binary_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , mor_img).toImageMsg());
        segmentation_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"bgr8" , image).toImageMsg());
        return;
    }
    auto contour = contours[0];
    auto moment = cv::moments(contour);
    int cx = int(moment.m10 / moment.m00);
    int cy = int(moment.m01/  moment.m00);

    cv::Point2i centriod_pt (cx,cy);
    cv::polylines(image,contour, true,cv::Scalar(0,255,0),2);
    cv::circle(image,centriod_pt,3,cv::Scalar(0,255,255),2);
    cv::circle(image,cv::Point(middle_x_,cy),3,cv::Scalar(0,255,0),2);

    std_msgs::Int16 data;
    data.data = 0;
    std::string text = "go straight";

    int max_y = escapeProcess(mor_img);
    cv::line(image,cv::Point(0,max_y),cv::Point(319,max_y),cv::Scalar(0,0,255),2);

    cv::line(image,cv::Point(0,tl_y_+prior_y_threshold_),cv::Point(319,tl_y_+prior_y_threshold_),cv::Scalar(255,0,0),2);
    bool prior_judge = priorProcess(mor_img);

    if (max_y - tl_y_ > min_escape_distance_ && mor_img.at<uchar>(239,0) == 255 && mor_img.at<uchar>(239,319) == 255)
    {
        text = "escape";
        data.data = right2_;
    }

    if (prior_judge)
    {
        text = "prior right";
        data.data = right2_;
    }


    if (cx < (middle_x_ - judge_range2_))
    {
        if (cx < (middle_x_ - judge_range3_))
              data.data = -left2_;
        else
          data.data = -left1_;
          
        text = "turn left value:"+std::to_string(data.data);
    }
    else if (cx > (middle_x_ + judge_range2_))
    {
        if (cx > (middle_x_ + judge_range3_))
            data.data = right2_;      
        else
          data.data = right1_; 
        
        text = "turn right value:"+ std::to_string(data.data);
    }



    cv::putText(image,text,cv::Point(10,10),1,1,cv::Scalar(255,255,0),2);
    direction_pub_.publish(data);
    binary_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , mor_img).toImageMsg());
    segmentation_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"bgr8" , image).toImageMsg());

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    Vision vis;
    vis.onInit();
    while (ros::ok())
    {
        ros::spinOnce();
    }
}
