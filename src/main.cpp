#include "../include/vision_pkg/header.h"

void Vision::onInit()
{
   image_sub_ = nh_.subscribe("/usb_cam/image_raw",1,&Vision::receiveCam,this);
   binary_pub_ = nh_.advertise<sensor_msgs::Image>("/vision/binary_publihser",1);
   segmentation_pub_ = nh_.advertise<sensor_msgs::Image>("/vision/segmentation_publihser",1);
   callback_ = boost::bind(&Vision::dynamicCallback, this, _1);
   server_.setCallback(callback_);
   cv::KalmanFilter KF(4, 2, 0);
   kf_ =  KF;
   kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);
   setIdentity(kf_.measurementMatrix);
   setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-1));
   setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-5));
   setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
   randn(kf_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));\
   measurement_ = cv::Mat::zeros(2, 1, CV_32F);
   init_ =true;
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
    rect_width_=config.rect_width;
    rect_height_=config.rect_height;
    distance_thresh_=config.distance_thresh;
}

void Vision::receiveCam(const sensor_msgs::ImageConstPtr &image)
{
    auto cv_image_ptr = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    imgProcess(cv_image_ptr->image);
}

inline int Vision::getDistance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
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
    cv::Mat label,centriod,status;
    auto * contours_ptr = new std::vector< std::vector< cv::Point> >();
    cv::connectedComponentsWithStats(mor_img,label,status,centriod,8);
    binary_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , mor_img).toImageMsg());
    int index = 0;
    double max_y=0;
    for (int i =0;i<centriod.cols;i++)
    {
        if(centriod.at<double>(i,1) > max_y)
        {
            max_y = centriod.at<double>(i,1);
            index = i;
        }

    }

    cv::Point2f centriod_pt(centriod.at<double>(index,0),centriod.at<double>(index,1));

    if (getDistance(prev_pt_,centriod_pt) > distance_thresh_ && !init_)
    {
        centriod_pt.x = predict_pt_.x;
        centriod_pt.y = prev_pt_.y;
    }


    cv::circle(image,centriod_pt,3,cv::Scalar(255,0,0),2);
    prev_pt_ = centriod_pt;
    cv::Mat prediction = kf_.predict();
    predict_pt_= cv::Point2f(prediction.at<double>(0),prediction.at<double>(1) );

    measurement_.at<double>(0) = centriod_pt.x;
    measurement_.at<double>(1) = centriod_pt.y;

    kf_.correct(measurement_);


    cv::circle(image,predict_pt_,3,cv::Scalar(0,255,255),2);

    std::cout<<predict_pt_<<std::endl;

    delete contours_ptr;
    segmentation_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"bgr8" , image).toImageMsg());
    init_ = false;
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
