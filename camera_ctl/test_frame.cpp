//=========cv_bridge接收图像例程===========
//接收格式如下
//created by hqy on 2020.1.16

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

void subCallBack(const sensor_msgs::ImageConstPtr &msg){
    try{
        cv::imshow("disp", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_frame");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("high_perform", 1, subCallBack);

    ros::spin();
    cv::destroyWindow("disp");
    return 0;
}