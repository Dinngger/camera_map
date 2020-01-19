//=========cv_bridge接收图像例程===========
//接收格式如下
//created by hqy on 2020.1.16

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
//#define DEBUG

char key = 0;

#ifdef DEBUG
	//打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
	std::string outputVideoPath = "/home/sentinel/cv_output1.avi";
	cv::Size sWH = cv::Size(1440, 1080);
	cv::VideoWriter outputVideo;
#endif

void subCallBack(const sensor_msgs::ImageConstPtr &msg){
    try{
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("disp", frame);
        key = cv::waitKey(1);
        #ifdef DEBUG
            outputVideo<<frame;
        #endif
    }
    catch (cv_bridge::Exception &e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv){

    #ifdef DEBUG
		outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 88.0, sWH);		
        printf("Waiting to start record...\n");
	#endif

    ros::init(argc, argv, "test_frame");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("high_perform", 1, subCallBack);

    while(ros::ok()){
        ros::spinOnce();
        if(key == 27) break;
    }
    #ifdef DEBUG
        outputVideo.release();
        printf("Video captured.\n");
    #endif
    cv::destroyWindow("disp");
    return 0;
}