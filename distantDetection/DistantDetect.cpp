#include "../universal/CameraCtl.hpp"
#include "../distance/LightMatch.hpp"
#include "../distance/ArmorPlate.hpp" 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <iostream>
#define DARK
//#define RECORD					//录像标识

using namespace std;

cv::VideoCapture cap;
cm::CameraCtl ctl;
ArmorPlate amp;
LightMatch match;
cv::Mat frame, screen, proced;
int test_count = 0;
double t_sum = 0, t_start = 0, t_end = 0;		//计时模块
char str[20], key = 0;
bool record_judge = false;

#ifdef RECORD
	std::string outputVideoPath = "cv_output.avi";
    cv::Size sWH = cv::Size(720, 540);
	cv::VideoWriter outputVideo;
#endif

void subCallBack(const sensor_msgs::ImageConstPtr &msg){
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::resize(frame, frame, cv::Size(720, 540));			//此处可以修改，近程时取图像resize后的图片，远程取ROI
	t_start = cv::getTickCount();
	if(frame.empty()) break;
	ctl.setExposureTime( match.low_exposure ? 7000 : 700);		
	if(match.lowExposure(frame, proced)){
		match.findPossible(proced);
		#ifdef DARK
			match.drawLights(frame);							//绘制所有灯条
			amp.matchAll(frame, match.matches, match.possibles);//查找匹配灯条
			amp.drawArmorPlates(frame);							//绘制装甲板
			t_end = cv::getTickCount();
			if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
			test_count++;
			cv::imshow("disp", frame);
			key=cv::waitKey(1);
			if(key==27) break;
			else if(key==' ') cv::waitKey(0);
		#endif
	}
	#ifndef DARK
	else{
		match.drawLights(frame);							//绘制所有灯条
		amp.matchAll(frame, match.matches, match.possibles);//查找匹配灯条
		amp.drawArmorPlates(frame);							//绘制装甲板
		#ifdef RECORD										//视频帧保存
			if(record_judge) {
				outputVideo<<frame;
				cv::circle(frame, cv::Point(20, 20), 10, cv::Scalar(0, 0, 255), -1);	//录像标识
			}
		#endif

		t_end = cv::getTickCount();
		if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
		test_count++;
		cv::imshow("disp", frame);
		key=cv::waitKey(1);
		if(key==27) break;
		
		#ifdef RECORD
			else if(key=='e') record_judge =! record_judge;
		#endif
		
		else if(key==' ') cv::waitKey(0);
	}
	#endif
	t_end = cv::getTickCount();
	if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
	test_count++;
}

//节点定义
int main(int argc, char ** argv){
	ros::init(argc, argv, "distance");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("high_perform", 1, )
	#ifdef RECORD
		//打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
		outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 30.0, sWH);
	#endif

	ctl.startGrabbing();
	cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);

	ros::spin();

	std::cout << "Average running time:" << t_sum / test_count - 1 << std::endl;

	#ifdef RECORD
        outputVideo.release();
    #endif

	cv::destroyAllWindows();
	return 0;
}
