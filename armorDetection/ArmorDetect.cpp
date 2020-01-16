#include <CameraCtl.hpp>
#include <ArmorPlate.hpp>
#include <iostream>
//#define DARK
//#define RECORD					//录像标识

using namespace std;

cv::VideoCapture cap;
CameraCtl ctl;
ArmorPlate amp;
cv::Mat frame, screen, proced, ROI;
int test_count = 0;
double sumTime = 0, startTime = 0, endTime = 0;	
char str[20], key = 0;
bool record_judge = false;

int main(){
	#ifdef RECORD
		//打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
    	std::string outputVideoPath = "cv_output.avi";
    	cv::Size sWH = cv::Size(720, 540);
		cv::VideoWriter outputVideo;
		outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 30.0, sWH);
	#endif
	ctl.startGrabbing();
	ctl.setGainMode(CONTINUOUS);
	ctl.setBGRBalance();
    ctl.setFrameRate(120);
	cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
	while(true){
		frame=ctl.getOpencvMat();
		cv::resize(frame, frame, cv::Size(720, 540));
		startTime = cv::getTickCount();
		if(frame.empty()) break;
		ctl.setExposureTime( amp.low_exposure ? 7000 : 600);
		if(amp.lowExposure(frame, proced)){
			amp.findPossible(proced);
			#ifdef DARK
				amp.drawArmorPlate(frame);
				endTime = cv::getTickCount();
				if (test_count)sumTime += (endTime - startTime) / cv::getTickFrequency() * 1000;
				test_count++;
				cv::imshow("disp", frame);
				key=cv::waitKey(1);
				if(key==27) break;
				else if(key==' ') cv::waitKey(0);
			#endif
		}
		#ifndef DARK
		else{
			amp.drawArmorPlate(frame);

			#ifdef RECORD				//视频帧保存
				if(record_judge) {
					outputVideo<<frame;
					cv::circle(frame, cv::Point(20, 20), 10, cv::Scalar(0, 0, 255), -1);	//录像标识
				}
			#endif

			endTime = cv::getTickCount();
			if (test_count)sumTime += (endTime - startTime) / cv::getTickFrequency() * 1000;
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
		endTime = cv::getTickCount();
		if (test_count)sumTime += (endTime - startTime) / cv::getTickFrequency() * 1000;
		test_count++;
		//cv::imshow("disp", frame);
		//key=cv::waitKey(1);
		//if(key==27) break;

	}

	#ifdef RECORD
        outputVideo.release();
    #endif

	std::cout << "Average running time:" << sumTime / test_count - 1 << std::endl;
	ctl.stopGrabbing();
	cv::destroyAllWindows();
	return 0;
}
