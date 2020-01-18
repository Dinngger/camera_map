#include <CameraCtl.hpp>
#include <ArmorPlate.hpp>
#include <iostream>
//#define DARK
//#define RECORD					//录像标识

using namespace std;


ArmorPlate::ArmorPlate(){
	mean_val=40.0;
	low_exposure=true;
}

ArmorPlate::~ArmorPlate(){
	;
}

void ArmorPlate::reset(){
	light_list.clear();
	possibles.clear();
	trapezoids.clear();
}

void ArmorPlate::findPossible(cv::Mat src){					//找出所有可能灯条，使用梯形匹配找出相匹配的灯条对
	reset();
	cv::Mat binary;
	cv::threshold(src, binary, 128, 255, CV_THRESH_BINARY);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);	//寻找图上轮廓
	for (int i = 0; i < contours.size(); ++i) {
		double area = cv::contourArea(contours[i]);
		if (area > 10) {		
			cv::RotatedRect light = cv::minAreaRect(contours[i]);
			possibles.push_back(light);
		}
	}
	for (int i = 0; i < possibles.size(); ++i) {
		cv::Point2f corners[4];
		possibles[i].points(corners);
		getTrapezoids(corners);								
	}
	getRealLight(possibles.size());
}

void ArmorPlate::preProcess(cv::Mat src, cv::Mat &high_exp, cv::Mat &low_exp){
	cv::Mat channels[3];
	cv::split(src, channels);
	cv::Mat res = channels[0];
	cv::threshold(res, high_exp, 85, 255, CV_THRESH_BINARY);
	cv::threshold(res, low_exp, 170, 255, CV_THRESH_BINARY);
}

void ArmorPlate::drawArmorPlate(cv::Mat &src) {
	for(int i=0;i<light_list.size();i+=2){
		cv::RotatedRect rect=getArmorPlate(light_list[i], light_list[i+1]);
		cv::Point2f pts[4];
		rect.points(pts);
		for(int j = 0; j<4; ++j ){
			cv::line(src, pts[j], pts[(j+1)%4], cv::Scalar(0, 255, 0));
		}
		cv::circle(src, rect.center, 2, cv::Scalar(0, 0, 255), -1);			//标定打击中心
	}
}

void ArmorPlate::getArmorRect(std::vector<cv::RotatedRect> &result) {
	for(int i=0; i<light_list.size(); i+=2) {
		cv::RotatedRect rect=getArmorPlate(light_list[i], light_list[i+1]);
		result.push_back(rect);
	}
}

bool ArmorPlate::lowExposure(cv::Mat src, cv::Mat &gray){
	low_exposure = !low_exposure;								//exposure模式交替设置，消除环境光影响
	cv::Scalar mean_values;
	cv::Mat channels[3];
	cv::split(src, channels);
	gray = channels[0];											//取出蓝色通道
	mean_values=cv::mean(gray);
	bool res=mean_values.val[0]< mean_val;
	mean_val=mean_val*0.95 + 0.05*mean_values.val[0];
	return res;			
}

void ArmorPlate::getRealLight(int size){
	bool used[size];						//是否经过了梯形匹配的标签
	bool flag[size][size];					//两灯条是否满足要求,是个对称矩阵，当[i][j],[j][i]为真时，两灯条匹配
	for (int i = 0; i < size; ++i) {		//初始化		
		used[i] = false;
        for(int j = 0; j < size; ++j)
		    flag[i][j] = false;
	}
	for (int i = 0; i < size; ++i) {		//梯形包含匹配
		cv::Point2f corners[4];
		possibles[i].points(corners);
		for (int j = 0; j<size*2; ++j) {
			if (isInTrapezoid(corners, trapezoids[j])) {
				flag[i][j/2] = true;
			}
		}
	}
	for (int i = 0; i<size; ++i) {			//将结果放入最终的容器light_list中
		if (used[i])						//已经匹配则跳过
			continue;
		for (int j = 0; j<size; ++j) {
			if (used[j])					
				continue;
			if (flag[i][j] == true && flag[j][i] == true) {
				light_list.push_back(possibles[i]);
				light_list.push_back(possibles[j]);
				used[i] = true;
				used[j] = true;
			}
		}
	}
	//std::cout<<"Result size:"<<light_list.size()<<std::endl;
}

//匹配的灯条其梯形将会互相包含
void ArmorPlate::getTrapezoids(cv::Point2f corners[4]){
	std::vector<cv::Point2f> left_trapezoid;
	std::vector<cv::Point2f> right_trapezoid;
	double d1 = getDistance(corners[0], corners[1]);
	double d2 = getDistance(corners[1], corners[2]);
	cv::Point2f trapezoid_points[4];								//梯形角点
	cv::Point2f direction_vectors[2];								//方向向量
	cv::Point2f midpoints[2];										//中点
	int i0 = d1<d2? 1:0;										//长所在边第一个顶点的位置
	midpoints[0] = (corners[i0] + corners[i0 + 1]) / 2;			//获得旋转矩形两条长上的中点
	midpoints[1] = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
	cv::Point2f vertical_vector = (midpoints[1]) - (midpoints[0]);	//获得垂直长方向的方向向量
	vertical_vector = (i0 == 0 ? d1 : d2) * PARAM1 * vertical_vector / sqrt(vertical_vector.x*vertical_vector.x + vertical_vector.y*vertical_vector.y);
	direction_vectors[0] = corners[i0 + 1] - corners[i0];
	direction_vectors[1] = corners[(i0 + 3) % 4] - corners[i0 + 2];
	left_trapezoid.push_back(corners[i0]);
	left_trapezoid.push_back(corners[i0 + 1]);
	left_trapezoid.push_back(midpoints[0] - vertical_vector + PARAM2 * direction_vectors[0]);
	left_trapezoid.push_back(midpoints[0] - vertical_vector - PARAM2 * direction_vectors[0]);
	right_trapezoid.push_back(corners[i0 + 2]);
	right_trapezoid.push_back(corners[(i0 + 3) % 4]);
	right_trapezoid.push_back(midpoints[1] + vertical_vector + PARAM2 * direction_vectors[1]);
	right_trapezoid.push_back(midpoints[1] + vertical_vector - PARAM2 * direction_vectors[1]);
	trapezoids.push_back(left_trapezoid);						//灯条左右两边将会拓展出两个梯形
	trapezoids.push_back(right_trapezoid);
}

bool ArmorPlate::isInTrapezoid(cv::Point2f corners[4], const std::vector<cv::Point2f> &trapezoid){
	int count = 0;
	for (int i = 0; i<4; i++) {
		if (cv::pointPolygonTest(trapezoid, corners[i], false) >= 0)
			count++;
	}
	if (count == 4) {
		return true;
	}
	return false;
}

float ArmorPlate::getDistance(cv::Point p1, cv::Point p2){
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	distance = sqrtf(distance);
	return distance;
}

cv::RotatedRect ArmorPlate::getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2){
	cv::Point2f pts1[4], pts2[4];
	r1.points(pts1); r2.points(pts2);
	std::vector<cv::Point2f> pts(8); pts.clear();
	for(int i=0; i<4; ++i){
		pts.push_back(pts1[i]);
		pts.push_back(pts2[i]);
	}
	return cv::minAreaRect(pts);
}
/*
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
*/