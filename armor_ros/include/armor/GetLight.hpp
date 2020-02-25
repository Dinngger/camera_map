/*
灯条检测模块-找到图上所有合适的灯条
作者：heqianyue
last date of modification:2020.1.13
修改内容：contourArea阈值条件
寻找灯条基本流程：
1.对输入图像进行色道分离，二值化，然后进行开闭运算：用MORPH_RECT
2.找到所有轮廓，存入contours中
3.从轮廓中拟合出无角度的灯条预选区
	从每个灯条预选区中再重新找轮廓，找到轮廓最大的那一个作为最终灯条
	寻找内轮廓时的阈值要较高
4.轮廓是否为灯条的判定模式：
    1.长宽比合适：大于1.5（保证动态模糊下能够识别），小于10
    2.面积合适：当灯条面积小于48且大于25时，若长宽比合适（此时需要大于2），则直接选中，
        若灯条面积大于48，则需要其轮廓面积和灯条面积比至少大于0.5
    3.角度合适：不会有角度适中的灯条（0~30,60~90）          //这条也许可以去掉，ROLL轴偏向很可能存在
5.如果一个外轮廓符合灯条外轮廓外形，且存在内轮廓：
    取内轮廓方法1.2倍为目标灯条（暂定）
    如果外轮廓过小则按照灯条匹配直接框选为灯条
6.显示取到的灯条
7.进行灯条匹配

1.本代码目前的问题存在于找灯条不准确
2.灯条匹配的条件过于宽松
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#define THRESHOLD_BLUE 180
#define THRESHOLD_RED 190

class GetLight{
public:
    GetLight();
    ~GetLight();
	void reset();
    void showLight(cv::Mat &screen);                            //显示所有灯条
	void mergeRect(cv::Rect rect);						//如果有临近矩形，则合并,没有则在list中新增
	void brightAdjust(cv::Mat &src, double alpha, double beta);	//亮度调整函数
	void imgProcess(cv::Mat src, cv::Mat &dst);                  //输入图像的处理
	int findPossible(cv::Mat screen, cv::Mat src);                              //寻找所有可能的灯条（预处理）
	//==========================trackerBar==============================
	void threshBlueCallBk(int thresh);
	void threshRoiCallBk(int thresh);
public:
    //std::vector<cv::RotatedRect> all_list;                    //所有可能的1灯条容器
    std::vector<cv::RotatedRect> light_list;                    //成功匹配的灯条容器
    std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Rect> bounds;								//cv::Rect，灯条预选区
	int thresh_blue;											//trackerBar可调参数
	int thersh_red;												//trackerBar可调参数
	int thresh_roi;												//待选区trackerBar可调参数
private:
	cv::Mat roiProcess(cv::Mat src);							//二次处理预选区
	int getRealLight(cv::Mat, cv::Rect rect, cv::RotatedRect &outRect);   //从灯条预选区取出灯条最终框选区,返回0时表示查找成功
    static bool isRatioFit(cv::RotatedRect rect);               //判断灯条的长宽比是否合理
    static bool isAngleFit(cv::RotatedRect rect);                        //判断灯条的角度是否合理
	static double getCenterDist(cv::Rect src, cv::Rect dst);	//返回两个旋转矩形的中心距离
};	

GetLight::GetLight(){
    light_list.resize(16);
    light_list.clear();
    contours.resize(32);
    contours.clear();
	thresh_blue = THRESHOLD_BLUE;
	thersh_red = THRESHOLD_RED;
	thresh_roi = 205;
}

GetLight::~GetLight(){
    ;
}

void GetLight::showLight(cv::Mat &screen){
	//char str[4];
    cv::Point2f pts[4];
    for(int i=0;i<light_list.size();++i){
        light_list[i].points(pts);
        for(int j=0;j<4;++j){
            line(screen, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 255, 255), 2);
        }
		/*===============DEBUG,灯条编号显示=================
		snprintf(str, 20, "%d", i);
		cv::putText(screen, str, cv::Point(light_list[i].center.x + 15, 
			light_list[i].center.y + 30),
			cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0));
		=================DEBUG,灯条编号显示=================*/
    }
}

void GetLight::reset(){
    //all_list.clear();
    contours.clear();
	light_list.clear();
	bounds.clear();
}

void GetLight::imgProcess(cv::Mat src, cv::Mat &dst){
	//cv::blur(src, src, cv::Size(3, 3));
	std::vector<cv::Mat> channels(3);
	cv::split(src, channels);
	//if ENEMY IS RED
	cv::Mat res = channels.at(0) - channels.at(2);
	cv::threshold(res, dst, thresh_blue, 255, cv::THRESH_BINARY);
	cv::dilate(dst, dst, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4)), cv::Point(-1, -1), 1);
	cv::erode(dst, dst, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
}

int GetLight::findPossible(cv::Mat screen, cv::Mat src){           //入参应该是从imgProcess来的二值图
    cv::findContours(src, contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    if(!contours.size()){
        return 1;
    }
	for (int i = 0; i < contours.size(); ++i) {								//按层级数查找
        float c_area=cv::contourArea(contours[i]);
        if(c_area > 5){													//轮廓面积要大于16
			mergeRect(cv::boundingRect(contours[i]));												//合并最终匹配灯条
        }
    }
	for (int i = 0; i < bounds.size(); ++i) {
		cv::RotatedRect light;
		getRealLight(screen, bounds[i], light);						//没有找到合适的内轮廓
		if (!isRatioFit(light) || !isAngleFit(light)) continue;
		light_list.push_back(light);
	}
	return 0;
}

void GetLight::brightAdjust(cv::Mat &src, double alpha, double beta) {
	int height = src.rows;
	int width = src.cols;
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			//获取RGB各通道像素值
			float b = src.at<cv::Vec3b>(row, col)[0];
			float g = src.at<cv::Vec3b>(row, col)[1];
			float r = src.at<cv::Vec3b>(row, col)[2];
			//对各通道像素值进行计算，实现对比度和亮度变换
			src.at<cv::Vec3b>(row, col)[0] = cv::saturate_cast<uchar>(b*alpha + beta);
			src.at<cv::Vec3b>(row, col)[1] = cv::saturate_cast<uchar>(g*alpha + beta);
			src.at<cv::Vec3b>(row, col)[2] = cv::saturate_cast<uchar>(r*alpha + beta);
		}
	}
}

void GetLight::mergeRect(cv::Rect rect) {			//如果有临近矩形，则合并,没有则在list中新增
	for (int i = 0; i < bounds.size(); ++i) {
		if (getCenterDist(rect, bounds[i]) <= 
			((rect.width+bounds[i].width)*1.25*(rect.width+bounds[i].width)*1.25)) {	//返回的是未开方的值
			std::vector<cv::Point> pointList = std::vector<cv::Point>{
				cv::Point(rect.x , rect.y),
				cv::Point(rect.x + rect.width, rect.y + rect.height),
				cv::Point(bounds[i].x ,bounds[i].y),
					cv::Point(bounds[i].x + bounds[i].width,
					bounds[i].y + bounds[i].height),
			};
			bounds[i] = cv::boundingRect(pointList);
			if (bounds[i].x + bounds[i].width > 720) bounds[i].width = 720 - bounds[i].x;
			else if (bounds[i].x < 0) bounds[i].x = 0;
			if (bounds[i].y + bounds[i].height > 540) bounds[i].height = 540 - bounds[i].y;
			else if (bounds[i].y < 0) bounds[i].y = 0;
			return;
		}
	}
	bounds.push_back(rect);
}

int GetLight::getRealLight(cv::Mat src, cv::Rect rect, cv::RotatedRect &outRect) {
	std::vector<std::vector<cv::Point>> cts(8); cts.clear();
	cv::Mat tmp = roiProcess(src(rect));
	cv::findContours(tmp, cts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	float max_area = 16, tmp_area = 0.0;
	int max_pos = -1;
	for (int i = 0; i < cts.size(); ++i) {
		tmp_area = cv::contourArea(cts[i]);
		if (tmp_area > max_area) {
			max_pos = i;
			max_area = tmp_area;
		}
	}
	if (max_pos != -1) {
		outRect = cv::minAreaRect(cts[max_pos]);
		outRect.center.x += rect.x;
		outRect.center.y += rect.y;
		return 0;
	}
	outRect = cv::RotatedRect(cv::Point2f(rect.x + rect.width / 2, rect.y + rect.height / 2),
		cv::Size(rect.width, rect.height), 0.0);
	outRect.size.width > outRect.size.height ? outRect.size.height /= 1.2 : outRect.size.width /= 1.2;
	return 1;
}

cv::Mat GetLight::roiProcess(cv::Mat src) {
	cv::Mat res;
	cv::cvtColor(src, res, cv::COLOR_BGR2GRAY);
	cv::threshold(res, res, thresh_roi, 255, cv::THRESH_BINARY);
	cv::dilate(res, res, cv::getStructuringElement(0, cv::Size(3, 3)), cv::Point(-1, -1), 3);
	cv::erode(res, res, cv::getStructuringElement(0, cv::Size(3, 3)));
	return res;
}

//=======================灯条框选条件bool值函数============================
bool GetLight::isRatioFit(cv::RotatedRect rect){
    float ratio=rect.size.width>rect.size.height? rect.size.width/rect.size.height :
        rect.size.height/rect.size.width;
    return (ratio>=1 && ratio <10);
}

bool GetLight::isAngleFit(cv::RotatedRect rect){
	//std::cout << "rect.angle:" << rect.angle << std::endl;
	return rect.size.width > rect.size.height ? rect.angle< -60 : (rect.angle - 90) < -60;
    //return abs(angle)<30 || abs(angle)>60;
}

double GetLight::getCenterDist(cv::Rect src, cv::Rect dst) {
	return (src.x - dst.x)*(src.x - dst.x) + (src.y - dst.y)*(src.y - dst.y);
}

//=============================TRACKERBAR deBug函数================================
void GetLight::threshBlueCallBk(int thresh) {
	thresh_blue = thresh;
}

void GetLight::threshRoiCallBk(int thresh) {
	thresh_roi = thresh;
}

