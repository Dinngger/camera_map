/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
实现者：雷达组-zlq
代码思路：雷达组-dxd
修改：雷达组-dxd
封装：hqy
last date of modification:2020.1.16 22:30
==============程序的输出有所改变================
原程序：一个vector，包含所有配对的灯条（成对地储存，会重复）
更改后：一个vector保存所有灯条，另一个vector<cv::Point>保存匹配信息：
	Point.x是第一个灯条的索引，Point.y是第二个灯条的索引

目前存在的问题：
	1.远距离不够稳定，灯条过小造成
	2.帧率问题
*/

#ifndef _LIGHT_MATCH_HPP
#define _LIGHT_MATCH_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "AimDeps.hpp"

// #define LIGHT_MATCH_DEBUG

class LightMatch{
public:
    LightMatch();
    ~LightMatch();
	void setEnemyColor(const bool _enemy_blue = false);			//重新设置敌人的颜色
	void findPossible(const float _a = 0.0);										//找到图上所有可能的灯条						
	void drawLights(cv::Mat &src);								//绘制灯条
	bool saveImg(cv::Mat src);									//图像预处理
public:															
	bool enemy_blue;											//敌人是否是蓝色的
	bool low_exposure;											//曝光率设置flag
	std::vector<aim_deps::Light> possibles;								//预选区集合
	//=================颜色参数==========================//		 
	int thresh_low;												//二值化阈值
	int thresh_high;											//二值化阈值
	//==================================================//				
																
	//灯条匹配装甲板函数将会使用此配对信息，以及灯条来匹配装甲板							
	std::vector<cv::Point> matches;								//会配对的数组
private:														
	void reset();												//重置
	void getRealLight(int size);								//找到预选灯条中实际的灯条
	void getTrapezoids(cv::Point2f corners[4]);					//取出灯条拓展梯形
	static bool isInTrapezoid(cv::Point2f corners[4], 			
		const std::vector<cv::Point2f> &trapezoid);				//点集是否能被梯形包围
	static float getDistance(cv::Point p1, cv::Point p2);		//图像两点间距离
	inline static int getThreshold(const float _a);			//通过灯条平均面积计算二值化阈值
private:														
	float mean_val;												//均值
	cv::Mat proced;												
	std::vector<std::vector<cv::Point2f>> trapezoids;			//灯条梯形集合
};

LightMatch::LightMatch(){
	low_exposure 	= false;
	mean_val 		= aim_deps::LIGHT_mean;
	setEnemyColor(true);											//初始设置敌人颜色
}

LightMatch::~LightMatch(){
	;
}

void LightMatch::setEnemyColor(const bool _enemy_blue){
	enemy_blue = _enemy_blue;
	if(enemy_blue){								//设置颜色阈值
		thresh_low = aim_deps::light_params.blue_thresh_low;
		thresh_high = aim_deps::light_params.blue_thresh_high;	
	}
	else{
		thresh_low = aim_deps::light_params.red_thresh_low;
		thresh_high = aim_deps::light_params.red_thresh_high;
	}
}

void LightMatch::reset(){
	matches.clear();
	possibles.clear();
	trapezoids.clear();
}

void LightMatch::findPossible(const float _a){								//找出所有可能灯条，使用梯形匹配找出相匹配的灯条对
	reset();
	cv::Mat binary;
	thresh_low = getThreshold(_a);
	cv::threshold(proced, binary, thresh_low, thresh_high, cv::THRESH_BINARY);		///THRESHOLD修改
	#ifdef LIGHT_MATCH_DEBUG
		cv::imshow("binary", binary);
	#endif
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);	//寻找图上轮廓

	for (size_t i = 0, _cnt = 0; i < contours.size(); ++i) {
		double area = cv::contourArea(contours[i]);
		if (area) {						//小于3*3像素的不被考虑	
			cv::RotatedRect light = cv::minAreaRect(contours[i]);
			possibles.emplace_back(aim_deps::Light(_cnt, light));
			++_cnt;
		}
	}
	for (size_t i = 0; i < possibles.size(); ++i) {
		cv::Point2f corners[4];
		possibles[i].box.points(corners);
		getTrapezoids(corners);								
	}
	getRealLight(possibles.size());
}

bool LightMatch::saveImg(cv::Mat src){
	cv::Mat channels[3];
	cv::split(src, channels);
	if(enemy_blue) proced = channels[0];						//取出蓝色通道
	else proced = channels[2];									//取出红色通道
	return true;
}

void LightMatch::getRealLight(int size){
	bool flag[size][size];					//两灯条是否满足要求,是个对称矩阵，当[i][j],[j][i]为真时，两灯条匹配
	for (int i = 0; i < size; ++i) {		//初始化		
        for(int j = 0; j < size; ++j)
		    flag[i][j] = false;
	}
	for (int i = 0; i < size; ++i) {		//梯形包含匹配
		cv::Point2f corners[4];
		possibles[i].box.points(corners);
		for (int j = 0; j<size*2; ++j) {
			if (isInTrapezoid(corners, trapezoids[j])) {
				flag[i][j/2] = true;
			}
		}
	}
	for (int i = 0; i<size; ++i) {						//将可能的匹配结果放入容器matches中
		for (int j = i+1; j<size; ++j) {
			if (flag[i][j] == true && flag[j][i] == true) {
				flag[i][j] = false;
				matches.emplace_back(cv::Point(i, j));		//最后只保存配对的信息，灯条只保存一次
			}
		}
	}
	//std::cout<<"Result :"<<matches<<std::endl;
}

//匹配的灯条其梯形将会互相包含
void LightMatch::getTrapezoids(cv::Point2f corners[4]){
	std::vector<cv::Point2f> left_trapezoid;
	std::vector<cv::Point2f> right_trapezoid;
	double d1 = getDistance(corners[0], corners[1]);
	double d2 = getDistance(corners[1], corners[2]);
	// cv::Point2f trapezoid_points[4];								//梯形角点
	cv::Point2f direction_vectors[2];								//方向向量
	cv::Point2f midpoints[2];										//中点
	int i0 = d1<d2? 1:0;										//长所在边第一个顶点的位置
	midpoints[0] = (corners[i0] + corners[i0 + 1]) / 2;			//获得旋转矩形两条长上的中点
	midpoints[1] = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
	cv::Point2f vertical_vector = (midpoints[1]) - (midpoints[0]);	//获得垂直长方向的方向向量
	vertical_vector = (i0 == 0 ? d1 : d2) * aim_deps::LIGHT_PARAM1 * vertical_vector /
		sqrt(vertical_vector.x*vertical_vector.x + vertical_vector.y*vertical_vector.y);
	direction_vectors[0] = corners[i0 + 1] - corners[i0];
	direction_vectors[1] = corners[(i0 + 3) % 4] - corners[i0 + 2];
	left_trapezoid.emplace_back(corners[i0]);
	left_trapezoid.emplace_back(corners[i0 + 1]);
	left_trapezoid.emplace_back(midpoints[0] - vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vectors[0]);
	left_trapezoid.emplace_back(midpoints[0] - vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vectors[0]);
	right_trapezoid.emplace_back(corners[i0 + 2]);
	right_trapezoid.emplace_back(corners[(i0 + 3) % 4]);
	right_trapezoid.emplace_back(midpoints[1] + vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vectors[1]);
	right_trapezoid.emplace_back(midpoints[1] + vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vectors[1]);
	trapezoids.emplace_back(left_trapezoid);						//灯条左右两边将会拓展出两个梯形
	trapezoids.emplace_back(right_trapezoid);
}

void LightMatch::drawLights(cv::Mat &src){
	cv::Point2f pts[4];
	char str[4];
	for(size_t i=0; i<possibles.size(); ++i){
		possibles[i].box.points(pts);
		for(int j=0; j<4;++j){
			cv::line(src, pts[j], pts[(j+1)%4], cv::Scalar(0, 0, 255));
			//snprintf(str, 4, "%d", j);
			//cv::putText(src, str, pts[j]+cv::Point2f(1,1), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
		}
		snprintf(str, 4, "%lu", i);
		cv::putText(src, str, pts[3], cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
	}
}

bool LightMatch::isInTrapezoid(cv::Point2f corners[4], const std::vector<cv::Point2f> &trapezoid){
	int count = 0;
	for (int i = 0; i<4; ++i) {
		if (cv::pointPolygonTest(trapezoid, corners[i], false) >= 0)
			++count;	}
	if (count == 4) {
		return true;
	}
	return false;
}

float LightMatch::getDistance(cv::Point p1, cv::Point p2){
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	distance = sqrtf(distance);
	return distance;
}

int LightMatch::getThreshold(const float _a){
	if(_a >= 150.0) return 135;
	else if(_a == 0.0) return 120;				//为0一般就是上一帧没有找到装甲板
	else if(_a <= 25.0) return 115;
	int res = (int)(-6.48011782e-04 * _a * _a + 2.76877761e-01 * _a + 1.08195876e+02);	//一个拟合函数
	return res;
}

#endif //_LIGHT_MATCH_HPP
