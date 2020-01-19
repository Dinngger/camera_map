/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
实现者：雷达组-zlq
代码思路：雷达组-dxd
修改：雷达组-dxd
封装：hqy
last date of modification:2020.1.13 19:58
新的思路：两个threshold阈值，达到一张图像同时处理成高低曝光度的效果
*/

#ifndef __ARMOR_PLATE_HPP
#define __ARMOR_PLATE_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#define PARAM1 4
#define PARAM2 1.5

class ArmorPlate{
public:
    ArmorPlate();
    ~ArmorPlate();
	void findPossible(cv::Mat src);									//找到图上所有可能的灯条
	void drawArmorPlate(cv::Mat &src);								//绘制所有的装甲板
	void getArmorRect(std::vector<cv::RotatedRect> &result);
	void preProcess(cv::Mat src, cv::Mat &high_exp, cv::Mat &low_exp);	//处理成高低阈值两个图像
	bool lowExposure(cv::Mat src, cv::Mat &gray);					//图像预处理
	static float getDistance(cv::Point p1, cv::Point p2);			//图像两点间距离
public:
	bool low_exposure;												//曝光率设置flag
private:
	void reset();													//重置
	void getRealLight(int size);									//从预选灯条中找到合适的最终灯条	
	void getTrapezoids(cv::Point2f corners[4]);						//取出灯条拓展梯形
	static bool isInTrapezoid(cv::Point2f corners[4], 
		const std::vector<cv::Point2f> &trapezoid);					//点集是否能被梯形包围
	cv::RotatedRect getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2);	//取得两个匹配灯条对应的装甲板
private:	
	float mean_val;													//均值
	std::vector<cv::RotatedRect> light_list;						//灯条集合
	std::vector<std::vector<cv::Point2f>> trapezoids;				//灯条梯形集合
	std::vector<cv::RotatedRect> possibles;							//预选区集合 			
};

#endif // __ARMOR_PLATE_HPP
