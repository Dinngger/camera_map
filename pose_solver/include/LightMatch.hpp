/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
实现者：雷达组-zlq
代码思路：雷达组-dxd
修改：雷达组-dxd
封装：hqy
last date of modification:2020.4.7
*/

#ifndef _LIGHT_MATCH_HPP
#define _LIGHT_MATCH_HPP

// #define LIGHT_CNT_TIME				//不需要计算灯条处理时间，则注释此行

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
	void setEnemyColor(const bool _enemy_blue = false);					//重新设置敌人的颜色
	void findPossible();												//找到图上所有可能的灯条						
	void drawLights(cv::Mat &src);										//绘制灯条
	void saveImg(cv::Mat src) {											//图像预处理
		cv::split(src, proced);
	}
public:															
	bool enemy_blue;													//敌人是否是蓝色的
	bool low_exposure;													//曝光率设置flag
	std::vector<aim_deps::Light> possibles;								//预选区集合
	//=================颜色参数==========================//		 
	int thresh_low;														//二值化阈值
	int thresh_high;													//二值化阈值
	//==================================================//				
																
	//灯条匹配装甲板函数将会使用此配对信息，以及灯条来匹配装甲板							
	std::vector<cv::Point> matches;										//会配对的数组
private:														
	void reset();														//重置
	/**
	 * @brief 二次阈值化（对于反光严重的位置仍然效果不好）
	 * @param index 当前灯条的下标
	 * @param area 一次阈值化时对应的轮廓面积
	 * @param ct 一次阈值化时对应的轮廓
	 * @param bin 如果需要把二次阈值化的图像绘制在原二值图上，需要此参数
	 */
	void doubleThresh(int &index, const float area, std::vector<cv::Point> ct, cv::Mat &bin);	//局部阈值化

	/**
	 * @brief 灯条预匹配
	 * @param size 灯条预选区的大小
	 * @param vexes corners[4]的集合，之前算过一次，无需再算一次
	 */
	void getRealLight(const int size, 
			const std::vector<cv::Point2f *> vexes);					//找到预选灯条中实际的灯条

	/**
	 * @brief 阈值化
	 * @param dst 输出单通道阈值化后的二值图,尚不明确src = dst会发生什么，尽量避免
	 * @param thresh 最低阈值
	 * @param diff_thresh b-r或者r-b的最小阈值
	 * @note 此阈值化结合了颜色过滤以及阈值化
	 */
	void threshold(cv::Mat &dst, const int thresh, const int diff_thresh = 15);

	void getTrapezoids(const cv::Point2f corners[2]);					//取出灯条拓展梯形
	static bool isInTrapezoid(cv::Point2f corners[2], 			
			const std::vector<cv::Point2f> &trapezoid);					//点集是否能被梯形包围
	inline static int getDoubleThresh(const float _a);				
	inline static float getDistance(const cv::Point p1, const cv::Point p2);//图像两点间距离
private:
	#ifdef LIGHT_CNT_TIME														
		double time_sum;
		int _cnt;
	#endif // LIGHT_CNT_TIME
	float mean_val;														//均值
	cv::Mat proced[3];												
	std::vector<std::vector<cv::Point2f>> trapezoids;					//灯条梯形集合
};

#endif //_LIGHT_MATCH_HPP
