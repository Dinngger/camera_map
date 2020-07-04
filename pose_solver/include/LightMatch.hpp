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

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <algorithm>
#include "AimDeps.hpp"
#include "LOG.hpp"

//#define DRAW_CONTOUR					//无需绘制灯条角度优化使用的轮廓点时注释此行
#define MULTI_THREAD					//非异步调试时请注释此行
//#define LIGHT_CNT_TIME				//不需要计算灯条处理时间，则注释此行
//#define LIGHT_MATCH_DEBUG			//无需输出调试信息则注释此行
#ifdef LIGHT_MATCH_DEBUG
	#define match_debug rmlog::LOG::printc
#else
	#define match_debug(...)
#endif //LIGHT_MATCH_DEBUG

class LightMatch{
public:
    LightMatch();
    ~LightMatch();
	void setEnemyColor(const bool _enemy_blue = false);					//重新设置敌人的颜色
	void findPossible();												//找到图上所有可能的灯条						
	void drawLights(cv::Mat &src) const;								//绘制灯条
	void saveImg(const cv::Mat& src){									//图像预处理
		cv::split(src, proced);
	}
public:															
	bool enemy_blue;													//敌人是否是蓝色的
	std::vector<aim_deps::Light> possibles;								//预选区集合
	//=================颜色参数==========================//		 
	int thresh_low;														//二值化阈值

	//==================================================//				
																
	//灯条匹配装甲板函数将会使用此配对信息，以及灯条来匹配装甲板							
	std::vector<cv::Point> matches;										//会配对的数组
private:							
	///线程函数	
	void contourProcess(const std::vector<std::vector<cv::Point> >& ct, int start, int step);

	void reset();														//重置
	/**
	 * @brief 二次阈值化（对于反光严重的位置仍然效果不好）
	 * @param index 当前灯条的下标
	 * @param ct 一次阈值化时对应的轮廓
	 * @param bbox 一次阈值化的bounding box
	 * @param valid 灯条是否有效
	 */
	void doubleThresh(const std::vector<cv::Point>& ct, cv::Rect& bbox, bool valid);	//局部阈值化

	/**
	 * @brief 灯条预匹配
	 * @param size 灯条预选区的大小
	 */
	void getRealLight(const int size);					//找到预选灯条中实际的灯条

	/**
	 * @brief 阈值化
	 * @param dst 输出单通道阈值化后的二值图,尚不明确src = dst会发生什么，尽量避免
	 * @param thresh 最低阈值
	 * @param diff_thresh b-r或者r-b的最小阈值
	 * @note 此阈值化结合了颜色过滤以及阈值化
	 */
	void threshold(cv::Mat &dst, int thresh, int diff_thresh = 20) const;

	void getTrapezoids(const cv::Point2f corners[2]);					//取出灯条拓展梯形

	/**
	 * @brief 角度优化另取一个contour
	 * @param rec 选区boundingBox
	 * @param ct 输入/输出 找到的方向轮廓点，用于优化角度
	 * @param is_small 是否是很小的灯条(默认是正常大小)
	 * @return 是否有符合要求的方向轮廓集合(yes > true)
	 */
	bool getDirection(const cv::Rect &rec, std::vector<cv::Point> &ct, bool is_small = false);

	/**
	 * @brief 给面积较大灯条使用的角度优化边缘提取 
	 * @param src 输入的图像
	 * @param ct 输入/输出 用于角度优化的轮廓点集合
	 * @return 是否找到符合要求的轮廓
	 */
	bool getBigDirection(const cv::Mat &src, std::vector<cv::Point> &ct) const;

	static bool isInTrapezoid(cv::Point2f corners[2], 			
			const std::vector<cv::Point2f> &trapezoid);					//点集是否能被梯形包围

	inline static bool isAngleValid(const aim_deps::LightBox &lb);		//角度是否合适(过滤杂灯条)
			
	inline static int getDoubleThresh(const float _a);					//局部阈值化的阈值函数		

	/**
	 * @brief 扩大一个ROI
	 * @return 如果扩大成功，则返回true, 反之扩大失败（说明选框在边界附近，可能导致亚像素检测失效）
	*/ 
	inline static bool extendRect(cv::Rect &rect, const cv::Size sz = cv::Size(6, 11));

	/// ==========================灯条优化========================== ///

	/**
	 * @brief 灯条角度优化(最小二乘, 让轮廓点到灯条的距离平方最小)
	 * @param contour 灯条轮廓点(根据轮廓点进行优化)
	 * @param l 灯条
	 * @param offset 因为轮廓提取根据ROI，丢失了在原图中的位置信息，需要外加位置信息
	 * @param contain 包络矩形的点集(轮廓点落在包络内才会被计算误差)
	 * @param weaken_coeff 对角度优化结果进行削弱(angle * weaken_coeff) 默认为1.0
	 */
	void readjustAngle(const std::vector<cv::Point>& contour, aim_deps::Light &l,
			cv::Point offset, double weaken_coeff = 1.0) const;

	/** 
	 * @brief 计算误差函数对修正角度的导数
	 * @param _vec 灯条线段的法向量
	 * @param ctr 灯条中点
	 * @param p 对应轮廓点
	 * @return d(总误差平方/2)/d(angle) 
	 */
	inline static double calcDiff(
		const cv::Point2f &_vec,
		const cv::Point2f &ctr,
		const cv::Point &p);

	/** @brief 计算二阶导数  */
	inline static double calcDiff2(
		const cv::Point2f &_vec,
		const cv::Point2f &ctr,
		const cv::Point &p);

	inline static double safeCast(double angle){
		while(angle < -2 * CV_PI){
			angle += 2 * CV_PI;
		}
		while(angle > 2 * CV_PI){
			angle -= 2 * CV_PI;
		}
		return angle;
	}

	///============================亚像素检测============================///
	/// 将cv::Point 轮廓转化为cv::Point2f 轮廓
	static void convertVector(const std::vector<cv::Point>& src, std::vector<cv::Point2f> &dst);

	///自适应确定亚像素检测的窗口大小
	static cv::Size decideSize(const cv::RotatedRect &rect);			

	/// std::sort使用的函数
	static bool sizeCmp(const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2){
		return c1.size() > c2.size();
	}

private:
	#ifdef LIGHT_CNT_TIME														
		double time_sum;
		int cnt;
	#endif // LIGHT_CNT_TIME
	std::mutex mtx;														// 锁
	cv::Mat proced[3];												
	std::vector<std::vector<cv::Point2f> > trapezoids;					//灯条梯形集合
public:
#ifdef DRAW_CONTOUR
	std::vector<std::vector<cv::Point> > cts_todraw;
	void drawContour(cv::Mat &src);
#endif 	// DRAW_CONTOUR
};
#endif //_LIGHT_MATCH_HPP
