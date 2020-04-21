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
// #define DRAW_CONTOUR
// #define LIGHT_CNT_TIME				//不需要计算灯条处理时间，则注释此行

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "AimDeps.hpp"
#include "LOG.hpp"

// #define LIGHT_MATCH_DEBUG
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
	void drawLights(cv::Mat &src);										//绘制灯条
	void saveImg(cv::Mat src){											//图像预处理
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
	void threshold(cv::Mat &dst, const int thresh, const int diff_thresh = 5);

	void getTrapezoids(const cv::Point2f corners[2]);					//取出灯条拓展梯形


	static bool isInTrapezoid(cv::Point2f corners[2], 			
			const std::vector<cv::Point2f> &trapezoid);					//点集是否能被梯形包围
			
	inline static int getDoubleThresh(const float _a);				

	/// @brief 扩大一个ROI
	/// @return 如果扩大成功，则返回true, 反之扩大失败（说明选框在边界附近，可能导致亚像素检测失效）
	inline static bool extendRect(cv::Rect &rect, const int pix = 7);

	/// ==========================灯条优化========================== ///
	void readjustAngle(std::vector<cv::Point> contour, aim_deps::Light &l,
				cv::Point offset, std::vector<cv::Point2f> *contain = nullptr);

	/// 将cv::Point 轮廓转化为cv::Point2f 轮廓
	static void convertVector(std::vector<cv::Point> src, std::vector<cv::Point2f> &dst);

	/** 
	 * @brief 提取灯条方向包络,使用很高的阈值，获取真实方向上的一个多边形顶点列表
	 * @note 使用这个多边形列表，在readjustAngle中，判断轮廓点是否在多边形中，如果在则可以用此点优化
	 * @param src 阈值化后的图像
	 * @param vexes 输入，输出，旋转矩形的顶点vector
	 * @return 是否有有效的多边形列表输出
	 */
	static bool extractDirect(cv::Mat src, std::vector<cv::Point2f> &vexes);

	/** @brief 计算轮廓点到线段的距离
	 * @param _vec 线段的单位化法向量
	 * @param ctr 线段中心（定点）
	 * @param p 轮廓点
	 * @return 总误差的平方/2
	 */
	inline static double calcError(cv::Point2f _vec, cv::Point2f ctr, cv::Point p);

	/** @brief 计算误差函数对修正角度的导数
	 * @param @ref calcError
	 * @return d(总误差平方/2)/d(angle) 
	 */
	inline static double calcDiff(cv::Point2f _vec, cv::Point2f ctr, cv::Point p);

	/// @brief 计算二阶导数
	inline static double calcDiff2(cv::Point2f _vec, cv::Point2f ctr, cv::Point p);

private:
	#ifdef LIGHT_CNT_TIME														
		double time_sum;
		int _cnt;
	#endif // LIGHT_CNT_TIME
	float mean_val;														//均值
	cv::Mat proced[3];												
	std::vector<std::vector<cv::Point2f>> trapezoids;					//灯条梯形集合
#ifdef DRAW_CONTOUR
public:
	std::vector<std::vector<cv::Point> > cts_todraw;
	std::vector<std::vector<bool> >	cts_valids;
	void drawContour(cv::Mat &src);
#endif 	// DRAW_CONTOUR
};
#endif //_LIGHT_MATCH_HPP
