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

#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "ErrorTerm.hpp"
#include "AimDeps.hpp"
#include "LOG.hpp"

#define LIGHT_CNT_TIME			// LightMatch   灯条提取模块         计算灯条处理时间
//#define LIGHT_MATCH_DEBUG		// LightMatch   灯条提取模块         灯条调试信息输出

class LightMatch{
using PtrPair = std::pair<const aim_deps::Light*, const aim_deps::Light*>;
public:
    LightMatch();
    ~LightMatch();
	void setEnemyColor(bool _enemy_blue, int thresh_low, int ch_diff, int filter);					//重新设置敌人的颜色

	void findPossible();												//找到图上所有可能的灯条						
	void drawLights(cv::Mat &src, char belongs[]) const;								//绘制灯条
	void saveImg(const cv::Mat& src){									//图像预处理
		cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		cv::split(src, proced);
	}
public:															
	int thresh_low;														//二值化阈值
	bool enemy_blue;													//敌人是否是蓝色的
																
	//灯条匹配装甲板函数将会使用此配对信息，以及灯条来匹配装甲板							
	std::vector<cv::Point> matches;										//会配对的数组
	std::vector<aim_deps::Light> possibles;								//预选区集合
private:	
	void reset() {
		matches.clear();
		possibles.clear();
		trapezoids.clear();
	}

	/**
	 * @brief 线程函数，多线程对找到的粗灯条轮廓进行细化处理(3线程并行) 
	 * @param ct 粗轮廓点集合
	 * @param start 从下标start开始
	 * @param step 间隔step个轮廓处理一次
	 * @param ptr 假设不为空指针 就计算环境亮度的表达式(灯条匹配框面积)
	 */
	void contourProcess(const std::vector<std::vector<cv::Point> >& ct, std::vector<aim_deps::Light>& lts);

	/**
	 * @brief 二次阈值化
	 * @param bbox 一次阈值化的bounding box
	 * @return 是否使用二次阈值的方式查找到灯条
	 */
	bool doubleThresh(cv::Rect& bbox, std::vector<aim_deps::Light>& lts);

	/**
	 * @brief 灯条预匹配
	 * @param size 灯条预选区的大小
	 */
	void getRealLight(std::vector<aim_deps::Light>& lts, std::vector<PtrPair>& prs);

	/**
	 * @brief 阈值化
	 * @param dst 输出单通道阈值化后的二值图,尚不明确src = dst会发生什么，尽量避免
	 * @note 此阈值化结合了颜色过滤以及阈值化
	 */
	void threshold(cv::Mat &dst, int diff_thresh = 20) const;

	/// @brief 获取灯条扩展梯形
	void getTrapezoids(const cv::Point2f corners[2]);

	/**
	 * @brief 获取大灯条的方向信息
	 * @param src 图像(ROI)
	 * @param ct 灯条边缘轮廓点
	 */
	bool getBigDirection(const cv::Mat &src, std::vector<cv::Point> &ct) const;

	/**
	 * @brief 灯条角度优化(最小二乘, 让轮廓点到灯条的距离平方最小)
	 * @param contour 灯条轮廓点(根据轮廓点进行优化)
	 * @param l 灯条
	 * @param offset 因为轮廓提取根据ROI，丢失了在原图中的位置信息，需要外加位置信息
	 * @param weaken_coeff 对角度优化结果进行削弱(angle * weaken_coeff) 默认为1.0
	 */
	void readjustAngle(const std::vector<cv::Point>& contour, aim_deps::Light &l,
			cv::Point offset, double weaken_coeff = 1.0) const;

	/**
	 * @brief 灯条扩散优化模型
	 * @param src 输入的图像ROI
	 * @param top [2] 初始值: 灯条的粗略上顶点
	 * @param ctr [3] [0,1]: 灯条粗略的中心 [2] 扩散量的变化
	 * @param radius 灯条向外发光扩散的距离
	 */
	bool lightDiffusion(cv::Mat& src, double* top, double* ctr, double radius);

	/**
	 * @brief 预处理ROI
	 * @param dst 预处理后的图像
	 * @param pts 输出的normalized图像点
	 */
    void readAndConvert(cv::Mat& dst, std::vector<double>& pts) const;

	/**
	 * @brief 对灯条做初始值估计
	 * @note 见 lightDiffusion
	 */
    bool betterInitialize(const cv::Mat& src, double* _top, double* _ctr) const;

	static bool isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid);
	/**
	 * @brief 扩大一个ROI
	 * @return 如果扩大成功，则返回true, 反之扩大失败（说明选框在边界附近，可能导致亚像素检测失效）
	*/ 
	inline static void extendRect(cv::Rect &rect, const cv::Size sz = cv::Size(6, 11));

	/// 判断bbox 是否合格：高 - 宽是否异常
	/// 当bbox越小时 高 - 宽的差阈值越小（噪声很容易对小bbox造成影响）
	inline static bool isGoodBoundingBox(const cv::Rect &rect);
	inline static bool isAngleValid(const aim_deps::LightBox &lb);
	inline static double calcDiff(const cv::Point2f &_vec, const cv::Point2f &ctr, const cv::Point2f &p);
	inline static double calcDiff2(const cv::Point2f &_vec, const cv::Point2f &ctr, const cv::Point2f &p);
	inline static double safeCast(double angle);
private:
	#ifdef LIGHT_CNT_TIME														
		double time_sum;
		int cnt;
	#endif // LIGHT_CNT_TIME
	std::mutex mtx;														// 锁
	cv::Mat proced[3];								
	cv::Mat gray;														// 灰度图
	std::vector<std::vector<cv::Point2f> > trapezoids;					//灯条梯形集合

	//=================与颜色有关的阈值设置量========================//
	int filter_thresh;					// 过滤阈值
	int chan_diff;						// 主通道与绿色通道的差异最小允许值

	ceres::Solver::Options opts;
};
#endif //_LIGHT_MATCH_HPP
