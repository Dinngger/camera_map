/**==========pnp测距测试模块，基于opencv solvePNP的iterative算法==============
 * @author: sentinel
 * last date of modification: 2020.3.2
 * 最后修改的内容：删除了几个没有用的函数和变量
*/

#ifndef _GET_POS_HPP
#define _GET_POS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "GimbalCtrl.hpp"
#include "AimDeps.hpp"

#define _HALF_LENGTH_SMALL	65.00
#define _HALF_LENGTH_BIG 	105.00
#define _HALF_HEIGHT		28.50

class GetPos{
public:
	GetPos();
	~GetPos();

	/**
	 * @brief 将距离解算与弹道模型，角度预测分开进行
	 * @param tar 最终的预选装甲板
	 * @param pitch 入参/输出 弹道模型计算的pitch偏移
	 * @param yaw 入参/输出 弹道模型计算的yaw偏移
	 * @param delay 射击子弹飞行延迟
	 */
	void calcBallistic(const aim_deps::Armor tar, float &pitch, float &yaw, float &delay);	
	//ratio:角度缩小控制率
	//比如角度过大，在发一次信息过程中转不完计算的角度，可以认为缩小每次发送的角度
	void batchProcess(std::vector<aim_deps::Armor> &tar_list);	//批量solvePNP（但是不解算弹道）
	void positionScore(aim_deps::Armor &tar);		//距离分数和旋转分数计算	

	/**
	 * @brief 从tar_list中获取rMats与tMats
	 * @param rmats rMats
	 * @param tmats tMats
	 * @param tar_list 通过其中的Armor的属性解出旋转矩阵以及取出平移向量
	 */
	void packUp(std::vector<cv::Mat> &rmats, std::vector<cv::Mat> &tmats, 
		const std::vector<aim_deps::Armor>& tar_list);		
public:
	cv::Mat tVec;									//平移矩阵(x, y, z)
private:
private:
	ballistic::GimbalCtrl g_ctrl;					//云台控制
	std::vector<cv::Point3f> objPoints_small;				//世界坐标系下的点（装甲板实际4点位置）
	std::vector<cv::Point3f> objPoints_big;				//世界坐标系下的点（装甲板实际4点位置）
	std::vector<cv::Point3f> intrinsic;				//(内参矩阵)
	std::vector<float> distCoeffs;					//畸变与切变向量（4参数）
	cv::Mat rVec;									//旋转向量
	cv::Mat insM;									//内参矩阵Mat形式
};

#endif //_GET_POS_HPP
