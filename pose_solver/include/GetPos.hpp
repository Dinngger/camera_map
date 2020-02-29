/**==========pnp测距测试模块，基于opencv solvePNP的iterative算法==============
 * @author: sentinel
 * last date of modification: 2020.1.13
 * problems:测距问题，z轴测距个人怀疑不准
 * 已知Problems：测距：z轴近距离时很准，远距离距离损失严重
 * 可以改进的地方：GetPos发送的角度，可以使用PID算法来减小超调
 *	1.虽说PID算法在电控处控制云台转动中使用到了，但个人认为视觉端也可以进行角度调节方面的优化
 *	但是存在的一个疑问就是：不清楚发送接收频率，PID也不是很明白该怎么用
 *	2.主要的改进方向：
 *		1.添加预测模块，先根据帧追踪确定装甲板的速度和加速度，进而确定pitch，yaw需要补偿的角度
 *		2.
 *	3.有关速度，角速度历史值推定的思路（注意所有的pitch,yaw都是增量式）
 *		所有的储存值都相当于是瞬时期望角速度（一帧内期望转的角速度）
 *		而old_x与now_x的差值相当于实际角速度，比如：
 *			1.在第k帧时需要转a1角度，而在k+1帧时需要转a2角度：
 *				若目标静止，则（a1-a2）为实际转过的角度
 *				若目标移动，则（a1-a2）包含了角度度以及装甲板速度两个信息
 *			2.实际计算的是期望以及期望的变化规律
 *				可以考虑RBF内核的SVM来处理这个问题
 *				虽然我担心的是时间问题,以后可以考虑并行计算优化
*/

#ifndef _GET_POS_HPP
#define _GET_POS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "GimbalCtrl.hpp"
#include "AimDeps.cc"
#define _HALF_LENGTH 65.00			
#define _HALF_HEIGHT 28.50			
#define PI 3.1415926535897
#define NULLPOINT cv::Point2f(0.0, 0.0)

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
	void getDistance(cv::Point2f pts[4], float &delay);			//solvePNP所在的函数	
	//ratio:角度缩小控制率
	//比如角度过大，在发一次信息过程中转不完计算的角度，可以认为缩小每次发送的角度
	float getPitchRotation();						//实际偏移
	float getYawRotation();							//实际偏移
	void batchProcess(std::vector<aim_deps::Armor> &tar_list);	//批量solvePNP（但是不解算弹道）
	void positionScore(aim_deps::Armor &tar);		//距离分数和旋转分数计算	

	/**
	 * @brief 从tar_list中获取rMats与tMats
	 * @param rmats rMats
	 * @param tmats tMats
	 * @param tar_list 通过其中的Armor的属性解出旋转矩阵以及取出平移向量
	 */
	void packUp(std::vector<cv::Mat> &rmats, std::vector<cv::Mat> &tmats, 
		const std::vector<aim_deps::Armor> tar_list);		
public:
	cv::Mat tVec;									//平移矩阵(x, y, z)
	//=====================可能要删除的部分===============
	float now_pitch;								//当前pitch增量
	float now_yaw;									//当前yaw增量
private:
private:
	ballistic::GimbalCtrl g_ctrl;					//云台控制
	std::vector<cv::Point3f> objPoints;				//世界坐标系下的点（装甲板实际4点位置）
	std::vector<cv::Point3f> intrinsic;				//(内参矩阵)
	std::vector<float> distCoeffs;					//畸变与切变向量（4参数）
	cv::Mat coeff;									//distCoeff Mat形式
	cv::Mat rVec;									//旋转向量
	cv::Mat insM;									//内参矩阵Mat形式
	//====================可能要删除的部分=================
	float old_pitch;								//pitch旧值
	float old_yaw;									//yaw旧值
};

GetPos::GetPos() {
	objPoints = std::vector<cv::Point3f>{
		cv::Point3f(-_HALF_LENGTH, _HALF_HEIGHT, 0),		//2,3,4,1象限顺序，与sortPoint一致
		cv::Point3f(-_HALF_LENGTH, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH, _HALF_HEIGHT, 0),
	};
	intrinsic = std::vector<cv::Point3f>{
		cv::Point3f(1776.67168581218, 0, 720),
		cv::Point3f(0, 1778.59375346543, 540),
		cv::Point3f(0, 0, 1)
	};
	insM = cv::Mat(intrinsic);
	insM.convertTo(insM, CV_64F);
	distCoeffs=std::vector<float>{
		-0.419212525827893, 
		0.175006995615751,
		0.00489209817799368,
		-0.00289049464268412
	};
	tVec.create(3, 1, CV_64F);
	rVec.create(3, 1, CV_64F);
	now_pitch=0.0; now_yaw=0.0; old_pitch=0.0; old_yaw=0.0;
	//pitches.resize(4);
	//yaws.resize(4);
	g_ctrl.Init(0, 7.95, -4.56, 0.0, 0.0, 16.0, 0.017772);//0.000814);
}

GetPos::~GetPos(){;}

void GetPos::calcBallistic(aim_deps::Armor tar, float &pitch, float &yaw, float &delay){
	g_ctrl.Transform(tar.t_vec, pitch, yaw, delay);	//弹道模型云台位置解算
}

void GetPos::batchProcess(std::vector<aim_deps::Armor> &tar_list){
	for(int i = 0; i<tar_list.size(); ++i){
		positionScore(tar_list[i]);
	}
}

//==========如果不事先做InputArray的处理，则可能报有关Mat type()的错误===========
//带有弹道模型的角度解算
void GetPos::getDistance(cv::Point2f pts[4], float &delay) {
	std::vector<cv::Point2f> tmp = {pts[0], pts[1], pts[2], pts[3]};
	cv::solvePnP(cv::InputArray(objPoints), cv::InputArray(tmp), 
		cv::InputArray(insM), cv::InputArray(distCoeffs), 
		cv::OutputArray(rVec), cv::OutputArray(tVec),
		false, cv::SOLVEPNP_ITERATIVE);
	//std::cout<<tVec<<std::endl;
	old_yaw = now_yaw; old_pitch = now_pitch;
	g_ctrl.Transform(cv::Point3f(tVec.at<double>(0, 0),					//弹道模型云台位置解算
		tVec.at<double>(1, 0), tVec.at<double>(2, 0)), now_pitch, now_yaw, delay);
}

//================随动模块(无弹道模型)====================

float GetPos::getYawRotation() {						//返回yaw实际偏移并计算角度控制值
	old_yaw=now_yaw;
	float theta_x = atan(tVec.at<double>(0, 0)/tVec.at<double>(2, 0));
	theta_x = theta_x * (180.0 / PI);
	//std::cout << "Result yaw:  " << theta_x << std::endl;
	if(abs(theta_x)>25) now_yaw=old_yaw;						//暂时的错误保护机制
	else {
		now_yaw = -theta_x;
	}          
	return theta_x;	
}

float GetPos::getPitchRotation() {					//返回pitch实际偏移并计算角度控制值
	old_pitch=now_pitch;
	float theta_y = atan(tVec.at<double>(1, 0)/tVec.at<double>(2, 0));
	theta_y = theta_y * (180.0 / PI);
	//std::cout << "Result pitch:  " << theta_y << std::endl;
	if(abs(theta_y)>25) now_pitch=old_pitch;
	else {
		now_pitch= -theta_y;						//暂时的错误保护机制
	}
	return theta_y;
}

void GetPos::positionScore(aim_deps::Armor &tar){
	std::vector<cv::Point2f> tmp = {tar.vertex[0], tar.vertex[1], tar.vertex[2], tar.vertex[3]};
	cv::solvePnP(cv::InputArray(objPoints), cv::InputArray(tmp), 
		cv::InputArray(insM), cv::InputArray(distCoeffs), 
		cv::OutputArray(rVec), cv::OutputArray(tVec),
		false, cv::SOLVEPNP_ITERATIVE);
	tar.t_vec = cv::Point3f(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
	tar.r_vec = rVec.clone();
}

void GetPos::packUp(std::vector<cv::Mat> &rmats, std::vector<cv::Mat> &tmats, 
	const std::vector<aim_deps::Armor> tar_list)
{
	rmats.clear();
	tmats.clear();
	for(int i = 0; i< tar_list.size(); ++i){
		cv::Mat vecr(3, 1, CV_64F), vect(3, 1, CV_64F);		
		/// cv::Point3f到cv::Mat
		vect.at<double>(0) = (double)tar_list[i].t_vec.x;
		vect.at<double>(1) = (double)tar_list[i].t_vec.y;
		vect.at<double>(2) = (double)tar_list[i].t_vec.z;
		cv::Mat rtmp;
		cv::Rodrigues(tar_list[i].r_vec, rtmp);
		rmats.emplace_back(rtmp);
		tmats.emplace_back(vect);
	}
}
#endif //_GET_POS_HPP
