/* pnp测距测试模块，基于opencv solvePNP的iterative算法
* author: sentinel
* last date of modification: 2020.1.13
* problems:测距问题，z轴测距个人怀疑不准
* 已知Problems：测距：z轴近距离时很准，远距离距离损失严重
* 可以改进的地方：GetPos发送的角度，可以使用PID算法来减小超调
	1.虽说PID算法在电控处控制云台转动中使用到了，但个人认为视觉端也可以进行角度调节方面的优化
	但是存在的一个疑问就是：不清楚发送接收频率，PID也不是很明白该怎么用
	2.主要的改进方向：
		1.添加预测模块，先根据帧追踪确定装甲板的速度和加速度，进而确定pitch，yaw需要补偿的角度
		2.
	3.有关速度，角速度历史值推定的思路（注意所有的pitch,yaw都是增量式）
		所有的储存值都相当于是瞬时期望角速度（一帧内期望转的角速度）
		而old_x与now_x的差值相当于实际角速度，比如：
			1.在第k帧时需要转a1角度，而在k+1帧时需要转a2角度：
				若目标静止，则（a1-a2）为实际转过的角度
				若目标移动，则（a1-a2）包含                    cv::circle(src, tar.pts[i], 2, cv::Scalar(0, 255, 255), -1);
了角度度以及装甲板速度两个信息
			2.实际计算的是期望以及期望的变化规律
				可以考虑RBF内核的SVM来处理这个问题
				虽然我担心的是时间问题,以后可以考虑并行计算优化
*/
#ifndef __GETPOS_HPP
#define __GETPOS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque>
#define _HALF_LENGTH 0.065			//70.00mm
#define _HALF_HEIGHT 0.0285			//30.00mm
#define PI 3.1415926535897
#define NULLPOINT cv::Point2f(0.0, 0.0)

cv::Point2f DEFAULT_NULL_POINTS[4] = {NULLPOINT, NULLPOINT,
	NULLPOINT, NULLPOINT};

struct pnpTarget{
    cv::Point2f pts[4];                     //多边形拟合
    cv::Point2f ctr;                        //装甲板中心
    int light_pos1 = 0;                     //灯条1的储存位置
    int light_pos2 = 0;                     //灯条2的储存位置
    int plate_num = 0;                      //装甲板标号
    bool valid = true;                      //是否是有效的装甲板

	pnpTarget(){}
    pnpTarget(cv::Point2f _pts[4], int _l1, int _l2, int _pNum):
        light_pos1(_l1), light_pos2(_l2), plate_num(_pNum), valid(true){
			for(int i=0; i<4;++i) pts[i]=_pts[i];							//逐点复制
            cv::Point2f ctr = (_pts[0]+_pts[1]+_pts[2]+_pts[3])/4;			//求中心位置（也许鸡肋）
        }
    
    bool operator == (pnpTarget tar){    //两个装甲板（是否不等：只要没有公用的灯条，就不相等）
        return (
            light_pos1 == tar.light_pos1 ||
            light_pos1 == tar.light_pos2 ||
            light_pos2 == tar.light_pos1 ||
            light_pos2 == tar.light_pos2
        );
    }

};

class GetPos{
public:
	GetPos();
	~GetPos();
	void getDistance(pnpTarget tar);			//solvePNP所在的函数	
	void getDistance(cv::RotatedRect rect);	//重载：可以对旋转矩形进行处理
	//ratio:角度缩小控制率
	//比如角度过大，在发一次信息过程中转不完计算的角度，可以认为缩小每次发送的角度
	float getPitchRotation();						//实际偏移
	float getYawRotation();							//实际偏移
public:
	cv::Mat rVec;									//旋转向量
	cv::Mat tVec;									//平移矩阵(x, y, z)
	cv::Mat rMat;									
	float now_pitch;								//当前pitch增量
	float now_yaw;									//当前yaw增量
	
	/*
	这两个队列的作用：储存历史pitch，yaw的历史值
	需要达到的目的，比如：
		当敌方移动太快时，假设敌方装甲板移出相机视野区，则检测不到装甲板
		正常情况下，没有装甲板要返回0.0，但实际上我们存有的历史值可以告诉我们
		装甲板的移动方向以及速度,可以根据此来推断一个对应角度而不是返回0.0

	创建一个大小为8的双端队列，保存pitch,yaw历史值
	能够根据历史值以及当前值推断云台的速度，角速度以及预测值
	SVM根据队列作为输入来确定预测角度
	*/
	std::deque<float> pitches;						//pitch队列
	std::deque<float> yaws;							//yaw队列
private:
	std::vector<cv::Point3f> objPoints;				//世界坐标系下的点（装甲板实际4点位置）
	std::vector<cv::Point3f> intrinsic;				//(内参矩阵)
	std::vector<float> distCoeffs;					//畸变与切变向量（4参数）
	cv::Mat coeff;									//distCoeff Mat形式
	cv::Mat insM;									//内参矩阵Mat形式
	float old_pitch;								//pitch旧值
	float old_yaw;									//yaw旧值
};

GetPos::GetPos() {
	objPoints = std::vector<cv::Point3f>{
		cv::Point3f(-_HALF_LENGTH, _HALF_HEIGHT, 0),		//2,1,4,3象限顺序，与sortPoint一致
		cv::Point3f(-_HALF_LENGTH, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH, _HALF_HEIGHT, 0),
	};
	intrinsic = std::vector<cv::Point3f>{
		cv::Point3f(1776.67168581218, 0, 720),//743.045791845325),
		cv::Point3f(0, 1778.59375346543, 540),//554.215561312513),
		cv::Point3f(0, 0, 1)
	};
	insM = cv::Mat(intrinsic);
	insM.convertTo(insM, CV_64F);
	distCoeffs=std::vector<float>{-0.419212525827893, 
		0.175006995615751, 0.00489209817799368, -0.00289049464268412};
	tVec.create(3, 1, CV_64F);
	rVec.create(3, 1, CV_64F);
	now_pitch=0.0; now_yaw=0.0; old_pitch=0.0; old_yaw=0.0;
	pitches.resize(4);
	yaws.resize(4);
}

GetPos::~GetPos(){;}

//==========如果不事先做InputArray的处理，则可能报有关Mat type()的错误===========
//带有弹道模型的角度解算

void GetPos::getDistance(pnpTarget tar) {
#define n 50
	static double lastz[n] = {0,};
	static double z = 0;
	//tVec.create(3, 1, CV_64F);
	//rVec.create(3, 1, CV_64F);

	std::vector<cv::Point2f> tmp = {tar.pts[0], tar.pts[1], tar.pts[2], tar.pts[3]};
	// std::cout<<"tar.pts[0]: "<<tar.pts[0]<<std::endl;
	// std::cout<<"tar.pts[1]: "<<tar.pts[1]<<std::endl;
	// std::cout<<"tar.pts[2]: "<<tar.pts[2]<<std::endl;
	// std::cout<<"tar.pts[3]: "<<tar.pts[3]<<std::endl;

	cv::solvePnP(cv::InputArray(objPoints), cv::InputArray(tmp), 
		cv::InputArray(insM), cv::InputArray(distCoeffs), 
		cv::OutputArray(rVec), cv::OutputArray(tVec),
		false, cv::SOLVEPNP_ITERATIVE);
	
	// std::cout<<"tVec: "<<tVec<<std::endl;
	
	cv::Rodrigues(rVec, rMat);
	double newz = tVec.at<double>(2, 0);
	z = 0.99 * z;
	for (int i=0; i<n; i++)
		z += 0.01 / n * lastz[i];
	if (z > 10000 || z < 0)
		z = newz;
	for (int i=0; i<n-1; i++)
		lastz[i] = lastz[i+1];
	lastz[n-1] = newz;
	tVec.at<double>(2, 0) = z;
	
	old_yaw = now_yaw; old_pitch = now_pitch;
	pitches.push_front(now_pitch);
	yaws.push_front(now_yaw);
}

void GetPos::getDistance(cv::RotatedRect rect) {
	//tVec.create(3, 1, CV_64F);
	//rVec.create(3, 1, CV_64F);
	cv::Point2f pts[4];
	rect.points(pts);
	std::vector<cv::Point2f> tmp = {pts[0], pts[1], pts[2], pts[3]};	
	cv::solvePnP(cv::InputArray(objPoints), cv::InputArray(tmp), 
		cv::InputArray(insM), cv::InputArray(distCoeffs), 
		cv::OutputArray(rVec), cv::OutputArray(tVec),
		false, cv::SOLVEPNP_ITERATIVE);
	//std::cout<<tVec<<std::endl;
	old_yaw = now_yaw; old_pitch = now_pitch;
	pitches.push_front(now_pitch);
	yaws.push_front(now_yaw);
}

float GetPos::getYawRotation() {						//返回yaw实际偏移并计算角度控制值
	old_yaw=now_yaw;
	float theta_x = atan(tVec.at<double>(0, 0)/tVec.at<double>(2, 0));
	theta_x = theta_x * (180.0 / PI);
	//std::cout << "Result yaw:  " << theta_x << std::endl;
	if(abs(theta_x)>25) now_yaw=old_yaw;						//暂时的错误保护机制
	else {
		now_yaw = -theta_x;
	}          
	yaws.push_front(now_yaw);        					//队列更新yaw值   
	return theta_x;	
}


//================随动模块(无弹道模型)====================
float GetPos::getPitchRotation() {					//返回pitch实际偏移并计算角度控制值
	old_pitch=now_pitch;
	float theta_y = atan(tVec.at<double>(1, 0)/tVec.at<double>(2, 0));
	theta_y = theta_y * (180.0 / PI);
	//std::cout << "Result pitch:  " << theta_y << std::endl;
	if(abs(theta_y)>25) now_pitch=old_pitch;
	else {
		now_pitch= -theta_y;						//暂时的错误保护机制
	}
	pitches.push_front(now_pitch);						//队列pitch更新
	return theta_y;
}
#endif // __GETPOS_HPP
