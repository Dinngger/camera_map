/*
装甲板灯条匹配模块
作者：heqianyue
last date of modification: 2020.1.13
问题：没有加入红色灯条识别，没有加入大装甲板识别

装甲板灯条匹配基本流程：
1.从GetLight类取得所有待匹配灯条
2.如果图上只有一辆车那还好说，只有小装甲也很简单，但是现在要综合考虑多种情况：
    O(n^2)时间的一一配对,将不设置匹配分值
3.灯条匹配条件：
    1.两灯条的角度：夹角小于12
    2.两灯条的长度比（不能过大或者过小2.8-1/2.8，否则都算作失配）
    3.两灯条的位置关系（center）: 
        1.两灯条的距离必须与灯条的长度保持关系（比值在0.6-10之间）
        2.两灯条的y轴offset必须小于32
    4.装甲板不能过小（小于64的装甲板不予认同）
    5.装甲板的长宽比（小装甲板不能过大：不能超过5.5）
    6.装甲板的角度（不会超过45+17=62度（平面最大角度45，17度飞坡））
        并且如果两灯条长度比偏离1，若装甲板矩形0度，则说明误匹配
4.灯条匹配完成过滤：
    正常情况下targetList中，会出现一些两个装甲板共用一个灯条的情况，此为重合匹配，需要过滤
    选取装甲板中心3*3的区域，蓝色通道内值之和更低的是误匹配装甲板，需要过滤
5.得到最终的target
6.在Mat上绘制出target
7.PNP解算
*/

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "AnglePre.hpp"
#include <opencv2/opencv.hpp>
#include "../universal/GetPos.hpp"
#include "../../serial_com/include/serial_com/parameters.h"
#define NULL_RECT cv::RotatedRect(cv::Point2f(0.0, 0.0), cv::Size(0,0), 0.0)
#define COEFF2 0.418648
#define COEFF0 1.4

struct light_group{
    int light1;
    int light2;
    light_group(int _l1, int _l2){
        if(_l1<_l2?0:1){
            light1=_l1;
            light2=_l2;
        }
        else{
            light1=_l2;
            light2=_l1;
        }
    } 
};

struct target{
    cv::RotatedRect rect;
    light_group lights;
    int enemy_num;
    int enemy_color;
	bool valid = true;
    target():rect(NULL_RECT), enemy_num(0),
         enemy_color(0), lights(light_group(0,0)), valid(true) {}
    target(cv::RotatedRect _r, int _l1, int _l2):rect(_r),
        enemy_num(0), enemy_color(0), lights(light_group(_l1, _l2)), valid(true){}
};

class MatchLight{
public:
	MatchLight();
	~MatchLight();
	void matchLights(std::vector<cv::RotatedRect> r_list);                  //O(n^2)灯条遍历匹配
	void filter(cv::Mat src);                                                          //过滤重合装甲板
	void showTargets(cv::Mat &src);                                         //在输入图像上绘制灯条和装甲板
	void reset();
    void init(ros::NodeHandle nh);                   //初始化publisher
public:
	std::vector<target> tar_list;                                           //目标容器
	GetPos pos_getter;                                                      //pnp测距模块
    //AnglePre ang_pre;                                                       //角度预测模块
    ros::Publisher pub;
private:
	void deCollision(cv::Mat src, int pos1, int pos2);						//消除共灯条的装甲板（重复选区）
	//TODO:deCollision可以拿空间去换时间，个人觉得运算内存应该够用，建立一个灯条为索引的二维矩阵
	bool close2White(cv::Mat src, cv::Point p1, cv::Point p2);						//点周围更加接近白色
	cv::RotatedRect getRect(cv::RotatedRect r1, cv::RotatedRect r2);        //返回初筛选的装甲板
	static bool isTooSmall(float area);
	static bool isValidRatio(cv::RotatedRect rect);                         //装甲板的长宽比
	static bool isValidAngle(float ang, float ratio);                                   //装甲板角度

																		   //以下函数主要涉及到灯条的合适程度
	static bool isLengthMatch(float len1, float len2);                    //装甲板灯条比例是否合适
	static bool isValidMatch(cv::RotatedRect r1,
		cv::RotatedRect r2, float &src_r, int i, int j);             //两个灯条空间位置是否合适,如果合适，有src_r（output）！=0.0
	static bool isPosMatch(float len1, float len2,
		float dist, float y1, float y2);								//位置关系判断
	static bool isAngleMatch(float ang);                                 //灯条角度是否合适
	static float centerDist(cv::RotatedRect r1, cv::RotatedRect r2);       //两灯条中心距离
	static float angleDiff(cv::RotatedRect r1, cv::RotatedRect r2);        //求两个rect的角度（谁让minAreaRect的角度那么扯淡呢）
};


MatchLight::MatchLight() {
	tar_list.resize(6);
	tar_list.clear();
}

MatchLight::~MatchLight() {
	;
}

void MatchLight::init(ros::NodeHandle nh){
	pub=nh.advertise<serial_com::parameters>("cameraData", 1000);
}

void MatchLight::matchLights(std::vector<cv::RotatedRect> r_list) {
	float ratio = 0.0;
	for (int i = 0; i < r_list.size(); ++i) {
		for (int j = i + 1; j < r_list.size(); ++j) {
			bool judge = true;	//DEBUG	
			if (isValidMatch(r_list[i], r_list[j], ratio, i, j)) {
				cv::RotatedRect res = getRect(r_list[i], r_list[j]);
				if (!isTooSmall(res.size.area()) &&
				isValidRatio(res) && isValidAngle(res.angle, ratio))
				tar_list.push_back(target(res, i, j));          //成功匹配
				//============DEBUG==============
				/*if (isTooSmall(res.size.area())) {
					printf("Failed to match %d & %d. Plate too small.\n", i, j); judge = false;
				}
				if (!isValidRatio(res)) {
					printf("Failed to match %d & %d. Plate Ratio out of Range.\n", i, j); judge = false;
				}
				if (!isValidAngle(res.angle, ratio)) {
					printf("Failed to match %d & %d. Plate invalid angle.\n", i, j); judge = false;
				}
				if (judge) {
					tar_list.push_back(target(res, i, j));
					printf("%d & %d is a match success.\n\n", i, j);
				}
				else {
					printf("\n");
				}*/
			}
		}
	}
}

void MatchLight::filter(cv::Mat src) {                                        //暂时不做处理              
	for (int i = 0; i < tar_list.size(); ++i) {
		if (tar_list[i].valid) {											//处理过就直接跳过
			for (int j = i + 1; j < tar_list.size(); ++j) {
				if (tar_list[j].valid) {
					deCollision(src, i, j);				//默认valid值true，若false则处理过了
				}
			}
		}
	}
}

void MatchLight::showTargets(cv::Mat &src) {
	cv::Point2f pts[4];
	char str[20];
	//绘制图像辅助线
	cv::line(src, cv::Point(360, 0), cv::Point(360, 540), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 270), cv::Point(720, 270), cv::Scalar(255, 0, 0));
	if(tar_list.size()){						//检测到有装甲板
		for (target tar : tar_list) {
			if (tar.valid) {
				tar.rect.points(pts);
				pos_getter.getDistance(tar.rect);
				for (int i = 0; i < 4; ++i) {
					cv::line(src, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 50), 2);
				}
				//绘制打击中心，绘制测距信息
				cv::circle(src, tar.rect.center, 3, cv::Scalar(0, 0, 255), -1);
				pos_getter.getDistance(tar.rect);			
				snprintf(str, 20, "Delta pitch:%f", pos_getter.now_pitch);
				cv::putText(src, str,cv::Point(30, 330),
					cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
				snprintf(str, 20, "Delta yaw:%f", pos_getter.now_yaw);
				cv::putText(src, str,cv::Point(30, 360),
					cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
				snprintf(str, 20, "Pitch:%f", pos_getter.getPitchRotation());
				cv::putText(src, str,cv::Point(30, 480),
					cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
				snprintf(str, 20, "Yaw: %f", pos_getter.getYawRotation());
				cv::putText(src, str,cv::Point(30, 510),
					cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
				snprintf(str, 20, "X: %f", pos_getter.tVec.at<double>(0, 0));
				cv::putText(src, str,cv::Point(30, 390),
					cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
				snprintf(str, 20, "Y: %f", pos_getter.tVec.at<double>(1, 0));
				cv::putText(src, str,cv::Point(30, 420),
					cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
				snprintf(str, 20, "Z: %f", pos_getter.tVec.at<double>(2, 0));
				cv::putText(src, str,cv::Point(30, 450),
					cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
				serial_com::parameters msg;
				msg.pitch=pos_getter.now_pitch; msg.yaw=pos_getter.now_yaw;			
				//msg.pitch += ang_pre.calcPitch(pos_getter.pitches)*0.2;		//加预测量(pitch)
				//msg.yaw += ang_pre.calcYaw(pos_getter.yaws)*0.2;				//加预测量（yaw)
				msg.v1=msg.v2=msg.v3=0.0;
				msg.status=1;
				pub.publish(msg);
				std::cout<<"Msg published."<<std::endl;
			}
		}
	}
	else{
		serial_com::parameters msg;
		msg.pitch=0.0; msg.yaw=0.0;		
		msg.v1=msg.v2=msg.v3=0.0;
		msg.status=1;
		pub.publish(msg);
		std::cout<<"No armorplate detected. Sent 0.0."<<std::endl;
	}
	//还有绘制数字识别结果，但是CNN没有写出，所以这一步先省略
	reset();
}

void MatchLight::reset() {
	tar_list.clear();
}

cv::RotatedRect MatchLight::getRect(cv::RotatedRect r1, cv::RotatedRect r2) {
	cv::Point2f pts1[4], pts2[4];
	r1.points(pts1); r2.points(pts2);
	std::vector<cv::Point2f> pointList(8);
	for (int i = 0; i < 4; ++i) { pointList[i] = pts1[i]; pointList[i + 4] = pts2[i]; }
	return cv::minAreaRect(pointList);
}

void MatchLight::deCollision(cv::Mat src, int pos1, int pos2) {
	//printf("Tar_list[%d]:l1:%d, l2:%d\n", pos1, tar_list[pos1].lights.light1, tar_list[pos1].lights.light2);
	//printf("Tar_list[%d]:l1:%d, l2:%d\n", pos2, tar_list[pos2].lights.light1, tar_list[pos2].lights.light2);
	if (tar_list[pos1].lights.light1 != tar_list[pos2].lights.light1 &&
		tar_list[pos1].lights.light1 != tar_list[pos2].lights.light2 &&
		tar_list[pos1].lights.light2 != tar_list[pos2].lights.light1 &&
		tar_list[pos1].lights.light2 != tar_list[pos2].lights.light2) {
		//printf("%d & %d. Not collision.\n", pos1, pos2);
		return;
	}
		
	if (close2White(src, tar_list[pos1].rect.center, tar_list[pos2].rect.center)) {
		tar_list[pos2].valid = false;
		//printf("Tar_list[%d] decollision: %d & %d.\n", pos2, tar_list[pos2].lights.light1, tar_list[pos2].lights.light2);
	}
	else {
		tar_list[pos1].valid = false;
		//printf("Tar_list[%d] decollision: %d & %d.\n", pos1, tar_list[pos1].lights.light1, tar_list[pos1].lights.light2);
	}
}

//TODO:
bool MatchLight::close2White(cv::Mat src, cv::Point p1, cv::Point p2) {
	int blue_aver1 = 0, blue_aver2 = 0;
	for (int i = p1.x - 1 >= 0 ? p1.x - 1 : 0; i < (p1.x + 2 <= 720 ? p1.x + 2 : 720); ++i) {
		for (int j = p1.y - 1 >= 0 ? p1.y - 1 : 0; j < (p1.y + 2 <= 540 ? p1.y + 2 : 540); ++j) {
			blue_aver1 += src.at<cv::Vec3b>(j, i)[0];				//蓝色通道信息
		}
	}
	for (int i = p2.x - 1 >= 0 ? p2.x - 1 : 0; i < (p2.x + 2 <= 720 ? p2.x + 2 : 720); ++i) {
		for (int j = p2.y - 1 >= 0 ? p2.y - 1 : 0; j < (p2.y + 2 <= 540 ? p2.y + 2 : 540); ++j) {
			blue_aver2 += src.at<cv::Vec3b>(j, i)[0];			//蓝色通道信息
		}
	}
	return (blue_aver1 >= blue_aver2);							//true则说明p1周围更接近白色
}

//=====================匹配判断bool函数===========================
bool MatchLight::isTooSmall(float area) {
	return area <= 64;
}

bool MatchLight::isValidRatio(cv::RotatedRect rect) {
	return (rect.size.width / rect.size.height<3 &&
		rect.size.width / rect.size.height>0.3333);
}

bool MatchLight::isValidAngle(float ang, float ratio) {
	if ((ratio >= 1.8 || ratio < 0.554) && ang <= 1.0) return false;
	return ang <= 62;
}

bool MatchLight::isValidMatch(cv::RotatedRect r1, cv::RotatedRect r2, float &src_r, int i, int j) {
	float len1 = r1.size.width > r1.size.height ? r1.size.width : r1.size.height,
		len2 = r2.size.width > r2.size.height ? r2.size.width : r2.size.height;
	float dist = centerDist(r1, r2), ang = angleDiff(r1, r2),
		ratio = len1 > len2 ? len1 / len2 : len2 / len1;
	bool judge = true;
	//printf("%d & %d ang:%f\n", i, j, ang);
	/*=====================DEBUG=========================
	if (!isLengthMatch(len1, len2)) {
		printf("Failed to match %d & %d. Length dismatch.\n", i, j); judge = false;
	}
	if (!isAngleMatch(ang)) {
		printf("Failed to match %d & %d. Angle dismatch %f.\n", i, j, abs(ang)); judge = false;
	}
	if (!isPosMatch(len1, len2, dist, r1.center.y, r2.center.y)) {
		printf("Failed to match %d & %d. Position dismatch.\n", i, j); judge = false;
	}
	if (judge) {
		//score = matchScore(abs(ang), abs(r1.center.y - r2.center.y), ratio, (len1+len2)/2/dist, i, j);
		//std::cout << "Score:" << score << std::endl;
		src_r = ratio;
		return true;
	}=====================DEBUG==========================*/
	if (isLengthMatch(len1, len2) && isAngleMatch(ang) &&
		isPosMatch(len1, len2, dist, r1.center.y, r2.center.y)) {
		src_r = ratio;
		return true;
	}
	//std::cout << "For some reasons, failed to match." << std::endl;
	src_r = 0.0;
	return false;
}

bool MatchLight::isLengthMatch(float len1, float len2) {
	return (len1 / len2 > 0.357&&len1 / len2 < 2.8);
}

bool MatchLight::isPosMatch(float len1, float len2, float dist, float y1, float y2) {
	float average = (len1 + len2) / 2.0;
	return (abs(y1-y2)<=45 && abs(y1-y2)<0.8*average && dist / average<10 && dist / average>0.6);
}

bool MatchLight::isAngleMatch(float ang) {
	return abs(ang) <= 12;
}

float MatchLight::centerDist(cv::RotatedRect r1, cv::RotatedRect r2) {
	float diffx = r1.center.x - r2.center.x,
		diffy = r1.center.y - r2.center.y;
	return sqrt(diffx*diffx + diffy*diffy);
}

float MatchLight::angleDiff(cv::RotatedRect r1, cv::RotatedRect r2) {
	float ang1 = r1.size.width > r1.size.height ? r1.angle : -r1.angle - 90;
	float ang2 = r2.size.width > r2.size.height ? r2.angle : -r2.angle - 90;
	return ang1-ang2;
}
