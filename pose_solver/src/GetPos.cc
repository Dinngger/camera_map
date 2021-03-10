/**==========pnp测距测试模块，基于opencv solvePNP的iterative算法==============
 * @author: sentinel
 * last date of modification: 2020.3.2
 * 最后修改的内容：删除了几个没有用的函数和变量
*/

#include "GetPos.hpp"

GetPos::GetPos() {
	objPoints_small = std::vector<cv::Point3f>{
		cv::Point3f(-_HALF_LENGTH_SMALL, _HALF_HEIGHT, 0),		//2,3,4,1象限顺序，与sortPoint一致
		cv::Point3f(-_HALF_LENGTH_SMALL, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_SMALL, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_SMALL, _HALF_HEIGHT, 0),
	};
	objPoints_big = std::vector<cv::Point3f>{
		cv::Point3f(-_HALF_LENGTH_BIG, _HALF_HEIGHT, 0),		//2,3,4,1象限顺序，与sortPoint一致
		cv::Point3f(-_HALF_LENGTH_BIG, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_BIG, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_BIG, _HALF_HEIGHT, 0),
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
	g_ctrl.Init(0, 7.95, -4.56, 0.0, 0.0, 16.0, 0.017772);//0.000814);
}

GetPos::~GetPos(){;}

void GetPos::calcBallistic(aim_deps::Armor tar, float &pitch, float &yaw, float &delay){
	g_ctrl.Transform(tar.t_vec, pitch, yaw, delay);	//弹道模型云台位置解算
}

void GetPos::batchProcess(std::vector<aim_deps::Armor> &tar_list){
	for(size_t i = 0; i<tar_list.size(); ++i){
		positionScore(tar_list[i]);
	}
}

void GetPos::positionScore(aim_deps::Armor &tar){
	std::vector<cv::Point2f> tmp = {tar.vertex[0], tar.vertex[1], tar.vertex[2], tar.vertex[3]};
	if(tar.Isbigarmor)
	{cv::solvePnP(cv::InputArray(objPoints_big), cv::InputArray(tmp), 
		cv::InputArray(insM), cv::InputArray(distCoeffs), 
		cv::OutputArray(rVec), cv::OutputArray(tVec),
		false, cv::SOLVEPNP_ITERATIVE);
	tar.t_vec = cv::Point3f(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
	tar.r_vec = rVec.clone();
	}
	else{
	cv::solvePnP(cv::InputArray(objPoints_small), cv::InputArray(tmp), 
		cv::InputArray(insM), cv::InputArray(distCoeffs), 
		cv::OutputArray(rVec), cv::OutputArray(tVec),
		false, cv::SOLVEPNP_ITERATIVE);
	tar.t_vec = cv::Point3f(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
	tar.r_vec = rVec.clone();
	}
	
}

void GetPos::packUp(std::vector<cv::Mat> &rmats, std::vector<cv::Mat> &tmats, 
	const std::vector<aim_deps::Armor>& tar_list)
{
	rmats.clear();
	tmats.clear();
	for(size_t i = 0; i< tar_list.size(); ++i){
		if(tar_list[i].armor_number != -1 && tar_list[i].valid){
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
}
