/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.1.18 23:50
 * 主要思路：设置几个判断条件：（2020.1.18日写）
 *      采用非旋转矩形进行标定（四边形匹配）
 *      其中Target的定义有所改变，包含的是一个四个点的数组（一个Point2f数组指针）
 *      数组的指针可能导致程序跑到不知道的地方crash掉，所以今晚的任务是修改所有的代码，
 *      把ROS的这个包完全变成使用distantDetection的包
 *      把指针处理方面写好，减轻明天的负担
 *      明天主要是测定参数，调整yaw轴位置，拍视频，拍视频的问题明天再想
 */

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <ros/ros.h>
#include "../universal/GetPos.hpp"             //给pnp解算函数预留的位置
#include "../../serial_com/include/serial_com/parameters.h"
#define VALID_RATIO         3.8             //装甲板长宽比上限
#define ANGLE_THRESH        12.0             //装甲板灯条角度差阈值

#ifdef DEBUG
    #define print printf
#else
    #define print(...)
#endif

class ArmorPlate{
public:
    ArmorPlate();
    ~ArmorPlate();
public:
    void matchAll(cv::Mat src, std::vector<cv::Point> matches,
        std::vector<cv::RotatedRect> lights);                           //匹配主函数
    void drawArmorPlates(cv::Mat &src);                                 //绘制装甲板
    void init(ros::NodeHandle nh);
public:
    GetPos pos_getter;
    ros::Publisher pub;                                                 //消息发布
    std::vector<Target> tar_list;                                       //目标装甲板
private:
    bool isRatioValid();                                                //中点连线（平方）的比是否合适
    bool isEdgesValid();                                                //两对边(平方)的比是否合适
    void match(cv::RotatedRect r1, 
        cv::RotatedRect r2, int pos1, int pos2);                        //匹配两个灯条
    void getMidPoints(cv::RotatedRect rect,
         cv::Point2f &p1, cv::Point2f &p2);                             //获取一个灯条的短边两中点
    void getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2);         //灯条匹配装甲板                         
    static bool isAngleMatch(cv::RotatedRect r1, cv::RotatedRect r2);   //灯条角度差合适吗    
    static float getPointDist(cv::Point2f p1, cv::Point2f p2);          //返回两点距离的平方
private:
    cv::Point2f points[4];                                              //装甲板点列的临时容器
};

ArmorPlate::ArmorPlate(){
    tar_list.resize(16);
    tar_list.clear();
    for(int i=0;i<4;++i) points[i]=NULLPOINT;
}

ArmorPlate::~ArmorPlate(){;}

void ArmorPlate::init(ros::NodeHandle nh){
    pub = nh.advertise<serial_com::parameters>("cameraData", 1000);
}

void ArmorPlate::matchAll(cv::Mat src, std::vector<cv::Point> matches,
std::vector<cv::RotatedRect> lights)
{
    tar_list.clear();
    for( int i = 0 ; i<matches.size() ; ++i){
        match(lights[matches[i].x], lights[matches[i].y],
            matches[i].x, matches[i].y);
        print("Matched:(%d), with matches(%d, %d).\n", i, matches[i].x, matches[i].y);
    }
    //filter(src);
    //=============debug:计算出最后有多少个valid的装甲板==================
    #ifdef DEBUG
    int _cnt = 0;
    for(int i=0;i<tar_list.size();i++){
        if(tar_list[i].valid) ++_cnt;
    }
    #endif
    //print("Target list size(%d), valid size(%d).\n", tar_list.size(), _cnt);
}

void ArmorPlate::match(cv::RotatedRect r1, cv::RotatedRect r2, int pos1, int pos2){
    if( !isAngleMatch(r1, r2)){
        print("Angle dismatch.(%d, %d).\n", pos1, pos2);
        return;
    }
    if(r1.center.x < r2.center.x)               //r1灯条在左侧
        getArmorPlate(r1, r2);
    else
        getArmorPlate(r2, r1);                  //始终保持第一个入参是x轴坐标小的灯条
    
    if(isRatioValid() && isEdgesValid()){
        print("Pushed in.(%d, %d).\n", pos1, pos2);
        //数字识别预留位置！！！！！！！！！！！！！！！！！
        tar_list.push_back(Target(points, pos1, pos2, 0));
    }
    else{
        print("Ratio dismatch.(%d, %d).\n", pos1, pos2);
    }
}

void ArmorPlate::drawArmorPlates(cv::Mat &src){
	char str[20];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    if(tar_list.size()){
        for (Target tar : tar_list) {
		    if (tar.valid) {
		    	for (int i = 0; i < 4; ++i) {
		    		cv::line(src, tar.pts[i], tar.pts[(i + 1) % 4], cv::Scalar(0, 255, 0));
                    snprintf(str, 20, "%d", i);
		    	    cv::putText(src, str, tar.pts[i],
		    		    cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 100, 0));
		    	}
		    	//绘制打击中心，绘制测距信息
		    	cv::circle(src, tar.ctr, 2, cv::Scalar(0, 0, 255), -1);
		    	pos_getter.getDistance(tar);			
		    	snprintf(str, 20, "Delta pitch:%f", pos_getter.now_pitch);
		    	cv::putText(src, str,cv::Point(30, 330),
		    		cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
		    	snprintf(str, 20, "Delta yaw:%f", pos_getter.now_yaw);
		    	cv::putText(src, str,cv::Point(30, 360),
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
                msg.pitch = pos_getter.now_pitch; msg.yaw = pos_getter.now_yaw;
                msg.v1 = msg.v2 = msg.v3 = 0.0;
		    	msg.status=1;
		    	pub.publish(msg);//装甲板选框的四个角点
		    	std::cout<<"Msg published."<<std::endl;
		    }
        }
    }
	else{
	    serial_com::parameters msg;
	    msg.pitch=0.0; msg.yaw=0.0;
	    msg.v1 = msg.v2=msg.v3=0.0;
	    msg.status=1;
	    pub.publish(msg);
	    std::cout<<"No armorplate detected. Sent 0.0."<<std::endl;
	}
	//还有绘制数字识别结果，但是CNN没有写出，所以这一步先省略
}

void ArmorPlate::getMidPoints(cv::RotatedRect rect, cv::Point2f &p1, cv::Point2f &p2){
    cv::Point2f tmp_p1, tmp_p2, corners[4];                                     //找出角点
    rect.points(corners);
    float d1 = getPointDist(corners[0], corners[1]);            //0/1点距离的平方
	float d2 = getPointDist(corners[1], corners[2]);            //1/2点距离的平方
	int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
    tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;			    //获得旋转矩形两条短边上的中点
	tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
    if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
        p2 = tmp_p1;    
        p1 = tmp_p2;
    }
    else{                                                       //必须是p1是处于上方的点，p2处于下方（y轴更大）
        p1 = tmp_p1;
        p2 = tmp_p2;
    }
}

void ArmorPlate::getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2){
    getMidPoints(r1, points[0], points[1]);             
    getMidPoints(r2, points[3], points[2]);
}

//-----------------------------------------------------------

bool ArmorPlate::isRatioValid(){                    //对边中点连线的长度平方比值是否合适
    float len1 = getPointDist((points[0]+points[1])/2, (points[2]+points[3])/2),
         len2 = getPointDist((points[0]+points[3])/2, (points[1]+points[2])/2);
    return (len1/len2 < 16.0 && len1/len2 > 0.0625);
}

bool ArmorPlate::isEdgesValid(){  //对边长度平方比值是否合适
    float edges[4];
    for(int i = 0; i<4;++i){
        edges[i]=getPointDist(points[i], points[(i+1)%4]);
    }
    bool judge1 = (edges[0]/edges[2]<6.25 && edges[0]/edges[2]>0.16),
        judge2 = (edges[1]/edges[3]<6.25 && edges[1]/edges[3]>0.16);
    return judge1 && judge2;
}

bool ArmorPlate::isAngleMatch(cv::RotatedRect r1, cv::RotatedRect r2){
    float ang1 = r1.size.width > r1.size.height ? r1.angle : -r1.angle - 90;
	float ang2 = r2.size.width > r2.size.height ? r2.angle : -r2.angle - 90;
    return abs(ang1-ang2)<ANGLE_THRESH;
}

float ArmorPlate::getPointDist(cv::Point2f p1, cv::Point2f p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}


