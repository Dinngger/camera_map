/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.1.16 14:59
 * 主要思路：设置几个判断条件：
 *      1.角度匹配：角度差必须极小（3度以内）
 *      2.装甲板灯条比例合适（长宽比）
 *      3.装甲板角度就不加限制了，万一翻车了还能打就行
 *      4.装甲板灯条比例合适：装甲板和灯条的长度比
 *      5.filter:消除共灯条情况：中心更接近白色的为灯条
 * 
 */

#ifndef __ARMOR_PLATE_HPP
#define __ARMOR_PLATE_HPP

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "../universal/GetPos.hpp"             //给pnp解算函数预留的位置
// #include "../universal/Numrcgn.hpp"
#define NULL_RECT cv::RotatedRect(cv::Point2f(0.0, 0.0), cv::Size(0,0), 0.0)
#define VALID_RATIO         3.8             //装甲板长宽比上限
#define ANGLE_THRESH        12.0             //装甲板灯条角度差阈值
//#define DEBUG
#ifdef DEBUG
    #define print printf
#else
    #define print(...)
#endif
/*
struct Target{
    cv::RotatedRect rect;
    int light_pos1 = 0;                     //灯条1的储存位置
    int light_pos2 = 0;                     //灯条2的储存位置
    int plate_num = 0;                      //装甲板标号
    bool valid = true;                      //是否是有效的装甲板
    Target(): rect(NULL_RECT), light_pos1(0), light_pos2(0), plate_num(0), valid(true){}
    Target(cv::RotatedRect _r, int _l1, int _l2, int _pNum): 
        rect(_r), light_pos1(_l1), light_pos2(_l2), plate_num(_pNum), valid(true){}
    
    bool operator == (Target tar){    //两个装甲板（是否不等：只要没有公用的灯条，就不相等）
        return (
            light_pos1 == tar.light_pos1 ||
            light_pos1 == tar.light_pos2 ||
            light_pos2 == tar.light_pos1 ||
            light_pos2 == tar.light_pos2
        );
    }

};
*/
class ArmorPlate{
public:
    ArmorPlate();
    ~ArmorPlate();
public:
    void matchAll(cv::Mat src, std::vector<cv::Point> matches,
        std::vector<cv::RotatedRect> lights);         //匹配主函数
    void drawArmorPlates(cv::Mat &src);                     //绘制装甲板
public:
    GetPos pos_getter;
    std::vector<Target> tar_list;                           //目标装甲板
private:
    void filter(cv::Mat src);                               //过滤共边装甲板
    void match(cv::Mat, cv::RotatedRect r1, 
        cv::RotatedRect r2, int pos1, int pos2);            //匹配两个灯条
    static bool isAngleValid(cv::RotatedRect rect);         //装甲板的角度是否合适（不会存在超过40度的装甲板）
    static bool isRatioValid(cv::RotatedRect rect);         //装甲板的长宽比合适吗
    static bool isAngleMatch(cv::RotatedRect r1, cv::RotatedRect r2);       //灯条角度差合适吗

    //判断p1,p2周围更亮接近白色的一个点,p1>p2则返回真
    static bool close2White(cv::Mat src, cv::Point2f p1, cv::Point2f p2);
    cv::Mat getNumImg(cv::Mat src, cv::RotatedRect rect);                                 //装甲板预选区切下一块ROI作为数字识别区
    cv::RotatedRect getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2);  //灯条匹配装甲板                         
private:
    // Numrcgn classifier;                                     //数字分类器     
};

ArmorPlate::ArmorPlate(){
    tar_list.resize(16);
    tar_list.clear();
}

ArmorPlate::~ArmorPlate(){;}

void ArmorPlate::matchAll(cv::Mat src, std::vector<cv::Point> matches,
    std::vector<cv::RotatedRect> lights){
    tar_list.clear();
    for( int i = 0 ; i<matches.size() ; ++i){
        match(src, lights[matches[i].x], lights[matches[i].y],
            matches[i].x, matches[i].y);
        print("Matched:(%d), with matches(%d, %d).\n", i, matches[i].x, matches[i].y);
    }
    filter(src);
    int _cnt = 0;
    //=============debug:计算出最后有多少个valid的装甲板==================
    for(int i=0;i<tar_list.size();i++){
        if(tar_list[i].valid) ++_cnt;
    }
    print("Target list size(%d), valid size(%d).\n", tar_list.size(), _cnt);
}

void ArmorPlate::filter(cv::Mat src){
    for(int i = 0; i < tar_list.size(); ++i){

        if(tar_list[i].valid){

            for(int j = i+1; j<tar_list.size(); ++j){

                if(tar_list[j].valid){

                    if(tar_list[i] == tar_list[j]){

                        if(close2White(src, tar_list[i].rect.center,
                            tar_list[j].rect.center)){
                            tar_list[j].valid = false;
                            print("Tar(%d, %d) is removed.\n",
                                tar_list[j].light_pos1, tar_list[j].light_pos2);
                        }
                        else{
                            tar_list[i].valid = false;
                            print("Tar(%d, %d) is removed.\n",
                                tar_list[i].light_pos1, tar_list[i].light_pos2);
                        }
                    }
                }
            }
        }
    }
}

void ArmorPlate::match(cv::Mat src, cv::RotatedRect r1, cv::RotatedRect r2, int pos1, int pos2){
    if( !isAngleMatch(r1, r2)){
        print("Angle dismatch.(%d, %d).\n", pos1, pos2);
        return;
    }
    cv::RotatedRect rect = getArmorPlate(r1, r2);
    if(isRatioValid(rect) && isAngleValid(rect)){
        cv::Point2f pts[4];
        rect.points(pts);
        print("Pushed in.(%d, %d).\n", pos1, pos2);
        tar_list.push_back(Target(rect, pos1, pos2, 
            0)); //classifier.Recognize(pts, getNumImg(src, rect))));  //输入rect, 灯条位置以及数字识别结果
    }
    else{
        print("Ratio dismatch.(%d, %d).\n", pos1, pos2);
    }
}

void ArmorPlate::drawArmorPlates(cv::Mat &src){
    cv::Point2f pts[4];
	char str[20];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (Target tar : tar_list) {
		if (tar.valid) {
			tar.rect.points(pts);
			pos_getter.getDistance(tar.rect);
			for (int i = 0; i < 4; ++i) {
				cv::line(src, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0));
			}
			//绘制打击中心，绘制测距信息
			cv::circle(src, tar.rect.center, 2, cv::Scalar(0, 0, 255), -1);
			pos_getter.getDistance(tar.rect);			
			snprintf(str, 20, "Delta pitch:%f", pos_getter.now_pitch);
			cv::putText(src, str,cv::Point(30, 720),
				cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
			snprintf(str, 20, "Delta yaw:%f", pos_getter.now_yaw);
			cv::putText(src, str,cv::Point(30, 750),
				cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
			snprintf(str, 20, "Pitch:%f", pos_getter.getPitchRotation());
			cv::putText(src, str,cv::Point(30, 780),
				cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
			snprintf(str, 20, "Yaw: %f", pos_getter.getYawRotation());
			cv::putText(src, str,cv::Point(30, 810),
				cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
			snprintf(str, 20, "X: %f", pos_getter.tVec.at<double>(0, 0));
			cv::putText(src, str,cv::Point(30, 840),
				cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
			snprintf(str, 20, "Y: %f", pos_getter.tVec.at<double>(1, 0));
			cv::putText(src, str,cv::Point(30, 870),
				cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
			snprintf(str, 20, "Z: %f", pos_getter.tVec.at<double>(2, 0));
			cv::putText(src, str,cv::Point(30, 900),
				cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
		}
	}
	//还有绘制数字识别结果，但是CNN没有写出，所以这一步先省略
}

cv::RotatedRect ArmorPlate::getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2){
	cv::Point2f pts1[4], pts2[4];
	r1.points(pts1); r2.points(pts2);
	std::vector<cv::Point2f> pts(8); pts.clear();
	for(int i=0; i<4; ++i){
		pts.push_back(pts1[i]);
		pts.push_back(pts2[i]);
	}
	return cv::minAreaRect(pts);
}

cv::Mat ArmorPlate::getNumImg(cv::Mat src, cv::RotatedRect rect){
    float ang = abs(rect.angle)<45 ? abs(rect.angle) : abs(rect.angle+90),
        width = (rect.size.width > rect.size.height ?
            rect.size.width:rect.size.height) * 0.8 * cos(ang),
        height = width*1.4;
    cv::Mat res; 
    cv::cvtColor(src(cv::Rect(rect.center.x-width/2,
        rect.center.y-height/2, width, height)), res, cv::COLOR_BGR2GRAY);
    cv::resize(res, res, cv::Size(50, 50));
    return res;
}

//-----------------------------------------------------------
bool ArmorPlate::isAngleValid(cv::RotatedRect rect){
    float ang = rect.size.width > rect.size.height ? rect.angle : -rect.angle - 90;    
    return abs(ang)<40;
}

bool ArmorPlate::isRatioValid(cv::RotatedRect rect){
    return (rect.size.width / rect.size.height<VALID_RATIO &&
		rect.size.width / rect.size.height>1/VALID_RATIO);
}

bool ArmorPlate::isAngleMatch(cv::RotatedRect r1, cv::RotatedRect r2){
    float ang1 = r1.size.width > r1.size.height ? r1.angle : -r1.angle - 90;
	float ang2 = r2.size.width > r2.size.height ? r2.angle : -r2.angle - 90;
    return abs(ang1-ang2)<ANGLE_THRESH;
}

bool ArmorPlate::close2White(cv::Mat src, cv::Point2f p1, cv::Point2f p2){
    int blue_aver1 = 0, blue_aver2 = 0;
	for (int i = p1.x - 1 >= 0 ? p1.x - 1 : 0; i < (p1.x + 2 <= 720 ? p1.x + 2 : 720); ++i) {
		for (int j = p1.y - 1 >= 0 ? p1.y - 1 : 0; j < (p1.y + 2 <= 540 ? p1.y + 2 : 540); ++j) {
			blue_aver1 += src.at<cv::Vec3b>(j, i)[0];			//蓝色通道信息
		}
	}
	for (int i = p2.x - 1 >= 0 ? p2.x - 1 : 0; i < (p2.x + 2 <= 720 ? p2.x + 2 : 720); ++i) {
		for (int j = p2.y - 1 >= 0 ? p2.y - 1 : 0; j < (p2.y + 2 <= 540 ? p2.y + 2 : 540); ++j) {
			blue_aver2 += src.at<cv::Vec3b>(j, i)[0];			//蓝色通道信息
		}
	}
	return (blue_aver1 >= blue_aver2);							//true则说明p1周围更接近白色
}

#endif // __ARMOR_PLATE_HPP