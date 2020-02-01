/*
裁判检测模块-找到图上所有合适的裁判系统
作者：zlq
last date of modification:2020.2.1 23:30
思路：裁判系统的灯条必定四周一点蓝色都不含有，故找出四周没有蓝色的灯条即可。
*/

#ifndef JUDGE_MATCH_HPP
#define JUDGE_MATCH_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

class JudgeDetection{
public:
    JudgeDetection();
    ~JudgeDetection();
	void getBlueRect(cv::Mat src);									//找到图上的蓝色点集并用旋转矩形框起来
    void getJudgeRect(std::vector<cv::RotatedRect> possibles);      //判断蓝色矩形是否与灯条矩形有交集
	void drawJudge(cv::Mat &src);					                //绘制裁判系统
public:
	std::vector<cv::RotatedRect> blues;							    //蓝色矩形集合
    std::vector<cv::RotatedRect> judges;							    //裁判系统集合

private:
	void reset();													//重置
    bool DoesRectangleContainPoint(cv::RotatedRect rectangle, cv::Point2f point);    //判断某个点是否在某个旋转矩形内
private:	
	float mean_val;													//均值
};

JudgeDetection::JudgeDetection(){
    ;
}

JudgeDetection::~JudgeDetection(){
	;
}

void JudgeDetection::reset(){
	blues.clear();
	judges.clear();
}

void JudgeDetection::getBlueRect(cv::Mat src){					//找出所有可能灯条，使用梯形匹配找出相匹配的灯条对
	reset();
	cv::Mat channels[3];
	cv::split(src, channels);
    cv::Mat blue_binary = (channels[0] - 0.4 * channels[1] - 0.7 * channels[2]> 50);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(blue_binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);	//寻找图上轮廓
	for (int i = 0; i < contours.size(); ++i) {
		double area = cv::contourArea(contours[i]);
		if (area > 7) {		
			cv::RotatedRect blue = cv::minAreaRect(contours[i]);
			blues.push_back(blue);
		}
	}
}

bool JudgeDetection::DoesRectangleContainPoint(cv::RotatedRect rectangle, cv::Point2f point) {
	//Get the corner points.
	cv::Point2f corners[4];
	rectangle.points(corners);
	//Convert the point array to a vector.
	cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
	std::vector<cv::Point2f> contour(corners, lastItemPointer);
	//Check if the point is within the rectangle.
	double indicator = cv::pointPolygonTest(contour, point, false);
	bool rectangleContainsPoint = (indicator >= 0);
	return rectangleContainsPoint;
}

void JudgeDetection::getJudgeRect(std::vector<cv::RotatedRect> possibles){
    for(int i=0;i<possibles.size();i++){
        for(int j=0;j<blues.size();j++){
            cv::Point2f corners[4];
            cv::Point2f corners2[4];
	        blues[j].points(corners);
            possibles[i].points(corners2);
            for(int k=0;k<4;k++){
                if(DoesRectangleContainPoint(possibles[i], corners[k]) || DoesRectangleContainPoint(blues[j], corners2[k]))
                    goto nextPossible;
            }
        }
        judges.push_back(possibles[i]);
        nextPossible: continue;
    }
}

void JudgeDetection::drawJudge(cv::Mat &src){
    cv::Point2f corners[4];
	char str[4];
	for(int i=0;i<judges.size();++i){
		judges[i].points(corners);
		for(int j=0; j<4;++j){
			cv::line(src, corners[j], corners[(j+1)%4], cv::Scalar(0, 255, 0), 2);
		}
    }
}

#endif // JUDGE_MATCH_HPP
