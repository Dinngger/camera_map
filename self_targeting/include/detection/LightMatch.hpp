/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
实现者：雷达组-zlq
代码思路：雷达组-dxd
修改：雷达组-dxd
封装：hqy
last date of modification:2020.1.16 22:30
==============程序的输出有所改变================
原程序：一个vector，包含所有配对的灯条（成对地储存，会重复）
更改后：一个vector保存所有灯条，另一个vector<cv::Point>保存匹配信息：
	Point.x是第一个灯条的索引，Point.y是第二个灯条的索引

目前存在的问题：
	1.远距离不够稳定，灯条过小造成
	2.帧率问题
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#define PARAM1 10
#define PARAM2 6

class LightMatch{
public:
    LightMatch();
    ~LightMatch();
	void findPossible(cv::Mat src);									//找到图上所有可能的灯条
	void drawLights(cv::Mat &src);									//绘制灯条
	bool lowExposure(cv::Mat src, cv::Mat &gray);					//图像预处理
public:
	int low_exposure;												//曝光率设置flag
	std::vector<cv::RotatedRect> possibles;							//预选区集合

	//灯条匹配装甲板函数将会使用此配对信息，以及灯条来匹配装甲板
	std::vector<cv::Point> matches;									//会配对的数组
private:
	void reset();													//重置
	void getRealLight(int size);									//找到预选灯条中实际的灯条
	void getTrapezoids(cv::Point2f corners[4]);						//取出灯条拓展梯形
	static bool isInTrapezoid(cv::Point2f corners[4], 
		const std::vector<cv::Point2f> &trapezoid);					//点集是否能被梯形包围
	static float getDistance(cv::Point p1, cv::Point p2);			//图像两点间距离
private:	
	float mean_val;													//均值
	std::vector<std::vector<cv::Point2f>> trapezoids;				//灯条梯形集合
};

LightMatch::LightMatch(){
	mean_val = 40.0;
	low_exposure = 0;
}

LightMatch::~LightMatch(){
	;
}

void LightMatch::reset(){
	matches.clear();
	possibles.clear();
	trapezoids.clear();
}

void LightMatch::findPossible(cv::Mat src){					//找出所有可能灯条，使用梯形匹配找出相匹配的灯条对
	reset();
	cv::Mat binary;
	cv::threshold(src, binary, 128, 255, CV_THRESH_BINARY);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);	//寻找图上轮廓
	for (int i = 0; i < contours.size(); ++i) {
		double area = cv::contourArea(contours[i]);
		if (area > 7) {		
			cv::RotatedRect light = cv::minAreaRect(contours[i]);
			possibles.push_back(light);
		}
	}
	for (int i = 0; i < possibles.size(); ++i) {
		cv::Point2f corners[4];
		possibles[i].points(corners);
		getTrapezoids(corners);								
	}
	getRealLight(possibles.size());
}

bool LightMatch::lowExposure(cv::Mat src, cv::Mat &gray){
	low_exposure = (++low_exposure)%3;							//2帧低曝光，1帧高曝光
	cv::Scalar mean_values;
	cv::Mat channels[3];
	cv::split(src, channels);
	gray = channels[0];											//取出蓝色通道
	mean_values=cv::mean(channels[2]);
	bool res=mean_values.val[0]< mean_val;
	mean_val=mean_val*0.993+ 0.007*mean_values.val[0];
	return res;			
}

void LightMatch::getRealLight(int size){
	bool flag[size][size];					//两灯条是否满足要求,是个对称矩阵，当[i][j],[j][i]为真时，两灯条匹配
	for (int i = 0; i < size; ++i) {		//初始化		
        for(int j = 0; j < size; ++j)
		    flag[i][j] = false;
	}
	for (int i = 0; i < size; ++i) {		//梯形包含匹配
		cv::Point2f corners[4];
		possibles[i].points(corners);
		for (int j = 0; j<size*2; ++j) {
			if (isInTrapezoid(corners, trapezoids[j])) {
				flag[i][j/2] = true;
			}
		}
	}
	for (int i = 0; i<size; ++i) {						//将可能的匹配结果放入容器matches中
		for (int j = i+1; j<size; ++j) {
			if (flag[i][j] == true && flag[j][i] == true) {
				flag[i][j] = false;
				matches.push_back(cv::Point(i, j));		//最后只保存配对的信息，灯条只保存一次
			}
		}
	}
	//std::cout<<"Result :"<<matches<<std::endl;
}

//匹配的灯条其梯形将会互相包含
void LightMatch::getTrapezoids(cv::Point2f corners[4]){
	std::vector<cv::Point2f> left_trapezoid;
	std::vector<cv::Point2f> right_trapezoid;
	double d1 = getDistance(corners[0], corners[1]);
	double d2 = getDistance(corners[1], corners[2]);
	cv::Point2f trapezoid_points[4];								//梯形角点
	cv::Point2f direction_vectors[2];								//方向向量
	cv::Point2f midpoints[2];										//中点
	int i0 = d1<d2? 1:0;										//长所在边第一个顶点的位置
	midpoints[0] = (corners[i0] + corners[i0 + 1]) / 2;			//获得旋转矩形两条长上的中点
	midpoints[1] = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
	cv::Point2f vertical_vector = (midpoints[1]) - (midpoints[0]);	//获得垂直长方向的方向向量
	vertical_vector = (i0 == 0 ? d1 : d2) * PARAM1 * vertical_vector / sqrt(vertical_vector.x*vertical_vector.x + vertical_vector.y*vertical_vector.y);
	direction_vectors[0] = corners[i0 + 1] - corners[i0];
	direction_vectors[1] = corners[(i0 + 3) % 4] - corners[i0 + 2];
	left_trapezoid.push_back(corners[i0]);
	left_trapezoid.push_back(corners[i0 + 1]);
	left_trapezoid.push_back(midpoints[0] - vertical_vector + PARAM2 * direction_vectors[0]);
	left_trapezoid.push_back(midpoints[0] - vertical_vector - PARAM2 * direction_vectors[0]);
	right_trapezoid.push_back(corners[i0 + 2]);
	right_trapezoid.push_back(corners[(i0 + 3) % 4]);
	right_trapezoid.push_back(midpoints[1] + vertical_vector + PARAM2 * direction_vectors[1]);
	right_trapezoid.push_back(midpoints[1] + vertical_vector - PARAM2 * direction_vectors[1]);
	trapezoids.push_back(left_trapezoid);						//灯条左右两边将会拓展出两个梯形
	trapezoids.push_back(right_trapezoid);
}

void LightMatch::drawLights(cv::Mat &src){
	cv::Point2f pts[4];
	char str[4];
	for(int i=0;i<possibles.size();++i){
		possibles[i].points(pts);
		for(int j=0; j<4;++j){
			cv::line(src, pts[j], pts[(j+1)%4], cv::Scalar(0, 0, 255));
		}
		snprintf(str, 4, "%d", i);
		cv::putText(src, str, pts[3], cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
	}
}

bool LightMatch::isInTrapezoid(cv::Point2f corners[4], const std::vector<cv::Point2f> &trapezoid){
	int count = 0;
	for (int i = 0; i<4; i++) {
		if (cv::pointPolygonTest(trapezoid, corners[i], false) >= 0)
			count++;
	}
	if (count == 4) {
		return true;
	}
	return false;
}

float LightMatch::getDistance(cv::Point p1, cv::Point p2){
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	distance = sqrtf(distance);
	return distance;
}
