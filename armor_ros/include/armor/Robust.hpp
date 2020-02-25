/*===============鲁棒性增强类（简单）================
*作者：hqy
*latest modification date:2020.1.13

存在的问题以及可以继续发展的方面：
	1.图像遍历太慢，要想办法改进brightInspect函数，对于输入图像估计亮度加速
	2.可能出现图像处理不自然的情况：亮度对比度突变以及高亮度下beta函数不收敛的情况
	3.改进方面：
		1.加速
		2.找到更好的亮度对比度函数模型，对输入图像进行处理
*/

#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#define NULLREC Rect(0,0,0,0)
//====alpha函数系数========
#define NUMA3 2.4367592e-07
#define NUMA2 -6.1522943e-05
#define NUMA1 -0.0083672
#define NUMA0 2.606281
//====beta函数系数=========
#define NUMB4 9.0326e-07
#define NUMB3 -0.0001443
#define NUMB2 0.00537
#define NUMB1 -0.1577
#define NUMB0 2.9301

using std::cout;
using std::endl;

//³����ά����
class Robust {
public:
	void brightInspect(cv::Mat src, int fps = 6);			//���ҶȻ�ͼ���ƽ������,ÿfps֡���һ��
	void alphaFunc() {					//alpha����:ȷ���Աȶȵ��ڵ�alphaϵ��
		alpha = NUMA3*pow(light, 3) + NUMA2*pow(light, 2) + NUMA1*pow(light, 1) + NUMA0;
		//cout << "Alpha:" << alpha << endl;
	}
	void betaFunc() {					//beta����:ȷ�����ȵ����е�betaϵ��
		beta = NUMB4*pow(light, 4) + NUMB3*pow(light, 3) + NUMB2*pow(light, 2) + NUMB1*pow(light, 1) + NUMB0;
		//cout << "Beta:" << beta << endl;
	}
public:
	double alpha = 1;
	double beta = 0;
private:
	double light = 0;
	int loopCount = 5;
	bool processFlag = true;
};


void Robust::brightInspect(cv::Mat src, int fps){
	++loopCount;
	if (loopCount%fps == 0) {
		loopCount = 0;
		int width = src.cols, height = src.rows;
		double sum = 0;
		std::vector<cv::Mat> channels;
		cv::split(src, channels);
		//cvtColor(src, tmp, COLOR_BGR2GRAY);
		for (int row = height/3; row < 2*height/3+1; row++) {
			for (int col = width/3; col < 2*width/3+1; col++) {
				sum += channels[0].at<uchar>(row, col);
			}
		}
		light = sum / (width/3+2) / (height/3+2);
		//cout << "Light Average:" << light << endl;
		alphaFunc();
		betaFunc();
	}
}
