#include "ArmorPlate.hpp"
#include "LightMatch.hpp"
#include "Judge.hpp"

using namespace cv;
using namespace std;

ArmorPlate amp;
LightMatch match;
JudgeDetection judge;
cv::Mat frame, screen;
GetPos getpos;
double t_sum = 0, t_start = 0, t_end = 0;		//计时模块
int test_count = 0;
char key = 0;

int main()
{
	//读取视频或摄像头
	cv::VideoCapture cap ("C:/Users/allegray/Videos/test.avi");
	double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::vector<cv::Mat>  point_answer;

    std::vector<cv::Mat> mats {cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F)};
    for (int w = 0; w < totalFrameNumber; w++)
    {
        cap.read(frame);
		t_start = cv::getTickCount();
		if(frame.empty())
            break;
        judge.getBlueRect(frame);
        match.findPossible(frame);
        judge.getJudgeRect(match.possibles);
        match.drawLights(frame);							    //绘制所有灯条
        judge.drawJudge(frame);                                 //绘制所有裁判系统
        //amp.matchAll(frame, match.matches, match.possibles);  //查找匹配灯条
        //amp.drawArmorPlates(frame);							//绘制装甲板
        
        t_end = cv::getTickCount();
        if (test_count) t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
        test_count++;
        namedWindow("disp",0);//创建窗口
        cv::resizeWindow("disp", 1080, 720); 
        cv::imshow("disp", frame);
        key = cv::waitKey(0);
        if(key==27)
            break;
        else if (key==' ')
            cv::waitKey(0);
		t_end = cv::getTickCount();
		if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
		test_count++;
	}

	std::cout << "Average running time:" << t_sum / test_count - 1 << std::endl;
	cv::destroyAllWindows();
	return 0;
}
