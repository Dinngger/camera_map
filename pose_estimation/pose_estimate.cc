#include <mutex>
#include "Viewer.h"
#include "LightMatch.hpp"
#include "ArmorPlate.hpp"
#include <thread>

cv::VideoCapture cap;
ArmorPlate amp;
LightMatch match;
cv::Mat frame, screen;
GetPos getpos;
int test_count = 0;
double t_sum = 0, t_start = 0, t_end = 0;		//计时模块
char str[20], key = 0;
bool record_judge = false;

int main(int argc, char* argv[])
{
    Viewer *viewer = new Viewer();
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);

    std::vector<cv::Mat> armor3dpoints;
    std::vector<cv::RotatedRect> result;
    cv::VideoCapture cap("../../test.avi");
    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::vector<cv::Mat>  point_answer;
    std::cout<<"frame: -1"<<std::endl;
    std::cout<<totalFrameNumber<<std::endl;

    std::vector<cv::Mat> mats {cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F)};
    cv::Mat frame;
    bool reset;
    for (int w = 0; w < totalFrameNumber; w++)
    {
        cap.read(frame);
		t_start = cv::getTickCount();
		if(frame.empty())
            break;
        match.findPossible(frame);
        match.drawLights(frame);							//绘制所有灯条
        amp.matchAll(frame, match.matches, match.possibles);//查找匹配灯条
        amp.drawArmorPlates(frame);							//绘制装甲板
        t_end = cv::getTickCount();
        if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
        test_count++;
        std::vector<cv::Mat> rMats = amp.rMats;
        std::vector<cv::Mat> tMats = amp.tMats;
        cv::imshow("disp", frame);
        std::vector<cv::Mat> Twcs;
        if(rMats.size() == 0){
            viewer->mDrawer.reset_flag = true;
            continue;
        }
        //std::cout <<"rMats: "<< rMats.size() << std::endl;
        for(int i=0; i<1; i++){
            cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
            cv::Mat temp2, Twc;
            cv::hconcat(rMats[i], tMats[i], temp2);
            cv::vconcat(temp2, temp, Twc);
            Twcs.push_back(Twc);
            
        }
        //std::cout << "Twcs.size(): " << Twcs.size() <<std::endl;
        viewer->mDrawer.SetCurrentArmorPoses(Twcs);
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
