#include <mutex>
#include "Viewer.h"
#include "LightMatch.hpp"
#include "ArmorPlate.hpp"
#include <thread>


int main(int argc, char* argv[])
{
    cv::VideoCapture cap("../../cv_output1.avi");
    ArmorPlate amp;
    LightMatch match;
    cv::Mat frame, screen;
    GetPos getpos;
    Viewer *viewer = new Viewer();
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);
    int test_count = 0;
    double t_sum = 0, t_start = 0, t_end = 0;		//计时模块
    char key = 0;

    std::vector< cv::Point3f > p3d;
    std::ifstream fp3d("../fp3d.txt");
    if (!fp3d){
        std::cout << "No p3d.text file\n";
        return -1;
    } else {
        while (!fp3d.eof()){
            double pt3[3] = {0};
            for (auto &p:pt3) {
                fp3d >> p;
            }
            p3d.push_back(cv::Point3f(pt3[0],pt3[1],pt3[2]));
        }
    }

    std::vector< cv::Point2f > p2d;
    cv::Mat K = (cv::Mat_<double> (3, 3) << 1776.67168581218, 0, 720,
                                            0, 1778.59375346543, 540,
                                            0, 0, 1 );
    std::vector<cv::Mat> armor3dpoints;
    std::vector<cv::RotatedRect> result;
    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::vector<cv::Mat>  point_answer;
    std::cout<<"frame: -1"<<std::endl;
    std::cout<<totalFrameNumber<<std::endl;

    std::vector<cv::Mat> mats {cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F)};

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
        for(int i=0; i<rMats.size(); i++){
            cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
            cv::Mat temp2, Twc;
            cv::hconcat(rMats[i], tMats[i], temp2);
            cv::vconcat(temp2, temp, Twc);
            Twcs.push_back(Twc);
        }
/*

        int iterations = 100;
        double cost = 0, lastCost = 0;
        int nPoints = p3d.size();

        bundleAdjustment ( p3d, p2d, K );
*/
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
