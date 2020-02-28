#include <mutex>
#include "Viewer.h"
#include "LightMatch.hpp"
#include "ArmorPlate.hpp"
#include "sampling.hpp"
#include "CarModule.hpp"
#include <thread>


int main(int argc, char* argv[])
{
    cv::VideoCapture cap("/mine/cv_output1.avi");
    CarModule module;
    ArmorPlate amp;
    LightMatch match;
    cv::Mat frame, screen;
    GetPos getpos;
    Viewer *viewer = new Viewer();
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);

    cv::Mat K = (cv::Mat_<double> (3, 3) << 1776.67168581218, 0, 720,
                                            0, 1778.59375346543, 540,
                                            0, 0, 1 );
    std::vector<cv::Mat> armor3dpoints;
    std::vector<cv::RotatedRect> result;
    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::vector<cv::Mat>  point_answer;
    std::cout<<"frame: -1"<<std::endl;
    std::cout<<totalFrameNumber<<std::endl;

    int count = 0;
    for (int w = 0; w < totalFrameNumber; w++)
    {
        cap.read(frame);
		if(frame.empty())
            break;
        count++;
        if (!isLowExposure(frame))
            continue;
        match.findPossible(frame);
        match.drawLights(frame);							//绘制所有灯条
        amp.matchAll(frame, match.matches, match.possibles);//查找匹配灯条
        amp.drawArmorPlates(frame);							//绘制装甲板

        module.create_predict(count);
        std::vector<LightBarP> light_bars;
        // TODO: add items into light_bars and do predict.
        // TODO: use armors failed to predict to areate new car module.
        bundleAdjustment(light_bars, module, K);

        // TODO: change to use the module to draw.
        std::vector<cv::Mat> rMats = amp.rMats;
        std::vector<cv::Mat> tMats = amp.tMats;
        std::vector<cv::Mat> Twcs;
        for(int i=0; i<rMats.size(); i++){
            cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
            cv::Mat temp2, Twc;
            cv::hconcat(rMats[i], tMats[i] / 1000, temp2);
            cv::vconcat(temp2, temp, Twc);
            Twcs.push_back(Twc);
        }
        // std::cout <<"Twcs[0]: "<< Twcs[0] << std::endl;
        viewer->mDrawer.SetCurrentArmorPoses(Twcs);

        cv::imshow("disp", frame);
        char key = cv::waitKey(0);
        if(key==27)
            break;
	}

	cv::destroyAllWindows();
    return 0;
}
