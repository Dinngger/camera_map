#include "LightMatch.hpp"
#include "ArmorPlate.hpp"
#include "GetPos.hpp"
#include "sampling.hpp"
#include "CarModule.hpp"
#include "Viewer.h"
#include <thread>
#include <mutex>


int main(int argc, char* argv[])
{
    std::vector<aim_deps::Armor> tar_list(16);                  /// tar_list不再是match内部的成员，而是一个更高层的类的成员
    tar_list.clear();                                           /// 
    cv::VideoCapture cap("cv_output1.avi");
    cv::Mat K = (cv::Mat_<double> (3, 3) << 1776.67168581218, 0, 720,
                                            0, 1778.59375346543, 540,
                                            0, 0, 1 );
    CarModule module(K);
    ArmorPlate amp;
    LightMatch match;
    cv::Mat frame, screen;
    GetPos pos_getter;                                          /// PNP解算模块
    Viewer *viewer = new Viewer(K.at<double>(0, 0), K.at<double>(1, 1));
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);

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

        match.saveImg(frame);
        match.findPossible();

        amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条
        pos_getter.batchProcess(tar_list);              ///外部pnp解算所有装甲板

        //传入时间预测此时装甲版的平面位置
        module.create_predict(count);
        //观测到的灯条
        std::vector<LightBarP> light_bars;
        // add items into light_bars and do predict.
        for (aim_deps::Light light : match.possibles) {
            LightBarP lbp(light.box);
            if (module.find_light(lbp))
                light_bars.emplace_back(lbp);
            else {
                // TODO: use armors failed to predict to areate new car module.
                ;
            }
        }
        //bundleAdjustment(light_bars, module, K);

        // TODO: change to use the module to draw.
        std::vector<cv::Mat> rMats;
        std::vector<cv::Mat> tMats;
        std::vector<cv::Mat> Twcs;
        pos_getter.packUp(rMats, tMats, tar_list);      ///取得rMats, tMats(内部clear这两个Mat容器)
        for(int i=0; i<rMats.size(); i++){
            cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
            cv::Mat temp2, Twc;
            cv::hconcat(rMats[i], tMats[i] / 1000, temp2);
            cv::vconcat(temp2, temp, Twc);
            Twcs.push_back(Twc);
        }
        // std::cout <<"Twcs[0]: "<< Twcs[0] << std::endl;

        match.drawLights(frame);							//绘制所有灯条
        amp.drawArmorPlates(frame, tar_list, 0);							//绘制装甲板
        viewer->mDrawer.SetCurrentArmorPoses(Twcs);
        cv::imshow("disp", frame);
        char key = cv::waitKey(0);
        if(key==27)
            break;
	}

	cv::destroyAllWindows();
    return 0;
}
