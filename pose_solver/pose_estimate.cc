#include "LightMatch.hpp"
#include "ArmorPlate.hpp"
#include "GetPos.hpp"
#include "sampling.hpp"
#include "CarModule.hpp"
#include "Viewer.h"
#include <thread>
#include <mutex>
#include <opencv2/core/eigen.hpp>

int main(int argc, char* argv[])
{
    std::vector<aim_deps::Armor> tar_list(16);                  /// tar_list不再是match内部的成员，而是一个更高层的类的成员
    tar_list.clear();                                           /// 
    cv::VideoCapture cap("../../cv_output1.avi");
    if (!cap.isOpened()) {
        printf("Unable to open video.\n");
        return 0;
    }
    cv::Matx<double, 3, 3> K  ( 1776.67168581218, 0, 720,
                                0, 1778.59375346543, 540,
                                0, 0, 1);
    CarModule module(K);
    ArmorPlate amp;
    LightMatch match;
    cv::Mat frame, screen;
    GetPos pos_getter;                                          /// PNP解算模块
    Viewer *viewer = new Viewer(K(0, 0), K(1, 1));
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);

    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::vector<cv::Mat>  point_answer;
    std::cout<<"frame: ";
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
        bool failed[match.possibles.size()];                //失败的标记
        for (int i = 0; i<match.possibles.size(); ++i) {
            failed[i] = true;
        }
        for (aim_deps::Armor armor: tar_list) {
            failed[armor.left_light.index] = false;
            failed[armor.right_light.index] = false;
            LightBarP lbp(armor.left_light.box);
            if (module.find_light(lbp)) {
                light_bars.emplace_back(lbp);
            } else {
                LightBarP t_lbp(armor.right_light.box);
                if (module.find_light(t_lbp)) {
                    ///TODO: 一个找到一个没找到的函数
                    ;
                } else {
                    module.add_car(armor.vertex);
                }
            }
        }
        for (int i = 0; i<match.possibles.size(); ++i) {
            if (failed[i]) {
                LightBarP lbp(match.possibles[i].box);
                if (module.find_light(lbp)) {
                    light_bars.emplace_back(lbp);
                } else {
                    ///TODO: 单独的灯条（新出现的）
                    ;
                }
            }
        }
        std::cout << "light_bars: " << light_bars.size() << std::endl;
        module.bundleAdjustment(light_bars);

        // TODO: change to use the module to draw.
        std::vector<cv::Mat> rMats, tMats, Twcs;
        pos_getter.packUp(rMats, tMats, tar_list);      ///取得rMats, tMats(内部clear这两个Mat容器)
        for(size_t i=0; i<rMats.size(); i++){
            cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
            cv::Mat temp2, Twc;
            cv::hconcat(rMats[i], tMats[i] / 1000, temp2);
            cv::vconcat(temp2, temp, Twc);
            Twcs.push_back(Twc);
        }
        // std::cout <<"Twcs[0]: "<< Twcs[0] << std::endl;

        match.drawLights(frame);							//绘制所有灯条
        amp.drawArmorPlates(frame, tar_list, 0);							//绘制装甲板


        std::vector<cv::Point3f> lbs;
        module.get_lbs(lbs);
        viewer->mDrawer.SetCurrentArmorPoses(Twcs, lbs);

        cv::imshow("disp", frame);
        char key = cv::waitKey(0);
        if(key==27)
            break;
	}

    viewer->RequestFinish();
	cv::destroyAllWindows();
    while (mpViewer->joinable())
        mpViewer->join();
    return 0;
}
