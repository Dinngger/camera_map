#include "branch/LightMatch.hpp"
#include "branch/ArmorPlate.hpp"
#include "branch/GetPos.hpp"
#include "sampling.hpp"
#include "CarModule.hpp"
#include "include/Viewer.h"
#include "BundleAdjust.hpp"
//#include "include/ba_g2o.hpp"
#include <thread>
#include <mutex>


int main(int argc, char* argv[])
{
    std::vector<aim_deps::Armor> tar_list(16);                  /// tar_list不再是match内部的成员，而是一个更高层的类的成员
    tar_list.clear();                                           /// 
    cv::VideoCapture cap("/mine/cv_output1.avi");
    //CarModule module;
    ArmorPlate amp;
    LightMatch match;
    cv::Mat frame, screen;
    GetPos pos_getter;                                          /// PNP解算模块
    Viewer *viewer = new Viewer();
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);

    cv::Mat K = (cv::Mat_<double> (3, 3) << 1776.67168581218, 0, 720,
                                            0, 1778.59375346543, 540,
                                            0, 0, 1 );
    std::vector<cv::Mat> armor3dpoints;
    std::vector<cv::RotatedRect> result;                        ///这个与上面这一行原来就有，但是好像暂时没作用？
    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::vector<cv::Mat>  point_answer;
    std::cout<<"frame: -1"<<std::endl;
    std::cout<<totalFrameNumber<<std::endl;

    ///ArmorPlate中rMats与tMats的替代：


    int count = 0;
    for (int w = 0; w < totalFrameNumber; w++)
    {
        cap.read(frame);
		if(frame.empty())
            break;
        count++;
        if (!isLowExposure(frame))
            continue;

        ///原有的LightMatch中又一次对frame做了split
        ///在isLowExposure中做一次split后，单通道图像被保存在proced中（相当于原来的LightMatch的gray），不用再传入frame
        match.findPossible();                         

        ///原有的ArmorPlate需要输入frame进行装甲板匹配判定，现在不需要了
        ///由于tar_list在外部，所以需要传入 
        amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条

        //module.create_predict(count);
        std::vector<LightBarP> light_bars;
        // TODO: add items into light_bars and do predict.
        // TODO: use armors failed to predict to areate new car module.
        //bundleAdjustment(light_bars, module, K);

        // TODO: change to use the module to draw.
        ///注意这个地方：amp在我写的模块里没有rMats和tMats这两个成员，（原来PNP解算是在类内部，这样不方便）
        ///rMats和tMats现在可以在ArmorPlate类外被解出,使用pos_getter的batchProcess
        ///但实际上，我定义的装甲板结构体中有对应装甲板的r_vector以及t_vector(是cv::Point3f不是Mat)
        ///我现在不知道这个是做什么的
        std::vector<cv::Mat> rMats;
        std::vector<cv::Mat> tMats;
        std::vector<cv::Mat> Twcs;
        pos_getter.batchProcess(tar_list);              ///外部pnp解算所有装甲板
        pos_getter.packUp(rMats, tMats, tar_list);      ///取得rMats, tMats(内部clear这两个Mat容器)
        for(int i=0; i<rMats.size(); i++){
            cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
            cv::Mat temp2, Twc;
            cv::hconcat(rMats[i], tMats[i] / 1000, temp2);
            cv::vconcat(temp2, temp, Twc);
            Twcs.push_back(Twc);
        }
        // std::cout <<"Twcs[0]: "<< Twcs[0] << std::endl;

        ///绘制装甲板改为了最后一步，需要先确定：最佳装甲板位置（绘制成绿色），其他装甲板为黄色
        match.drawLights(frame);							//绘制所有灯条
        /// 0 是暂时的最佳装甲板在tar_list中的下标号，由于这个模块没有决策，所以直接填0
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
