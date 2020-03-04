/**
 * Top module of Pose Solver
 * created date: 2020.3.3
 * by Dinger
 */

#ifndef __POSE_SOLVER_HPP
#define __POSE_SOLVER_HPP

#include "LightMatch.hpp"
#include "ArmorPlate.hpp"
#include "GetPos.hpp"
#include "sampling.hpp"
#include "CarModule.hpp"

class PoseSolver
{
private:
    std::vector<aim_deps::Armor> tar_list;
    cv::Matx<double, 3, 3> K;
    CarModule module;
    ArmorPlate amp;
    LightMatch match;
    GetPos pos_getter;                     /// PNP解算模块
public:
    PoseSolver(cv::Matx<double, 3, 3> &K);
    int run(const cv::Mat &frame, double time);
    int getTwcs(std::vector<cv::Mat> &Twcs);
    int draw(cv::Mat &frame);
    int get_lbs(std::vector<cv::Point3f> &lbs);
};

PoseSolver::PoseSolver(cv::Matx<double, 3, 3> &K) :
    tar_list(16),
    K (K),
    module(K)
{
    tar_list.clear();
}

int PoseSolver::run(const cv::Mat &frame, double time)
{
    if (!isLowExposure(frame))
        return 0;
    match.saveImg(frame);
    match.findPossible();

    amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条
    pos_getter.batchProcess(tar_list);              ///外部pnp解算所有装甲板

    //传入时间预测此时装甲版的平面位置
    module.create_predict(time);
    //观测到的灯条
    std::vector<LightBarP> light_bars;
    bool failed[match.possibles.size()];                //失败的标记
    for (int i = 0; i<match.possibles.size(); ++i) {
        failed[i] = true;
    }
    for (aim_deps::Armor armor: tar_list) {
        failed[armor.left_light.index] = false;
        failed[armor.right_light.index] = false;
        LightBarP lbpl(armor.left_light.box);
        LightBarP lbpr(armor.right_light.box);
        bool find_left = module.find_light(lbpl);
        bool find_right = module.find_light(lbpr);
        if (find_left)
            light_bars.emplace_back(lbpl);
        if (find_right)
            light_bars.emplace_back(lbpr);
        if (!(find_left || find_right))
            ///TODO: use pnp result to fix this error.
            module.add_car(armor.vertex);
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
    module.bundleAdjustment(light_bars);
}

int PoseSolver::getTwcs(std::vector<cv::Mat> &Twcs)
{
    std::vector<cv::Mat> rMats, tMats;
    pos_getter.packUp(rMats, tMats, tar_list);   ///取得rMats, tMats(内部clear这两个Mat容器)
    for(size_t i=0; i<rMats.size(); i++){
        cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
        cv::Mat temp2, Twc;
        cv::hconcat(rMats[i], tMats[i] / 1000, temp2);
        cv::vconcat(temp2, temp, Twc);
        Twcs.push_back(Twc);
    }
    return 0;
}

int PoseSolver::draw(cv::Mat &frame)
{
    match.drawLights(frame);							//绘制所有灯条
    amp.drawArmorPlates(frame, tar_list, 0);		    //绘制装甲板
    return 0;
}

int PoseSolver::get_lbs(std::vector<cv::Point3f> &lbs)
{
    module.get_lbs(lbs);
    return 0;
}

#endif // __POSE_SOLVER_HPP
