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
#include "CarModule.hpp"
#include "NNSearch.hpp"
#include "CarMatch.hpp"
#include "newCarModule.hpp"

Armor3d toArmor3d(const aim_deps::Armor& armor);

class PoseSolver
{
private:
    cv::Matx<double, 3, 3> K;
    CarModule module;
    newCarModule new_module;
    ArmorPlate amp;
    GetPos pos_getter;                     /// PNP解算模块
    CarMatch carMatch;
    std::vector<LightBarP> match_result;
public:
    std::vector<aim_deps::Armor> tar_list;
    LightMatch match;
    PoseSolver(cv::Matx<double, 3, 3> &K, double time);
    int run(const cv::Mat &frame, double time);
    int newrun(const cv::Mat &frame,double time);
    int getTwcs(std::vector<cv::Mat> &Twcs);
    int draw(cv::Mat &frame);
    int get_lbs(std::vector<cv::Point3d> &lbs);
};

#endif // __POSE_SOLVER_HPP
