/**
 * Top module of Pose Solver
 * created date: 2020.3.3
 * by Dinger
 */

#ifndef __POSE_SOLVER_HPP
#define __POSE_SOLVER_HPP

#include <queue>
#include "LightMatch.hpp"
#include "ArmorPlate.hpp"
#include "GetPos.hpp"
#include "sampling.hpp"
#include "CarModule.hpp"

Armor3d toArmor3d(const aim_deps::Armor& armor) {
    Armor3d _armor;
    _armor.t = Eigen::Vector3d(armor.t_vec.x / 1000, armor.t_vec.y / 1000, armor.t_vec.z / 1000);
    cv::Mat rotation;
    cv::Rodrigues(armor.r_vec, rotation);
    Eigen::Map<Eigen::Matrix3d> eR((double*)rotation.data);
    eR.transposeInPlace();
    _armor.r = Eigen::Quaterniond(eR);
    return _armor;
}

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
    int get_lbs(std::vector<cv::Point3d> &lbs);
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
    match.saveImg(frame);
    match.findPossible();

    amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条
    pos_getter.batchProcess(tar_list);              ///外部pnp解算所有装甲板

    std::vector<LightBarP> predict2d;
    std::cout << "predict num: " << module.create_predict(time, predict2d) << " ";
    for (const LightBarP& lbp : predict2d)
        std::cout << lbp.armor_id << "+" << lbp.lb_id << ", ";
    std::cout << std::endl;
    bool found[predict2d.size()];
    for (bool& f : found)
        f = false;
    std::vector<LightBarP> light_bar_pairs;
    struct Lbp_ptr {
        int id;
        int tar_id;
        double distance;
        Lbp_ptr(int id, int tar_id, double distance) :
            id(id), tar_id(tar_id), distance(distance) {}
        bool operator>(const Lbp_ptr &x) const {
            return distance > x.distance;
        }
    };
    auto getDist = [&predict2d, &found](int& min_id, double& min_dist, const LightBarP& lbp) {
        for (size_t j=0; j<predict2d.size(); j++) {
            if (!found[j]) {
                double dist = (lbp.center - predict2d[j].center).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    min_id = j;
                }
            }
        }
    };
    std::priority_queue<Lbp_ptr, std::vector<Lbp_ptr>, std::greater<Lbp_ptr>> heap;
    for (size_t i=0; i<match.possibles.size(); ++i) {
        aim_deps::LightBox &lb = match.possibles[i].box;
        LightBarP lbp(lb.center, lb.vex[0] - lb.center);
        // better to use kd tree
        int min_id = -1;
        double min_dist = 1e10;
        getDist(min_id, min_dist, lbp);
        heap.emplace(i, min_id, min_dist);
    }
    while (heap.top().distance < 80) {
        Lbp_ptr tmp = heap.top();
        std::cout << "id: " << tmp.id << " tar: " << tmp.tar_id << " dist: " << tmp.distance << "\n";
        heap.pop();
        aim_deps::LightBox &lb = match.possibles[tmp.id].box;
        LightBarP lbp(lb.center, lb.vex[0] - lb.center);
        if (!found[tmp.tar_id]) {
            found[tmp.tar_id] = true;
            light_bar_pairs.emplace_back(predict2d[tmp.tar_id], lbp);
            if (heap.empty())
                break;
        } else {
            // better to use kd tree
            int min_id = -1;
            double min_dist = 1e10;
            getDist(min_id, min_dist, lbp);
            heap.emplace(tmp.id, min_id, min_dist);
        }
    }
    module.bundleAdjustment(light_bar_pairs, time);
    std::set<int> id_set;
    while (!heap.empty()) {
        id_set.insert(heap.top().id);
        heap.pop();
    }
    for (const aim_deps::Armor &armor : tar_list) {
        if (!armor.valid)
            continue;
        if (id_set.count(armor.left_light.index) && id_set.count(armor.right_light.index)) {
            Armor3d a3d = toArmor3d(armor);
            module.add_car(a3d);
        }
    }
    return 0;
}

int PoseSolver::getTwcs(std::vector<cv::Mat> &Twcs)
{
    std::vector<cv::Mat> rMats, tMats;
    pos_getter.packUp(rMats, tMats, tar_list);   ///取得rMats, tMats(内部clear这两个Mat容器)
    std::cout << "armor number: " << rMats.size() << "\n";
    for (size_t i=0; i<rMats.size(); i++) {
        cv::Mat temp =(cv::Mat_<double>(1,4) << 0,0,0,1);
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

int PoseSolver::get_lbs(std::vector<cv::Point3d> &lbs)
{
    module.get_lbs(lbs);
    return 0;
}

#endif // __POSE_SOLVER_HPP
