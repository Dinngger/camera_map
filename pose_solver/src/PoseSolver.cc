/**
 * Top module of Pose Solver
 * created date: 2020.3.3
 * by Dinger
 */

#include "PoseSolver.hpp"

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

    carMatch.runMatch(match.possibles);
    for (const aim_deps::Light& light : match.possibles){
        std::cout<<"index="<<light.index<<", angle="<<light.box.angle<<", len="<<light.box.length<<", center="<<light.box.center<<", upvex="<<light.box.vex[0]<<", downvex="<<light.box.vex[1]<<std::endl;
    }

    for (size_t i=0;i<match.possibles.size();i++){
        for (size_t j=i+1;j<match.possibles.size();j++){
            float dist = aim_deps::getPointDist(match.possibles[i].box.center, match.possibles[j].box.center);
            // float len = std::max(match.possibles[i].box.length, match.possibles[j].box.length);
            float len = (match.possibles[i].box.length + match.possibles[j].box.length)/2;
            float ratio = dist/len/len;
            std::cout<<"i="<<match.possibles[i].index<<", j="<<match.possibles[j].index<<", ratio="<<ratio<<std::endl;
        }
    }

    for (const aim_deps::Armor& armor : tar_list) {
        if (armor.valid) {
            match.possibles[armor.left_light.index].isLeft = 0;
            match.possibles[armor.right_light.index].isLeft = 1;
        }
    }

    // 在上一帧中寻找匹配
    NNSearch search_last_frame;
    for (const LightBarP& lbp : match_result)
        search_last_frame.getTargetLBPs().push_back(lbp);
    search_last_frame.finishSetTarget();
    search_last_frame.runSearch(match.possibles, 80);

    std::set<int> id_set1;
    search_last_frame.getHeapIdSet(id_set1);
    for (size_t i=0; i<match.possibles.size(); i++) {
        if (match.possibles[i].isLeft < 0)
            id_set1.insert(i);
    }

    NNSearch search_module;
    module.create_predict(time, search_module.getTargetLBPs(), search_last_frame.getResultLBPs());

    std::cout << "\033[33m" << "predict result: ";
    for (const LightBarP& temp_lbp : search_module.getTargetLBPs())
        std::cout << temp_lbp.car_id << temp_lbp.armor_id << temp_lbp.lb_id << ", ";
    std::cout << "\033[0m\n";

    search_module.finishSetTarget();
    search_module.runSearch(match.possibles, 100, id_set1);

    match_result.clear();
    std::cout << "\033[34m" << "find in last frame:\n";
    for (const LightBarP& lbp : search_last_frame.getResultLBPs()) {
        match_result.push_back(lbp);
        std::cout << lbp.car_id << lbp.armor_id << lbp.lb_id << ": " << lbp.center(0) << ", " << lbp.center(1) << "; ";
    }
    std::cout << "\033[0m\n";
    std::cout << "\033[32m" << "find in module:\n";
    for (const LightBarP& lbp : search_module.getResultLBPs()) {
        match_result.push_back(lbp);
        std::cout << lbp.car_id << lbp.armor_id << lbp.lb_id << ": " << lbp.center(0) << ", " << lbp.center(1) << "; ";
    }
    std::cout << "\033[0m\n";
    for (size_t i=0; i<match_result.size(); i++) {
        for (size_t j=i+1; j<match_result.size(); j++) {
            if (match_result[i].car_id == match_result[j].car_id && match_result[i].armor_id == match_result[j].armor_id &&
                match_result[i].lb_id != match_result[j].lb_id) {
                if ((match_result[i].lb_id < match_result[j].lb_id && match_result[i].center(0) > match_result[j].center(0)) ||
                    (match_result[i].lb_id > match_result[j].lb_id && match_result[i].center(0) < match_result[j].center(0))) {
                    std::swap(match_result[i].lb_id, match_result[j].lb_id);
                }
            }
        }
    }

    module.bundleAdjustment(match_result, time);

    std::set<int> id_set2;
    search_module.getHeapIdSet(id_set2);
    for (const aim_deps::Armor &armor : tar_list) {
        if (!armor.valid)
            continue;
        if (id_set2.count(armor.left_light.index) && id_set2.count(armor.right_light.index)) {
            Armor3d a3d = toArmor3d(armor);
            int new_car_id = module.add_car(a3d);
            int light_id = armor.left_light.index;
            for (int i=0; i<2; i++) {
                const aim_deps::LightBox &lb = match.possibles[light_id].box;
                match_result.emplace_back(new_car_id, 0, i, lb.center, lb.vex[0] - lb.center);
                light_id = armor.right_light.index;
            }
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
