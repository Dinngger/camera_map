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

PoseSolver::PoseSolver(cv::Matx<double, 3, 3> &K, double time) :
    K (K),
    module(K, time)
{
}

int PoseSolver::run(const cv::Mat &frame, double time)
{
    match.saveImg(frame);
    match.findPossible();

    carMatch.transformerMatch(match.possibles);
    std::vector<int> cars(0);                      // 如果需要禁用每辆车的二分图构建，则传入一个空的cars即可
    // for (size_t i = 0; i < cars.size(); i++) {
    //     cars[i] = carMatch.r_trans[i];
    // }
    amp.matchAll(cars, match.matches, match.possibles, tar_list);
    pos_getter.batchProcess(tar_list);              ///外部pnp解算所有装甲板
    if (carMatch.carsPossible.size() < 1)
        return 0;
    std::vector<std::vector<LightBarP> > carsPossible;
    for (uint i = 0, n_cars = 0; i < carMatch.carsPossible.size(); i++) {
        if (carMatch.carsPossible[i].first < 0)
            continue;
        carsPossible.push_back(std::vector<LightBarP>());

        carsPossible.back().clear();

        if (carMatch.carsPossible[i].lightPossibles.size() <= 1) {
            n_cars ++;
            continue;
        }
        int first_index = carMatch.carsPossible[i].first_index;
        std::cout << "car: " << carMatch.carsPossible[i].first << " light: ";
        for (size_t j=0; j<carMatch.carsPossible[i].lightPossibles.size(); j++) {
            const aim_deps::LightBox &lb = carMatch.carsPossible[i].lightPossibles[j].box;
            LightBarP lbp(lb.center, lb.vex[0] - lb.center);
            if ((j & 0x1) == (first_index & 0x1))               // 与index同奇偶性，则标志应该相同
                lbp.lb_id = carMatch.carsPossible[i].first;
            else                                                // 与index不同奇偶性，则标志不同
                lbp.lb_id = 1 - carMatch.carsPossible[i].first;
            // hqy yyds!
            std::cout << lbp.lb_id << " ";
            carsPossible[n_cars].emplace_back(lbp);
        }
        n_cars ++;
        std::cout << "\n";
    }

    std::vector<LightBarP> predict2d;
    module.create_predict(time, predict2d);

    std::cout << "\033[33m" << "predict result: ";
    for (const LightBarP& temp_lbp : predict2d)
        std::cout << temp_lbp.car_id << temp_lbp.armor_id << temp_lbp.lb_id << ", ";
    std::cout << "\033[0m\n";

    for (std::vector<LightBarP>& car : carsPossible) {
        if (car.size() <= 1)
            continue;
        for (LightBarP& lbp : car) {
            double min_dist = 1e5;
            int min_id = -1;
            for (size_t j=0; j<predict2d.size(); j++) {
                if (predict2d[j].lb_id >= 0 && lbp.lb_id >= 0) {
                    if (lbp.lb_id != predict2d[j].lb_id)
                        continue;
                }
                double dist = (lbp.center - predict2d[j].center).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    min_id = j;
                }
            }
            if (min_id >= 0 && min_dist < 60) {
                lbp.car_id = predict2d[min_id].car_id;
                lbp.armor_id = predict2d[min_id].armor_id;
                lbp.lb_id = predict2d[min_id].lb_id;
            }
        }
    }

    for (std::vector<LightBarP>& car : carsPossible) {
        if (car.size() <= 1)
            continue;
        std::cout << "car: ";
        std::map<int, int> car_id_num;
        for (LightBarP& lbp : car) {
            if (car_id_num.count(lbp.car_id) == 0) {
                car_id_num[lbp.car_id] = 1;
            } else {
                car_id_num[lbp.car_id]++;
            }
        }
        int max_car_id = -1, max_car_num = 0;
        for (const std::pair<int, int>& pair : car_id_num) {
            if (pair.second > max_car_num) {
                max_car_id = pair.first;
                max_car_num = pair.second;
            }
        }
        for (LightBarP& lbp : car) {
            lbp.car_id = max_car_id;
        }
        std::cout << max_car_id << " armor: ";

        std::map<int, int> armor_id_num;
        for (size_t i=0; i<car.size(); i++) {
            int _armor_id;
            std::cout << car[i].armor_id << ":" << car[i].lb_id << " ";
            if (car[i].armor_id < 0)
                _armor_id = (car[i].lb_id + i) % 2 - 2;
            else
                _armor_id = (car[i].armor_id * 2 + car[i].lb_id - i + 8) % 8;
            if (armor_id_num.count(_armor_id) == 0) {
                armor_id_num[_armor_id] = 1;
            } else {
                armor_id_num[_armor_id]++;
            }
        }
        int max_armor_id = -1, max_armor_num = 0;
        for (const std::pair<int, int>& pair : armor_id_num) {
            if (pair.second > max_armor_num) {
                max_armor_id = pair.first;
                max_armor_num = pair.second;
            }
        }
        if (max_armor_id < 0)
            max_armor_id += 2;
        std::cout << "final_lb: ";
        for (size_t i=0; i<car.size(); i++) {
            car[i].armor_id = ((max_armor_id + i) % 8) / 2;
            car[i].lb_id = (max_armor_id + i) % 2;
            std::cout << car[i].armor_id << car[i].lb_id << " ";
        }
        std::cout << "\n";
    }

    module.bundleAdjustment(carsPossible, time);
    printf("finished ba\n");
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
    match.drawLights(frame, carMatch.r_trans);							//绘制所有灯条
    amp.drawArmorPlates(frame, tar_list, 0);		    //绘制装甲板
    return 0;
}

int PoseSolver::get_lbs(std::vector<cv::Point3d> &lbs)
{
    module.get_lbs(lbs);
    return 0;
}
