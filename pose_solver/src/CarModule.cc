/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.4.9 23:54
*/

#include <set>
#include <queue>
#include "CarModule.hpp"

Eigen::Vector2d CarModule::projectPoint(Eigen::Vector3d p3) const
{
    p3 = K * p3 / p3(2);
    return Eigen::Vector2d(p3(0), p3(1));
}

int CarModule::bundleAdjustment(const std::vector<std::vector<LightBarP>> &division,
                                double time)
{
    double delta_time = time - module_time;
    for (const std::vector<LightBarP>& car : division) {
        int i = car[0].car_id;
        while (i >= (int)cars.size())
            add_car();
        if (!cars[i].car_valid)
            continue;
        cars[i].bundleAdjustment(car, K, delta_time);
    }
    module_time = time;
    return 0;
}

// Predict 2d light bar with time, @return number of 2d light bar.
int CarModule::create_predict(double time, std::vector<LightBarP>& predict2d,
                              const std::vector<LightBarP>& found_set) const
{
    struct LBPointer {
        int car_id, armor_id, lb_id;
        LBPointer(int car_id=0, int armor_id=0, int lb_id=0) :
            car_id(car_id), armor_id(armor_id), lb_id(lb_id) {}
        LBPointer(const LightBarP& lbp) :
            car_id(lbp.car_id), armor_id(lbp.armor_id), lb_id(lbp.lb_id) {}
        bool operator<(const LBPointer& lbp) const {
            if (car_id != lbp.car_id)
                return car_id < lbp.car_id;
            else {
                if (armor_id != lbp.armor_id)
                    return armor_id < lbp.armor_id;
                else
                    return lb_id < lbp.lb_id;
            }
        }
    };
    std::set<LBPointer> LBPset;
    for (const LightBarP& lbp : found_set)
        LBPset.insert(lbp);
    int num = 0;
    for (size_t c=0; c < cars.size(); c++) {
        if (!cars[c].car_valid)
            continue;
        Eigen::Quaterniond car_r;
        Eigen::Vector3d car_t;
        cars[c].predict(time - module_time, car_r, car_t);
        Eigen::Matrix3d car_R = car_r.matrix();
        for (size_t i=0; i<4; i++) {
            Eigen::Matrix3d armor_R = cars[c].armor[i].r.matrix();
            const Eigen::Vector3d& armor_t = cars[c].armor[i].t;
            Eigen::Vector3d rotated_armor_point[4];
            for (size_t j=0; j<4; j++) {
                rotated_armor_point[j] = car_R * (armor_R * armor_module[j] + armor_t) + car_t;
            }
            for (size_t j=0; j<2; j++) {
                LBPointer this_lbp(c, i, j), left_lbp, right_lbp;
                if (j == 0) {
                    right_lbp = LBPointer(c, i, 1);
                    if (i == 0)
                        left_lbp = LBPointer(c, 3, 1);
                    else
                        left_lbp = LBPointer(c, i-1, 1);
                } else {
                    left_lbp = LBPointer(c, i, 0);
                    if (i == 3)
                        right_lbp = LBPointer(c, 0, 0);
                    else
                        right_lbp = LBPointer(c, i+1, 0);
                }
                if ((!LBPset.count(this_lbp)) && (LBPset.count(left_lbp) || LBPset.count(right_lbp))) {
                    Eigen::Vector2d point2d[2];
                    point2d[0] = projectPoint(rotated_armor_point[2*j]);
                    point2d[1] = projectPoint(rotated_armor_point[2*j+1]);
                    predict2d.emplace_back(c, i, j, (point2d[0] + point2d[1])/2, (point2d[0] - point2d[1])/2);
                    num++;
                }
            }
        }
    }
    return num;
}

// Predict 2d light bar with time, @return number of 2d light bar.
int CarModule::create_predict(double time, std::vector<LightBarP>& predict2d) const {
    struct LbpComp {
        LightBarP lbp;
        double dist;
        LbpComp(const LightBarP& _lbp, double _dist) {
            lbp = _lbp;
            dist = _dist;
        }
        bool operator<(const LbpComp& lbpc) const {
            return dist > lbpc.dist;
        }
    };
    for (size_t c=0; c < cars.size(); c++) {
        if (!cars[c].car_valid)
            continue;
        std::priority_queue<LbpComp> heap;
        Eigen::Quaterniond car_r;
        Eigen::Vector3d car_t;
        cars[c].predict(time - module_time, car_r, car_t);
        Eigen::Matrix3d car_R = car_r.matrix();
        for (size_t i=0; i<4; i++) {
            Eigen::Matrix3d armor_R = cars[c].armor[i].r.matrix();
            const Eigen::Vector3d& armor_t = cars[c].armor[i].t;
            Eigen::Vector3d rotated_armor_point[4];
            for (size_t j=0; j<4; j++) {
                rotated_armor_point[j] = car_R * (armor_R * armor_module[j] + armor_t) + car_t;
            }
            for (int j=0; j<2; j++) {
                Eigen::Vector2d point2d[2];
                point2d[0] = projectPoint(rotated_armor_point[2*j]);
                point2d[1] = projectPoint(rotated_armor_point[2*j+1]);
                heap.push(LbpComp(LightBarP(c, i, j, (point2d[0] + point2d[1])/2, (point2d[0] - point2d[1])/2),
                    (rotated_armor_point[2*j](2) + rotated_armor_point[2*j+1](2))/2));
            }
        }
        for (int i=0; i<6; i++) {
            predict2d.emplace_back(heap.top().lbp);
            heap.pop();
        }
    }
    return 0;
}

int CarModule::add_car()
{
    Car c;
    c.confidence[0] = 1;
    c.t = Eigen::Vector3d(0, 0, 5);
    c.armor[0].t = Eigen::Vector3d(0, 0, -0.25);
    c.armor[0].r = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 12, Eigen::Vector3d(-1, 0, 0)));
    Eigen::Quaterniond _r(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, -1, 0)));
    Eigen::Matrix3d _R = _r.matrix();
    for (int i=1; i<4; i++) {
        c.armor[i].t = _R * c.armor[i-1].t;
        c.armor[i].r = _r * c.armor[i-1].r;
    }
    c.r.setIdentity();
    c.update_state();
    cars.push_back(c);
    std::cout << "\033[42m"<< "successfully add a car! ";
    std::cout << "now car number: " << cars.size() << "\033[0m\n";
    return cars.size() - 1;
}

void CarModule::get_lbs(std::vector<cv::Point3d> &lbs) const {
    for (size_t i=0; i<cars.size(); i++) {
        if (!cars[i].car_valid)
            continue;
        Eigen::Matrix3d car_R = cars[i].r.matrix();
        Eigen::Vector3d car_t = cars[i].t;
        for (int j=0; j<4; j++) {
            Eigen::Matrix3d armor_R = cars[i].armor[j].r.matrix();
            Eigen::Vector3d armor_t = cars[i].armor[j].t;
            for (int k=0; k<4; k++) {
                Eigen::Vector3d tmp_p = car_R * (armor_R * armor_module[k] + armor_t) + car_t;
                lbs.emplace_back(tmp_p(0), tmp_p(1), tmp_p(2));
            }
        }
    }
}
