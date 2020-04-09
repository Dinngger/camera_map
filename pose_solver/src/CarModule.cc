/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.4.9 23:54
*/

#include "CarModule.hpp"

Eigen::Vector2d CarModule::projectPoint(Eigen::Vector3d p3) const
{
    p3 = K * p3 / p3(2);
    return Eigen::Vector2d(p3(0), p3(1));
}

int CarModule::bundleAdjustment(const std::vector<LightBarP> &light_bars,
                                double time)
{
    double delta_time = time - module_time;
    if (cars.size() < 1)
        return 0;
    std::vector<LightBarP> light_bars_car[cars.size()];
    for (const LightBarP& lbp : light_bars) {
        light_bars_car[lbp.car_id].push_back(lbp);
    }
    for (size_t i=0; i<cars.size(); i++) {
        int size = light_bars_car[i].size();
        // cars[i].car_info = (cars[i].car_info + size) / 2;
        if (size > 0)
            cars[i].bundleAdjustment(light_bars_car[i], K, delta_time);
    }
    module_time = time;
    return 0;
}

// Predict 2d light bar with time, @return number of 2d light bar.
int CarModule::create_predict(double time, std::vector<LightBarP>& predict2d) const
{
    int num = 0;
    for (size_t c=0; c < cars.size(); c++) {
        Eigen::Quaterniond car_r;
        Eigen::Vector3d car_t;
        cars[c].predict(time - module_time, car_r, car_t);
        Eigen::Matrix3d car_R = car_r.matrix();
        for (size_t i=0; i<4; i++) {
            Eigen::Matrix3d armor_R = cars[c].armor[i].r.matrix();
            const Eigen::Vector3d& armor_t = cars[c].armor[i].t;
            Eigen::Vector3d rotated_armor_point[4];
            for (size_t j=0; j<4; j++) {
                rotated_armor_point[j] = car_R * (armor_R * armor_module[j] + armor_t);
            }
            for (size_t j=0; j<2; j++) {
                if (((rotated_armor_point[2*j] + rotated_armor_point[2*j+1]) / 2)(2) <= 0) {
                    Eigen::Vector2d lbp[2];
                    lbp[0] = projectPoint(rotated_armor_point[2*j] + car_t);
                    lbp[1] = projectPoint(rotated_armor_point[2*j+1] + car_t);
                    predict2d.emplace_back(c, i, j, (lbp[0] + lbp[1])/2, (lbp[0] - lbp[1])/2);
                    num++;
                }
            }
        }
    }
    return num;
}

int CarModule::add_car(const Armor3d& _armor)
{
    Car c;
    c.t = _armor.t + Eigen::Vector3d(0, 0, 0.25);
    c.armor[0].t = Eigen::Vector3d(0, 0, -0.25);
    c.armor[0].r = _armor.r;
    Eigen::Quaterniond _r(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, -1, 0)));
    Eigen::Matrix3d _R = _r.matrix();
    for (int i=1; i<4; i++) {
        c.armor[i].t = _R * c.armor[i-1].t;
        c.armor[i].r = _r * c.armor[i-1].r;
    }
    c.r.setIdentity();
    c.regularzation();
    c.update_state();
    cars.push_back(c);
    std::cout << "successfully add a car! ";
    std::cout << "now car number: " << cars.size() << std::endl;
    return 0;
}

void CarModule::get_lbs(std::vector<cv::Point3d> &lbs) const {
    for (size_t i=0; i<cars.size(); i++) {
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
