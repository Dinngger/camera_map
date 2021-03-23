#include <set>
#include <queue>
#include "newCarModule.hpp"

int newCarModule::add_newcar(Eigen::Vector2d t,double r)
{
    newcar Newcar(t,r);
    newcars.push_back(Newcar);
    return 0;
}

void newCarModule::get_lbs(std::vector<cv::Point3d> &lbs) const {
    for (size_t i=0; i<newcars.size(); i++) {
        double car_R = newcars[i].r;
        Eigen::Vector2d car_t = newcars[i].t;
        for (int k=0; k<4; k++) {
            Eigen::Vector3d tmp_p = car_R * armor_module[k] + car_t;
            lbs.emplace_back(tmp_p(0), tmp_p(1), tmp_p(2));
        }
    }
}
