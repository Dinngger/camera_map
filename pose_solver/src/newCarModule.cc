#include <set>
#include <queue>
#include "newCarModule.hpp"

int newCarModule::add_newcar(Eigen::Vector3d t,double r,std::vector<cv::Point2f> Points)
{
    newcar Newcar(t,r);
    newcars.push_back(Newcar);
    std::cout<<"\033[33m"<<"add a car!"<<"\033[0m"<<std::endl;
    return 0;
}

void newCarModule::get_lbs(std::vector<cv::Point3d> &lbs) const {
    for (size_t i=0; i<newcars.size(); i++) {
        double r = newcars[i].r;
        Eigen::Vector3d car_t = newcars[i].t;
        car_t(0)+=0.2*cos(r);
        car_t(2)-=0.2*sin(r);           //此处没考虑清楚r的正负，可能有bug
        Eigen::Matrix3d car_R = Eigen::AngleAxisd(r, Eigen::Vector3d(0, 1, 0)).matrix();
        for (int k=0; k<4; k++) {
            Eigen::Vector3d tmp_p = car_R * armor_module[k] + car_t;
            lbs.emplace_back(tmp_p(0), tmp_p(1), tmp_p(2));
        }
    }
}
