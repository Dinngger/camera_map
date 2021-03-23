#ifndef __NEWCARCAR_MODULE_HPP
#define __NEWCARCAR_MODULE_HPP
#include "CarModule.hpp"
#include "AimDeps.hpp"


class newcar {
    private:
    Armor3d armor;
    //double confidence;
    Eigen::Vector2d t;
    Eigen::Vector2d dt;
    Eigen::Vector2d last_t;
    double r;
    double dr;
    double last_r;
    static Eigen::Matrix3d up(const Eigen::Vector3d& a) {
        Eigen::Matrix3d res;
        res << 0, -a(2), a(1),
               a(2), 0, -a(0),
               -a(1), a(0), 0;
        return res;
    }
    static Eigen::Matrix3d exp(const Eigen::Vector3d& fi) {
        double theta = fi.norm();
        if (theta < 1e-10)
            return Eigen::Matrix3d::Identity();
        Eigen::Vector3d a = fi / theta;
        return cos(theta)*Eigen::Matrix3d::Identity() + (1 - cos(theta))*(a*a.transpose()) + sin(theta)*up(a);
    }
    static Eigen::Matrix3d jacobi(const Eigen::Vector3d& fi) {
        double theta = fi.norm();
        if (theta < 1e-10)
            return Eigen::Matrix3d::Identity();
        Eigen::Vector3d a = fi / theta;
        return sin(theta)/theta*Eigen::Matrix3d::Identity() + (1 - sin(theta) / theta)*(a*a.transpose()) + (1 - cos(theta))/theta*up(a);
    }
    public:
    int update_state();
    newcar(Eigen::Vector2d st,double sr){
        t=st;
        r=sr;
        update_state();
        dt=Eigen::Vector2d::Zero();
        dr=0.0;
    }
    newcar(){
        t=Eigen::Vector2d::Zero();
        dt=Eigen::Vector2d::Zero();
        last_t=Eigen::Vector2d::Zero();
        r=0.0;
        dr=0.0;
        last_r=0.0;
        //confidence=0.0;
    }
    int bundleAdjustment(double delta_time,Eigen::Vector2d real_t,double real_r);
    int update_state(double delta_time);    // time from last update;
    int predict(double delta_time, double &pre_r, Eigen::Vector2d &pre_t) const  ;  // time to the future
    friend class newCarModule;
};

class newCarModule {
    private:
    //Eigen::Matrix3d K;
    //Eigen::Vector2d projectPoint(Eigen::Vector3d p3) const;
    public:
    std::vector<newcar> newcars;
    double module_time;
    newCarModule(double module_time) : module_time(module_time) {}
    int add_newcar(Eigen::Vector2d t,double r);
    void get_lbs(std::vector<cv::Point3d> &lbs) const;
};

#endif