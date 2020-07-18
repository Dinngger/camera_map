/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.2.25 00:33
*/

#ifndef __CAR_MODULE_HPP
#define __CAR_MODULE_HPP

#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

///IMPORTENT: change this to the average time between two frames.
#define DELTA_TIME 1

template<typename T>
T& interpolate(T& a, const T& b, double t) {
    a = a * (1 - t) + b * t;
    return a;
}

const Eigen::Vector3d armor_module[4] = {
    Eigen::Vector3d(-0.065, -0.0285, 0),
    Eigen::Vector3d(-0.065,  0.0285, 0),
    Eigen::Vector3d( 0.065, -0.0285, 0),
    Eigen::Vector3d( 0.065,  0.0285, 0)
};

// p[0] is the upper point.
struct Armor3d
{
    int car_id=-1, armor_id=-1;
    // double info=1;    // info belongs to [0, 1], 0 means perfectly know, 1 means know nothing.
    Eigen::Quaterniond r;
    Eigen::Vector3d t;
};

struct LightBarP
{
    int car_id, armor_id, lb_id;
    Eigen::Vector2d center;
    Eigen::Vector2d p;  // the vector from center to the upper point.
    LightBarP(cv::Point2f _center, cv::Point2f _p) :
        car_id(-1),
        armor_id(-1),
        lb_id(-1) {
        center = Eigen::Vector2d(_center.x, _center.y);
        p = Eigen::Vector2d(_p.x, _p.y);
    }
    LightBarP(int car_id, int armor_id, int lb_id, cv::Point2f _center, cv::Point2f _p) :
        car_id(car_id),
        armor_id(armor_id),
        lb_id(lb_id) {
        center = Eigen::Vector2d(_center.x, _center.y);
        p = Eigen::Vector2d(_p.x, _p.y);
    }
    LightBarP(int car_id, int armor_id, int lb_id, Eigen::Vector2d center, Eigen::Vector2d p) :
        car_id(car_id),
        armor_id(armor_id),
        lb_id(lb_id),
        center(center),
        p(p) {
    }
    LightBarP(const LightBarP& lbp1, const LightBarP& lbp2) :
        car_id(lbp1.car_id),
        armor_id(lbp1.armor_id),
        lb_id(lbp1.lb_id),
        center(lbp2.center),
        p(lbp2.p) {
    }
    Eigen::Vector2d operator[](int i) const {
        if (i == 0)
            return center + p;
        else
            return center - p;
    }
};

class KeyFrame
{
public:
    bool valid;
    double score;
    Eigen::Quaterniond kf_r;
    Eigen::Vector3d kf_t;
    std::vector<LightBarP> lbps;
    KeyFrame() : valid(false), score(100) {}
};

// lb[0] was the first light bar to be seen.
class Car
{
private:
    int color=-1;
    int number=-1; // -1 means unknow
    Armor3d armor[4];
    double confidence_sum=0;
    double confidence[4];  // 0 to 1

    Eigen::Quaterniond dr;
    Eigen::Quaterniond last_r;
    Eigen::Quaterniond r;
    Eigen::Vector3d ddt;
    Eigen::Vector3d last_dt;
    Eigen::Vector3d dt;
    Eigen::Vector3d last_t;
    Eigen::Vector3d t;

    KeyFrame kfs[4];

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
    bool car_valid=true;
    double car_info=1;  // Average number of light bars recently seen.
    Car() {
        ddt = Eigen::Vector3d::Zero();
        last_dt = Eigen::Vector3d::Zero();
        dt = Eigen::Vector3d::Zero();
        last_t = Eigen::Vector3d::Zero();
        last_r.setIdentity();
        dr.setIdentity();
        for (int i=0; i<4; i++)
            confidence[i] = 0;
    }
    int bundleAdjustment(const std::vector<LightBarP> &light_bars,
                         const Eigen::Matrix3d &K,
                         double delta_time);
    int update_state();
    int update_state(double delta_time);    // time from last update;
    int predict(double delta_time, Eigen::Quaterniond &pre_r,
                Eigen::Vector3d &pre_t, bool linear=false) const;     // time to the future
    friend class CarModule;
};

class CarModule
{
private:
    double module_time;
    Eigen::Matrix3d K;
    std::vector<Car> cars;
    Eigen::Vector2d projectPoint(Eigen::Vector3d p3) const;

public:
    CarModule(cv::Matx<double, 3, 3> &K) : K(K.val) {
        this->K.transposeInPlace();
        std::cout << "K: \n" << this->K << std::endl;
    }
    int add_car(const Armor3d& _armor);
    int create_predict(double time, std::vector<LightBarP>& predict2d,
                       const std::vector<LightBarP>& found_set) const;
    int bundleAdjustment(const std::vector<LightBarP> &light_bars,
                         double time);
    void get_lbs(std::vector<cv::Point3d> &lbs) const;
};

#endif // __CAR_MODULE_HPP
