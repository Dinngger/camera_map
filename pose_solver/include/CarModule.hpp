/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.2.25 00:33
*/

#ifndef __CAR_MODULE_HPP
#define __CAR_MODULE_HPP

#include <vector>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
/*
#include "g2o/core/solver.h"
#include <g2o/core/block_solver.h>
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
*/
// #include "Ruler.hpp"

///IMPORTENT: change this to the average time between two frames.
#define DELTA_TIME 1

double getDistance(const cv::Point2f p1, const cv::Point2f p2) {
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

template<typename T>
T& interpolate(T& a, const T& b, double t) {
    a = a * (1 - t) + b * t;
    return a;
}

const Eigen::Vector3d armor_module[4] = {
    Eigen::Vector3d(-0.065,  0.0285, 0),
    Eigen::Vector3d(-0.065, -0.0285, 0),
    Eigen::Vector3d( 0.065,  0.0285, 0),
    Eigen::Vector3d( 0.065, -0.0285, 0)
};

// p[0] is the upper point.
struct Armor3d
{
    int car_id=-1, armor_id=-1;
    double info=1;    // info belongs to [0, 1], 0 means perfectly know, 1 means know nothing.
    Eigen::Quaterniond r;
    Eigen::Vector3d t;
};

struct LightBarP
{
    int car_id, armor_id, lb_id;
    Eigen::Vector2d center;
    Eigen::Vector2d p;  // the vector from center to the upper point.
    LightBarP(cv::RotatedRect box) {
        cv::Point2f corners[4];                                     //找出角点
        box.points(corners);
        float d1 = getDistance(corners[0], corners[1]);             //0/1点距离的平方
        float d2 = getDistance(corners[1], corners[2]);             //1/2点距离的平方
        int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
        cv::Point2f tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;	//获得旋转矩形两条短边上的中点
        cv::Point2f tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
        cv::Point2f cv_center = (tmp_p1 + tmp_p2) / 2;
        center = Eigen::Vector2d(cv_center.x, cv_center.y);
        if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
            cv::Point2f cv_p = (tmp_p1 - tmp_p2) / 2;
            p = Eigen::Vector2d(cv_p.x, cv_p.y);
        } else{                                                     //必须是p1是处于上方的点，p2处于下方（y轴更大）
            cv::Point2f cv_p = (tmp_p2 - tmp_p1) / 2;
            p = Eigen::Vector2d(cv_p.x, cv_p.y);
        }
    }
    LightBarP(int car_id, int armor_id, int lb_id, Eigen::Vector2d center, Eigen::Vector2d p) :
        car_id(car_id),
        armor_id(armor_id),
        lb_id(lb_id),
        center(center),
        p(p) {
    }
};

// lb[0] was the first light bar to be seen.
class Car
{
private:
    int color=-1;
    int number=-1; // -1 means unknow
    Armor3d armor[4];

    Eigen::Quaterniond dr;
    Eigen::Quaterniond last_r;
    Eigen::Quaterniond r;
    Eigen::Vector3d ddt;
    Eigen::Vector3d last_dt;
    Eigen::Vector3d dt;
    Eigen::Vector3d last_t;
    Eigen::Vector3d t;

public:
    double car_info=1;  // Average number of light bars recently seen.
    Car() {
        ddt = Eigen::Vector3d::Zero();
        last_dt = Eigen::Vector3d::Zero();
        dt = Eigen::Vector3d::Zero();
        last_t = Eigen::Vector3d::Zero();
        last_r.setIdentity();
        dr.setIdentity();
    }
    int regularzation();
    int bundleAdjustment(std::vector<Armor3d> &_armors,
                         std::vector<LightBarP> &light_bars,
                         const Eigen::Matrix3d &K,
                         double delta_time);
    int update_state();
    int update_state(double delta_time);    // time from last update;
    int predict(double delta_time, Eigen::Quaterniond &pre_r,
                Eigen::Vector3d &pre_t, bool linear=false) const;     // time to the future
    friend class CarModule;
};

int Car::update_state()
{
    last_t = t;
    last_r = r;
    return 0;
}

int Car::update_state(double delta_time)
{
    dt = 0.5 * dt + (last_t - t) / delta_time;
    last_t = t;
    ddt = 0.5 * ddt + (last_dt - dt) / delta_time;
    last_dt = dt;
    dr = dr.slerp(0.5, Eigen::Quaterniond::Identity().slerp(DELTA_TIME/delta_time, (last_r.conjugate() * r)));
    last_r = r;
    return 0;
}

int Car::predict(double delta_time, Eigen::Quaterniond &pre_r, Eigen::Vector3d &pre_t, bool linear) const
{
    pre_t = t + dt * delta_time;
    if (!linear)
        pre_t += ddt * delta_time * delta_time / 2;
    pre_r = last_r.conjugate() * r * r;
    return 0;
}

int Car::regularzation()
{
    // update t
    Eigen::Vector3d car_center = (armor[0].t + armor[1].t + armor[2].t + armor[3].t) / 4;
    t += car_center;
    for (int i=0; i<4; i++) {
        armor[i].t -= car_center;
    }

    // update r
    double t_len[4];
    for(int i=0; i<4; i++) {
        t_len[i] = armor[i].t.norm();
    }
    Eigen::Vector3d t_reset[4];
    t_reset[0] << 0, 0, -t_len[0];
    t_reset[1] << t_len[1], 0, 0;
    t_reset[2] << 0, 0, t_len[2];
    t_reset[3] << -t_len[3], 0, 0;
    Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
    for (int i=0; i<4; i++)
        T += t_reset[i] * armor[i].t.transpose();
    T /= 4;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix3d z_transfor = Eigen::Matrix3d::Identity();
        z_transfor(1, 1) = -1;
        R = z_transfor * R;
    }
    std::cout << "R: " << R.determinant() << std::endl;
    Eigen::Quaterniond qR(R);
    std::cout << "qR: " << qR.norm() << std::endl;
    for (int i=0; i<4; i++) {
        armor[i].r = qR * armor[i].r;
    }
    r = Eigen::Quaterniond(r.matrix() * R.transpose());
    return 0;
}

int Car::bundleAdjustment ( std::vector<Armor3d> &_armors,
                            std::vector<LightBarP> &light_bars,
                            const Eigen::Matrix3d &K,
                            double delta_time)
{
    if (_armors.size() > 0) {
        Eigen::Matrix3d _R = Eigen::Matrix3d::Zero();
        for (size_t i=0; i<_armors.size(); i++) {
            _R += (_armors[i].r * armor[_armors[i].armor_id].r.conjugate()).matrix();
        }
        _R /= _armors.size();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(_R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        r = Eigen::Quaterniond(svd.matrixU() * svd.matrixV().transpose());
        std::cout << "r: " << r.norm() << std::endl;
        r.x() = 0;
        r.z() = 0;
        r.normalize();
        Eigen::Vector3d new_t(0, 0, 0);
        for (size_t i=0; i<_armors.size(); i++) {
            new_t += _armors[i].t - r.matrix() * armor[_armors[i].armor_id].t;
        }
        new_t /= _armors.size();
        t = new_t;
        for (size_t i=0; i<_armors.size(); i++) {
            armor[_armors[i].armor_id].r = r.conjugate() * _armors[i].r;
            armor[_armors[i].armor_id].t = r.matrix().transpose() * (_armors[i].t - t);
        }
    }
    regularzation();
    update_state(delta_time);
    return 0;
}

class CarModule
{
private:
    double module_time;
    std::vector<LightBarP> predict2d;
    Eigen::Matrix3d K;
    std::vector<Car> cars;
    Eigen::Vector2d projectPoint(Eigen::Vector3d p3);

public:
    CarModule(cv::Matx<double, 3, 3> &K) : K(K.val) {
        this->K.transposeInPlace();
        std::cout << "K: \n" << this->K << std::endl;
    }
    int add_car(const Armor3d& _armor);
    int create_predict(double time);
    bool find_light(LightBarP &lbp);
    bool find_armor(Armor3d &armor);
    int bundleAdjustment(const std::vector<Armor3d> &armor3ds,
                         const std::vector<LightBarP> &light_bars,
                         double time);
    void get_lbs(std::vector<cv::Point3d> &lbs);
};

Eigen::Vector2d CarModule::projectPoint(Eigen::Vector3d p3)
{
    p3 = K * p3 / p3(2);
    // std::cout << "plane point :" << p3 << std::endl;
    return Eigen::Vector2d(p3(0), p3(1));
}

int CarModule::bundleAdjustment(const std::vector<Armor3d> &armor3ds,
                                const std::vector<LightBarP> &light_bars,
                                double time)
{
    double delta_time = time - module_time;
    if (cars.size() < 1)
        return 0;
    std::vector<LightBarP> light_bars_car[cars.size()];
    std::vector<Armor3d> armor3ds_car[cars.size()];
    for (LightBarP lbp : light_bars) {
        light_bars_car[lbp.car_id].push_back(lbp);
    }
    for (Armor3d a3d : armor3ds) {
        armor3ds_car[a3d.car_id].push_back(a3d);
    }
    for (size_t i=0; i<cars.size(); i++) {
        int size = light_bars_car[i].size() + armor3ds_car[i].size() * 2;
        // cars[i].car_info = (cars[i].car_info + size) / 2;
        if (size > 0)
            cars[i].bundleAdjustment(armor3ds_car[i], light_bars_car[i], K, delta_time);
    }
    module_time = time;
    return 0;
}

int CarModule::create_predict(double time)
{
    predict2d.clear();
    for (size_t c=0; c < cars.size(); c++) {
        Eigen::Quaterniond car_r;
        Eigen::Vector3d car_t;
        cars[c].predict(time - module_time, car_r, car_t);
        Eigen::Matrix3d car_R = car_r.matrix();
        for (size_t i=0; i<4; i++) {
            Eigen::Matrix3d armor_R = cars[c].armor[i].r.matrix();
            Eigen::Vector3d& armor_t = cars[c].armor[i].t;
            Eigen::Vector3d rotated_armor_point[4];
            for (size_t j=0; j<4; j++) {
                rotated_armor_point[i] = car_R * (armor_R * armor_module[i] + armor_t);
            }
            for (size_t j=0; j<2; j++) {
                if (((rotated_armor_point[2*j] + rotated_armor_point[2*j+1]) / 2)(2) <= 0) {
                    Eigen::Vector2d lbp[2];
                    lbp[0] = projectPoint(rotated_armor_point[2*j] + car_t);
                    lbp[1] = projectPoint(rotated_armor_point[2*j+1] + car_t);
                    predict2d.emplace_back(c, i, j, (lbp[0] + lbp[1])/2, (lbp[0] - lbp[1])/2);
                }
            }
        }
    }
    return 0;
}

int CarModule::add_car(const Armor3d& _armor)
{
    Car c;
    c.t = _armor.t + Eigen::Vector3d(0, 0, 0.3);
    std::cout << "c.t: " << c.t << std::endl;
    c.armor[0].t = Eigen::Vector3d(0, 0, -0.3);
    c.armor[0].r = _armor.r;
    Eigen::Quaterniond _r(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, -1, 0)));
    Eigen::Matrix3d _R = _r.matrix();
    for (int i=1; i<4; i++) {
        c.armor[i].t = _R * c.armor[i-1].t;
        c.armor[i].r = _r * c.armor[i-1].r;
    }
    c.regularzation();
    c.update_state();
    cars.push_back(c);
    std::cout << "successfully add a car!\n";
    return 0;
}

/**@return if or not found
 * @lbp input light bar info and output the id
*/
bool CarModule::find_light(LightBarP &lbp)
{
    if (predict2d.size() == 0)
        return false;
    double min_distance = 1000;
    int min_index = -1;
    for (size_t i=0; i<predict2d.size(); i++){
        double distance = (predict2d[i].center - lbp.center).norm();
        if (distance < min_distance){
            min_distance = distance;
            min_index = i;
        }
    }
    if (min_distance > 50) {
        return false;
    } else {
        lbp.car_id = predict2d[min_index].car_id;
        lbp.armor_id = predict2d[min_index].armor_id;
        lbp.lb_id = predict2d[min_index].lb_id;
        return true;
    }
}

bool CarModule::find_armor(Armor3d &armor)
{
    double min_distance = 1000;
    int car_id=0, armor_id=0;
    for (size_t i=0; i<cars.size(); i++) {
        Eigen::Matrix3d car_R = cars[i].r.matrix();
        Eigen::Vector3d car_t = cars[i].t;
        for (size_t j=0; j<4; j++) {
            double distance = (armor.t - (car_R * cars[i].armor[j].t + car_t)).norm();
            if (distance < min_distance) {
                min_distance = distance;
                car_id = i;
                armor_id = j;
            }
        }
    }
    if (min_distance > 0.3) {
        return false;
    } else {
        armor.car_id = car_id;
        armor.armor_id = armor_id;
        return true;
    }
}

void CarModule::get_lbs(std::vector<cv::Point3d> &lbs) {
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

#endif // __CAR_MODULE_HPP
