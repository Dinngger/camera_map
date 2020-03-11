/** Ruler.hpp
 * created time : 2020.3.3
 * by Dinger
 */

#ifndef __RULER_HPP
#define __RULER_HPP

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

template<typename ErrorType>
class FourPointRule
{
protected:
    Eigen::Vector3d* p[4];
    double* info[4];
    double rate;
    bool inner_info;
public:
    FourPointRule(double rate=0.05) : rate(rate) {}
    int setPoint (  Eigen::Vector3d* p1, Eigen::Vector3d* p2,
                    Eigen::Vector3d* p3, Eigen::Vector3d* p4,
                    double* info1=nullptr, double* info2=nullptr,
                    double* info3=nullptr, double* info4=nullptr) {
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        info[0] = info1;
        info[1] = info2;
        info[2] = info3;
        info[3] = info4;
        inner_info = !(info1 && info2 && info3 && info4);
        return 0;
    }
    virtual ErrorType error() const;
    virtual int backPropagate() {std::cout << "No backPropagate!\n"; return 0;}
    virtual int backPropagate(const ErrorType &error) {std::cout << "No backPropagate!\n"; return 0;}
};

class Centroid : public FourPointRule<Eigen::Vector3d>
{
public:
    Eigen::Vector3d error() const {
        return (*p[0] + *p[1] + *p[2] + *p[3]) / 4;
    }
    int backPropagate(const Eigen::Vector3d &error) override {
        for (int i=0; i<4; i++) {
            if (inner_info)
                *p[i] += error * rate;
            else
                *p[i] += error * *info[i] * rate;
        }
        return 0;
    }
};

class SymmetryRule : public FourPointRule<double>
{
public:
    double error() const {
        return  (*p[0] +*p[2]).norm() + (*p[1] + *p[3]).norm()
                + abs((*p[2] - *p[0]).dot(*p[3] - *p[1]));
    }
    int backPropagate() override {
        Eigen::Vector3d p_reset[4];
        p_reset[0] << 0, 0, -(*p[0]).norm();
        p_reset[1] << (*p[1]).norm(), 0, 0;
        p_reset[2] << 0, 0, (*p[2]).norm();
        p_reset[3] << -(*p[3]).norm(), 0, 0;
        for (int i=0; i<4; i++) {
            if (inner_info)
                *p[i] = *p[i] * (1 - rate) + p_reset[i] * rate;
            else
                *p[i] = *p[i] * (1 - rate * *info[i]) + p_reset[i] * rate * *info[i];
        }
        return 0;
    }
};

class ArmorRule : public FourPointRule<double>
{
private:
    Eigen::Vector3d _armor_module[4];
public:
    ArmorRule() : FourPointRule() {
        _armor_module[0] = Eigen::Vector3d(-0.065,  0.0285, 0);
        _armor_module[1] = Eigen::Vector3d(-0.065, -0.0285, 0);
        _armor_module[2] = Eigen::Vector3d( 0.065,  0.0285, 0);
        _armor_module[3] = Eigen::Vector3d( 0.065, -0.0285, 0);
    }
    double error() const {
        return (abs((*p[1] - *p[0]).norm() - 0.057) +
                abs((*p[3] - *p[2]).norm() - 0.057) +
                abs((*p[2] - *p[0]).norm() - 0.13) +
                abs((*p[3] - *p[1]).norm() - 0.13) +
                abs((*p[3] - *p[0]).norm() - (*p[2] - *p[1]).norm()));
    }
    int backPropagate() override {
        Eigen::Vector3d middle = (*p[0] + *p[1] + *p[2] + *p[3]) / 4;
        Eigen::Matrix3d T;
        for (int i=0; i<4; i++) {
            T += (*p[i] - middle) * _armor_module[i].transpose();
        }
        T /= 4;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);
        std::cout << "ruler test point110\n";
        Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
        std::cout << "ruler test point112\n";
        for (int i=0; i<4; i++) {
            if (inner_info) {
                std::cout << "ruler test point115\n";
                *p[i] = *p[i] * (1 - rate) + (R * _armor_module[i] + middle) * rate;
            } else {
                std::cout << "ruler test point118\n";
                *p[i] = *p[i] * (1 - rate * *info[i]) + (R * _armor_module[i] + middle) * rate * *info[i];
            }
        }
        return 0;
    }
};

#endif // __RULER_HPP
