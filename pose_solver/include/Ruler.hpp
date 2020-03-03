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
    double info[4];
public:
    FourPointRule();
    int setPoint (  Eigen::Vector3d* p1, Eigen::Vector3d* p2,
                    Eigen::Vector3d* p3, Eigen::Vector3d* p4,
                    double info1, double info2, double info3, double info4) {
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        info[0] = info1;
        info[1] = info2;
        info[2] = info3;
        info[3] = info4;
    }
    virtual ErrorType error() const;
    virtual int backPropagate() {std::cout << "No backPropagate!\n"}
    virtual int backPropagate(const ErrorType &error) {std::cout << "No backPropagate!\n"}
};

class Centroid : public FourPointRule<Eigen::Vector3d>
{
public:
    Eigen::Vector3d error() const {
        return (*p[0] + *p[1] + *p[2] + *p[3]) / 4;
    }
    int backPropagate(const Eigen::Vector3d &error) override;
};

class SymmetryRule : public FourPointRule<double>
{
public:
    double error() const {
        return  (*p[0] +*p[2]).norm() + (*p[1] + *p[3]).norm()
                + (*p[2] - *p[0]).dot(*p[3] - *p[1]);
    }
    int backPropagate() override;
};

class ArmorRule : public FourPointRule<double>
{
    double error() const {
        return ((*p[1] - *p[0]).norm() - 0.057 +
                (*p[3] - *p[2]).norm() - 0.057 +
                (*p[2] - *p[0]).norm() - 0.13 +
                (*p[3] - *p[1]).norm() - 0.13 +
                (*p[3] - *p[0]).norm() - (*p[2] - *p[1]).norm());
    }
    int backPropagate() override;
};

#endif // __RULER_HPP
