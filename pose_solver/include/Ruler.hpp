/** Ruler.hpp
 * created time : 2020.3.3
 * by Dinger
 * This file is no use now!!
 * and become my code trash.
 */

#ifndef __RULER_HPP
#define __RULER_HPP

#include <iostream>
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

template<typename ErrorType>
class FourPointRule
{
protected:
    Eigen::Vector3d p[4];
    double info[4];
    double rate;
    bool inner_info;
public:
    FourPointRule(double rate=0.05) : rate(rate) {}
    int setPoint (  const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                    const Eigen::Vector3d& p3, const Eigen::Vector3d& p4,
                    const double& info1, const double& info2,
                    const double& info3, const double& info4) {
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        info[0] = info1;
        info[1] = info2;
        info[2] = info3;
        info[3] = info4;
        inner_info = false;
        return 0;
    }
    int setPoint (  const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                    const Eigen::Vector3d& p3, const Eigen::Vector3d& p4) {
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        inner_info = true;
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
        return (p[0] + p[1] + p[2] + p[3]) / 4;
    }
    int backPropagate(const Eigen::Vector3d &error) override {
        for (int i=0; i<4; i++) {
            if (inner_info) {
                p[i] += error * rate;
            } else {
                double _info = info[i];
                p[i] += error * _info * rate;
            }
        }
        return 0;
    }
};

class SymmetryRule : public FourPointRule<double>
{
public:
    double error() const {
        return  (p[0] +p[2]).norm() + (p[1] + p[3]).norm()
                + abs((p[2] - p[0]).dot(p[3] - p[1]));
    }
    int backPropagate() override {
        Eigen::Vector3d p_reset[4];
        p_reset[0] << 0, 0, -(p[0]).norm();
        p_reset[1] << (p[1]).norm(), 0, 0;
        p_reset[2] << 0, 0, (p[2]).norm();
        p_reset[3] << -(p[3]).norm(), 0, 0;
        for (int i=0; i<4; i++) {
            if (inner_info) {
                p[i] = p[i] * (1 - rate) + p_reset[i] * rate;
            } else {
                double _info = info[i];
                p[i] = p[i] * (1 - rate * _info) + p_reset[i] * rate * _info;
            }
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
        return (abs((p[1] - p[0]).norm() - 0.057) +
                abs((p[3] - p[2]).norm() - 0.057) +
                abs((p[2] - p[0]).norm() - 0.13) +
                abs((p[3] - p[1]).norm() - 0.13) +
                abs((p[3] - p[0]).norm() - (p[2] - p[1]).norm()));
    }
    int backPropagate() override {
        Eigen::Vector3d middle = (p[0] + p[1] + p[2] + p[3]) / 4;
        Eigen::Matrix3d T;
        for (int i=0; i<4; i++) {
            T += (p[i] - middle) * _armor_module[i].transpose();
        }
        T /= 4;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
        for (int i=0; i<4; i++) {
            if (inner_info) {
                p[i] = p[i] * (1 - rate) + (R * _armor_module[i] + middle) * rate;
            } else {
                double _info = info[i];
                p[i] = p[i] * (1 - rate * _info);
                p[i] += (R * _armor_module[i] + middle) * rate * _info;
            }
        }
        return 0;
    }
};

/*
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);    //关提示
    // pose 维度为 6, landmark 维度为 3
    // 创建一个线性求解器LinearSolver
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =
        g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
    // 创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    // 定义图的顶点和边。并添加到SparseOptimizer中
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(r, t));
    optimizer.addVertex(pose);

    // 设置相机内参
    g2o::CameraParameters* camera = new g2o::CameraParameters(
            K(0,0), Eigen::Vector2d(K(0,2), K(1,2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    int index=1;
    std::vector<g2o::VertexSBAPointXYZ*> points;
    ///TODO: add armor size edge.
    ///TODO: add infomation and robust core.
    for (const LightBarP& lbp:light_bars)
    {
        LightBar lb = lbs[lbp.lb_id];
        for (int i=0; i<2; i++) {
            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
            point->setId(index);
            point->setEstimate(Eigen::Vector3d(lb.p[i](0), lb.p[i](1), lb.p[i](2)));
            point->setMarginalized(true);
            optimizer.addVertex(point);
            points.push_back(point);

            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId(index);
            edge->setVertex(0, point);
            edge->setVertex(1, pose);
            edge->setMeasurement(Eigen::Vector2d(lbp.p[i](0), lbp.p[i](1)));  //设置观测值
            edge->setParameterId(0,0);
            Eigen::Matrix2d infor = Eigen::Matrix2d::Identity();
            edge->setInformation(infor);
            optimizer.addEdge(edge);
            index++;
        }
    }

    for (int opt=0; opt<1; opt++) {
        // 设置优化参数，开始执行优化
        optimizer.initializeOptimization();
        optimizer.setVerbose(true);
        optimizer.optimize(1);
        #define RATE 0.5

        g2o::SE3Quat new_pose = pose->estimate();
        r = r.slerp(RATE, new_pose.rotation());
        interpolate(t, new_pose.translation(), RATE);
        pose->setEstimate(g2o::SE3Quat(r, t));

        index = 1;
        for (size_t i=0; i<light_bars.size(); i++) {
            for (int j=0; j<2; j++) {
                g2o::Vector3 new_lb = points[index-1]->estimate();
                interpolate(lbs[light_bars[i].lb_id].p[j], Eigen::Vector3d(new_lb(0), new_lb(1), new_lb(2)), RATE);
                Eigen::Vector3d _p = lbs[light_bars[i].lb_id].p[j];
                points[index-1]->setEstimate(Eigen::Vector3d(_p(0), _p(1), _p(2)));
                index++;
            }
        }
        double error;
        for (size_t i=0; i<5; i++) {
            error = ruler();
        }
        std::cout << "ruler error: " << error << "\n";
        if (optimizer.chi2() < 100)
            break;
    }
*/

#endif // __RULER_HPP
