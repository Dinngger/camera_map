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
#include "g2o/core/solver.h"
#include <g2o/core/block_solver.h>
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "Ruler.hpp"

///IMPORTENT: change this to the average time between two frames.
#define DELTA_TIME 1

double getDistance(const cv::Point2f p1, const cv::Point2f p2) {
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

// p[0] is the upper point.
struct LightBar
{
    double info;    // info belongs to [0, 1], 0 means perfectly know, 1 means know nothing.
    Eigen::Vector3d p[2];
    LightBar (  Eigen::Vector3d p1=Eigen::Vector3d::Zero(),
                Eigen::Vector3d p2=Eigen::Vector3d::Zero()) :
        info(1) {
        p[0] = p1;
        p[1] = p2;
    }
    Eigen::Vector3d center() {
        return (p[0] + p[1]) / 2;
    }
};

struct LightBarP
{
    int car_id, lb_id;
    Eigen::Vector2d center;
    Eigen::Vector2d p[2];
    LightBarP(cv::RotatedRect box) {
        cv::Point2f tmp_p1, tmp_p2, corners[4];                     //找出角点
        box.points(corners);
        float d1 = getDistance(corners[0], corners[1]);             //0/1点距离的平方
        float d2 = getDistance(corners[1], corners[2]);             //1/2点距离的平方
        int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
        tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;			    //获得旋转矩形两条短边上的中点
        tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
        if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
            p[1] = Eigen::Vector2d(tmp_p1.x, tmp_p1.y);
            p[0] = Eigen::Vector2d(tmp_p2.x, tmp_p2.y);
        }
        else{                                                       //必须是p1是处于上方的点，p2处于下方（y轴更大）
            p[0] = Eigen::Vector2d(tmp_p1.x, tmp_p1.y);
            p[1] = Eigen::Vector2d(tmp_p2.x, tmp_p2.y);
        }
        center = (p[0] + p[1]) / 2;
    }
    LightBarP(int car_id, int lb_id, Eigen::Vector2d p0, Eigen::Vector2d p1) :
        car_id(car_id),
        lb_id(lb_id) {
        p[0] = p0;
        p[1] = p1;
        center = (p0 + p1) / 2;
    }
};

// lb[0] was the first light bar to be seen.
class Car
{
private:
    int color=-1;
    int number=-1; // -1 means unknow
    LightBar lbs[8];

    Eigen::Quaterniond dr;
    Eigen::Quaterniond last_r;
    Eigen::Quaterniond r;
    Eigen::Vector3d ddt;
    Eigen::Vector3d last_dt;
    Eigen::Vector3d dt;
    Eigen::Vector3d last_t;
    Eigen::Vector3d t;

    ArmorRule ar[4];
    Centroid cen[4];
    Eigen::Vector3d centroid[4];
    SymmetryRule sr;
public:
    double info=1;  // how much information we know about this car. it belongs to [0, 1].
    Car() {
        for (int i=0; i<4; i++) {
            ar[i].setPoint(&lbs[i].p[0], &lbs[i].p[1], &lbs[i+1].p[0], &lbs[i+1].p[1],
                            &lbs[i].info, &lbs[i].info, &lbs[i+1].info, &lbs[i+1].info);
            cen[i].setPoint(&lbs[i].p[0], &lbs[i].p[1], &lbs[i+1].p[0], &lbs[i+1].p[1],
                            &lbs[i].info, &lbs[i].info, &lbs[i+1].info, &lbs[i+1].info);
        }
        sr.setPoint(&centroid[0], &centroid[1], &centroid[2], &centroid[3]);
        ddt = Eigen::Vector3d::Zero();
        last_dt = Eigen::Vector3d::Zero();
        dt = Eigen::Vector3d::Zero();
        last_t = Eigen::Vector3d::Zero();
        last_r.setIdentity();
        dr.setIdentity();
    }
    int regularzation();
    double ruler();
    int bundleAdjustment(const std::vector<LightBarP> &light_bars, const Eigen::Matrix3d &K, double delta_time);
    int update_state();
    int update_state(double delta_time);    // time from last update;
    int predict(double delta_time, Eigen::Quaterniond &pre_r, Eigen::Vector3d &pre_t, bool linear=false) const;     // time to the future
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
    dr = dr.slerp(0.5, Eigen::Quaterniond::Identity().slerp(DELTA_TIME/delta_time, (last_r.inverse() * r)));
    last_r = r;
    return 0;
}

int Car::predict(double delta_time, Eigen::Quaterniond &pre_r, Eigen::Vector3d &pre_t, bool linear) const
{
    pre_t = t + dt * delta_time;
    if (!linear)
        pre_t += ddt * delta_time * delta_time / 2;
    pre_r = last_r.inverse() * r * r;
    return 0;
}

int Car::regularzation()
{
    // update t
    Eigen::Vector3d armor_center[4];
    for(int i=0; i<4; i++) {
        armor_center[i] = cen[i].error();
    }
    Eigen::Vector3d car_center = (armor_center[0] + armor_center[1] + armor_center[2] + armor_center[3]) / 4;
    t += car_center;
    for (int i=0; i<8; i++) {
        lbs[i].p[0] -= car_center;
        lbs[i].p[1] -= car_center;
    }

    // update r
    double t_len[4];
    for(int i=0; i<4; i++) {
        armor_center[i] -= car_center;
        t_len[i] = armor_center[i].norm();
    }
    Eigen::Vector3d t_reset[4];
    t_reset[0] << 0, 0, -t_len[0];
    t_reset[1] << t_len[1], 0, 0;
    t_reset[2] << 0, 0, t_len[2];
    t_reset[3] << -t_len[3], 0, 0;
    Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
    for (int i=0; i<4; i++)
        T += t_reset[i] * armor_center[i].transpose();
    T /= 4;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(T, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    for (int i=0; i<8; i++) {
        lbs[i].p[0] = R * lbs[i].p[0];
        lbs[i].p[1] = R * lbs[i].p[1];
    }
    Eigen::Quaterniond q(r.matrix() * R.inverse());
    r = q;
    return 0;
}

// return error.
double Car::ruler()
{
    double error_sum = 0;
    for (int i=0; i<4; i++) {
        error_sum += ar[i].error();
        ar[i].backPropagate();
        centroid[i] = cen[i].error();
    }
    std::cout << "ruler testpoint 1\n";
    error_sum += sr.error();
    std::cout << "ruler testpoint 2\n";
    sr.backPropagate();
    std::cout << "ruler testpoint 3\n";
    for (int i=0; i<4; i++) {
        cen[i].backPropagate(centroid[i]);
    }
    return error_sum;
}

int Car::bundleAdjustment(const std::vector<LightBarP> &light_bars, const Eigen::Matrix3d &K, double delta_time)
{
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);    //关提示
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
    for (const LightBarP lbp:light_bars)
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
            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
            edge->setVertex(1, pose);
            edge->setMeasurement(Eigen::Vector2d(lbp.p[i](0), lbp.p[i](1)));  //设置观测值
            edge->setParameterId(0,0);
            Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
            info(0, 0) = 0.1;
            edge->setInformation(info);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // 设置优化参数，开始执行优化
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(10);

    // 输出优化结果
    g2o::SE3Quat new_pose = pose->estimate();
    r = new_pose.rotation();
    t = new_pose.translation();

    index = 1;
    for (size_t i=0; i<light_bars.size(); i++) {
        for (int j=0; j<2; j++) {
            g2o::Vector3 new_lb = points[index-1]->estimate();
            lbs[light_bars[i].lb_id].p[j] = Eigen::Vector3d(new_lb(0), new_lb(1), new_lb(2));
            index++;
        }
    }
    // double error = ruler();
    // while (abs(ruler() - error) > 0.5);
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
    int add_car(const Eigen::Vector3d armor[4]);
    int create_predict(double time);
    bool find_light(LightBarP &lbp);
    int bundleAdjustment(const std::vector<LightBarP> &light_bars, double time);
    void get_lbs(std::vector<cv::Point3d> &lbs);
};

Eigen::Vector2d CarModule::projectPoint(Eigen::Vector3d p3)
{
    p3 = K * p3 / p3(2);
    // std::cout << "plane point :" << p3 << std::endl;
    return Eigen::Vector2d(p3(0), p3(1));
}

int CarModule::bundleAdjustment(const std::vector<LightBarP> &light_bars, double time)
{
    double delta_time = time - module_time;
    if (cars.size() < 1)
        return 0;
    std::vector<LightBarP> light_bars_car[cars.size()];
    for (LightBarP lbp : light_bars) {
        light_bars_car[lbp.car_id].push_back(lbp);
        double* infop = &(cars[lbp.car_id].lbs[lbp.lb_id].info);
        *infop *= 0.9;
        if (*infop < 0.1)
            *infop = 0.1;
    }
    for (size_t i=0; i<cars.size(); i++) {
        int size = light_bars_car[i].size();
        cars[i].info = (cars[i].info + size) / 2;
        if (size > 0)
            cars[i].bundleAdjustment(light_bars_car[i], K, delta_time);
    }
    module_time = time;
    return 0;
}

int CarModule::create_predict(double time)
{
    for (size_t c=0; c < cars.size(); c++) {
        Eigen::Quaterniond pre_r;
        Eigen::Vector3d pre_t;
        cars[c].predict(time - module_time, pre_r, pre_t);
        LightBar rotated_lbs[8];
        for (int i=0; i<8; i++) {
            double* infop = &(cars[c].lbs[i].info);
            *infop *= 1.2;
            if (*infop > 1)
                *infop = 1;
            rotated_lbs[i] = LightBar(  pre_r.matrix() * cars[c].lbs[i].p[0],
                                        pre_r.matrix() * cars[c].lbs[i].p[1]);
            if (rotated_lbs[i].center()(2) <= 0) {
                Eigen::Vector2d lbp[2];
                lbp[0] = projectPoint(rotated_lbs[i].p[0] + pre_t);
                lbp[1] = projectPoint(rotated_lbs[i].p[1] + pre_t);
                predict2d.emplace_back(c, i, lbp[0], lbp[1]);
            }
        }
    }
    return 0;
}

int CarModule::add_car(const Eigen::Vector3d armor[4])
{
    Car c;
    c.t = (armor[0] + armor[1] + armor[2] + armor[3]) / 4 + Eigen::Vector3d(0, 0, 0.3);
    for (int i=0; i<2; i++) {
        for (int j=0; j<2; j++)
            c.lbs[i].p[j] = armor[2*i+j] - c.t;
    }
    // Eigen::AngleAxisd eaa(M_PI / 2, Eigen::Vector3d(0, -1, 0));
    // Eigen::Matrix3d K = eaa.matrix();
    Eigen::Matrix3d K;
    K << 0, 0, -1,
         0, 1, 0,
         1, 0, 0;
    // std::cout << "add car K:\n" << K << "\n";
    for (int i=2; i<8; i++) {
        for (int j=0; j<2; j++)
            c.lbs[i].p[j] = K * c.lbs[i-2].p[j];
    }
    // double error = c.ruler();
    // std::cout << "error: " << error << std::endl;
    std::cout << "successfully ruler!\n";
    // while (abs(c.ruler() - error) > 0.5);
    std::cout << "successfully ruler!\n";
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
    double DISTANCE_THRESHOLD = 50;
    if (predict2d.size() == 0)
        return false;
    double min_distance = 1000, distance;
    int min_index = -1;
    for (size_t i=0; i<predict2d.size(); i++){
        distance = (predict2d[i].center - lbp.center).norm();
        if (distance < min_distance){
            min_distance = distance;
            min_index = i;
        }
    }
    if (min_distance > DISTANCE_THRESHOLD) 
        return false;
    else {
        lbp.car_id = predict2d[min_index].car_id;
        lbp.lb_id = predict2d[min_index].lb_id;
        return true;
    }
}

void CarModule::get_lbs(std::vector<cv::Point3d> &lbs) {
    for (size_t i=0; i<cars.size(); i++) {
        Eigen::Matrix3d _R = cars[i].r.matrix();
        Eigen::Vector3d _t = cars[i].t;
        for (int j=0; j<8; j++) {
            for (int k=0; k<2; k++) {
                Eigen::Vector3d tmp = _R * cars[i].lbs[j].p[k] + _t;
                lbs.emplace_back(tmp(0), tmp(1), tmp(2));
            }
        }
    }
}

#endif // __CAR_MODULE_HPP
