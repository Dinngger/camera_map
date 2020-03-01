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

// TODO: change all opencv Mat to Eigen matrix.

// p[0] is the upper point.
struct LightBar
{
    Eigen::Vector3d p[2];
};

struct LightBarP
{
    int car_id, lb_id;
    Eigen::Vector2d center;
    Eigen::Vector2d p[2];
    LightBarP(cv::RotatedRect box) {
        // TODO: fill this function.
    }
};

// lb[0] was the first light bar to be seen.
struct Car
{
    int color=-1;
    int number=-1; // -1 means unknow
    double info=1;  // how much information we know about this car. it belongs to [0, 1].
    Eigen::Quaternion<double> r;
    Eigen::Matrix<double, 3, 1> t;
    LightBar lbs[8];
    int Regularzation();
    int bundleAdjustment(const std::vector<LightBarP> &light_bars, const Eigen::Matrix3d &K);
};

int Car::Regularzation()
{
    // TODO: make t be the mid of the lbs, and the lbs[0] is the light bar faced to -z.
    return 0;
}

int Car::bundleAdjustment(const std::vector<LightBarP> &light_bars, const Eigen::Matrix3d &K)
{
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
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
    // TODO: add armor size edge.
    // TODO: add infomation and robust core.
    for (const LightBarP lbp:light_bars)
    {
        LightBar lb = lbs[lbp.lb_id];
        for (int i=0; i<2; i++) {
            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
            point->setId(index);
            point->setEstimate(Eigen::Vector3d(lb.p[i](0), lb.p[i](1), lb.p[i](2)));
            point->setMarginalized(true);
            // point->setInformation(Eigen::Matrix3d::Identity());
            optimizer.addVertex(point);
            points.push_back(point);

            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId(index);
            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
            edge->setVertex(1, pose);
            edge->setMeasurement(Eigen::Vector2d(lbp.p[i](0), lbp.p[i](1)));  //设置观测值
            edge->setParameterId(0,0);
            edge->setInformation(Eigen::Matrix2d::Identity());
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

    Regularzation();
    return 0;
}

class CarModule
{
private:
    double module_time;
    std::vector<LightBarP> predict2d;
    Eigen::Matrix3d K;

public:
    CarModule(cv::Matx<double, 3, 3> &K) : K(K.val) {std::cout << "K: \n" << K << std::endl;}
    std::vector<Car> cars;
    int add_car(const std::vector<cv::Point3f> &armor);
    int create_predict(double time);
    int find_light(LightBarP &lbp);
    int bundleAdjustment(const std::vector<LightBarP> &light_bars);
};

int CarModule::bundleAdjustment(const std::vector<LightBarP> &light_bars)
{
    if (cars.size() < 1)
        return 0;
    std::vector<LightBarP> light_bars_car[cars.size()];
    for (LightBarP lbp : light_bars) {
        light_bars_car[lbp.car_id].push_back(lbp);
    }
    for (size_t i=0; i<cars.size(); i++) {
        int size = light_bars_car[i].size();
        cars[i].info = (cars[i].info + size) / 2;
        if (size > 0) 
            cars[i].bundleAdjustment(light_bars_car[i], K);
    }
    return 0;
}

int CarModule::create_predict(double time)
{
    // TODO: 计算此时的pose，投影出平面的装甲板
    return 0;
}

int CarModule::add_car(const std::vector<cv::Point3f> &armor)
{
    Car c;
    cars.push_back(c);
    return 0;
}

double getDistance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2){
	double distance;
	distance = powf((p1(0) - p2(0)), 2) + powf((p1(1) - p2(1)), 2);
	distance = sqrtf(distance);
	return distance;
}

/**@return if or not found
 * @lbp input light bar info and output the id
*/
int CarModule::find_light(LightBarP &lbp)
{
    double DISTANCE_THRESHOLD = 50;
    if (predict2d.size() == 0)
        return -1;
    double min_distance = 100000, distance;
    int min_index = -1;
    for (size_t i=0; i<predict2d.size(); i++){
        distance = getDistance(predict2d[i].center, lbp.center);
        if (distance < min_distance){
            min_distance = distance;
            min_index = i;
        }
    }
    if (min_distance > DISTANCE_THRESHOLD) 
        return -1;
    else {
        lbp.car_id = predict2d[min_index].car_id;
        lbp.lb_id = predict2d[min_index].lb_id;
        return 0;
    }
}

#endif // __CAR_MODULE_HPP
