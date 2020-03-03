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

double getDistance(const cv::Point2f p1, const cv::Point2f p2) {
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

// p[0] is the upper point.
struct LightBar
{
    double info=-1;
    Eigen::Vector3d p[2];
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
    }
};

// lb[0] was the first light bar to be seen.
class Car
{
private:
    int color=-1;
    int number=-1; // -1 means unknow
    LightBar lbs[8];

    Eigen::Quaternion<double> r;
    Eigen::Matrix<double, 3, 1> t;

    ArmorRule ar[4];
    Centroid cen[4];
    Eigen::Vector3d centroid[4];
    SymmetryRule sr;
public:
    double info=1;  // how much information we know about this car. it belongs to [0, 1].
    Car() {
        for (int i=0; i<4; i++) {
            ar[i].setPoint(&lbs[i].p[0], &lbs[i].p[1], &lbs[i+1].p[0], &lbs[i+1].p[1],
                            lbs[i].info, lbs[i].info, lbs[i+1].info, lbs[i+1].info);
            cen[i].setPoint(&lbs[i].p[0], &lbs[i].p[1], &lbs[i+1].p[0], &lbs[i+1].p[1],
                            lbs[i].info, lbs[i].info, lbs[i+1].info, lbs[i+1].info);
        }
        sr.setPoint(&centroid[0], &centroid[1], &centroid[2], &centroid[3], 1, 1, 1, 1);
    }
    int Regularzation();
    int bundleAdjustment(const std::vector<LightBarP> &light_bars, const Eigen::Matrix3d &K);

    friend class CarModule;
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
    ///TODO: add armor size edge.
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
    std::vector<Car> cars;

public:
    CarModule(cv::Matx<double, 3, 3> &K) : K(K.val) {std::cout << "K: \n" << K << std::endl;}
    int add_car(const std::vector<cv::Point3f> &armor);
    int create_predict(double time);
    int find_light(LightBarP &lbp);
    int bundleAdjustment(const std::vector<LightBarP> &light_bars);
    void get_lbs(std::vector<cv::Point3f> &lbs);
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
        distance = (predict2d[i].center - lbp.center).norm();
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

void CarModule::get_lbs(std::vector<cv::Point3f> &lbs) {
    for (int i=0; i<cars.size(); i++) {
        for (int j=0; j<8; j++) {
            for (int k=0; k<2; k++) {
                Eigen::Vector3d tmp = cars[i].lbs[j].p[k];
                lbs.emplace_back(tmp(0), tmp(1), tmp(2));
            }
        }
    }
}

#endif // __CAR_MODULE_HPP
