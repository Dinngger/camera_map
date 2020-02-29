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
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


// p[0] is the upper point.
struct LightBar
{
    cv::Point3f p[2];
};

struct LightBarP
{
    int car_id, lb_id;
    cv::Point2f center;
    cv::Point2f p[2];
    LightBarP(cv::RotatedRect box) {
        // TODO: fill this function.
    }
};

// lb[0] was the first light bar to be seen.
struct Car
{
    int color=-1;
    int number=-1; // -1 means unknow
    LightBar lbs[8];
};

class CarModule
{
private:
    double module_time;
    std::vector<LightBarP> predict2d;
    cv::Mat K;

public:
    CarModule(const cv::Mat &K) {this->K = K;}
    std::vector<Car> cars;
    int add_car(const std::vector<cv::Point3f> &armor);
    int create_predict(double time);
    int find_light(LightBarP &lbp);
    // int bundleAdjustment(const std::vector<LightBarP> &light_bars);
};

int CarModule::create_predict(double time)
{
    // TODO: 计算此时的pose，投影出平面的装甲板
    return 0;
}

int bundleAdjustment(const std::vector<LightBarP> &light_bars)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block63;  // pose 维度为 6, landmark 维度为 3
    // 创建一个线性求解器LinearSolver
    Block63::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block63::PoseMatrixType>();

    // 创建BlockSolver。并用上面定义的线性求解器初始化
    Block63* solver_ptr = new Block63 ( std::unique_ptr<Block63::LinearSolverType>(linearSolver) );

    // 创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block63>(solver_ptr) );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 定义图的顶点和边。并添加到SparseOptimizer中
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();//初始位姿为单位矩阵
    pose->setId (0);
    pose->setEstimate ( g2o::SE3Quat() );
    optimizer.addVertex (pose);
/*
    // 设置相机内参
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0);
    camera->setId(0);
    optimizer.addParameter ( camera );

    int index=1;
    for(const LightBarP lbp:light_bars)
    {
        LightBar lb = cars[lbp.car_id].lbs[lbp.lb_id];
        for (int i=0; i<2; i++) {
            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
            point->setId(index);
            point->setEstimate(Eigen::Vector3d(lb.p[i].x, lb.p[i].y, lb.p[i].z));
            point->setMarginalized(true);
            // point->setInformation(Eigen::Matrix3d::Identity());
            optimizer.addVertex(point);

            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId ( index );
            edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
            edge->setVertex ( 1, pose );
            edge->setMeasurement ( Eigen::Vector2d (lbp.p[i].x, lbp.p[i].y) );  //设置观测值
            edge->setParameterId ( 0,0 );
            edge->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( edge );
            index++;
        }
    }

    // 设置优化参数，开始执行优化
    optimizer.setVerbose ( false );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );

    // 输出优化结果
    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;*/
    return 0;
}

int CarModule::add_car(const std::vector<cv::Point3f> &armor)
{
    Car c;

    double x1,x2,y1,y2;
    x1=armor[0].x;
    x2=armor[1].x;
    y1=armor[0].y;
    y2=armor[1].y;
    //Assume that distance(armor[0],armor[1])==long side
    double length = 1;//length of car
    double len=sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));//len of armor
    double k_cos=(x2-x1)/sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    double k_sin=(y2-y1)/sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

    LightBar l7,l8;//the front lightbars
    l7.p[0]=armor[0];
    l7.p[1]=armor[3];
    l8.p[0]=armor[1];
    l8.p[1]=armor[2];
    c.lbs[0]=l7;
    c.lbs[1]=l8;

    LightBar l1,l2;// the two back lightbars
    l1.p[0]=armor[0]+cv::Point3f(length*k_sin,-length*k_cos,0);
    l1.p[1]=armor[3]+cv::Point3f(length*k_sin,-length*k_cos,0);
    l2.p[0]=armor[1]+cv::Point3f(length*k_sin,-length*k_cos,0);
    l2.p[1]=armor[2]+cv::Point3f(length*k_sin,-length*k_cos,0);
    c.lbs[2]=l1;
    c.lbs[3]=l2;

    LightBar l3,l4;//one of left or right
    cv::Point3f center1=(armor[0]+armor[1])/2+cv::Point3f(length/2*k_sin,-length/2*k_cos,0)+cv::Point3f(length/2*k_cos,-length/2*k_sin,0);
    cv::Point3f center2=(armor[2]+armor[3])/2+cv::Point3f(length/2*k_sin,-length/2*k_cos,0)+cv::Point3f(length/2*k_cos,-length/2*k_sin,0);
    l3.p[0]=center1+cv::Point3f(len*k_sin,-len*k_cos,0);
    l3.p[1]=center1+cv::Point3f(len*k_sin,-len*k_cos,0);
    l4.p[0]=center2-cv::Point3f(len*k_sin,-len*k_cos,0);
    l4.p[1]=center2-cv::Point3f(len*k_sin,-len*k_cos,0);
    c.lbs[4]=l3;
    c.lbs[5]=l4;

    LightBar l5,l6;//another left or right
    cv::Point3f center3=(armor[0]+armor[1])/2+cv::Point3f(length/2*k_sin,-length/2*k_cos,0)-cv::Point3f(length/2*k_cos,-length/2*k_sin,0);
    cv::Point3f center4=(armor[2]+armor[3])/2+cv::Point3f(length/2*k_sin,-length/2*k_cos,0)-cv::Point3f(length/2*k_cos,-length/2*k_sin,0);
    l5.p[0]=center3+cv::Point3f(len*k_sin,-len*k_cos,0);
    l5.p[1]=center3+cv::Point3f(len*k_sin,-len*k_cos,0);
    l6.p[0]=center4-cv::Point3f(len*k_sin,-len*k_cos,0);
    l6.p[1]=center4-cv::Point3f(len*k_sin,-len*k_cos,0);
    c.lbs[6]=l5;
    c.lbs[7]=l6;

    cars.push_back(c);
    return 0;
}

double getDistance(cv::Point p1, cv::Point p2){
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	distance = sqrtf(distance);
	return distance;
}

/**@return if or not found
 * @lbp input light bar info and output the id
*/
int CarModule::find_light(LightBarP &lbp)
{
    double DISTANCE_THRESHOLD = 50;
    if (predict2d.size()==0)
        return -1;
    double min_distance = 100000, distance;
    int min_index = -1;
    for(int i=0;i<predict2d.size();i++){
        distance = getDistance(predict2d[i].center, lbp.center);
        if(distance < min_distance){
            min_distance = distance;
            min_index = i;
        }
    }
    if(min_distance > DISTANCE_THRESHOLD) 
        return -1;
    else{
        lbp.car_id = predict2d[min_index].car_id;
        lbp.lb_id = predict2d[min_index].lb_id;
        return 0;
    }
}

#endif // __CAR_MODULE_HPP
