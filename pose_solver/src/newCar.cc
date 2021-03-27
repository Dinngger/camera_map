#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "CarModule.hpp"
#include "newCarModule.hpp"

int newcar::update_state()
{
    last_t = t;
    last_r = r;
    return 0;
}
int newcar::update_state(double delta_time)
{
    if (delta_time == 0)
        return 0;
    dt = 0.9 * dt + 0.1 * (last_t - t) / delta_time;
    last_t = t;
    dr = 0.9 * dr + 0.1 * (last_r - r) / delta_time;
    last_r = r;
    return 0;
}
int newcar::predict(double delta_time, double &pre_r, Eigen::Vector3d &pre_t) const
{
    pre_t = t + dt * delta_time;
    pre_r = r + dr * delta_time;
    return 0;
}
struct Edge2Edge {                                          //重投影误差：传入过去一帧的装甲板观测结果 优化过去一帧的x,z,r,将其重投影到平面上，与观测点的位置进行比较，残差是位置的差值
    const Eigen::Matrix3d K;
    const cv::Point2f Point;
    const int Id;
    Edge2Edge(const Eigen::Matrix3d K,cv::Point2f Point,const int Id):
        K(K),Point(Point),Id(Id){}
    template <typename T>
    bool operator()(const T* const t_x,const T* const t_y,const T* const t_z, const T* const r, T* residuals) const {
        T x=t_x[0]+0.2*cos(r[0]),z=t_z[0]-0.2*sin(r[0]);           //此处没考虑清楚r的正负，可能有bug
        T y=t_y[0];
        //还需要加上装甲板偏差，按照Id
        switch(Id)
        {
            case 0:{
            x+=-0.065;y+=-0.0285;
            break;}
            case 1:{
            x+=-0.065;y+=0.0285;
            break;}
            case 2:{
            x+=0.065;y+=-0.0285;
            break;}
            case 3:{
            x+=0.065;y+=0.0285;
            break;}
        }
        Eigen::Matrix<T,3,1> p{x,y,z};
        p = K.cast<T>() * p;
        p /= p(2);
        residuals[0]=T(x)-T(Point.x);
        residuals[1]=T(z)-T(Point.y);
        return true;
    }
    static ceres::CostFunction *Create(
        const Eigen::Matrix3d K,const cv::Point2f Point,const int Id) {
        return new ceres::AutoDiffCostFunction<Edge2Edge, 2, 1, 1, 1, 1>(
            new Edge2Edge(K,Point,Id)
        );
    }
};

struct Edge2Energy {
    const double delta_time;
    Edge2Energy(
        const double delta_time
    ):
    delta_time(delta_time){}
    template <typename T>
    bool operator()(const T* const l_x,const T* const l_z, const T* const l_r,const T* const x,const T* const z, const T* const r, T* residuals) const {
        residuals[0]=((x[0]-l_x[0])/delta_time)*((x[0]-l_x[0])/delta_time);
        residuals[1]=T(15)*((r[0]-l_r[0])/delta_time)*((r[0]-l_r[0])/delta_time);
        return true;
    }
    static ceres::CostFunction *Create(const double delta_time) {
        return new ceres::AutoDiffCostFunction<Edge2Energy, 2, 1, 1, 1, 1, 1, 1>(
            new Edge2Energy(delta_time)
        );
    }
};
int newcar::bundleAdjustment(double delta_time)
{
    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
    double t_x[10],t_z[10],t_r[10];
    double t_y;
    //frameHead 链表下包含过去的10帧和现在的一帧 有可能不满十一帧
    pastFrame*p;
    p=frameHead;
    int FrameNum=0;
    int FrameNum2=0;
    while(p->next)
    {
        p=p->next;
        for(int j=0;j<4;j++){
            cv::Point2f Point=p->vertex[j];
            ceres::CostFunction* cost = Edge2Edge::Create(K,Point,j);
            problem.AddResidualBlock(cost,loss_function,&t_x[FrameNum],&t_y,&t_z[FrameNum],&t_r[FrameNum]);
        }
        FrameNum++;
    }
    if(FrameNum>2)
    {
        while(FrameNum2<FrameNum-1)
        {
            ceres::CostFunction* cost = Edge2Energy::Create(delta_time);
            problem.AddResidualBlock(cost,loss_function,&t_x[FrameNum2],&t_z[FrameNum2],&t_r[FrameNum2],&t_x[FrameNum2+1],&t_z[FrameNum2+1],&t_r[FrameNum2+1]);
            FrameNum2++;
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-2;
    options.max_num_iterations = 50;
    options.num_threads = 1;
    ceres::Solver::Summary summary;
    std::cout<<"\033[35mCeres result:\n";
    ceres::Solve(options, &problem, &summary);
    std::cout<<"module_time: "<<delta_time<<"\nt_x: "<<t_x[FrameNum]<<"\nt_y: "<<t_y<<"\nt_z: "<<t_z[FrameNum]<<"\nt_r: "<<t_r[FrameNum2];
    t=Eigen::Vector3d(t_x[FrameNum],t_y,t_z[FrameNum]);
    r=t_r[FrameNum];
    return 0;
}