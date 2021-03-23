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
int newcar::predict(double delta_time, double &pre_r, Eigen::Vector2d &pre_t) const
{
    pre_t = t + dt * delta_time;
    pre_r = r + dr * delta_time;
    return 0;
}
struct Edge2Edge {
    const Eigen::Vector2d last_t,real_t;
    const double last_r,real_r;
    const double delta_time;
    Edge2Edge(
        const Eigen::Vector2d last_t,const Eigen::Vector2d real_t,
        const double last_r,const double real_r,const double delta_time
    ):
    last_t(last_t),real_t(real_t),last_r(last_r),real_r(real_r),delta_time(delta_time){}
    template <typename T>
    bool operator()(const T* const t_x,const T* const t_z, const T* const r, T* residuals) const {
        residuals[0]=T(real_t[0])-t_x[0];
        residuals[1]=T(real_t[1])-t_y[1];
        residuals[2]=T(real_r)-r;
        residuals[3]=t_x[0]-T(last_t[0]);
        residuals[4]=t_z[0]-T(last_t[0]);
        residuals[5]=r[0]-T(last_r);
        return true;
    }
    static ceres::CostFunction *Create(
        const Eigen::Vector2d last_t,const Eigen::Vector2d real_t,
        const double last_r,const double real_r,const double delta_time) {
        return new ceres::AutoDiffCostFunction<Edge2Edge, 6, 1, 1, 1>(
            new Edge2Edge(last_t,real_t,last_r,real_r,delta_time)
        );
    }
};
int newcar::bundleAdjustment(double delta_time,Eigen::Vector2d real_t,double real_r)
{
    double t_x=t[0],t_z=t[1];
    double pre_r=r;
    ceres::Problem problem;
    ceres::CostFunction* cost = Edge2Edge::Create(last_t,real_t,last_r,real_r,delta_time);
    problem.AddResidualBlock(cost,NULL,&t_x,&t_z,&pre_r);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-2;
    options.max_num_iterations = 50;
    options.num_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    r=pre_r;
    t[0]=t_x;
    t[1]=t_z;
    return 0;
}