/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.4.9 23:54
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "CarModule.hpp"

int Car::update_state()
{
    last_t = t;
    last_r = r;
    return 0;
}

int Car::update_state(double delta_time)
{
    dt = 0.9 * dt + 0.1 * (last_t - t) / delta_time;
    last_t = t;
    ddt = 0.9 * ddt + 0.1 * (last_dt - dt) / delta_time;
    last_dt = dt;
    dr = dr.slerp(0.1, Eigen::Quaterniond::Identity().slerp(DELTA_TIME/delta_time, (last_r.conjugate() * r)));
    last_r = r;
    return 0;
}

int Car::predict(double delta_time, Eigen::Quaterniond &pre_r, Eigen::Vector3d &pre_t, bool linear) const
{
// #define PREDICT
#ifdef PREDICT
    pre_t = t + dt * delta_time;
    if (!linear)
        pre_t += ddt * delta_time * delta_time / 2;
    pre_r = r * Eigen::Quaterniond::Identity().slerp(delta_time/DELTA_TIME, dr);
#else
    pre_t = t;
    pre_r = r;
#endif
    return 0;
}

int Car::regularzation()
{
    r.normalize();
    // update t
    Eigen::Vector3d car_center = (armor[0].t + armor[1].t + armor[2].t + armor[3].t) / 4;
    t += r.matrix() * car_center;
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
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix3d sigma = Eigen::Matrix3d::Identity();
        sigma(2, 2) = -1;
        R = svd.matrixU() * sigma * svd.matrixV().transpose();
    }
    for (int i=0; i<4; i++) {
        armor[i].r = Eigen::Quaterniond(R * armor[i].r.normalized().matrix()).normalized();
        armor[i].t = R * armor[i].t;
    }
    r = Eigen::Quaterniond(r.matrix() * R.transpose()).normalized();
    return 0;
}

struct Edge2EdgeError {
    const Eigen::Matrix3d K;
    const Eigen::Vector3d pt3d;
    const Eigen::Vector2d pt2d;
    Edge2EdgeError(
        const Eigen::Matrix3d K,
        const Eigen::Vector3d pt3d,
        const Eigen::Vector2d pt2d) :
        K(K), pt3d(pt3d), pt2d(pt2d) {}
    template <typename T>
    bool operator()(const T* const _q1, const T* const _t1, const T* const _q2, const T* const _t2, T* residuals) const {
        Eigen::Matrix<T, 3, 1> p{T(pt3d[0]), T(pt3d[1]), T(pt3d[2])};
        Eigen::Matrix<T, 3, 1> t1{_t1[0], _t1[1], _t1[2]};
        Eigen::Matrix<T, 3, 1> t2{_t2[0], _t2[1], _t2[2]};
        Eigen::Quaternion<T> q1{_q1[3], _q1[0], _q1[1], _q1[2]};
        Eigen::Quaternion<T> q2{_q2[3], _q2[0], _q2[1], _q2[2]};
        p = q1 * (q2 * p + t2) + t1;
        p = K * p;
        p /= p(2);
        residuals[0] = p(0) - pt2d(0);
        residuals[1] = p(1) - pt2d(1);
        return true;
    }
};

struct HistoryError {
    const Eigen::Quaterniond q;
    const Eigen::Vector3d t;
    HistoryError(
        const Eigen::Quaterniond q,
        const Eigen::Vector3d t) :
        q(q), t(t) {}
    template <typename T>
    bool operator()(const T* const _q, const T* const _t, T* residuals) const {
        for (int i=0; i<4; i++)
            residuals[i] = (_q[(i+1)%4] - (T)q.coeffs()(i)) * 300.0;
        for (int i=0; i<3; i++)
            residuals[4+i] = (_t[i] - (T)t(i)) * 200.0;
        return true;
    }
};

template <typename T>
void printVector(const T& a, int n) {
    for (int i=0; i<n; i++)
        std::cout << a(i) << ", ";
}

int Car::bundleAdjustment ( const std::vector<LightBarP> &light_bars,
                            const Eigen::Matrix3d &K,
                            double delta_time)
{
    predict(delta_time, r, t);
    ceres::Problem problem;
    ceres::LossFunction* loss_function (new ceres::SoftLOneLoss(1));
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
    problem.AddParameterBlock(r.coeffs().data(), 4, quaternion_local_parameterization);
    bool isValid[8] = {false, };
    for (const LightBarP &lbp : light_bars) {
        for (size_t i=0; i<2; i++) {
            Eigen::Vector3d p = armor_module[lbp.lb_id*2+i];
            ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<Edge2EdgeError, 2, 4, 3, 4, 3>(
                new Edge2EdgeError(K, p, lbp[i])
            );
            problem.AddResidualBlock(cost, loss_function, r.coeffs().data(), t.data(),
                armor[lbp.armor_id].r.coeffs().data(), armor[lbp.armor_id].t.data());
        }
        isValid[lbp.armor_id * 2 + lbp.lb_id] = true;
    }
    for (int i=0; i<4; i++) {
        if (kfs[i].valid) {
            problem.AddParameterBlock(kfs[i].kf_r.coeffs().data(), 4, quaternion_local_parameterization);
            for (const LightBarP &lbp : kfs[i].lbps) {
                for (size_t i=0; i<2; i++) {
                    Eigen::Vector3d p = armor_module[lbp.lb_id*2+i];
                    ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<Edge2EdgeError, 2, 4, 3, 4, 3>(
                        new Edge2EdgeError(K, p, lbp[i])
                    );
                    problem.AddResidualBlock(cost, loss_function, kfs[i].kf_r.coeffs().data(), kfs[i].kf_t.data(),
                        armor[lbp.armor_id].r.coeffs().data(), armor[lbp.armor_id].t.data());
                }
            }
        }
    }
    Eigen::Vector3d t0 = Eigen::Vector3d(0, 0, -0.25);
    Eigen::Quaterniond r0 = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond r_90(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, -1, 0)));
    Eigen::Matrix3d R_90 = r_90.matrix();
    for (int i=0; i<4; i++) {
        if (i > 0) {
            t0 = R_90 * t0;
            r0 = r_90 * r0;
        }
        std::cout << "armor_r: ";
        printVector(armor[i].r.coeffs(), 4);
        std::cout << "t: ";
        printVector(armor[i].t, 3);
        std::cout << "\n";
        if (isValid[i*2] || isValid[i*2+1]) {
            ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<HistoryError, 7, 4, 3>(
                new HistoryError(r0, t0)
            );
            problem.AddResidualBlock(cost, loss_function, armor[i].r.coeffs().data(), armor[i].t.data());
            problem.SetParameterization(armor[i].r.coeffs().data(), quaternion_local_parameterization);
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-2;
    options.max_num_iterations = 10;
    options.num_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    regularzation();
    update_state(delta_time);
    Eigen::Vector3d angle = r.toRotationMatrix().eulerAngles(2, 1, 0);
    int kfs_cnt = 0;
    for (int i=0; i<4; i++) {
        if (isValid[i*2] && isValid[i*2+1]) {
            double d1 = angle(0) - i * 90;
            double d2 = 180 - abs(d1);
            if (d1 > 0)
                d2 *= -1.0;
            d1 = d1 < d2 ? d1 : d2;
            if ((kfs[i].valid && kfs[i].score >= d1) || !kfs[i].valid) {
                std::cout << "update kfs: " << i << "\n";
                kfs[i].valid = true;
                kfs[i].score = d1;
                kfs[i].kf_r = r;
                kfs[i].kf_t = t;
                kfs[i].lbps.assign(light_bars.begin(), light_bars.end());
            }
        }
        if (kfs[i].valid)
            kfs_cnt++;
    }
    std::cout << "kfs cnt: " << kfs_cnt << "\n";
    return 0;
}
