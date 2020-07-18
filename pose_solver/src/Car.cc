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

template<typename T>
void printVector(const T& vec, int n) {
    for (int i=0; i<n; i++)
        std::cout << vec(i) << ", ";
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
        ceres::AngleAxisRotatePoint(_q2, p.data(), p.data());
        Eigen::Matrix<T, 3, 1> t2{_t2[0], _t2[1], _t2[2]};
        p += t2;
        ceres::AngleAxisRotatePoint(_q1, p.data(), p.data());
        Eigen::Matrix<T, 3, 1> t1{_t1[0], _t1[1], _t1[2]};
        p += t1;
        p = K * p;
        p /= p(2);
        residuals[0] = p(0) - pt2d(0);
        residuals[1] = p(1) - pt2d(1);
        return true;
    }
    static ceres::CostFunction *Create(
        const Eigen::Matrix3d K_,
        const Eigen::Vector3d pt3d_,
        const Eigen::Vector2d pt2d_) {
        return new ceres::AutoDiffCostFunction<Edge2EdgeError, 2, 3, 3, 3, 3>(
            new Edge2EdgeError(K_, pt3d_, pt2d_)
        );
    }
};

struct HistoryError {
    const Eigen::Vector3d q;
    const Eigen::Vector3d t;
    const double alpha;
    HistoryError(
        const Eigen::Vector3d q,
        const Eigen::Vector3d t,
        const double alpha) :
        q(q), t(t), alpha(alpha) {}
    template <typename T>
    bool operator()(const T* const _q, const T* const _t, T* residuals) const {
        for (int i=0; i<3; i++)
            residuals[i] = (_q[i] - (T)q(i)) * alpha;
        for (int i=0; i<3; i++)
            residuals[3+i] = (_t[i] - (T)t(i)) * alpha;
        return true;
    }
    static ceres::CostFunction *Create(
        const Eigen::Vector3d q_,
        const Eigen::Vector3d t_,
        const double alpha_) {
        return new ceres::AutoDiffCostFunction<HistoryError, 6, 3, 3>(
            new HistoryError(q_, t_, alpha_)
        );
    }
};

int Car::bundleAdjustment ( const std::vector<LightBarP> &light_bars,
                            const Eigen::Matrix3d &K,
                            double delta_time)
{
    predict(delta_time, r, t);
    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
    bool isValid[8] = {false, };
    bool isValidAll[8] = {false, };
    Eigen::Vector3d armor_aa[4];
    for (int i=0; i<4; i++)
        armor_aa[i] = Eigen::AngleAxisd(armor[i].r).axis() * Eigen::AngleAxisd(armor[i].r).angle();
    Eigen::Vector3d r_aa = Eigen::AngleAxisd(r).axis() * Eigen::AngleAxisd(r).angle();
    for (const LightBarP &lbp : light_bars) {
        for (size_t i=0; i<2; i++) {
            Eigen::Vector3d p = armor_module[lbp.lb_id*2+i];
            ceres::CostFunction* cost = Edge2EdgeError::Create(K, p, lbp[i]);
            problem.AddResidualBlock(cost, loss_function, r_aa.data(), t.data(),
                armor_aa[lbp.armor_id].data(), armor[lbp.armor_id].t.data());
        }
        isValid[lbp.armor_id * 2 + lbp.lb_id] = true;
        isValidAll[lbp.armor_id * 2 + lbp.lb_id] = true;
    }
    Eigen::Vector3d kfs_r_aa[4];
    for (int i=0; i<4; i++) {
        if (kfs[i].valid) {
            kfs_r_aa[i] = Eigen::AngleAxisd(kfs[i].kf_r).axis() * Eigen::AngleAxisd(kfs[i].kf_r).angle();
            for (const LightBarP &lbp : kfs[i].lbps) {
                for (size_t j=0; j<2; j++) {
                    Eigen::Vector3d p = armor_module[lbp.lb_id*2+j];
                    ceres::CostFunction* cost = Edge2EdgeError::Create(K, p, lbp[j]);
                    problem.AddResidualBlock(cost, loss_function, kfs_r_aa[i].data(), kfs[i].kf_t.data(),
                        armor_aa[lbp.armor_id].data(), armor[lbp.armor_id].t.data());
                }
                isValidAll[lbp.armor_id * 2 + lbp.lb_id] = true;
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
        if (isValidAll[i*2] || isValidAll[i*2+1]) {
            ceres::CostFunction* cost = HistoryError::Create(Eigen::AngleAxisd(r0).axis() * Eigen::AngleAxisd(r0).angle(), t0, 1);
            problem.AddResidualBlock(cost, loss_function, armor_aa[i].data(), armor[i].t.data());
        } else {
            ceres::CostFunction* cost = HistoryError::Create(Eigen::AngleAxisd(r0).axis() * Eigen::AngleAxisd(r0).angle(), t0, 1);
            problem.AddResidualBlock(cost, loss_function, armor_aa[i].data(), armor[i].t.data());
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

    r = Eigen::Quaterniond(Eigen::AngleAxisd(r_aa.norm(), r_aa.normalized()));
    for (int i=0; i<4; i++) {
        armor[i].r = Eigen::Quaterniond(Eigen::AngleAxisd(armor_aa[i].norm(), armor_aa[i].normalized()));
        if (kfs[i].valid)
            kfs[i].kf_r = Eigen::Quaterniond(Eigen::AngleAxisd(kfs_r_aa[i].norm(), kfs_r_aa[i].normalized()));
    }

    std::cout << "r=";
    printVector(r.coeffs(), 4);
    std::cout << "\n";
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
            if ((kfs[i].valid && kfs[i].score > d1) || !kfs[i].valid) {
                std::cout << "update kfs: " << i << "angle: " << angle(0) << "\n";
                kfs[i].valid = true;
                kfs[i].score = d1;
                kfs[i].kf_r = r;
                kfs[i].kf_t = t;
                kfs[i].lbps.assign(light_bars.begin(), light_bars.end());
            }
        }
        if (kfs[i].valid) {
            kfs_cnt++;
            std::cout << "kf" << i << ": r=";
            printVector(kfs[i].kf_r.coeffs(), 4);
            std::cout << "\n";
        }
    }
    std::cout << "kfs cnt: " << kfs_cnt << "\n";
    std::cout << "r=";
    printVector(r.coeffs(), 4);
    std::cout << "\n";
    return 0;
}
