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
#define PREDICT
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

template <typename T>
void printVector(const T& vec, int n) {
    for (int i=0; i<n; i++)
        std::cout << vec(i) << ", ";
}

struct Edge2EdgeErrorX {
    const Eigen::Matrix3d K;
    const Eigen::Vector3d pt3d;
    const Eigen::Vector2d pt2d;
    const Eigen::Quaterniond armor_r;
    const int direction;
    Edge2EdgeErrorX(
        const Eigen::Matrix3d K,
        const Eigen::Vector3d pt3d,
        const Eigen::Vector2d pt2d,
        const Eigen::Quaterniond armor_r,
        const int direction) :
        K(K), pt3d(pt3d), pt2d(pt2d), armor_r(armor_r), direction(direction) {}
    template <typename T>
    bool operator()(const T* const _q1, const T* const _t1, const T* const tx, T* residuals) const {
        Eigen::Matrix<T, 3, 1> p{T(pt3d[0]), T(pt3d[1]), T(pt3d[2])};
        p = armor_r.matrix().cast<T>() * p;
        p(0) += T(direction) * tx[0];
        ceres::AngleAxisRotatePoint(_q1, p.data(), p.data());
        Eigen::Matrix<T, 3, 1> t1{_t1[0], _t1[1], _t1[2]};
        p += t1;
        p = K.cast<T>() * p;
        p /= p(2);
        residuals[0] = p(0) - pt2d(0);
        residuals[1] = p(1) - pt2d(1);
        return true;
    }
    static ceres::CostFunction *Create(
        const Eigen::Matrix3d K_,
        const Eigen::Vector3d pt3d_,
        const Eigen::Vector2d pt2d_,
        const Eigen::Quaterniond armor_r,
        const int direction) {
        return new ceres::AutoDiffCostFunction<Edge2EdgeErrorX, 2, 3, 3, 1>(
            new Edge2EdgeErrorX(K_, pt3d_, pt2d_, armor_r, direction)
        );
    }
};

struct Edge2EdgeErrorZ {
    const Eigen::Matrix3d K;
    const Eigen::Vector3d pt3d;
    const Eigen::Vector2d pt2d;
    const Eigen::Quaterniond armor_r;
    const int direction;
    Edge2EdgeErrorZ(
        const Eigen::Matrix3d K,
        const Eigen::Vector3d pt3d,
        const Eigen::Vector2d pt2d,
        const Eigen::Quaterniond armor_r,
        const int direction) :
        K(K), pt3d(pt3d), pt2d(pt2d), armor_r(armor_r), direction(direction) {}
    template <typename T>
    bool operator()(const T* const _q1, const T* const _t1, const T* const tz, const T* const ty, T* residuals) const {
        Eigen::Matrix<T, 3, 1> p{T(pt3d[0]), T(pt3d[1]), T(pt3d[2])};
        p = armor_r.matrix().cast<T>() * p;
        p(1) += ty[0];
        p(2) += T(direction) * tz[0];
        ceres::AngleAxisRotatePoint(_q1, p.data(), p.data());
        Eigen::Matrix<T, 3, 1> t1{_t1[0], _t1[1], _t1[2]};
        p += t1;
        p = K.cast<T>() * p;
        p /= p(2);
        residuals[0] = p(0) - pt2d(0);
        residuals[1] = p(1) - pt2d(1);
        return true;
    }
    static ceres::CostFunction *Create(
        const Eigen::Matrix3d K_,
        const Eigen::Vector3d pt3d_,
        const Eigen::Vector2d pt2d_,
        const Eigen::Quaterniond armor_r,
        const int direction) {
        return new ceres::AutoDiffCostFunction<Edge2EdgeErrorZ, 2, 3, 3, 1, 1>(
            new Edge2EdgeErrorZ(K_, pt3d_, pt2d_, armor_r, direction)
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
    Eigen::Vector3d r_aa = Eigen::AngleAxisd(r).axis() * Eigen::AngleAxisd(r).angle();
    double tx = armor[1].t(0);
    double ty = armor[0].t(1);
    double tz = -armor[0].t(2);
    for (const LightBarP &lbp : light_bars) {
        for (size_t i=0; i<2; i++) {
            Eigen::Vector3d p = armor_module[lbp.lb_id*2 + i];
            if (lbp.armor_id % 2 == 1) {
                ceres::CostFunction* cost = Edge2EdgeErrorX::Create(K, p, lbp[i],
                    armor[lbp.armor_id].r, lbp.armor_id > 2 ? -1 : 1);
                problem.AddResidualBlock(cost, loss_function, r_aa.data(), t.data(), &tx);
            } else {
                ceres::CostFunction* cost = Edge2EdgeErrorZ::Create(K, p, lbp[i],
                    armor[lbp.armor_id].r, lbp.armor_id > 1 ? 1 : -1);
                problem.AddResidualBlock(cost, loss_function, r_aa.data(), t.data(), &tz, &ty);
            }
        }
        isValid[lbp.armor_id * 2 + lbp.lb_id] = true;
    }
    Eigen::Vector3d kfs_r_aa[4];
    for (int i=0; i<4; i++) {
        if (kfs[i].valid) {
            kfs_r_aa[i] = Eigen::AngleAxisd(kfs[i].kf_r).axis() * Eigen::AngleAxisd(kfs[i].kf_r).angle();
            for (const LightBarP &lbp : kfs[i].lbps) {
                for (size_t j=0; j<2; j++) {
                    Eigen::Vector3d p = armor_module[lbp.lb_id*2+j];
                    if (lbp.armor_id % 2 == 1) {
                        ceres::CostFunction* cost = Edge2EdgeErrorX::Create(K, p, lbp[j],
                            armor[lbp.armor_id].r, lbp.armor_id > 2 ? -1 : 1);
                        problem.AddResidualBlock(cost, loss_function, kfs_r_aa[i].data(), kfs[i].kf_t.data(), &tx);
                    } else {
                        ceres::CostFunction* cost = Edge2EdgeErrorZ::Create(K, p, lbp[j],
                            armor[lbp.armor_id].r, lbp.armor_id > 1 ? 1 : -1);
                        problem.AddResidualBlock(cost, loss_function, kfs_r_aa[i].data(), kfs[i].kf_t.data(), &tz, &ty);
                    }
                }
            }
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-2;
    options.max_num_iterations = 50;
    options.num_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    r = Eigen::Quaterniond(Eigen::AngleAxisd(r_aa.norm(), r_aa.normalized()));
    armor[0].t << 0, ty, -tz;
    armor[1].t << tx, 0, 0;
    armor[2].t << 0, ty, tz;
    armor[3].t << -tx, 0, 0;
    for (int i=0; i<4; i++) {
        if (kfs[i].valid)
            kfs[i].kf_r = Eigen::Quaterniond(Eigen::AngleAxisd(kfs_r_aa[i].norm(), kfs_r_aa[i].normalized()));
    }

    std::cout << "r=";
    printVector(r.coeffs(), 4);
    std::cout << "\n";
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
