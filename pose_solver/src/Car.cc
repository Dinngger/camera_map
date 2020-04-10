/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.4.9 23:54
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CarModule.hpp"

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
    dr = dr.slerp(0.5, Eigen::Quaterniond::Identity().slerp(DELTA_TIME/delta_time, (last_r.conjugate() * r)));
    last_r = r;
    return 0;
}

int Car::predict(double delta_time, Eigen::Quaterniond &pre_r, Eigen::Vector3d &pre_t, bool linear) const
{
#ifdef PREDICT
    pre_t = t + dt * delta_time;
    if (!linear)
        pre_t += ddt * delta_time * delta_time / 2;
    pre_r = last_r.conjugate() * r * r;
#else
    pre_t = t;
    pre_r = r;
#endif
    return 0;
}

int Car::regularzation()
{
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
        armor[i].r = Eigen::Quaterniond(R * armor[i].r.matrix());
        armor[i].t = R * armor[i].t;
    }
    r = Eigen::Quaterniond(r.matrix() * R.transpose());
    return 0;
}

int Car::computeGradient(const std::vector<LightBarP> &light_bars,
                         bool isObserved[],
                         int &sum,
                         Eigen::Matrix<double, 6, 1> gradient[],
                         const Eigen::Matrix3d &K,
                         double &error_sum) const {
    double fx = K(0, 0);
    double fy = K(1, 1);
    for (const LightBarP &lbp : light_bars) {
        Eigen::Matrix3d armor_R = armor[lbp.armor_id].r.matrix();
        const Eigen::Vector3d& armor_t = armor[lbp.armor_id].t;
        isObserved[lbp.armor_id*2+lbp.lb_id] = true;
        sum++;
        gradient[lbp.armor_id*2+lbp.lb_id] = Eigen::Matrix<double, 6, 1>::Zero();
        for (size_t i=0; i<2; i++) {
            Eigen::Vector3d _p = r.matrix() * (armor_R * armor_module[lbp.lb_id*2+i] + armor_t) + t;
            Eigen::Matrix<double, 1, 3> gradient_l[2];
            gradient_l[0] << fx/_p(2), 0, -fx*_p(0)/(_p(2)*_p(2));
            gradient_l[1] << 0, fy/_p(2), -fy*_p(1)/(_p(2)*_p(2));
            Eigen::Matrix<double, 3, 6> gradient_r;
            gradient_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            gradient_r.block<3, 3>(0, 3) = -up(_p);
            _p = K * _p / _p(2);
            Eigen::Vector2d error = Eigen::Vector2d(_p(0), _p(1)) - lbp[i];
            error_sum += pow(error.norm(), 2);
            for (int j=0; j<2; j++) {
                Eigen::Matrix<double, 6, 1> tmp = (gradient_l[j]*gradient_r).transpose();
                tmp = tmp * 1e-8 * error(j); // / pow(tmp.norm(), 2));
                gradient[lbp.armor_id*2+lbp.lb_id] += tmp;
            }
        }
    }
    return 0;
}

int Car::bundleAdjustment ( const std::vector<LightBarP> &light_bars,
                            const Eigen::Matrix3d &K,
                            double delta_time)
{
// #define BA_DEBUG
#ifdef  BA_DEBUG
    double old_error;
    cv::Matx<uint8_t, 1000, 1000> img = cv::Matx<uint8_t, 1000, 1000>::zeros();
    auto get_y = [](int x) {
        return x < 0 ? 0 : (x > 999 ? 999 : x);
    };
    auto get_x = [](int x) {
        return x < 0 ? 0 : (x > 999 ? 999 : x);
    };
#endif

    double error_sum = 0;
    Eigen::Matrix<double, 6, 1> moment[8];
    Eigen::Matrix<double, 6, 1> moment_sum = Eigen::Matrix<double, 6, 1>::Zero();
    double moment2[8];
    double moment2_sum = 0;
    for (int i=0; i<8; i++) {
        moment[i] = Eigen::Matrix<double, 6, 1>::Zero();
        moment2[i] = 0;
    }

    for (int cnt=0; cnt<1000; cnt++) {
#ifdef BA_DEBUG
        old_error = error_sum;
#endif
        error_sum = 0;
        bool isObserved[8] = {false,};
        Eigen::Matrix<double, 6, 1> gradient[8];
        int sum = 0;
        computeGradient(light_bars, isObserved, sum, gradient, K, error_sum);
        Eigen::Matrix<double, 6, 1> gradient_sum = Eigen::Matrix<double, 6, 1>::Zero();
        for (int i=0; i<8; i++) {
            if (isObserved[i]) {
                gradient_sum += gradient[i];
            }
        }
        // optimization(cnt, gradient, gradient_sum, isObserved,
        //              moment, moment_sum, moment2, moment2_sum);

#define BETA1 0.9
#define BETA2 0.999
        moment_sum =;

        if (cnt < 250) {
            t -= 3 * jacobi(gradient_sum.block<3, 1>(3, 0)) * gradient_sum.block<3, 1>(0, 0);
        } else if (cnt < 500) {
            r = Eigen::Quaterniond(exp(gradient_sum.block<3, 1>(3, 0)) * r.matrix());
            r.normalize();
        } else {
            Eigen::Matrix3d car_R = r.matrix();
            Eigen::Matrix3d car_R_T = car_R.transpose();
            for (int i=0; i<4; i++) {
                Eigen::Matrix<double, 6, 1> gradient_armor = Eigen::Matrix<double, 6, 1>::Zero();
                int sum_armor = 0;
                for (int j=0; j<2; j++) {
                    if (isObserved[i*2+j]) {
                        sum_armor++;
                        gradient_armor += gradient[i*2+j];
                    }
                }
                if (sum_armor > 0) {
                    gradient_armor *= 2.0 / sum_armor;
                    if (cnt < 750) {
                        armor[i].t -= car_R_T * jacobi(gradient_armor.block<3, 1>(3, 0)) * gradient_armor.block<3, 1>(0, 0);
                    } else {
                        armor[i].r = Eigen::Quaterniond(car_R_T * exp(gradient_armor.block<3, 1>(3, 0)) * car_R * armor[i].r.matrix());
                        armor[i].r.normalize();
                    }
                }
            }
        }

        if (cnt == 0)
            std::cout << "sum: " << sum << " error: " << error_sum << "\n";

#ifdef BA_DEBUG
        img(get_y(999 - (int)(gradient_sum.norm()*1e6)), cnt) = 128;
        img(get_y(999 - (int)(error_sum/10)), cnt) = 255;
        // img(get_y(999 - (int)(error_sum*2)), get_x((int)((t[0]-0.2)*5000+500))) = 200;
    }
    cv::imshow("error", img);
    cv::waitKey(1);
#else
    }
#endif
    std::cout << " error: " << error_sum << "\n";
    regularzation();
    update_state(delta_time);
    return 0;
}
