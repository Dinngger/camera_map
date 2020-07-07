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

int Car::bundleAdjustment ( const std::vector<LightBarP> &light_bars,
                            const Eigen::Matrix3d &K,
                            double delta_time)
{
    predict(delta_time, r, t);

    enum StateType {
        SCT = 0,    // State of changing Car T
        SCTR,   // State of changing Car T & R
        SAT,    // State of changing Armor T
        SATR,   // State of changing Armor T & R
        END
    };

    double error_sum = 1e8;
    double obv_error = 1e8;
    double old_error;
    double old_obv_error;
    Eigen::Matrix<double, 6, 1> moment[4];
    Eigen::Matrix<double, 6, 1> moment_sum;
    double moment2[4];
    double moment2_sum = 0;
    StateType state = SCT;
    bool reset = true;
    int cnt = 0;
    int state_cnt = 0;

    while (state != END) {
        old_error = error_sum;
        old_obv_error = obv_error;
        bool isObserved[8] = {false,};
        Eigen::Matrix<double, 6, 1> gradient[8];
        double error[8] = {0,};
        int sum = 0;

        // computeGradient
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
                Eigen::Vector2d error2d = Eigen::Vector2d(_p(0), _p(1)) - lbp[i];
                error[lbp.armor_id*2+lbp.lb_id] += pow(error2d.norm(), 2);
                for (int j=0; j<2; j++) {
                    Eigen::Matrix<double, 6, 1> tmp = (gradient_l[j]*gradient_r).transpose();
                    tmp = tmp * 1e-8 * error2d(j);
                    gradient[lbp.armor_id*2+lbp.lb_id] += tmp;
                }
            }
        }

        error_sum = 0;
        obv_error = 0;
        Eigen::Matrix<double, 6, 1> gradient_sum = Eigen::Matrix<double, 6, 1>::Zero();
        double rate_sum = 0;
        int observed_sum = 0;
        for (int i=0; i<4; i++) {
            for (int j=0; j<2; j++) {
                if (isObserved[2*i+j]) {
                    rate_sum += confidence[i];
                    observed_sum += 1;
                    obv_error += error[2*i+j];
                    error_sum += error[2*i+j] * confidence[i];
                    gradient_sum += gradient[2*i+j] * confidence[i];
                }
            }
        }
        obv_error *= 2.0 / observed_sum;
        error_sum *= 2.0 / rate_sum;
        gradient_sum *= 1.0 / rate_sum;

#define BETA1 0.9
#define BETA2 0.99
        if (reset) {
            moment_sum = gradient_sum;
            moment2_sum = pow(gradient_sum.norm(), 2);
        } else {
            moment_sum = BETA1 * moment_sum + (1 - BETA1) * gradient_sum;
            moment2_sum = BETA2 * moment2_sum + (1 - BETA2) * pow(gradient_sum.norm(), 2);
        }
        gradient_sum = moment_sum / (sqrt(moment2_sum) + 1e-8) * 5e-4;
        if (state == SCT || state == SCTR) {
            t -= jacobi(gradient_sum.block<3, 1>(3, 0)) * gradient_sum.block<3, 1>(0, 0);
        }
        if (state == SCTR) {
            r = Eigen::Quaterniond(exp(gradient_sum.block<3, 1>(3, 0) * -0.5) * r.matrix());
            r.normalize();
        }
        Eigen::Matrix3d car_R = r.matrix();
        Eigen::Matrix3d car_R_T = car_R.transpose();

        Eigen::Quaterniond transform90_r(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, -1, 0)));
        Eigen::Quaterniond standard_r = Eigen::Quaterniond::Identity();
        for (int i=0; i<4; i++) {
            Eigen::Matrix<double, 6, 1> gradient_armor = Eigen::Matrix<double, 6, 1>::Zero();
            int sum_armor = 0;
            for (int j=0; j<2; j++) {
                if (isObserved[i*2+j]) {
                    sum_armor++;
                    gradient_armor += gradient[i*2+j];
                }
            }
            /* Add gradient to standard car module
                    t       r
                0: 0,0,z
                1: x,0,0
                2: 0,0,z
                3: x,0,0
            */
            if (sum_armor != 0)
                gradient_armor *= 2.0 / sum_armor;

            // 修正t
            Eigen::Vector3d armor_t_error = armor[i].t * 0.05;
            if (i % 2 == 0)
                armor_t_error(2) = 0;
            else
                armor_t_error(0) = 0;
            armor[i].t -= armor_t_error;


            if (reset) {
                moment[i] = gradient_armor;
                moment2[i] = pow(gradient_armor.norm(), 2);
            } else {
                moment[i] = BETA1 * moment[i] + (1 - BETA1) * gradient_armor;
                moment2[i] = BETA2 * moment2[i] + (1 - BETA2) * pow(gradient_armor.norm(), 2);
            }
            gradient_armor = moment[i] / (sqrt(moment2[i]) + 1e-8) * 1e-3;
            if (state == SAT || state == SATR)
                armor[i].t -= car_R_T * jacobi(gradient_armor.block<3, 1>(3, 0)) * gradient_armor.block<3, 1>(0, 0);
            if (state == SATR) {
                // 修正r
                // armor[i].r = armor[i].r.slerp(0.0001, standard_r).normalized();
                // standard_r = transform90_r * standard_r;
                // armor[i].r.normalize();

                armor[i].r = Eigen::Quaterniond(car_R_T * exp(gradient_armor.block<3, 1>(3, 0) * -0.25) *
                                                car_R * armor[i].r.matrix());
            }
        }

        reset = false;
        double delta_error = state < SAT ? (old_error - error_sum) : (old_obv_error - obv_error);
        StateType next_state = state;
        int mincnt = 50;
        if (state == SCTR)
            mincnt = 200;
        if (state == SATR)
            mincnt = 200;
        double min_delte_error = 0.01;
        // if (state > SCTR)
        //     min_delte_error = confidence_sum / 5;
        if (abs(delta_error) < min_delte_error && state_cnt > mincnt) {
            next_state = (StateType)((int)next_state + 1);
            state_cnt = 0;
        }

        if (cnt == 0)
            std::cout << "sum: " << sum << " error: " << obv_error << "\n";
        cnt++;
        state_cnt++;

        state = next_state;
        if (cnt >= 1e4) {
            std::cout << "cnt >= 1e4 !!! state: " << state << "\n";
            state = END;
        }
        if (state == END) {
            double new_confidence_sum = 0;
            for (int i=0; i<4; i++) {
                confidence[i] = confidence[i] * 0.6 + 0.4 * pow((isObserved[2*i] + isObserved[2*i+1]) / 2, 2);
                new_confidence_sum += confidence[i];
            }
            new_confidence_sum /= 4;
            if (new_confidence_sum > confidence_sum)
                confidence_sum = confidence_sum * 0.8 + new_confidence_sum * 0.2;
            else
                confidence_sum = confidence_sum * 0.95 + new_confidence_sum * 0.05;
            std::cout << "confidence sum: " << confidence_sum << "\n";
        }
        regularzation();
#ifdef BA_DEBUG
#ifdef PROCESS_DEBUG
        cv::imshow("error", img);
        cv::waitKey(0);
    }
#else
    }
    cv::imshow("error", img);
#endif
#else
    }
#endif

    std::cout << " error: " << obv_error << "\n";
    update_state(delta_time);

    return 0;
}
