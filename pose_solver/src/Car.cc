/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.4.9 23:54
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CarModule.hpp"

// #define PROCESS_DEBUG

#ifdef PROCESS_DEBUG
#include "Viewer.h"
#include <thread>
#include <mutex>
#endif

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

int Car::bundleAdjustment ( const std::vector<LightBarP> &light_bars,
                            const Eigen::Matrix3d &K,
                            double delta_time)
{
#ifdef PROCESS_DEBUG
    Viewer *viewer = new Viewer(K(0, 0), K(1, 1));
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);
#endif

    enum StateType {
        SCT = 0,    // State of changing Car T
        SCTR,   // State of changing Car T & R
        SAT,    // State of changing Armor T
        SATR,   // State of changing Armor T & R
        END
    };
#define BA_DEBUG
#ifdef  BA_DEBUG
    cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    auto get_y = [](int x) -> int {
        return x < 0 ? 0 : (x > 999 ? 999 : x);
    };
    auto get_x = [](int x) -> int {
        return x < 0 ? 0 : (x > 999 ? x % 1000 : x);
    };
#endif

    double error_sum = 1e8;
    double obv_error = 1e8;
    double old_error;
    double old_obv_error;
    Eigen::Matrix<double, 6, 1> moment[4];
    Eigen::Matrix<double, 6, 1> moment_sum;
    double moment2[4];
    double moment2_sum = 0;
    StateType state = SCT;
    StateType state_max = SCT;
    bool reset = true;
    int cnt = 0;
    int state_cnt = 0;
    double state_error = 0;

    while (state != END) {
#ifdef BA_DEBUG
        old_error = error_sum;
        old_obv_error = obv_error;
#endif
        bool isObserved[8] = {false,};
        Eigen::Matrix<double, 6, 1> gradient[8];
        double error[8] = {0,};
        int sum = 0;

        // computeGradient
        ///TODO: add change punishment
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
        gradient_sum *= 2.0 / rate_sum;

#define BETA1 0.95
#define BETA2 0.98
        if (reset) {
            moment_sum = gradient_sum;
            moment2_sum = pow(gradient_sum.norm(), 2);
        } else {
            moment_sum = BETA1 * moment_sum + (1 - BETA1) * gradient_sum;
            moment2_sum = BETA2 * moment2_sum + (1 - BETA2) * pow(gradient_sum.norm(), 2);
        }
        gradient_sum = moment_sum / (sqrt(moment2_sum) + 1e-8) * 1e-3;
        if (state == SCT || state == SCTR) {
            t -= jacobi(gradient_sum.block<3, 1>(3, 0)) * gradient_sum.block<3, 1>(0, 0);
        }
        if (state == SCTR) {
            r = Eigen::Quaterniond(exp(gradient_sum.block<3, 1>(3, 0) * -1) * r.matrix());
        }
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
            if (sum_armor == 0)
                continue;
            gradient_armor *= 2.0 / sum_armor;
            if (reset) {
                moment[i] = gradient_armor;
                moment2[i] = pow(gradient_armor.norm(), 2);
            } else {
                moment[i] = BETA1 * moment[i] + (1 - BETA1) * gradient_armor;
                moment2[i] = BETA2 * moment2[i] + (1 - BETA2) * pow(gradient_armor.norm(), 2);
            }
            gradient_armor = moment[i] /(sqrt(moment2[i]) + 1e-8) * 1e-3;
            if (state == SAT || state == SATR)
                armor[i].t -= car_R_T * jacobi(gradient_armor.block<3, 1>(3, 0)) * gradient_armor.block<3, 1>(0, 0);
            if (state == SATR) {
                armor[i].r = Eigen::Quaterniond(car_R_T * exp(gradient_armor.block<3, 1>(3, 0) * -1) *
                                                car_R * armor[i].r.matrix());
            }
        }

        reset = false;
        double delta_error = state < SAT ? (old_error - error_sum) : (old_obv_error - obv_error);
        state_error += delta_error;
        StateType next_state = state;
        state_max = state_max > state ? state_max : state;
        double error_min = 0.01;
        switch (state) {
        case SCT:
            error_min = 0.1;
        case SCTR:
        case SAT:
        case SATR:
            if (delta_error < error_min && state_cnt > 50) {
                next_state = (StateType)((int)next_state + 1);
                state_cnt = 0;
                state_error = 0;
            }
            // else if (state_error > 30 && state == SAT) {
            //     next_state = (StateType)((int)next_state - 2);
            //     state_error = 0;
            //     state_cnt = 0;
            // }
            break;
        default:
            printf("ERROR: Invalid State\n");
            next_state = SCT;
            break;
        }

        if (cnt == 0)
            std::cout << "sum: " << sum << " error: " << obv_error << "\n";
        cnt++;
        state_cnt++;

#ifdef PROCESS_DEBUG
        std::vector<cv::Mat> Twcs;
        std::vector<cv::Point3d> lbs;
        for (int j=0; j<4; j++) {
            Eigen::Matrix3d armor_R = armor[j].r.matrix();
            Eigen::Vector3d armor_t = armor[j].t;
            for (int k=0; k<4; k++) {
                Eigen::Vector3d tmp_p = r.matrix() * (armor_R * armor_module[k] + armor_t) + t;
                lbs.emplace_back(tmp_p(0), tmp_p(1), tmp_p(2));
            }
        }
        viewer->mDrawer.SetCurrentArmorPoses(Twcs, lbs);
#endif

#ifdef BA_DEBUG
        cv::Vec3b g_color, e_color;
        if (state == SCT) {
            g_color = cv::Vec3b(255, 180, 180);
            e_color = cv::Vec3b(255, 0, 0);
        } else if (state == SCTR) {
            g_color = cv::Vec3b(180, 255, 180);
            e_color = cv::Vec3b(0, 255, 0);
        } else if (state == SAT) {
            g_color = cv::Vec3b(180, 180, 255);
            e_color = cv::Vec3b(0, 0, 255);
        } else {
            g_color = cv::Vec3b(180, 255, 255);
            e_color = cv::Vec3b(0, 255, 255);
        }
        img.at<cv::Vec3b>(get_y(999 - (int)(gradient_sum.norm()*5e5)), get_x(cnt)) = g_color;
        img.at<cv::Vec3b>(get_y(999 - (int)(state < SAT ? error_sum : obv_error)), get_x(cnt)) = e_color;
#endif
        state = next_state;
        if (cnt >= 1e4) {
            std::cout << "cnt >= 1e4 !!!\n";
            state = END;
        }
        if (state == END) {
            for (int i=0; i<4; i++) {
                confidence[i] = confidence[i] * 0.8 + 0.2 * pow((isObserved[2*i] + isObserved[2*i+1]) / 2, 2);
            }
        }
    }
#ifdef BA_DEBUG
    cv::imshow("error", img);
    cv::waitKey(1);
#endif
    std::cout << " error: " << obv_error << "\n";
    regularzation();
    update_state(delta_time);

#ifdef PROCESS_DEBUG
    viewer->RequestFinish();
    while (mpViewer->joinable())
        mpViewer->join();
#endif

    return 0;
}
