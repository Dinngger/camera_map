/* 误差下降控制器
* author: Dinger
* create date: 2020.4.10 00:34
*/

#include "CarModule.hpp"

int Car::optimization(const int cnt,
                      const Eigen::Matrix<double, 6, 1> gradient[],
                      const Eigen::Matrix<double, 6, 1>& gradient_sum,
                      const bool isObserved[],
                      Eigen::Matrix<double, 6, 1> moment[],
                      Eigen::Matrix<double, 6, 1>& moment_sum,
                      double moment2[],
                      double& moment2_sum) {
    return 0;
}
