#ifndef __ERROR_TERM_HPP
#define __ERROR_TERM_HPP
/**
 * @brief 灯条建模 V4 (扩散模型)
 * @author hqy
 * @date 2021.3.12
 */ 
#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <vector>
#define __ERROR_TERM_A 2.5

class ErrorTerm {
public:
    ErrorTerm(const std::vector<double>& vals, int col, int row, double radius):
        values(vals), img_col(col), img_row(row)
    {
            _a = __ERROR_TERM_A;
            _b = radius;
            cx = double(img_col) / 2;
            cy = double(img_row) / 2;
    }

    ~ErrorTerm(){;}

    /// @brief top: 上顶点 ctr: 中点 + 扩散变化
    template <typename T>
    bool operator() (const T* const _top, const T* const _ctr, T* residual) const {
        Eigen::Matrix<T, 2, 1> top(_top[0], _top[1]);
        Eigen::Matrix<T, 2, 1> ctr(_ctr[0], _ctr[1]);
        Eigen::Matrix<T, 2, 1> t2c = ctr - top;         // top -> center
        T t2c_norm = t2c.norm();                        // 法向量需要单位化
        Eigen::Matrix<T, 2, 1> bottom = ctr + t2c;
        Eigen::Matrix<T, 2, 1> normal(t2c(1) / t2c_norm, - t2c(0) / t2c_norm);
        T b = T(_b) + _ctr[2];                           // ctr[2] 不是中心,是扩散半径
        T results(0);
        for (int i = 0; i < img_row; i++) {
            int base = i * img_col;
            for (int j = 0; j < img_col; j++) {
                Eigen::Matrix<T, 2, 1> t2n(T(j) - top(0), T(i) - top(1)), b2n(T(j) - bottom(0), T(i) - bottom(1));
                T t_len = t2n.norm(), b_len = b2n.norm(); 
                bool bigger = (b_len >= t_len);
                T prod = bigger ? t2n.dot(t2c) : b2n.dot(- t2c);  // 更加靠近顶部 ? 内积求夹角cos : -t2c = b2c
                T dist = T(0);
                if (prod >= T(0)) {                        // 90度夹角之内
                    dist = bigger ? ceres::abs(t2n.dot(normal)) : ceres::abs(b2n.dot(normal));
                }
                else {
                    dist = bigger ? t_len : b_len;
                }
                T decay = T(1) / (ceres::exp(T(_a) * (dist - b)) + T(1));     // 计算光线衰减
                T value = T(values[base + j]);
                results += ceres::pow(value - decay, 2);
            }
        }

        /// 添加一些惩罚项
        residual[0] = results;
        T extra_loss(0.0);
        borderPunish<T>(top(0), top(1), extra_loss);
        borderPunish<T>(bottom(0), bottom(1), extra_loss);
        rangePunish<T>(b, extra_loss);
        residual[0] += extra_loss;
        return true;
    }

    static ceres::CostFunction* Create(const std::vector<double>& vals, int col, int row, double radius) {
        return new ceres::AutoDiffCostFunction<ErrorTerm, 1, 2, 3>(
            new ErrorTerm(vals, col, row, radius)
        );
    }

private:
    /// ============== 惩罚化的弱约束问题 ==============
    /// 边界惩罚
    template <typename T>
    inline void borderPunish(const T& x, const T& y, T& term) const {
        if (x < T(0.0)) {
            term += ceres::pow(x, 2);
        }
        else if (x > T(img_col)) {
            term += ceres::pow(x - T(img_col), 2);
        }
        if (y < T(0.0)) {
            term += ceres::pow(y, 2);
        }
        else if (y > T(img_row)) {
            term += ceres::pow(y - T(img_row), 2);
        }
    }

    /// 扩散范围惩罚
    template <typename T>
    inline void rangePunish(const T& b, T& term) const {
        if (b < T(1.1)) {
            term += T(9.0) * (T(1.1) - b);
        }
        else if (b > T(2.4)) {
            term += T(9.0) * (b - T(2.4));
        }
    }

public:
    const std::vector<double>& values;
    int img_col;
    int img_row;

    double cx;
    double cy;
    double _a;
    double _b;
};

#endif  //__ERROR_TERM_HPP