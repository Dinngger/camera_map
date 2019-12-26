/*
 * GaussEliminationWithPivoting.hpp
 * 
 * Created Date: Sunday, September 29th 2019, 12:58:35 pm
 * by Dinger
 */

#ifndef __GAUSS_ELIMINATION_WITH_PIVOTING_HPP
#define __GAUSS_ELIMINATION_WITH_PIVOTING_HPP

#include <vector>
#include "Matrix.hpp"

template <typename T>
class GaussEliminationWithPivoting
{
private:
    Matrix<T> _Ab;
public:
    GaussEliminationWithPivoting() {}
    GaussEliminationWithPivoting(Matrix<T> Ab) : _Ab(Ab) {}
    inline void setMatrixAb(Matrix<T> Ab) {_Ab = Ab;}
    bool solve(std::vector<T>& result) const;
};

template <typename T>
bool GaussEliminationWithPivoting<T>::solve(std::vector<T>& result) const
{
    if (_Ab.rows() != _Ab.cols() - 1) {
        std::cout << "Wrong input!\n";
        return false;
    }
    int n = _Ab.rows();
    Matrix<T> Ab = _Ab;
    for (int k=0; k < n-1; k++) {
        int max_r = k;
        T max_value = abs(Ab.access(k, k));
        for (int i=k+1; i<n; i++) {
            if (abs(Ab.access(i, k)) > max_value) {
                max_r = i;
                max_value = abs(Ab.access(i, k));
            }
        }
        Ab.swapRow(k, max_r);
        if (max_value <= 1e-10) {
            std::cout << "Wrong! Singular matrix!\n";
            return false;
        }
        for (int i=k+1; i<n; i++) {
            Ab.access(i, k) /= Ab.access(k, k);
            for (int j=k+1; j<n+1; j++) {
                Ab.access(i, j) -= Ab.access(i, k) * Ab.access(k, j);
            }
        }
    }
    for (int k=n-1; k>=0; k--) {
        Ab.access(k, n) /= Ab.access(k, k);
        for (int i=k-1; i>=0; i--) {
            Ab.access(i, n) -= Ab.access(k, n) * Ab.access(i, k);
        }
    }
    for (int i=0; i<n; i++) {
        result[i] = Ab.access(i, n);
    }
    return true;
}

#endif // __GAUSS_ELIMINATION_WITH_PIVOTING_HPP
