/*
 * Matrix.hpp
 * 
 * Created Date: Sunday, September 29th 2019, 1:21:37 pm
 * by Dinger
 */

#ifndef __MATRIX_HPP
#define __MATRIX_HPP

#include <iostream>
#include <vector>

#define abs(x) ((x) >= 0 ? (x) : -(x))

typedef unsigned int uint;

template <typename T>
class Matrix {
private:
    uint _rows;
    uint _cols;
    uint _size;
    std::vector<T> _data;
public:
    Matrix();
    Matrix(uint rows, uint cols);
    Matrix(const Matrix<T> &a) : _rows(a._rows), _cols(a._cols), _size(a._size) {
        _data.resize(_size);
        std::copy(a._data.begin(), a._data.end(), _data.begin());
    };
    ~Matrix() {}
    inline uint rows() const {return _rows;}
    inline uint cols() const {return _cols;}
    inline T& access(uint row, uint col) {
        return _data[col * _rows + row];
    }
    inline T read(uint row, uint col) const {
        return _data[col * _rows + row];
    }
    void swapRow(uint i, uint j);
    void swapCol(uint i, uint j);
    void say() const;
};

template <typename T>
Matrix<T>::Matrix():
        _rows(0), _cols(0) {
    _size = _rows * _cols;
    _data.resize(_size);
}

template <typename T>
Matrix<T>::Matrix(uint rows, uint cols):
        _rows(rows), _cols(cols) {
    _size = rows * cols;
    _data.resize(_size);
}

template <typename T>
void Matrix<T>::swapRow(uint i, uint j) {
    if (i == j)
        return;
    for(uint col = 0; col < _cols; col++) {
        std::swap(access(i, col), access(j, col));
    }
}

template <typename T>
void Matrix<T>::swapCol(uint i, uint j) {
    if (i == j)
        return;
    for(uint row = 0; row < _rows; row++) {
        std::swap(access(row, i), access(row, j));
    }
}

template <typename T>
void Matrix<T>::say() const
{
    for (int i=0; i<_rows; i++) {
        for (int j=0; j<_cols; j++)
            std::cout << read(i, j) << " ";
        std::cout << std::endl;
    }
}

#endif // __MATRIX_HPP
