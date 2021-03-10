#ifndef _CAR_MATCH_HPP
#define _CAR_MATCH_HPP

#include <map>
#include <chrono>
#include <vector>
#include <stack>
#include <algorithm>
#include <sys/stat.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <numeric>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "AimDeps.hpp"
#define BUFFER_SIZE 65
#define EXIST_INDEX 52

struct CarPossible
{
    int first = -1;
    std::vector<aim_deps::Light> lightPossibles;
};

union transBuffer{
    float data[BUFFER_SIZE];
    char buf[4 * BUFFER_SIZE];
};

class CarMatch
{
private:    
    int in_fd;
    int out_fd;
    int nCar = 0;
    transBuffer w_trans;
public:
    char r_trans[13];
    std::vector<CarPossible> carsPossible;
public:
    /**
     * @brief 灯条输入之后，先转化为对应的中点向量格式，与Python文件交互输出结果保存到div中
     */
    void transformerMatch(const std::vector<aim_deps::Light> &hLights);

    CarMatch();
    ~CarMatch();
};

#endif //_CAR_MATCH_HPP
