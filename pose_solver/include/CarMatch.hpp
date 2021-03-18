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
    int first_index = -1;
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
    transBuffer w_trans;
public:
    char r_trans[13];
    std::vector<CarPossible> carsPossible;
public:
    void transformerMatch(const std::vector<aim_deps::Light> &lights);

    void infoExchange(const std::vector<aim_deps::Light> &lights);

    CarMatch();
    ~CarMatch();
};

#endif //_CAR_MATCH_HPP
