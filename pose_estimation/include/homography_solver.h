#ifndef homography_solver
#define homography_solver

#include <queue>
#include <thread>
#include "frame.h"

class Homography_solver{
public:
    Homography_solver();
    ~Homography_solver();
public:
    LLC_Frame frames[1000];
    double sumTime, startTime, endTime;
    cv::Mat proced;
    ArmorPlate amp;
    int get_homography_matrix(int w, cv::Mat &frame, std::vector<cv::Mat> &mat);
};

#endif // homography_solver
