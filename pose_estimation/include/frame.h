#ifndef FRAME
#define FRAME

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include "opencv2/objdetect.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <queue>
#include <ArmorPlate.hpp>

using namespace std;

class LLC_Frame{
public:
    LLC_Frame();
    LLC_Frame(int x);
    ~LLC_Frame();
    void find_max();
public:
    int Frame_count;
    bool is_ok;
    cv::RotatedRect max_rect;
    cv::Mat img;
    cv::Point2f left_up_point;
    std::vector<cv::RotatedRect> result;
    cv::Point2f short_point;
    cv::Point2f long_point;
    cv::Point2f duijiao_point;
};
#endif // FRAME