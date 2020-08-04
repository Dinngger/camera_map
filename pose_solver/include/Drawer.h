#ifndef __DRAWER_H
#define __DRAWER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pangolin/pangolin.h>
#include <mutex>

class Drawer
{
public:
    Drawer();
    void clear() {mArmorPoses.clear();}
    void reset();
    void DrawCurrentCamera(const std::vector<pangolin::OpenGlMatrix> &Twcs, const std::vector<cv::Point3d> &lbs);
    void SetCurrentArmorPoses(const std::vector<cv::Mat> &Tcws, const std::vector<cv::Point3d> &lbs,const cv::Mat &frame);
    void GetCurrentOpenGLCameraMatrix(std::vector<pangolin::OpenGlMatrix> &Ms);
    std::vector<cv::Point3d> GetLightBars();
    std::vector<cv::Point3d> mlbs;
    cv::Mat high_frame;
private:
    std::vector<cv::Mat> mArmorPoses, adjacent_frames_Tcw;
    std::mutex mMutexCamera;
};

#endif // __DRAWER_H
