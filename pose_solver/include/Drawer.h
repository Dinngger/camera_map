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
    void DrawCurrentCamera(std::vector<pangolin::OpenGlMatrix> Twcs, std::vector<cv::Point3f> lbs);
    void SetCurrentArmorPoses(const std::vector<cv::Mat> &Tcws, std::vector<cv::Point3f> lbs);
    void GetCurrentOpenGLCameraMatrix(std::vector<pangolin::OpenGlMatrix> &Ms);
    std::vector<cv::Point3f> mlbs;
private:
    std::vector<cv::Mat> mArmorPoses, adjacent_frames_Tcw;
    std::mutex mMutexCamera;
};

#endif // __DRAWER_H
