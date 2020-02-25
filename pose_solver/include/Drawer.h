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
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentArmorPoses(const std::vector<cv::Mat> &Tcws);
    void GetCurrentOpenGLCameraMatrix(std::vector<pangolin::OpenGlMatrix> &Ms);
private:
    std::vector<cv::Mat> mArmorPoses, adjacent_frames_Tcw;
    std::mutex mMutexCamera;
};

#endif // __DRAWER_H
