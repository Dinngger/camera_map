
#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pangolin/pangolin.h>
#include <mutex>

class Drawer
{
public:
    Drawer();

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
private:
    float mGraphLineWidth;
    float mCameraSize;
    float mCameraLineWidth;
    cv::Mat mCameraPose;
    std::mutex mMutexCamera;
};

#endif // RAWER_H
