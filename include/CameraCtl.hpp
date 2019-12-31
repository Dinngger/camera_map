/**
 * file CameraCtl.hpp
 * The CameraCtl Class for using hikvision camera in opencv easily.
 * by Dinger
 * last change: 2019.12.10
 */

#ifndef __CAMERA_CTL_HPP
#define __CAMERA_CTL_HPP

#include <stdio.h>
#include <string.h>
#include "MvCameraControl.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>

using namespace cv;

#define MAX_IMAGE_DATA_SIZE   (40*1024*1024)

class CameraCtl
{
private:
    int nRet;
    void* handle;
    unsigned char * pData;
    bool grabing;
    bool printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
public:
    CameraCtl();
    ~CameraCtl();
    int startGrabbing();
    int stopGrabbing();
    Mat getOpencvMat(int Msec=1000);
    int setExposureTime(double t);
};

#endif // __CAMERA_CTL_HPP
