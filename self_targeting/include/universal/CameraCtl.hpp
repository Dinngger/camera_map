/**
 * file CameraCtl.hpp
 * The CameraCtl Class for using hikvision camera in opencv easily.
 * basics: by Dinger
 * complement:hqy
 * integraion: gqr
 * last change: 2020.1.16 10:07
 * All the params of the camera are integrated into: CameraCtl
 * Except "setExposureTime" which will be used in armor detection.
 */

#ifndef __CAMERA_CTL_HPP
#define __CAMERA_CTL_HPP

#include <stdio.h>
#include <string.h>
#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "CameraParam.cc"

using namespace cv;

namespace cm{

#define MAX_IMAGE_DATA_SIZE   (40*1024*1024)
int frame_empty = 0;
Mat Frame;

class CameraCtl
{
private:
    int nRet;
    unsigned char * pData;
    bool grabing;
    bool printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    VideoWriter writer;
    CameraParam Defalut_Param;
    //pthread_t nThreadID;
public:
    CameraCtl();
    ~CameraCtl();
    //static void *HKWorkThread(void *p_handle);
    int startGrabbing();
    int stopGrabbing();
    int setExposureTime(const float t);
    Mat getOpencvMat();
public:
    void* handle;
};
}; // namespace Camera

namespace cm{
bool CameraCtl::printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo){
        printf("%s\n" , "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE){               //网络设备调用
		// 打印当前相机ip和用户自定义名字
		// print current ip and user defined name
        printf("%s %x\n" , "nCurrentIp:" , pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                   //当前IP
        printf("%s %s\n" , "chUserDefinedName:" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);     //用户定义名
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE){           //usb设备调用
        printf("UserDefinedName:%s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else{
        printf("Not supported.\n");
    }
    return true;
}

CameraCtl::CameraCtl() {
    nRet = MV_OK;
    handle = NULL;
    pData = new unsigned char [MAX_IMAGE_DATA_SIZE];
    grabing = false;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
	// enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        printf("MV_CC_EnumDevices failed! nRet [%x]\n", nRet);
        return;
    }
    unsigned int nIndex = 0;
    if (stDeviceList.nDeviceNum > 0) {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo) {
                break;
            }
            printDeviceInfo(pDeviceInfo);
        }
    } 
    else {
        printf("Find No Device!\n");
        return;
    }
    // 选择设备并创建句柄
	// select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet) {
        printf("MV_CC_CreateHandle failed! nRet [%x]\n", nRet);
        return;
    }

    // 打开设备
	// open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet){
        printf("MV_CC_OpenDevice failed! nRet [%x]\n", nRet);
        return;
    }
    nRet = MV_CC_SetHeight(handle, Defalut_Param.Height);
    nRet = MV_CC_SetWidth(handle, Defalut_Param.Width);
    if (MV_OK!=nRet)
    {
        printf("MV_CC_SetSize failed! nRet [%x]\n", nRet);
        return;
    }

    //设置帧率
    nRet = MV_CC_SetFrameRate(handle, Defalut_Param.Fps);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetFrameRate failed! nRet [%x]\n", nRet);
        return;
    }

    //设置曝光时间
    nRet = MV_CC_SetExposureTime(handle, Defalut_Param.Exposure_Time);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetExposureTime failed! nRet [%x]\n", nRet);
        return;
    }

    //设置增益模式,使用固定值增益，不进行自动增益
    //nRet = MV_CC_SetGainMode(handle, Defalut_Param.Gain_Mode);
    //if (MV_OK != nRet)
    //{
    //    printf("MV_CC_SetGainmode failed! nRet [%x]\n", nRet);
    //    return;
    //}

    //设置增益固定值
    nRet = MV_CC_SetGain(handle, Defalut_Param.Gain);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetGain failed! nRet [%x]\n", nRet);
        return;
    }
    
    //设置gamma值
    nRet = MV_CC_SetGamma(handle, Defalut_Param.Gamma);

    nRet=MV_CC_SetBalanceRatioBlue(handle, Defalut_Param.b_balance);
    nRet=MV_CC_SetBalanceRatioGreen(handle, Defalut_Param.g_balance);
    nRet=MV_CC_SetBalanceRatioRed(handle, Defalut_Param.r_balance);
    if(nRet != MV_OK){
        printf("Failed to set balance ratio. nRet [%x]\n", nRet);
        return;
    }

    //设置曝光模式，使用固定曝光时间或是手动调节时间，不使用自动曝光
    //nRet = MV_CC_SetExposureAutoMode(handle, Defalut_Param.Auto_Exposure_Mode);
    //if (MV_OK != nRet)
    //{
    //   printf("MV_CC_SetExposure failed! nRet [%x]\n", nRet);
    //    return;
    //}

    //枚举设备
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet){
        printf("MV_CC_SetTriggerMode failed! nRet [%x]\n", nRet);
        return;
    }
}

CameraCtl::~CameraCtl() {
    if (grabing)
        stopGrabbing();
	if (pData) {
		delete pData;	
		pData = NULL;
	}
    // 销毁句柄
	// destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet){
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    }
}

int CameraCtl::startGrabbing() {
    // 开始取流
	// start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet){
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }
    grabing = true;
    return 0;
}

int CameraCtl::stopGrabbing() {
    // 停止取流
	// end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_StopGrabbing failed! nRet [%x]\n", nRet);
        return -1;
    }
    grabing = false;
    return 0;
}

int CameraCtl::setExposureTime(const float t){             //设置曝光时间
    nRet=MV_CC_SetExposureTime(handle, t);
    if(nRet != MV_OK){
        printf("Failed to set exposure time. nRet [%x]\n", nRet);
        return -1;
    }
    return 0;
}

//此函数用于低性能取图像，帧率上限60fps,只用于简单DEBUG
Mat CameraCtl::getOpencvMat() {
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned int nDataSize = MAX_IMAGE_DATA_SIZE;

	nRet = MV_CC_GetImageForBGR(handle, pData, nDataSize, &stImageInfo, Defalut_Param.nMsec);
    Mat img;
	if (nRet == MV_OK) {
		// (stImageInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
        img = Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
        //cv::resize(img, img, cv::Size(720, 540));
    }
    else
    {
        printf("MV_CC_GetImage failed!\n");
    }
    return img;

}
}

#endif // __CAMERA_CTL_HPP