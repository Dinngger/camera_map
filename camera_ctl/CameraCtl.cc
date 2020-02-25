#include "CameraCtl.hpp"

bool CameraCtl::printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("%s\n" , "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
		// 打印当前相机ip和用户自定义名字
		// print current ip and user defined name
        printf("%s %x\n" , "nCurrentIp:" , pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                   //当前IP
        printf("%s %s\n\n" , "chUserDefinedName:" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);     //用户定义名
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

CameraCtl::CameraCtl() {
    nRet = MV_OK;
    handle = NULL;
    pData = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
    grabing = false;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
	// enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
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
    } else {
        printf("Find No Devices!\n");
        return;
    }

    scanf("%d", &nIndex);

    // 选择设备并创建句柄
	// select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return;
    }

    // 打开设备
	// open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return;
    }

	nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return;
    }
 
    nRet = MV_CC_SetFloatValue(handle, "Gain", 17);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetGain fail! nRet [%x]\n", nRet);
        return;
    }
}

CameraCtl::~CameraCtl() {
    if (grabing)
        stopGrabbing();
	if (pData) {
		free(pData);	
		pData = NULL;
	}
    // 销毁句柄
	// destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    }
	printf("exit\n");
}

int CameraCtl::startGrabbing() {
    // 开始取流
	// start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
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
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }
    grabing = false;
    return 0;
}

Mat CameraCtl::getOpencvMat(int Msec) {
	MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned int nDataSize = MAX_IMAGE_DATA_SIZE;

	nRet = MV_CC_GetImageForBGR(handle, pData, nDataSize, &stImageInfo, Msec);
    Mat img;
	if (nRet == MV_OK) {
		// (stImageInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
		img = Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
	} else {
        printf("MV_CC_GetImage fail!\n");
    }
    return img;
}

int CameraCtl::setExposureTime(double t) { 
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", t);
    if (MV_OK == nRet)
    {
        // printf("set exposure time OK!\n\n");
        return 0;
    } else {
        printf("set exposure time failed! nRet [%x]\n\n", nRet);
        return -1;
    }
}
