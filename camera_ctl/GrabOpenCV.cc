#include "CameraCtl.hpp"

int main(int argc, char* argv[])
{
    CameraCtl camCtl;
    camCtl.startGrabbing();
    printf("Press any key to exit.\n");
    bool lowExposureTime = true;
    while (true) {
        Mat img = camCtl.getOpencvMat();
        camCtl.setExposureTime(lowExposureTime ? 8000 : 1000);
        lowExposureTime = !lowExposureTime;
        imshow("Image", img);
        if (waitKey(1) > 0)
            break;
    }
    waitKey(0);
    return 0;
}
