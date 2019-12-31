#include "CameraCtl.hpp"

int main(int argc, char* argv[])
{
    CameraCtl camCtl;
    camCtl.startGrabbing();
    printf("Press any key to exit.\n");
    while (true) {
        Mat img = camCtl.getOpencvMat();
        imshow("Image", img);
        if (waitKey(1) > 0)
            break;
    }
    waitKey(0);
    return 0;
}
