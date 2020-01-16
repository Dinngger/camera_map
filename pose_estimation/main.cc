#include "Viewer.h"

int initMat(cv::Mat &mat, float (*m)[4])
{
    for (int i=0; i<mat.rows; i++) {
        for (int j=0; j<mat.cols; j++) {
            mat.at<float>(i, j) = m[i][j];
        }
    }
    return 0;
}

int main(int argc, char* argv[])
{
    Viewer viewer;
    cv::Mat pos(4, 4, CV_32F);
    float m[][4]={
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    };
    initMat(pos, m);
    viewer.mDrawer.SetCurrentCameraPose(pos);
    viewer.Run();
    return 0;
}
