#include "CameraCtl.hpp"
#include <vector>

int main(int argc, char* argv[])
{
    CameraCtl camCtl;
    camCtl.startGrabbing();
    printf("Press any key to exit.\n");
    bool lowExposureTime = true;
    std::vector<RotatedRect> rrects;
    float mean_mean = 40;
    while (true) {
        Mat img = camCtl.getOpencvMat();
        camCtl.setExposureTime(lowExposureTime ? 7000 : 50);
        lowExposureTime = !lowExposureTime;
		Mat grayImage;
		Mat binary = img.clone();
        cvtColor(img, grayImage, COLOR_BGR2GRAY);
        Scalar meanValue;
        meanValue = mean(grayImage);
        printf("\rmean: %f\t", meanValue.val[0]);
        bool lowExposure = meanValue.val[0] < mean_mean;
        mean_mean = mean_mean * 0.95 + 0.05 * meanValue.val[0];

        if (lowExposure) {
            rrects.clear();
            threshold(grayImage, binary, 253, 255, CV_THRESH_BINARY);
 
            std::vector<std::vector<Point>> contours;
            std::vector<Vec4i> hierarchy;
            findContours(grayImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

            for (int i = 0;i < contours.size();i++)
            {
                //绘制轮廓的最小外结矩形
                RotatedRect rrect = minAreaRect(contours[i]);
                rrects.push_back(rrect);
            }
            // imshow("lowExposureImage", img);
            // if (waitKey(1) > 0)
            //     break;
        } else {
            for(int i = 0;i<rrects.size();i++){
                Point2f corners[4];
                rrects[i].points(corners);
                for (int j = 0;j < 4;j++) {
                    line(img, corners[j], corners[(j + 1) % 4], Scalar(0, 0, 255), 2);
                }
            }
            imshow("Image", img);
            if (waitKey(1) > 0)
                break;
        }
    }
    return 0;
}
