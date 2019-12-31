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
        if (lowExposureTime) {
            imshow("Image", img);
            if (waitKey(1) > 0)
                break;
        } 
		Mat grayImage;
		binary = img.clone();
        cvtColor(img, grayImage, COLOR_BGR2GRAY);
		threshold(img, binary, 30, 200.0, CV_THRESH_BINARY);
		
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
		
		vector<RotatedRect> rrects;
		for (int i = 0;i < contours.size();i++)
		{
			//绘制轮廓的最小外结矩形
			RotatedRect rrect = minAreaRect(contours[i]);
			rrects.push_back(rrect);
		}
		for(int i = 0;i<rrects.size();i++){
			Point2f corners[4];
			rrects[i].points(corners);
			for (int j = 0;j < 4;j++) {
				line(img, corners[j], corners[(j + 1) % 4], Scalar(0, 0, 255), 2);
			}
		}
		imshow("binary", binary);
		imshow("img", img);
		waitKey(0);
    }
    waitKey(0);
    return 0;
}
