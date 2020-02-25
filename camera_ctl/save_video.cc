#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;
#include "CameraCtl.hpp"

int save_video(int argc, char* argv[]) {
	CameraCtl camCtl;
	camCtl.startGrabbing();
	printf("Press any key to exit.\n");
	string path = "test.avi";
	camCtl.setExposureTime(400);
	Mat img = camCtl.getOpencvMat();
	CvVideoWriter* writer = cvCreateVideoWriter("test.avi",CV_FOURCC('M','J','P','G'), 25, img.size(), true);
	int count = 0;
	while (true) {
		Mat img = camCtl.getOpencvMat();
		IplImage ipl_img = img;
		count++;
        cvWriteFrame(writer, &ipl_img);
		imshow("img",img);
		if (waitKey(1) == 'q')
			break;
	}
	cvReleaseVideoWriter(&writer);
	cvDestroyWindow("img");
	cout<<"done"<<endl;
	return 0;
}
