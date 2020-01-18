#include <mutex>
#include "Viewer.h"
#include "homography_solver.h"

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
    Viewer *viewer = new Viewer();
    std::thread* mpViewer = new thread(&Viewer::Run, viewer);

    Homography_solver *Hsolve = new Homography_solver();
    std::vector<cv::Mat> armor3dpoints;
    std::vector<cv::RotatedRect> result;
    cv::VideoCapture cap("../../test.avi");
    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::vector<cv::Mat>  point_answer;
    std::cout<<"frame: -1"<<std::endl;
    std::cout<<totalFrameNumber<<std::endl;

    std::vector<cv::Mat> mats {cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F),cv::Mat(4, 4, CV_32F)};
    cv::Mat frame;

    for (int w = 0; w < totalFrameNumber; w++)
    {
        cap.read(frame);
        if (Hsolve->get_homography_matrix(w, frame, mats) == 1)
            continue;
        viewer->mDrawer.SetCurrentArmorPoses(mats);
        cv::imshow("frame", frame);
        cv::waitKey(0);
    }
    return 0;
}
