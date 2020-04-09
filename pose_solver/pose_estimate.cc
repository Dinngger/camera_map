#define SHOW_FRAME
#define SHOW_MODULE

#include "PoseSolver.hpp"

#ifdef SHOW_MODULE
#include "Viewer.h"
#include <thread>
#include <mutex>
#endif

int main(int argc, char* argv[])
{
    cv::Matx<double, 3, 3> K  ( 1776.67168581218, 0, 720,
                                0, 1778.59375346543, 540,
                                0, 0, 1);
    PoseSolver ps(K);

#ifdef SHOW_MODULE
    Viewer *viewer = new Viewer(K(0, 0), K(1, 1));
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);
#endif

    cv::VideoCapture cap("/home/sentinel/camera_map/pose_solver/cv_output1.avi");
    if (!cap.isOpened()) {
        printf("Unable to open video.\n");
        return 0;
    }

    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout<<"frame: ";
    std::cout<<totalFrameNumber<<std::endl;

    cv::Mat frame;
    int count = 0;
    for (int w = 0; w < totalFrameNumber; w++)
    {
        cap.read(frame);
		if(frame.empty())
            break;
        if(!isLowExposure(frame))
            continue;
        count++;
        ps.run(frame, count);

#ifdef SHOW_MODULE
        std::vector<cv::Mat> Twcs;
        ps.getTwcs(Twcs);
        // std::cout <<"Twcs[0]: "<< Twcs[0] << std::endl;
        std::vector<cv::Point3d> lbs;
        ps.get_lbs(lbs);
        viewer->mDrawer.SetCurrentArmorPoses(Twcs, lbs);
#endif

#ifdef SHOW_FRAME
        ps.draw(frame);
        cv::imshow("disp", frame);
        char key = cv::waitKey(0);
        if(key==27)
            break;
#endif
	}
#ifdef SHOW_FRAME
	cv::destroyAllWindows();
#endif
#ifdef SHOW_MODULE
    viewer->RequestFinish();
    while (mpViewer->joinable())
        mpViewer->join();
#endif
    return 0;
}
