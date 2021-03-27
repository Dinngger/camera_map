#define SHOW_FRAME
#define SHOW_MODULE

#include "params.hpp"
#include "PoseSolver.hpp"

#ifdef SHOW_MODULE
#include "Viewer.h"
#include <thread>
#include <mutex>
#endif

int main(int argc, char* argv[])
{
    proj_path = argv[0];
    proj_path = proj_path.substr(0, proj_path.size() - 6) + "../../";
    cv::Matx<double, 3, 3> K  ( 1776.67168581218, 0, 720,
                                0, 1778.59375346543, 540,
                                0, 0, 1);
    PoseSolver ps(K, 0);

#ifdef SHOW_MODULE
    Viewer *viewer = new Viewer("main", K(0, 0), K(1, 1));
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);
#endif

    cv::VideoCapture cap(video_low_path);
    if (!cap.isOpened()) {
        printf("Unable to open video.\n");
        return 0;
    }
#ifdef SHOW_MODULE
    cv::VideoCapture cap_high(video_high_path);
    if (!cap_high.isOpened()) {
        printf("Unable to open video.\n");
        return 0;
    }
#endif

    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout<<"frame: ";
    std::cout<<totalFrameNumber<<std::endl;

    cv::Mat frame;
    cv::Mat high_frame;
    for (int w = 0; w < totalFrameNumber; w++) {
        std::cout << "\033[32m" << "frame: " << w << "\n";
        std::cout << "\033[0m";
        cap.read(frame);
		if (frame.empty())
            break;
        ps.newrun(frame, w);

#ifdef SHOW_MODULE
        std::vector<cv::Mat> Twcs;
        ps.getTwcs(Twcs);
        // std::cout <<"Twcs[0]: "<< Twcs[0] << std::endl;
        std::vector<cv::Point3d> lbs;
        ps.get_lbs(lbs);
        cap_high.read(high_frame);
        if(high_frame.empty())
            break;
        viewer->mDrawer.SetCurrentArmorPoses(Twcs, lbs,high_frame);
#endif

#ifdef SHOW_FRAME
        //ps.draw(frame);
        ps.drawNewCarModule(frame);
        cv::imshow("disp", frame);
        char key = cv::waitKey(0);
        if(key==27)
            break;
#else
        cv::waitKey(34);
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
