// #define SHOW_FRAME
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
    PoseSolver ps(K, 0);

#ifdef SHOW_MODULE
    Viewer *viewer = new Viewer("main", K(0, 0), K(1, 1));
    std::thread* mpViewer = new std::thread(&Viewer::Run, viewer);
#endif
#define path0a "/home/dinger/mine/Dataset/videos/disp_low2.avi"
#define path0b "/home/dinger/mine/Dataset/videos/output_high.avi"
#define path1 "/home/sentinel/camera_map/pose_solver/cv_output1.avi"
#define path2a "/home/sentinel/videos/disp_low2.avi"
#define path2b "/home/sentinel/videos/output_high.avi"
#define path3 "/home/allegray/videos/disp_low2.avi"
#define path4 "/home/xjturm/rm2020/videos/disp_low2.avi"
#define path5 "../../../output_low.avi"
#define path6 "../../../output_high.avi"
    cv::VideoCapture cap(path0a);
    if (!cap.isOpened()) {
        printf("Unable to open video.\n");
        return 0;
    }
#ifdef SHOW_MODULE
    cv::VideoCapture cap_high(path0b);
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
    for (int w = 0; w < totalFrameNumber; w++)
    {
        std::cout << "\033[32m" << "frame: " << w << "\n";
        std::cout << "\033[0m";
        cap.read(frame);
		if(frame.empty())
            break;
        if(!isLowExposure(frame))
            continue;
        if(w==256||w==261||w==599||w==608||w==644||w==645||w==646||w==647||w==649||w==836) continue;
        if (w < 38)
            continue;
        ps.run(frame, w);

#ifdef SHOW_MODULE
        std::vector<cv::Mat> Twcs;
        ps.getTwcs(Twcs);
        // std::cout <<"Twcs[0]: "<< Twcs[0] << std::endl;
        std::vector<cv::Point3d> lbs;
        ps.get_lbs(lbs);
        if (w > 38) {
            cap_high.read(high_frame);
            if(high_frame.empty())
                break;
            viewer->mDrawer.SetCurrentArmorPoses(Twcs, lbs,high_frame);
        }
#endif

#ifdef SHOW_FRAME
        ps.draw(frame);
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
