#include "homography_solver.h"
#include <mutex>


Homography_solver::Homography_solver(){
    sumTime = 0;
    startTime = 0;
    endTime = 0;
}

Homography_solver::~Homography_solver(){
    ;
}

int Homography_solver::get_homography_matrix(int w, cv::Mat &frame, std::vector<cv::Mat> &mats){
    std::mutex mtx;
    mtx.lock();
    std::cout<<"frame: "<< w <<std::endl;
    startTime = cv::getTickCount();
    std::vector<cv::RotatedRect> result;
    //draw armor
    LLC_Frame my_frame(w);
    amp.lowExposure(frame, proced);
    amp.findPossible(proced);
    amp.drawArmorPlate(frame);
    amp.getArmorRect(my_frame.result);
    //draw end
    frames[w]=my_frame;
    my_frame.find_max();
    std::vector<cv::Point2f> points1,points2;

    if (!my_frame.is_ok)
    {
        cout<<"no armor"<<endl;
        return 1;
    }
    points1.push_back(my_frame.left_up_point);
    points1.push_back(my_frame.short_point);
    points1.push_back(my_frame.long_point);
    points1.push_back(my_frame.duijiao_point);
    points2.push_back(cv::Point2f(0,0));
    points2.push_back(cv::Point2f(0,5.6));
    points2.push_back(cv::Point2f(13.2,0));
    points2.push_back(cv::Point2f(13.2,5.6));
    cv::Mat homography_matrix = cv::findHomography ( points1, points2, cv::RANSAC, 3 );

    if (homography_matrix.rows==0)
    {
        cout<<"no homography matrix"<<endl;
        return 1;
    }
    //-- 分解单应矩阵
    std::vector<cv::Mat> r, t, n, Twcs;
    double intrinsic[9] = { 1770.71,       0,          738.37,
                            0,       1768.77,   610.32,
                            0,              0,          1}; //Xtion内参
    cv::Mat K(3, 3, CV_64F, intrinsic); //相机内参
    cv::decomposeHomographyMat(homography_matrix, K, r, t, n);
    
    int index = 1;
    for(int i=0;i<t.size()-3;i++){
        cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
        cv::Mat temp2, Twc;
        cv::hconcat(r[i], t[i], temp2);
        cv::vconcat(temp2, temp, Twc);
        Twcs.push_back(Twc);
    }

    mats = Twcs;


    /*
    for(int i=0; i<r.size(); ++i) {
        //cout << "======== " << i << " ========" << endl;
        //cout << "rotation" << i << " = " << endl;
        //cout << r[i] << endl;
        //cout << "translation" << i << " = " << endl;
        //cout << t[i] << endl;
        for (int j=0;j<4;j++)
        {
            double frame_point[3]={points2[j].x,points2[j].y,0};
            cv::Mat Point_Mat(3,1,CV_64FC1,frame_point);
            //cout<<"the point is "<<Point_Mat<<endl;
            cv::Mat answer_temp_Mat=r[i]*Point_Mat+t[i];
            //cout<<"answer is"<<answer_temp_Mat<<endl;
            point_answer.push_back(answer_temp_Mat);
            //cout<<"point=="<<t[i]<<endl;
        }
    }
    */


    endTime = cv::getTickCount();
    cout<<endTime-startTime<<endl;
    return 0;
}


