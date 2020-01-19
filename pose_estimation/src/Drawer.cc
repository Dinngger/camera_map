#include "Drawer.h"
#include <pangolin/pangolin.h>
#include <mutex>

// #define HAVE_GLES

Drawer::Drawer()
{
    mGraphLineWidth = 0.9;
    mCameraSize = 0.8;
    mCameraLineWidth = 3;
    reset_flag = false;
}

void Drawer::reset(){
    glPopMatrix();
    glLoadIdentity();
    glPushMatrix();
}

void Drawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const double &w = mCameraSize;
    const double h = w*0.75;
    const double z = w*0.6;
    if(reset_flag)
        glLoadIdentity();
    else{
        glPushMatrix();

        #ifdef HAVE_GLES
            glMultMatrixf(Twc.m);
        #else
            glMultMatrixd(Twc.m);
        #endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void Drawer::SetCurrentArmorPoses(const std::vector<cv::Mat> &Tcws)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);


    /*
    // 计算两帧之间的变换矩阵
    cv::Mat last_Rwc(3,3,CV_32F);
    cv::Mat last_twc(3,1,CV_32F);
    cv::Mat Rwc(3,3,CV_32F);
    cv::Mat twc(3,1,CV_32F);
    cv::Mat temp_Rwc;
    std::vector<cv::Mat> adjacent_frames_Tcw;
    if(mArmorPoses.size()){
        for(int i=0;i<Tcws.size();i++){     
            last_Rwc = mArmorPoses[i].rowRange(0,3).colRange(0,3);
            last_twc = mArmorPoses[i].rowRange(0,3).col(3);
            Rwc = Tcws[i].rowRange(0,3).colRange(0,3);
            twc = Tcws[i].rowRange(0,3).col(3);
            cv::invert(last_Rwc, temp_Rwc);
            Rwc = Rwc*temp_Rwc;
            twc = twc-last_twc;

            cv::Mat temp =(cv::Mat_<double>(1,4)<<0,0,0,1);
            cv::Mat temp2, Twc;
            cv::hconcat(Rwc, twc, temp2);
            cv::vconcat(temp2, temp, Twc);
            adjacent_frames_Tcw.push_back(Twc);
        }
        std::cout<<"adjacent_frames_Tcw size: "<<adjacent_frames_Tcw.size()<<std::endl;
        mArmorPoses.clear();
        for(int i=0;i<adjacent_frames_Tcw.size();i++){
            mArmorPoses.push_back(adjacent_frames_Tcw[i].clone());
        }
    }
    else{
        mArmorPoses.clear();
        for(int i=0;i<Tcws.size();i++){
            mArmorPoses.push_back(Tcws[i].clone());
        }
    }
    */


    //计算每帧到已知装甲板的变换矩阵
    mArmorPoses.clear();
    // std::cout << "Tcws.size(): " << Tcws.size() <<std::endl;
    for(int i=0;i<Tcws.size();i++){
        mArmorPoses.push_back(Tcws[i].clone());
    }
}

void Drawer::GetCurrentOpenGLCameraMatrix(std::vector<pangolin::OpenGlMatrix> &Ms, float &mViewpointX, float &mViewpointY, float &mViewpointZ)
{
    if(mArmorPoses.size())
    {
        cv::Mat Rwc(3,3,CV_64F);
        cv::Mat twc(3,1,CV_64F);
        
        //std::cout << "mArmorPoses.size(): " << mArmorPoses.size() <<std::endl;
        for(int i=0;i<mArmorPoses.size();i++){
            pangolin::OpenGlMatrix M;
            {
            std::unique_lock<std::mutex> lock(mMutexCamera);
            Rwc = mArmorPoses[i].rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mArmorPoses[i].rowRange(0,3).col(3);
            }
            
            mViewpointX = twc.at<double>(0);
            mViewpointY = twc.at<double>(1);
            mViewpointZ = twc.at<double>(2);

            M.m[0] = Rwc.at<double>(0,0);
            M.m[1] = Rwc.at<double>(1,0);
            M.m[2] = Rwc.at<double>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<double>(0,1);
            M.m[5] = Rwc.at<double>(1,1);
            M.m[6] = Rwc.at<double>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<double>(0,2);
            M.m[9] = Rwc.at<double>(1,2);
            M.m[10] = Rwc.at<double>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<double>(0);
            M.m[13] = twc.at<double>(1);
            M.m[14] = twc.at<double>(2);
            M.m[15]  = 1.0;
            Ms.push_back(M);
        }
        //std::cout << "Ms: " << Ms.size() <<std::endl;
    }
    else{
        pangolin::OpenGlMatrix M;
        M.SetIdentity();
        Ms.push_back(M);
    }
       
}
