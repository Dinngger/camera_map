#include "Drawer.h"
#include <pangolin/pangolin.h>
#include <mutex>

// #define HAVE_GLES

Drawer::Drawer()
{
}

void Drawer::reset(){
    glPopMatrix();
    glLoadIdentity();
    glPushMatrix();
}

void Drawer::DrawCurrentCamera(const std::vector<pangolin::OpenGlMatrix> &Twcs, const std::vector<cv::Point3d> &lbs)
{
    const double &w = 0.065;
    const double h = 0.0285;
    const double z = 0;

    for(size_t i=0; i<Twcs.size(); i++){
        glPushMatrix();
        #ifdef HAVE_GLES
            glMultMatrixf(Twc.m);
        #else
            glMultMatrixd(Twcs[i].m);
        #endif
        glLineWidth(3);
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
    
    
    glLineWidth(3);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    for(size_t i=0; i<lbs.size(); i+=2){
        glVertex3f(lbs[i].x, lbs[i].y, lbs[i].z);
        glVertex3f(lbs[i+1].x, lbs[i+1].y, lbs[i+1].z);
    }
    glEnd();
}

void Drawer::SetCurrentArmorPoses(const std::vector<cv::Mat> &Tcws, const std::vector<cv::Point3d> &lbs)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);

    //计算每帧到已知装甲板的变换矩阵
    mArmorPoses.clear();
    for(size_t i=0; i<Tcws.size(); i++){
        mArmorPoses.push_back(Tcws[i].clone());
    }
    for(size_t i=0; i<lbs.size(); i++){
        mlbs.push_back(lbs[i]);
    }
}

void Drawer::GetCurrentOpenGLCameraMatrix(std::vector<pangolin::OpenGlMatrix> &Ms)
{
    for (size_t i=0; i<mArmorPoses.size(); i++) {
        pangolin::OpenGlMatrix M;
        std::unique_lock<std::mutex> lock(mMutexCamera);

        // std::cout << "M: \n";
        for (int j=0; j<4; j++) {
            for (int k=0; k<4; k++) {
                M.m[4*k+j] = mArmorPoses[i].at<double>(j,k);
                // std::cout << M.m[4*k+j] << ' ';
            }
            // std::cout << std::endl;
        }
        Ms.push_back(M);
    }
}

std::vector<cv::Point3d> Drawer::GetLightBars()
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    return mlbs;
}
