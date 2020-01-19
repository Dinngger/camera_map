
#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <mutex>

Viewer::Viewer():
    mbFinishRequested(false),
    mbFinished(true),
    mbStopped(true),
    mbStopRequested(false)
{
    float fps = 30;
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = 640;
    mImageHeight = 480;

    mViewpointX = 4.0;
    mViewpointY = 2.18558;
    mViewpointZ = -6.26*2;
    mViewpointF = 500;
}

void Viewer::clear()
{
    mDrawer.clear();
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    //pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    //pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    std::vector<pangolin::OpenGlMatrix> Twcs;
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    Twcs.push_back(Twc);

    //cv::namedWindow("Current Frame");

    bool bFollow = false;
    while(1)
    {
        Twcs.clear();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mDrawer.GetCurrentOpenGLCameraMatrix(Twcs, mViewpointX, mViewpointY, mViewpointZ);


        /*
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twcs[0]);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }
        //pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ*1.5, 0,0,0,0.0,-1.0, 0.0);
        //s_cam.Follow(Twcs[0]);
        */
        

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //std::cout << "Twcs.size(): " << Twcs.size() <<std::endl;
        for(int i=0;i<Twcs.size();i++){
            mDrawer.DrawCurrentCamera(Twcs[i]);
        }
        pangolin::FinishFrame();

        // cv::Mat im = mpFrameDrawer->DrawFrame();
        // cv::imshow("ORB-SLAM2: Current Frame",im);
        // cv::waitKey(mT);

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }
    
    SetFinish();
}

void Viewer::RequestFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    std::unique_lock<std::mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopped = false;
}
