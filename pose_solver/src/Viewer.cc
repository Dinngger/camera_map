
#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <mutex>

Viewer::Viewer(std::string name, double fu, double fv, int w, int h):
    name(name),
    w(w),
    h(h),
    fu(fu),
    fv(fv),
    mbFinishRequested(false),
    mbFinished(true),
    mbStopped(true),
    mbStopRequested(false)
{
    float fps = 30;
    if(fps<1)
        fps=30;
    mT = 1e3/fps;
}

void Viewer::clear()
{
    mDrawer.clear();
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind(name ,1024*10/7, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    //pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    //pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);


    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(w,h,fu,fv,w/2,h/2,0.1,1000),
                pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0,-1,0)
                );

    pangolin::OpenGlRenderState s_cam1(
                pangolin::ProjectionMatrix(w,h,fu,fv,w/2,h/2,0.1,1000),
                pangolin::ModelViewLookAt(0,0,0, 0,0,1, 0,-1,0)
                );
    
    pangolin::OpenGlRenderState s_cam2(
                pangolin::ProjectionMatrix(w,h,fu,fv,w/2,h/2,0.1,1000),
                pangolin::ModelViewLookAt(0,5,5, 0,0,5, 0,0,1)
                );

    pangolin::OpenGlRenderState s_cam3(
                pangolin::ProjectionMatrix(w,h,fu,fv,w/2,h/2,0.1,1000),
                pangolin::ModelViewLookAt(0,-0.3,-1,0,0,1, 0,-1,0)
                );
    // Define Camera Render Object (for view / scene browsing)
    
    pangolin::View& d_cam1 = pangolin::Display("cam1")
            .SetBounds(0.5, 1.0, 0.7, 1.0, 1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam1));

    pangolin::View& d_cam2 = pangolin::Display("cam2")
            .SetBounds(0.0, 0.5, 0.7, 1.0, 1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam2));


    pangolin::View& d_img1 = pangolin::Display("image")
            .SetBounds(0, 1, 0, 0.7)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    std::vector<pangolin::OpenGlMatrix> Twcs;

    pangolin::Display("multi")
            .SetBounds(0.0, 1.0, 0.0, 1.0)
            // .SetLayout(pangolin::LayoutEqual)
            .AddDisplay(d_cam1)
            .AddDisplay(d_img1)
            .AddDisplay(d_cam2);

    pangolin::GlTexture imageTexture(1440,1080,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);

    while(1)
    {
        Twcs.clear();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        
        mDrawer.GetCurrentOpenGLCameraMatrix(Twcs);
        // mDrawer.DrawCurrentCamera(Twcs, mDrawer.mlbs);

        imageTexture.Upload(mDrawer.high_frame.data,GL_BGR,GL_UNSIGNED_BYTE);
        //display the image
        d_img1.Activate();

        glColor3f(1.0,1.0,1.0);
        imageTexture.RenderToViewportFlipY();

        d_cam1.Activate(s_cam1);
        mDrawer.DrawCurrentCamera(Twcs, mDrawer.mlbs);
        d_cam2.Activate(s_cam2);
        mDrawer.DrawCurrentCamera(Twcs, mDrawer.mlbs);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //std::cout << "Twcs.size(): " << Twcs.size() <<std::endl;

        pangolin::FinishFrame();

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

    pangolin::DestroyWindow(name);
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
