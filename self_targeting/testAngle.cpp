#include "universal/GimbalCtrl.hpp"
#include <iostream>
#include <cmath>

int main(){
    float pitch = 0.0, yaw = 0.0;
    ballistic::GimbalCtrl g_ctrl;
    g_ctrl.Init(0, 30.0, -20.0, 0.5, 0.0, 12.85);
    for(int i = -500; i<= 500; i+= 100){
        for(int j = 500; j>=-500; j-=100){                 //由于y轴是在图像上向上为负
            g_ctrl.Transform(cv::Point3f(i, j, 5000), pitch, yaw);
            std::cout<<"Result for(x, y, z):"<<(float)i/1000<<", "<<
                (float)j/1000<<", "<<5.0<<std::endl;
            std::cout<<"Pitch & Yaw:"<<pitch<<", "<<yaw<<std::endl<<std::endl;
        }
    }
    /*std::cout<<"Three times for error:"<<std::endl;
    for(int j = 500; j >=-500; j-=500){
        g_ctrl.Transform(cv::Point3f(0, j, 5000), pitch, yaw);
        std::cout<<std::endl;
    }*/
    return 0;
}