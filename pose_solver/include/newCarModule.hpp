#ifndef __NEWCARCAR_MODULE_HPP
#define __NEWCARCAR_MODULE_HPP
#include "CarModule.hpp"
#include "AimDeps.hpp"
#include <queue>

struct pastFrame
{
    cv::Point2f vertex[4];
    struct pastFrame* next;
};//在头节点删除 尾节点插入 head是空的头节点
class newcar {
    private:
    Armor3d armor;
    Eigen::Vector3d t;
    Eigen::Vector3d dt;
    Eigen::Vector3d last_t;
    double r;
    double dr;
    double last_r;
    Eigen::Matrix3d K;
    public:
    int frameNumber;
    //std::vector<cv::Point2f[4]> pastFrame;
    pastFrame* frameHead;
    cv::Point2f NowPoints[4];
    int update_state();
    Eigen::Vector2d projectPoint(Eigen::Vector3d p3) const;
    newcar(Eigen::Vector3d st,double sr){
        t=st;
        r=sr;
        update_state();
        dt=Eigen::Vector3d::Zero();
        dr=0.0;
        K<<1776.67168581218, 0, 720,
             0, 1778.59375346543, 540,
             0, 0, 1;
        frameNumber=0;
        frameHead=(pastFrame*)malloc(sizeof(pastFrame));
        frameHead->next=NULL;
    }
    newcar(){   //not used
        t=Eigen::Vector3d::Zero();
        dt=Eigen::Vector3d::Zero();
        last_t=Eigen::Vector3d::Zero();
        r=0.0;
        dr=0.0;
        last_r=0.0;
        frameNumber=10;
        frameHead=(pastFrame*)malloc(sizeof(pastFrame));
        if(frameHead!=NULL)
        {
            std::cout<<"framehead success"<<std::endl;
        }
        frameHead->next=NULL;
    }
    int bundleAdjustment(double delta_time);
    int update_state(double delta_time);    // time from last update;
    int predict(double delta_time, double &pre_r, Eigen::Vector3d &pre_t) const  ;  // time to the future
    friend class newCarModule;
};

class newCarModule {
    private:
    Eigen::Matrix3d K;
    public:
    std::vector<newcar> newcars;
    double module_time;
    newCarModule(double module_time) : module_time(module_time) {
        std::cout<<"newcarModule initalized"<<std::endl;
    }
    int add_newcar(Eigen::Vector3d t,double r,std::vector<cv::Point2f> Points);
    void get_lbs(std::vector<cv::Point3d> &lbs) const;
};

#endif