/* 车辆姿态估计模型
* author: Dinger
* create date: 2020.2.25 00:33
*/

#ifndef __CAR_MODULE_HPP
#define __CAR_MODULE_HPP

#include <vector>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;

// p[0] is the upper point.
struct LightBar
{
    Point3f p[2];
};

struct LightBarP
{
    int car_id, lb_id;
    Point2f center;
};

// lb[0] was the first light bar to be seen.
struct Car
{
    int color=-1;
    int number=-1; // -1 means unknow
    LightBar lb[8];
};

class CarModule
{
private:
    double module_time;
    vector<LightBarP> predict2d;
public:
    vector<Car> cars;
    int add_car(const vector<Point3f> &armor);
    int create_predict(double time);
    int find_light(LightBarP &lbp);
};

int CarModule::add_car(const vector<Point3f> &armor)
{
    Car c;

    double x1,x2,y1,y2;
    x1=armor[0].x;
    x2=armor[1].x;
    y1=armor[0].y;
    y2=armor[1].y;
    //Assume that distance(armor[0],armor[1])==long side
    double length=400;//length of car
    double len=sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));//len of armor
    double k_cos=(x2-x1)/sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    double k_sin=(y2-y1)/sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

    LightBar l7,l8;//the front lightbars
    l7.p[0]=armor[0];
    l7.p[1]=armor[3];
    l8.p[0]=armor[1];
    l8.p[1]=armor[2];
    c.lb[0]=l7;
    c.lb[1]=l8;

    LightBar l1,l2;// the two back lightbars
    l1.p[0]=armor[0]+Point3f(length*k_sin,-length*k_cos,0);
    l1.p[1]=armor[3]+Point3f(length*k_sin,-length*k_cos,0);
    l2.p[0]=armor[1]+Point3f(length*k_sin,-length*k_cos,0);
    l2.p[1]=armor[2]+Point3f(length*k_sin,-length*k_cos,0);
    c.lb[2]=l1;
    c.lb[3]=l2;

    LightBar l3,l4;//one of left or right
    Point3f center1=(armor[0]+armor[1])/2+Point3f(length/2*k_sin,-length/2*k_cos,0)+Point3f(length/2*k_cos,-length/2*k_sin,0);
    Point3f center2=(armor[2]+armor[3])/2+Point3f(length/2*k_sin,-length/2*k_cos,0)+Point3f(length/2*k_cos,-length/2*k_sin,0);
    l3.p[0]=center1+Point3f(len*k_sin,-len*k_cos,0);
    l3.p[1]=center1+Point3f(len*k_sin,-len*k_cos,0);
    l4.p[0]=center2-Point3f(len*k_sin,-len*k_cos,0);
    l4.p[1]=center2-Point3f(len*k_sin,-len*k_cos,0);
    c.lb[4]=l3;
    c.lb[5]=l4;

    LightBar l5,l6;//another left or right
    Point3f center3=(armor[0]+armor[1])/2+Point3f(length/2*k_sin,-length/2*k_cos,0)-Point3f(length/2*k_cos,-length/2*k_sin,0);
    Point3f center4=(armor[2]+armor[3])/2+Point3f(length/2*k_sin,-length/2*k_cos,0)-Point3f(length/2*k_cos,-length/2*k_sin,0);
    l5.p[0]=center3+Point3f(len*k_sin,-len*k_cos,0);
    l5.p[1]=center3+Point3f(len*k_sin,-len*k_cos,0);
    l6.p[0]=center4-Point3f(len*k_sin,-len*k_cos,0);
    l6.p[1]=center4-Point3f(len*k_sin,-len*k_cos,0);
    c.lb[6]=l5;
    c.lb[7]=l6;

    cars.push_back(c);
    return 0;
}

double getDistance(cv::Point p1, cv::Point p2){
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	distance = sqrtf(distance);
	return distance;
}

/**@return if or not found
 * @lbp input light bar info and output the id
*/
int CarModule::find_light(LightBarP &lbp)
{
    double DISTANCE_THRESHOLD = 50;
    if (predict2d.size()==0)
        return -1;
    double min_distance = 100000, distance;
    int min_index = -1;
    for(int i=0;i<predict2d.size();i++){
        distance = getDistance(predict2d[i].center, lbp.center);
        if(distance < min_distance){
            min_distance = distance;
            min_index = i;
        }
    }
    if(min_distance > DISTANCE_THRESHOLD) 
        return -1;
    else{
        lbp.car_id = predict2d[min_index].car_id;
        lbp.lb_id = predict2d[min_index].lb_id;
        return 0;
    }
}

#endif // __CAR_MODULE_HPP
