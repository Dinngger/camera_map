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


double getDistance(cv::Point p1, cv::Point p2){
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	distance = sqrtf(distance);
	return distance;
}

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
    LightBar lb[4];
};

class CarModule
{
private:
    double module_time;
    vector<LightBarP> predict2d;
public:
    vector<Car> cars;
    int add_car(vector<Point3f> armor);
    int create_predict(double time);
    int find_light(LightBarP &lbp);
};

int CarModule::add_car(vector<Point3f> armor)
{
    Car c;
    // TODO: create a car behind the armor.
    /**let other armors be the same size as this one.
    */
    cars.push_back(c);
    return 0;
}

/**@return if or not found
 * @lbp input light bar info and output the id
*/
int CarModule::find_light(LightBarP &lbp)
{
    double DISTANCE_THRESHOLD = 50;
    if(predict2d.size()==0) return 0;
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
        return 0;
    else{
        lbp.car_id = predict2d[min_index].car_id;
        lbp.lb_id = predict2d[min_index].lb_id;
        return 1;
    }
}

#endif // __CAR_MODULE_HPP
