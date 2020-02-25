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
    // TODO: nearest search
}

#endif // __CAR_MODULE_HPP
