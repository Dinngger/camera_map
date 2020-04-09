/* 误差下降控制器
* author: Dinger
* create date: 2020.4.10 00:34
*/

#include "CarModule.hpp"

/*
struct ErrorCtl {
    int cnt;
    double error_sum;
    bool r, t, ar, at;
    int update();
};
*/

int ErrorCtl::update() {
    r = false;
    t = false;
    ar = false;
    at = false;
    if (cnt < 250)
        t = true;
    else if (cnt < 500)
        r = true;
    else if (cnt < 750)
        at = true;
    else
        ar = true;
    return 0;
}
