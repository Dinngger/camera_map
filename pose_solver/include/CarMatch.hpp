#ifndef _CAR_MATCH_HPP
#define _CAR_MATCH_HPP

#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "AimDeps.hpp"

struct CarPossible {
    bool first;
    std::vector<aim_deps::Light> lightPossibles;
};

class CarMatch{
    private:
        CarPossible emptyCar;
        bool constraint(std::vector<CarPossible> lightPossibles);
    public:
        std::vector<CarPossible> tempCars, carsPossible;
        void runMatch(const std::vector<aim_deps::Light>& Lights);
};

#endif     //_CAR_MATCH_HPP
