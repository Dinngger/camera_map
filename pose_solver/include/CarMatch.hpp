#ifndef _CAR_MATCH_HPP
#define _CAR_MATCH_HPP

#include <vector>
#include <stack>
#include <algorithm>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "AimDeps.hpp"

struct CarPossible {
    bool first;
    float chance;
    std::vector<aim_deps::Light> lightPossibles;
};

class CarMatch{
    private:
        int nCar=0;
        std::vector<aim_deps::Light> lightStack;
        std::vector<std::vector<aim_deps::Light>> divideClass;
        void clear();
        bool constraint(int i);
        void backTrack(int t, std::vector<aim_deps::Light>& Lights);
        int findMaxCar();
        void sortLight();
        void divisionLoad();
    public:
        std::vector<std::vector<CarPossible>> division;
        void runMatch(std::vector<aim_deps::Light>& Lights);
        void printCars();
        void printDivision();
};

#endif     //_CAR_MATCH_HPP
