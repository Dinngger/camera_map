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
    float error;
    std::vector<aim_deps::Light> lightPossibles;
};

class CarMatch{
    private:
        int nCar=0;
        std::vector<aim_deps::Light> lightStack;
        std::vector<std::vector<aim_deps::Light>> divideClass;
        std::vector<cv::Point> matches;
        cv::Point2f points[4];
        std::vector<float> errors;
        void clear();
        bool constraint(int i);
        void backTrack(int t, std::vector<aim_deps::Light>& Lights);
        int findMaxCar();
        void sortLight();
        void divisionLoad();
        float getRatio(float l1, float l2);
        float getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2);

        float armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2);
        float noArmorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2);
        float oneLight();
        float twoLight(std::vector<aim_deps::Light> lightPossibles);
        float threeLight(std::vector<aim_deps::Light> lightPossibles);
        float fourLight(std::vector<aim_deps::Light> lightPossibles);
        void calError();
        void printError();
    public:
        std::vector<std::vector<CarPossible>> division;
        void runMatch(std::vector<aim_deps::Light>& Lights);
        void printCars();
        void printDivision();
};

#endif     //_CAR_MATCH_HPP
