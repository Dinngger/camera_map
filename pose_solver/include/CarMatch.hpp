#ifndef _CAR_MATCH_HPP
#define _CAR_MATCH_HPP

#include <vector>
#include <stack>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "AimDeps.hpp"

struct CarPossible
{
    bool first;
    float error;
    std::vector<aim_deps::Light> lightPossibles;
};

struct CarError
{
    bool isOneLight = false;
    int nLight;
    float sumError;
    float armor1RatioError = 0.0, armor1DiffAngleError = 0.0, armor1HeightError = 0.0, armor1LenRatioError = 0.0; // 第一个装甲板的数据
    float armor2RatioError = 0.0, armor2DiffAngleError = 0.0, armor2HeightError = 0.0, armor2LenRatioError = 0.0;
    float noArmor1RatioError = 0.0, noArmor1HeightError = 0.0, noArmor1LenRatioError = 0.0;
    float noArmor2RatioError = 0.0, noArmor2HeightError = 0.0, noArmor2LenRatioError = 0.0;
    float solidError = 0.0;
};

struct CarsError
{
    float carsErrorValue = 0.0;
    std::vector<CarError> carsError;
};

class CarMatch
{
private:
    int nCar = 0;
    std::vector<aim_deps::Light> lightStack;
    std::vector<std::vector<aim_deps::Light>> divideClass;
    std::vector<cv::Point> matches;
    cv::Point2f points[4];
    std::vector<CarsError> divisionError;
    cv::Mat src;

    void clear();
    bool constraint(int i);
    void backTrack(int t, std::vector<aim_deps::Light> &Lights);
    int findMaxCar();
    void sortLight();
    void divisionLoad();

    float getRatio(float l1, float l2);
    float getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2);

    void armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarError &carError, bool firstArmor);
    void noArmorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarError &carError, bool firstNoArmor, int lefthigh);
    CarError oneLight(std::vector<aim_deps::Light> lightPossibles);
    CarError twoLight(std::vector<aim_deps::Light> lightPossibles);
    CarError threeLight(std::vector<aim_deps::Light> lightPossibles);
    CarError fourLight(std::vector<aim_deps::Light> lightPossibles);
    void calError();
    void printError();
    void drawCar(const std::vector<aim_deps::Light> &lightPossibles);
    void sortErrors();
    bool isE1more(CarError &e1, CarError &e2);
    float sumError(CarError &error);

public:
    std::vector<std::vector<CarPossible>> division;
    void runMatch(std::vector<aim_deps::Light> &Lights);
    void getImage(cv::Mat img);
    void printCars();
    void printDivision();
};

#endif //_CAR_MATCH_HPP
