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
    std::vector<aim_deps::Light> lightPossibles;

    bool isOneLight = false;
    int nLight;
    float sumError = 0.0;
    float armor1RatioError = 0.0, armor1DiffAngleError = 0.0, armor1HeightError = 0.0, armor1CenterAngleError = 0.0, armor1LenRatioError = 0.0; // 第一个装甲板的数据
    float armor2RatioError = 0.0, armor2DiffAngleError = 0.0, armor2HeightError = 0.0, armor2CenterAngleError = 0.0, armor2LenRatioError = 0.0;
    float noArmor1RatioError = 0.0, noArmor1HeightError = 0.0, noArmor1CenterAngleError = 0.0, noArmor1LenRatioError = 0.0;
    float noArmor2RatioError = 0.0, noArmor2HeightError = 0.0, noArmor2CenterAngleError = 0.0, noArmor2LenRatioError = 0.0;
    float solidError = 0.0;
    float betweenFourError = 0.0;
    float threeLightAngleRatio = 0.0, fourLightAngleRatio = 0.0;
    float biasError = 1000.0;
};

struct CarsPossible
{
    float carsErrorValue = 0.0;
    float betweenError = 0.0;
    int nCar = 0;
    std::vector<CarPossible> carsPossible;
};

class CarMatch
{
private:
    int nCar = 0;
    std::vector<aim_deps::Light> lightStack;
    std::vector<std::vector<aim_deps::Light>> divideClass;
    std::vector<cv::Point> matches;
    cv::Point2f points[4];

private:
    void clear();
    bool constraint(int i);
    void sortError();
    void backTrack(int t, std::vector<aim_deps::Light> &Lights);
    int findMaxCar();
    void sortLight();
    void divisionLoad();

    float getRatio(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2);
    void armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarPossible &carError, bool firstArmor, int nlgt);
    void noArmorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarPossible &carError, bool firstNoArmor, int leftSingle, int nlgt);
    float oneLight(CarPossible &carPossible);
    float twoLight(CarPossible &carPossible);
    float threeLight(CarPossible &carPossible);
    float fourLight(CarPossible &carPossible);
    void betweenFourLightError(const CarPossible &carPossible, CarPossible &e);
    void fourLightAngleRatio(const CarPossible &carPossible, CarPossible &e, int ne);
    void calError();
    void printError();
    void sortErrors();
    bool isE1more(CarPossible &e1, CarPossible &e2);
    float sumError(CarPossible &error);
    void setError(CarPossible &carPossible, const CarPossible &e, bool first);
    float betweenError(CarsPossible &carsPossible);
    bool overlap(const CarPossible &carPossible1, const CarPossible &carPossible2);
    bool isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid);
    bool isArmor(aim_deps::Light &l1, aim_deps::Light &l2);
    void getTrapezoids(const cv::Point2f corners[2], std::vector<std::vector<cv::Point2f>> &trapezoid);

public:
    std::vector<CarsPossible> division;

public:
    void runMatch(std::vector<aim_deps::Light> &Lights);
    void printCars();
    void printDivision();
};

#endif //_CAR_MATCH_HPP
