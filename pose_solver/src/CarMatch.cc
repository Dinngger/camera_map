#define DEBUG0
// #define DEBUG1
// #define DEBUG2

#include "../include/CarMatch.hpp"
#include "PoseSolver.hpp"

float getPointDist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void CarMatch::clear()
{
    lightStack.clear();
    division.clear();
}

void CarMatch::runMatch(std::vector<aim_deps::Light> &Lights, cv::Mat src, int count, std::vector<aim_deps::Armor> tar_list)
{
    frameCount = count;
    armorList = tar_list;
    std::cout << "Lights.size: " << Lights.size() << std::endl;
    clear();
    sortLights(Lights);
    // printLightInfo(Lights);
    // drawLights(Lights, src);
    backTrack(0, Lights);
    calError();
    sortErrors();
    std::cout << "division.size: " << division.size() << std::endl;
    int cnt = 0;
    for (const CarsPossible &cars : division)
    {
#ifdef DEBUG0
        if (cnt > 0)
            break;
        std::cout << "cnt=" << cnt << std::endl;
        printCarsError(cars);
        // drawCars(cars, src);
        cnt++;
#endif
#ifdef DEBUG1
        std::cout << "cnt=" << cnt << std::endl;
        printCarsError(cars);
        drawCars(cars, src);
        cnt++;
#endif //DEBUG1
#ifdef DEBUG2
        for (size_t i = 0; i < cars.carsPossible.size(); i++)
        {
            if (cars.carsPossible[i].lightPossibles.size() == 4 &&
                // cars.carsPossible[i].lightPossibles[0].box.center.x > 1078 &&
                // cars.carsPossible[i].lightPossibles[0].box.center.x < 1079 &&
                // cars.carsPossible[i].lightPossibles[1].box.center.x > 1052 &&
                // cars.carsPossible[i].lightPossibles[1].box.center.x < 1053 &&
                // cars.carsPossible[i].lightPossibles[2].box.center.x > 1094 &&
                // cars.carsPossible[i].lightPossibles[2].box.center.x < 1095 &&
                cars.carsPossible[i].lightPossibles[3].box.center.x > 1272 &&
                cars.carsPossible[i].lightPossibles[3].box.center.x < 1273)
            {
                std::cout << "cnt=" << cnt << std::endl;
                printCarsError(cars);
                drawCars(cars, src);
                cnt++;
            }
        }
#endif //DEBUG2
    }
}

bool compareX(aim_deps::Light l1, aim_deps::Light l2)
{
    // if (l1.carNum == l2.carNum)
    return l1.box.center.x < l2.box.center.x;
    // else
    //     return l1.carNum < l2.carNum;
}

void CarMatch::sortLights(std::vector<aim_deps::Light> &Lights)
{
    std::sort(Lights.begin(), Lights.end(), compareX);
    for (size_t i = 0; i < Lights.size(); i++)
    {
        Lights[i].carMatchIndex = i;
    }
}

void CarMatch::drawLights(const std::vector<aim_deps::Light> &Lights, cv::Mat &src)
{
    char str[2];
    for (size_t i = 0; i < Lights.size(); i++)
    {
        snprintf(str, 2, "%lu", i);
        cv::putText(src, str, Lights[i].box.vex[0] + cv::Point2f(1, 1),
                    cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
        cv::circle(src, Lights[i].box.vex[0], 0, cv::Scalar(255, 0, 255), -1);
        cv::circle(src, Lights[i].box.vex[1], 0, cv::Scalar(255, 0, 255), -1);
        match_debug(rmlog::F_GREEN, "Light ", Lights[i].index, " with lenth: ", Lights[i].box.length,
                    ", angle: ", Lights[i].box.angle);
    }
}
void CarMatch::printLightInfo(const std::vector<aim_deps::Light> &Lights)
{
    for (size_t i = 0; i < Lights.size(); i++)
    {
        std::cout << "index=" << i << ", angle=" << Lights[i].box.angle << ", len=" << Lights[i].box.length << ", center=" << Lights[i].box.center << ", upvex=" << Lights[i].box.vex[0] << ", downvex=" << Lights[i].box.vex[1] << std::endl;
    }
    for (size_t i = 0; i < Lights.size(); i++)
    {
        for (size_t j = i + 1; j < Lights.size(); j++)
        {
            float dist = aim_deps::getPointDist(Lights[i].box.center, Lights[j].box.center);
            // float len = std::max(Lights[i].box.length, Lights[j].box.length);
            float len = (Lights[i].box.length + Lights[j].box.length) / 2;
            float diffH = fabs(Lights[i].box.center.y - Lights[j].box.center.y);
            cv::Point2f vertical = (Lights[i].box.center - Lights[j].box.center);
            float verticalAngle = fabs(atan(vertical.y / vertical.x)) / 3.1415926 * 180;
            float angle = (Lights[i].box.angle - Lights[j].box.angle);
            float ratio = dist / len / len;
            float ratioH = diffH / len;
            std::cout << "i=" << i << ", j=" << j << ", ratio=" << ratio << ", ratioH=" << ratioH << ", angle=" << angle << ", verticalAngle=" << verticalAngle << std::endl;
        }
    }
}

bool compareCarsErrorValue(CarsPossible e1, CarsPossible e2)
{
    return e1.carsErrorValue < e2.carsErrorValue;
}

void CarMatch::sortErrors()
{
    std::sort(division.begin(), division.end(), compareCarsErrorValue);
}

void CarMatch::drawCars(const CarsPossible &cars, cv::Mat &src)
{
    cv::Mat src2 = src.clone();
    for (size_t i = 0; i < cars.carsPossible.size(); i++)
    {
        if (cars.carsPossible[i].lightPossibles.size() == 1)
            continue;
        for (const aim_deps::Light &light : cars.carsPossible[i].lightPossibles)
        {
            cv::line(src2, light.box.vex[0], light.box.vex[1], colors[i % 4], 3);
            // if (carPossibles.back()<500 && lightPossibles.size() == 4) cv::waitKey(0);
            // else cv::waitKey(1);
        }
    }
    cv::imshow("carPossible", src2);
    cv::waitKey(0);
}

void CarMatch::calError()
{
    int likeFourFlag = 0;
    float carError = 0.0;
    for (CarsPossible &cars : division)
    {
        // std::cout<<"division "<<i<<std::endl;
        for (CarPossible &car : cars.carsPossible)
        {
            switch (car.lightPossibles.size())
            {
            case 1:
                if (car.likeFourLight)
                {
                    car.likeFourLight = false;
                }
                carError = oneLight(car);
                break;
            case 2:
                if (car.likeFourLight)
                {
                    car.likeFourLight = false;
                }
                carError = twoLight(car);
                break;
            case 3:
                if (car.likeFourLight)
                {
                    car.likeFourLight = false;
                }
                carError = threeLight(car);
                break;
            case 4:
                if (car.likeFourLight)
                {
                    likeFourFlag++;
                }
                carError = fourLight(car);
                break;
            default:
                std::cout << "=================================" << car.lightPossibles.size() << "========================\n";
            }
            cars.carsErrorValue += carError;
        }
        cars.betweenError = betweenError(cars);
        cars.carsErrorValue += cars.betweenError;
        for (int i = 0; i < likeFourFlag; i++)
        {
            cars.carsErrorValue *= 0.1;
        }
        likeFourFlag = 0;
        cars.nCar = cars.carsPossible.size();
    }
}

bool CarMatch::overlap(std::vector<aim_deps::Light> ls1, std::vector<aim_deps::Light> ls2)
{
    // bool isOverlap = false;
    std::vector<cv::Point2f> polygon;
    for (size_t i = 0; i < ls1.size(); i++)
    {
        for (size_t j = i + 1; j < ls1.size(); j++)
        {
            if (isArmor(ls1[i], ls1[j]))
            {
                polygon.push_back(ls1[i].box.vex[0]);
                polygon.push_back(ls1[i].box.vex[1]);
                polygon.push_back(ls1[j].box.vex[1]);
                polygon.push_back(ls1[j].box.vex[0]);
                for (const aim_deps::Light &l2 : ls2)
                    if (cv::pointPolygonTest(polygon, l2.box.center, false) > 0)
                    {
                        polygon.clear();
                        return true;
                    }
                polygon.clear();
            }
        }
    }
    for (size_t i = 0; i < ls2.size(); i++)
    {
        for (size_t j = i + 1; j < ls2.size(); j++)
        {
            if (isArmor(ls2[i], ls2[j]))
            {
                polygon.push_back(ls2[i].box.vex[0]);
                polygon.push_back(ls2[i].box.vex[1]);
                polygon.push_back(ls2[j].box.vex[1]);
                polygon.push_back(ls2[j].box.vex[0]);
                for (const aim_deps::Light &l1 : ls1)
                    if (cv::pointPolygonTest(polygon, l1.box.center, false) > 0)
                    {
                        polygon.clear();
                        return true;
                    }
                polygon.clear();
            }
        }
    }
    return false;
    // float car1MeanHeight = 0.0, car1MeanLen = 0.0, car2MeanHeight = 0.0, car2MeanLen = 0.0;
    // for (const aim_deps::Light &light : carPossible1.lightPossibles)
    // {
    //     car1MeanHeight += light.box.center.y;
    //     car1MeanLen += light.box.length;
    // }
    // car1MeanHeight /= carPossible1.lightPossibles.size();
    // car1MeanLen /= carPossible1.lightPossibles.size();
    // for (const aim_deps::Light &light : carPossible2.lightPossibles)
    // {
    //     car2MeanHeight += light.box.center.y;
    //     car2MeanLen += light.box.length;
    // }
    // car2MeanHeight /= carPossible2.lightPossibles.size();
    // car2MeanLen /= carPossible2.lightPossibles.size();

    // float car1Left, car1Right, car2Left, car2Right;
    // if (ls1.size() == 1)
    // {
    //     car1Left = std::min(ls1[0].box.vex[0].x, ls1[0].box.vex[1].x);
    //     car1Right = std::max(ls1.back().box.vex[0].x, ls1.back().box.vex[1].x);
    //     car2Left = ls2[1].box.center.x;
    //     car2Right = ls2.end()[-2].box.center.x;
    // }
    // else if (ls2.size() == 1)
    // {
    //     car1Left = ls1[1].box.center.x;
    //     car1Right = ls1.end()[-2].box.center.x;
    //     car2Left = std::min(ls2[0].box.vex[0].x, ls2[0].box.vex[1].x);
    //     car2Right = std::max(ls2.back().box.vex[0].x, ls2.back().box.vex[1].x);
    // }
    // else
    // {
    //     car1Left = ls1[0].box.center.x;
    //     car1Right = ls1.back().box.center.x;
    //     car2Left = ls2[0].box.center.x;
    //     car2Right = ls2.back().box.center.x;
    // }
    // // std::cout<<"car1Left="<<car1Left<<", car1Right="<<car1Right<<", car2Left="<<car2Left<<", car2Right="<<car2Right<<std::endl;
    // if (ls1.size() == 1 || ls2.size() == 1)
    // {
    //     // float diffH = fabs(car1MeanHeight - car2MeanHeight) / std::min(car1MeanLen, car2MeanLen);
    //     if (isOverlap)
    //     {
    //         if ((car1Left < car2Right && car1Left > car2Left && car1Right > car2Right) || (car1Right > car2Left && car1Left < car2Left && car1Right < car2Right))
    //         {
    //             return true;
    //         }
    //         if ((car1Left < car2Left && car1Right > car2Right) || (car2Left < car1Left && car2Right > car1Right))
    //         {
    //             return true;
    //         }
    //     }
    // }
    // else if (isOverlap)
    // {
    //     if ((car1Left < car2Right && car1Left > car2Left && car1Right > car2Right) || (car1Right > car2Left && car1Left < car2Left && car1Right < car2Right))
    //     {
    //         return true;
    //     }
    //     if ((car1Left < car2Left && car1Right > car2Right) || (car2Left < car1Left && car2Right > car1Right))
    //     {
    //         return true;
    //     }
    // }
    // return false;
}

bool CarMatch::isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid)
{
    for (int i = 0; i < 2; ++i)
    {
        if (cv::pointPolygonTest(trapezoid, corners[i], false) < 0)
            return false;
    }
    /// 2个点都找到，才能返回true
    return true;
}

//匹配的灯条其梯形将会互相包含
void CarMatch::getTrapezoids(const cv::Point2f corners[2], std::vector<std::vector<cv::Point2f>> &trapezoids)
{
    std::vector<cv::Point2f> left_trapezoid;
    std::vector<cv::Point2f> right_trapezoid;
    cv::Point2f midpoint = (corners[0] + corners[1]) / 2;
    cv::Point2f direction_vector; //方向向量
    float d = sqrt(aim_deps::getPointDist(corners[0], corners[1]));
    cv::Point2f vertical_vector = cv::Point2f(corners[1].y - corners[0].y,
                                              corners[0].x - corners[1].x); //获得垂直长方向的方向向量
    vertical_vector = d * aim_deps::LIGHT_PARAM1 * vertical_vector /
                      sqrt(vertical_vector.x * vertical_vector.x + vertical_vector.y * vertical_vector.y);
    direction_vector = corners[1] - corners[0]; //平行于长边方向的方向向量
    left_trapezoid.emplace_back(corners[0]);
    left_trapezoid.emplace_back(corners[1]);
    left_trapezoid.emplace_back(midpoint - vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vector);
    left_trapezoid.emplace_back(midpoint - vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vector);
    right_trapezoid.emplace_back(corners[0]);
    right_trapezoid.emplace_back(corners[1]);
    right_trapezoid.emplace_back(midpoint + vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vector);
    right_trapezoid.emplace_back(midpoint + vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vector);
    trapezoids.emplace_back(left_trapezoid); //灯条左右两边将会拓展出两个梯形
    trapezoids.emplace_back(right_trapezoid);
}

bool CarMatch::isArmor(aim_deps::Light &l1, aim_deps::Light &l2)
{
    std::vector<std::vector<cv::Point2f>> trapezoids1;
    std::vector<std::vector<cv::Point2f>> trapezoids2;
    getTrapezoids(l1.box.vex, trapezoids1);
    getTrapezoids(l2.box.vex, trapezoids2);
    if ((isInTrapezoid(l1.box.vex, trapezoids2[0]) && isInTrapezoid(l2.box.vex, trapezoids1[1])) ||
        (isInTrapezoid(l2.box.vex, trapezoids1[0]) && isInTrapezoid(l1.box.vex, trapezoids2[1])))
    {
        return true;
    }
    return false;
}

float CarMatch::betweenError(CarsPossible &carsPossible)
{
    for (size_t i = 0; i < carsPossible.carsPossible.size(); i++)
    {
        for (size_t j = i + 1; j < carsPossible.carsPossible.size(); j++)
        {
            // if (carsPossible.carsPossible[i].lightPossibles.size() == 1 && carsPossible.carsPossible[j].lightPossibles.size() == 1)
            // {
            //     if (isArmor(carsPossible.carsPossible[i].lightPossibles[0], carsPossible.carsPossible[j].lightPossibles[0]))
            //         return 150000000;
            // }
            if (overlap(carsPossible.carsPossible[i].lightPossibles, carsPossible.carsPossible[j].lightPossibles))
            {
                return 140000000;
            }
        }
    }
    return 0;
}

float CarMatch::getRatio(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2)
{ //对边中点连线的长度平方比值是否合适
    float len1 = aim_deps::getPointDist(b1.center, b2.center),
          len2 = (b1.length + b2.length) / 2,
          ratio = len1 / (len2 * len2);
    return ratio;
}

bool CarMatch::isTarArmor(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2)
{
    float dist;
    for (aim_deps::Armor armor : armorList)
    {
        if (armor.armor_number != -1 && armor.valid)
        {
            dist = 0;
            dist += getPointDist(b1.vex[0], armor.left_light.box.vex[0]);
            dist += getPointDist(b1.vex[1], armor.left_light.box.vex[1]);
            dist += getPointDist(b2.vex[0], armor.right_light.box.vex[0]);
            dist += getPointDist(b2.vex[1], armor.right_light.box.vex[1]);
            if (dist < 1)
                return true;
        }
    }
    return false;
}

void CarMatch::armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarPossible &carPossible, bool firstArmor, int nlgt)
{
    float diffAngleError, heightError, lenRatioError, ratioError, centerAngleError;
    bool isArmor = false;
    if (isTarArmor(b1, b2))
    {
        diffAngleError = 0;
        heightError = 0;
        lenRatioError = 0;
        ratioError = 0;
        centerAngleError = 0;
        isArmor = true;
    }
    else
    {
        diffAngleError = fabs(b1.angle - b2.angle);
        diffAngleError = diffAngleError * diffAngleError / 5;
        cv::Point2f vertical = (b1.center - b2.center);
        float angle = fabs(atan(vertical.y / vertical.x)) / 3.1415926 * 180;
        if (angle > 35)
            centerAngleError = 120000000;
        else
            centerAngleError = 0;
        float diffH = fabs(b1.center.y - b2.center.y);
        heightError = diffH * diffH * 10;
        float meanLen = (b1.length + b2.length) / 2;
        if (diffH / meanLen > 1.5)
            heightError = 110000000;
        lenRatioError = (b1.length < b2.length ? b2.length / b1.length : b1.length / b2.length);
        lenRatioError = lenRatioError * lenRatioError * 10;
        // if (lenRatioError > 2.2)
        // {
        //     lenRatioError = lenRatioError * lenRatioError * lenRatioError * 100;
        // }
        // else
        // {
        //     lenRatioError = lenRatioError * lenRatioError * 10;
        // }
        // lenRatioError *= lenRatioError;
        float ratio = getRatio(b1, b2);
        ratioError = 0.0;
        if (ratio > 30)
            ratioError = 180000000;
        // if (ratio < 19.36 && ratio > 14.44)
        //     ratioError = fabs(ratio - 0.0);
        // else
        // ratioError = (ratio - 3.0) * (ratio - 3.0) * 0;
        if (nlgt == 2)
        {
            diffAngleError *= 0.5;
            heightError *= 0.5;
            lenRatioError *= 0.5;
            ratioError *= 0.5;
            centerAngleError *= 0.5;
        }
    }
    if (firstArmor)
    {
        carPossible.armor1DiffAngleError = diffAngleError;
        carPossible.armor1HeightError = heightError;
        carPossible.armor1LenRatioError = lenRatioError;
        carPossible.armor1RatioError = ratioError;
        carPossible.armor1CenterAngleError = centerAngleError;
        carPossible.isTarArmor1 = isArmor;
    }
    else
    {
        carPossible.armor2DiffAngleError = diffAngleError;
        carPossible.armor2HeightError = heightError;
        carPossible.armor2LenRatioError = lenRatioError;
        carPossible.armor2RatioError = ratioError;
        carPossible.armor2CenterAngleError = centerAngleError;
        carPossible.isTarArmor2 = isArmor;
    }

    // std::cout<<"armorError: "<<"ratioError="<<ratioError<<", diffAngleError="<<diffAngleError<<", heightError="<<heightError<<", lenRatioError="<<lenRatioError<<std::endl;
}

void CarMatch::noArmorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarPossible &carPossible, bool firstNoArmor, int leftSingle, int nlgt)
{
    float lenRatioError = (b1.length < b2.length ? b2.length / b1.length : b1.length / b2.length);
    // if (lenRatioError < 1.5)
    lenRatioError = lenRatioError * lenRatioError * 10;
    // else if (lenRatioError >= 1.5)
    // {
    //     lenRatioError = lenRatioError * lenRatioError * lenRatioError * 1000;
    // }
    // else
    // {
    //     lenRatioError = lenRatioError * lenRatioError * 10;
    // }
    cv::Point2f vertical = (b1.center - b2.center);
    float centerAngleError;
    float angle = fabs(atan(vertical.y / vertical.x)) / 3.1415926 * 180;
    if (angle > 71)
        centerAngleError = 130000000;
    else
        centerAngleError = 0;

    float diffH = fabs(b1.center.y - b2.center.y);
    float heightError = diffH;
    float meanLen = (b1.length + b2.length) / 2;
    if (diffH / meanLen > 1.5)
        heightError = 170000000;
    if (heightError != 170000000)
    {
        if (nlgt >= 3)
        {

            switch (leftSingle)
            {
            case -1:
                break;
            case 0:
                if ((b2.vex[1].y + 20 > b1.vex[0].y && b2.vex[1].y < b1.vex[1].y) ||
                    (b2.vex[0].y < b1.vex[1].y && b2.vex[0].y > b1.vex[0].y))
                    heightError = 0.0;
                else
                    heightError *= 10;
                if (b1.angle - b2.angle < -12)
                    carPossible.threeLightAngleRatio = 0.51;
                else if (b1.angle - b2.angle > 10)
                    carPossible.threeLightAngleRatio = 100;
                break;
            case 1:
                if ((b1.vex[1].y + 20 > b2.vex[0].y && b1.vex[1].y < b2.vex[1].y) ||
                    (b1.vex[0].y < b2.vex[1].y && b1.vex[0].y > b2.vex[0].y))
                    heightError = 0.0;
                else
                    heightError *= 10;
                if (b1.angle - b2.angle > 10)
                    carPossible.threeLightAngleRatio = 101;
                else if (b1.angle - b2.angle < -12)
                    carPossible.threeLightAngleRatio = 0.52;
                break;
            }
        }
    }

    float ratio = getRatio(b1, b2);
    float ratioError = 0.0;
    if (ratio > 54)
    {
        ratioError = 190000000;
    }
    else if (ratio > 30 && ratio <= 50)
    {
        ratioError = ratio * ratio * 10;
    }

    if (nlgt == 2)
    {
        heightError *= 2;
        lenRatioError *= 2;
        ratioError *= 2;
        centerAngleError *= 2;
    }

    if (firstNoArmor)
    {
        carPossible.noArmor1HeightError = heightError;
        carPossible.noArmor1LenRatioError = lenRatioError;
        carPossible.noArmor1RatioError = ratioError;
        carPossible.noArmor1CenterAngleError = centerAngleError;
    }
    else
    {
        carPossible.noArmor2HeightError = heightError;
        carPossible.noArmor2LenRatioError = lenRatioError;
        carPossible.noArmor2RatioError = ratioError;
        carPossible.noArmor2CenterAngleError = centerAngleError;
    }
    // std::cout<<"noArmorError: "<<"ratioError="<<ratioError<<", heightError="<<heightError<<", lenRatioError="<<lenRatioError<<std::endl;
}

float CarMatch::sumError(CarPossible &error)
{
    float errorValue = 0.0;
    errorValue += error.armor1DiffAngleError;
    errorValue += error.armor1HeightError;
    errorValue += error.armor1LenRatioError;
    errorValue += error.armor1CenterAngleError;
    errorValue += error.armor1RatioError;
    errorValue += error.armor2DiffAngleError;
    errorValue += error.armor2CenterAngleError;
    errorValue += error.armor2HeightError;
    errorValue += error.armor2LenRatioError;
    errorValue += error.armor2RatioError;
    errorValue += error.noArmor1CenterAngleError;
    errorValue += error.noArmor1HeightError;
    errorValue += error.noArmor1LenRatioError;
    errorValue += error.noArmor1RatioError;
    errorValue += error.noArmor2CenterAngleError;
    errorValue += error.noArmor2HeightError;
    errorValue += error.noArmor2LenRatioError;
    errorValue += error.noArmor2RatioError;
    errorValue += error.solidError;
    errorValue += error.betweenFourError;
    errorValue += error.biasError;
    if (error.fourLightAngleRatio != 0)
        errorValue *= error.fourLightAngleRatio;
    if (error.threeLightAngleRatio != 0)
        errorValue *= error.threeLightAngleRatio;
    if (error.nLight == 2)
        errorValue = errorValue / 8;
    if (error.nLight == 3)
        errorValue = errorValue / 20;
    if (error.nLight == 4)
        errorValue = errorValue / 50;
    if (error.isTarArmor1)
        errorValue = errorValue / 10;
    if (error.isTarArmor2)
        errorValue = errorValue / 10;
    error.sumError = errorValue;
    return errorValue;
}

bool CarMatch::isE1more(CarPossible &e1, CarPossible &e2)
{
    float error1, error2;
    error1 = sumError(e1);
    error2 = sumError(e2);
    return error1 > error2 ? true : false;
}

void CarMatch::setError(CarPossible &carPossible, const CarPossible &e, bool first)
{
    carPossible.armor1DiffAngleError = e.armor1DiffAngleError;
    carPossible.armor1CenterAngleError = e.armor1CenterAngleError;
    carPossible.armor1HeightError = e.armor1HeightError;
    carPossible.armor1LenRatioError = e.armor1LenRatioError;
    carPossible.armor1RatioError = e.armor1RatioError;
    carPossible.armor2DiffAngleError = e.armor2DiffAngleError;
    carPossible.armor2CenterAngleError = e.armor2CenterAngleError;
    carPossible.armor2HeightError = e.armor2HeightError;
    carPossible.armor2LenRatioError = e.armor2LenRatioError;
    carPossible.armor2RatioError = e.armor2RatioError;
    carPossible.noArmor1CenterAngleError = e.noArmor1CenterAngleError;
    carPossible.noArmor1HeightError = e.noArmor1HeightError;
    carPossible.noArmor1LenRatioError = e.noArmor1LenRatioError;
    carPossible.noArmor1RatioError = e.noArmor1RatioError;
    carPossible.noArmor2CenterAngleError = e.noArmor2CenterAngleError;
    carPossible.noArmor2HeightError = e.noArmor2HeightError;
    carPossible.noArmor2LenRatioError = e.noArmor2LenRatioError;
    carPossible.noArmor2RatioError = e.noArmor2RatioError;
    carPossible.fourLightAngleRatio = e.fourLightAngleRatio;
    carPossible.threeLightAngleRatio = e.threeLightAngleRatio;
    carPossible.betweenFourError = e.betweenFourError;
    carPossible.biasError = e.biasError;
    carPossible.nLight = e.nLight;
    carPossible.isTarArmor1 = e.isTarArmor1;
    carPossible.isTarArmor2 = e.isTarArmor2;
    carPossible.sumError = e.sumError;
    carPossible.first = first;
}

float CarMatch::oneLight(CarPossible &carPossible)
{
    carPossible.nLight = 1;
    carPossible.isOneLight = true;
    carPossible.solidError = carPossible.lightPossibles[0].box.length * carPossible.lightPossibles[0].box.length;
    carPossible.sumError = carPossible.solidError;
    return carPossible.sumError;
}

float CarMatch::twoLight(CarPossible &carPossible)
{
    CarPossible e1, e2;
    e1.nLight = 2;
    e2.nLight = 2;
    armorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e1, true, 2);
    noArmorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e2, true, -1, 2);
    if (isE1more(e1, e2))
    {
        setError(carPossible, e2, false);
    }
    else
    {
        setError(carPossible, e1, true);
    }
    return carPossible.sumError;
}

float CarMatch::threeLight(CarPossible &carPossible)
{
    CarPossible e1, e2;
    e1.nLight = 3;
    e2.nLight = 3;
    armorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e1, true, 3);
    noArmorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e1, true, 0, 3);
    noArmorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e2, true, 1, 3);
    armorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e2, true, 3);
    if (isE1more(e1, e2))
    {
        setError(carPossible, e2, false);
    }
    else
    {
        setError(carPossible, e1, true);
    }
    return carPossible.sumError;
}

float CarMatch::fourLight(CarPossible &carPossible)
{
    CarPossible e1, e2;
    e1.nLight = 4;
    e2.nLight = 4;
    armorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e1, true, 4);
    noArmorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e1, true, -1, 4);
    armorError(carPossible.lightPossibles[2].box, carPossible.lightPossibles[3].box, e1, false, 4);
    noArmorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e2, true, 1, 4);
    armorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e2, true, 4);
    noArmorError(carPossible.lightPossibles[2].box, carPossible.lightPossibles[3].box, e2, false, 0, 4);
    betweenFourLightError(carPossible, e1);
    betweenFourLightError(carPossible, e2);
    fourLightAngleRatio(carPossible, e1, 1);
    fourLightAngleRatio(carPossible, e2, 2);
    if (isE1more(e1, e2))
    {
        setError(carPossible, e2, false);
    }
    else
    {
        setError(carPossible, e1, true);
    }
    return carPossible.sumError;
}

void CarMatch::fourLightAngleRatio(const CarPossible &carPossible, CarPossible &e, int ne)
{
    // for (size_t i = 0; i < carPossible.lightPossibles.size(); i++)
    // {
    //     for (size_t j = i + 1; j < carPossible.lightPossibles.size(); j++)
    //     {
    //         if ((carPossible.lightPossibles[i].box.center.y - carPossible.lightPossibles[j].box.center.y) > 40)
    //         {
    //             e.fourLightAngleRatio = 105;
    //             return;
    //         }
    //     }
    // }

    if (ne == 1)
    {
        float diffH1 = fabs(carPossible.lightPossibles[0].box.center.y - carPossible.lightPossibles[1].box.center.y);
        float diffH2 = fabs(carPossible.lightPossibles[2].box.center.y - carPossible.lightPossibles[3].box.center.y);
        float meanLen1 = (carPossible.lightPossibles[0].box.length + carPossible.lightPossibles[1].box.length) / 2;
        float meanLen2 = (carPossible.lightPossibles[2].box.length + carPossible.lightPossibles[3].box.length) / 2;
        float len0 = carPossible.lightPossibles[0].box.length,
              len1 = carPossible.lightPossibles[1].box.length,
              len2 = carPossible.lightPossibles[2].box.length,
              len3 = carPossible.lightPossibles[3].box.length;
        float lenRatio1 = len0 < len1 ? len1 / len0 : len0 / len1;
        float lenRatio2 = len2 < len3 ? len3 / len2 : len2 / len3;
        if (lenRatio1 > 2.3 || lenRatio2 > 2.3)
        {
            e.fourLightAngleRatio = 107;
            return;
        }
        if (diffH1 / meanLen1 > 2 || diffH2 / meanLen2 > 2)
        {
            e.fourLightAngleRatio = 106;
            return;
        }
        float diffAngle1 = fabs(carPossible.lightPossibles[0].box.angle - carPossible.lightPossibles[1].box.angle);
        float diffAngle2 = fabs(carPossible.lightPossibles[2].box.angle - carPossible.lightPossibles[3].box.angle);
        if ((diffAngle1 > 15 || diffAngle2 > 15) || (diffAngle1 > 10 && diffAngle2 > 10))
        {
            e.fourLightAngleRatio = 104;
            return;
        }
        float diffAngle = carPossible.lightPossibles[1].box.angle - carPossible.lightPossibles[2].box.angle;
        if (diffAngle < -10)
        {
            e.fourLightAngleRatio = 0.1;
            return;
        }
        else if (diffAngle > 10)
        {
            e.fourLightAngleRatio = 102;
            return;
        }
    }
    else
    {
        float diffArmorAngle = fabs(carPossible.lightPossibles[1].box.angle - carPossible.lightPossibles[2].box.angle);
        if (diffArmorAngle > 10)
        {
            e.fourLightAngleRatio = 108;
            return;
        }
        float diffAngle1 = carPossible.lightPossibles[0].box.angle - carPossible.lightPossibles[1].box.angle;
        float diffAngle2 = carPossible.lightPossibles[2].box.angle - carPossible.lightPossibles[3].box.angle;
        if (diffAngle1 > 10 || diffAngle2 > 10)
        {
            e.fourLightAngleRatio = 103;
            return;
        }
        // else if(diffAngle1 >2 || diffAngle2>2){
        //     e.fourLightAngleRatio = 9;
        //     return;
        // }
        else if (diffAngle1 < -12 || diffAngle2 < -12)
        {
            e.fourLightAngleRatio = 0.12;
            return;
        }
    }
}

void CarMatch::betweenFourLightError(const CarPossible &carPossible, CarPossible &e)
{
    float diffH1 = carPossible.lightPossibles[0].box.center.y - carPossible.lightPossibles[1].box.center.y;
    float diffH2 = carPossible.lightPossibles[3].box.center.y - carPossible.lightPossibles[2].box.center.y;
    float meanLen1 = (carPossible.lightPossibles[0].box.length + carPossible.lightPossibles[1].box.length) / 2;
    float meanLen2 = (carPossible.lightPossibles[2].box.length + carPossible.lightPossibles[3].box.length) / 2;
    if (fabs(diffH1 / meanLen1) > 0.8 && fabs(diffH2 / meanLen2) > 0.8)
        if (diffH1 * diffH2 < 0)
            e.betweenFourError = 160000000;
}

int CarMatch::findMaxCarNum()
{
    for (size_t j = 0; j < mapCarLights.size(); j++)
        if (mapCarLights[j].lightPossibles.size() == 0)
            return j - 1;
    return mapCarLights.size() - 1;
}

void CarMatch::divisionPush(std::map<int, CarPossible> &mapCarLights)
{
    int nCar = 0;
    std::map<int, CarPossible>::iterator mapCarLight;
    for (mapCarLight = mapCarLights.begin(); mapCarLight != mapCarLights.end(); mapCarLight++)
    {
        if (mapCarLight->second.lightPossibles.size() > 0)
        {
            tempCarPossible = mapCarLight->second;
            tempCarsPossible.carsPossible.push_back(tempCarPossible);
            nCar++;
            // if (mapCarLight->second.likeFourLight)
            //     mapCarLight->second.likeFourLight = false;
        }
    }
    tempCarsPossible.nCar = nCar;
    division.push_back(tempCarsPossible);
    tempCarsPossible.carsPossible.clear();
}

bool CarMatch::checkTwoLightError(int i)
{
    CarPossible tempCar;
    tempCar = mapCarLights[i];
    float twoError = twoLight(tempCar);
    if (twoError > 10000)
    {
        // std::cout << "****************index=";
        // for (size_t j = 0; j < tempCar.lightPossibles.size(); j++)
        // {
        //     std::cout << tempCar.lightPossibles[j].index << ", ";
        // }
        // std::cout << "\n";
        // printCarError(tempCar);
        return true;
    }
    return false;
}

bool CarMatch::checkThreeLightError(int i)
{
    CarPossible tempCar;
    tempCar = mapCarLights[i];
    float threeError = threeLight(tempCar);
    if (threeError > 10000)
    {
        // std::cout << "****************index=";
        // for (size_t j = 0; j < tempCar.lightPossibles.size(); j++)
        // {
        //     std::cout << tempCar.lightPossibles[j].index << ", ";
        // }
        // std::cout << "\n";
        // printCarError(tempCar);
        return true;
    }
    return false;
}

bool CarMatch::checkFourLightError(int i)
{
    CarPossible tempCar;
    tempCar = mapCarLights[i];
    float fourError = fourLight(tempCar);
    if (fourError > 10000)
    {
        // std::cout << "****************index=";
        // for (size_t j = 0; j < tempCar.lightPossibles.size(); j++)
        // {
        //     std::cout << tempCar.lightPossibles[j].index << ", ";
        // }
        // std::cout << "\n";
        // printCarError(tempCar);
        return true;
    }
    else if (fourError < 5)
    {
        mapCarLights[i].likeFourLight = true;
    }
    return false;
}

void CarMatch::backTrack(int t, std::vector<aim_deps::Light> &Lights)
{
    // if (t > 10)
    //     return;
    if (lightStack.size() == Lights.size())
    {
        divisionPush(mapCarLights);
        // std::cout<<"========push=========\n";
    }
    else
    {
        for (int i = 0; i <= nCar; i++)
        {
            if (fabs(Lights[t].box.angle) > 50)
                return;
            if (mapCarLights[i].lightPossibles.size() == 4)
            {
                continue;
            }
            if (mapCarLights[i].likeFourLight)
            {
                mapCarLights[i].likeFourLight = false;
            }
            Lights[t].carNum = i;
            lightStack.push_back(Lights[t]);
            mapCarLights[i].lightPossibles.push_back(Lights[t]);
            switch (mapCarLights[i].lightPossibles.size())
            {
            case 1:
            {
                break;
            }
            case 2:
            {
                if (checkTwoLightError(i))
                {
                    lightStack.pop_back();
                    mapCarLights[i].lightPossibles.pop_back();
                    continue;
                }
                break;
            }
            case 3:
            {
                if (checkThreeLightError(i))
                {
                    lightStack.pop_back();
                    mapCarLights[i].lightPossibles.pop_back();
                    continue;
                }
                break;
            }
            case 4:
            {
                if (checkFourLightError(i))
                {
                    lightStack.pop_back();
                    mapCarLights[i].lightPossibles.pop_back();
                    continue;
                }
                break;
            }
            }

            if (lightStack.size() < Lights.size() && findMaxCarNum() == nCar)
                nCar++;

            // std::cout << "break1: lightStack.size: " << lightStack.size() << ", ncar=" << nCar << ", i=" << i << ", t=" << t << std::endl;
            // for (size_t j = 0; j < lightStack.size(); j++)
            //     std::cout << "index=" << lightStack[j].carMatchIndex << ", lightNum=" << j << ", carNum=" << lightStack[j].carNum << std::endl;

            // if (lightStack.size() == Lights.size() && mapCarLights[i].lightPossibles.size() == 4)
            // {

            //     if (checkFourLightError(i))
            //         return;
            // }
            backTrack(t + 1, Lights);
            lightStack.pop_back();
            mapCarLights[i].lightPossibles.pop_back();
            if (nCar - 2 == findMaxCarNum()) // 如果把归属最大车号的灯条pop出去了，车数就要减一
                nCar--;

            // std::cout << "break2: lightStack.size: " << lightStack.size() << ", ncar=" << nCar << ", i=" << i << ", t=" << t << std::endl;
            // for (size_t j = 0; j < lightStack.size(); j++)
            //     std::cout << "index=" << lightStack[j].carMatchIndex << ", lightNum=" << j << ", carNum=" << lightStack[j].carNum << std::endl;
        }
    }
}

void CarMatch::printCarError(const CarPossible &car)
{
    // std::cout << "sumError=" << car.sumError << std::endl;
    std::cout << "nLight=" << car.nLight << std::endl;
    std::cout << "first=" << car.first << std::endl;
    // std::cout << "isOneLight=" << car.isOneLight << std::endl;
    // std::cout << "armor1DiffAngleError=" << car.armor1DiffAngleError << std::endl;
    // std::cout << "armor1CenterAngleError=" << car.armor1CenterAngleError << std::endl;
    // std::cout << "armor1HeightError=" << car.armor1HeightError << std::endl;
    // std::cout << "armor1LenRatioError=" << car.armor1LenRatioError << std::endl;
    // std::cout << "armor1RatioError=" << car.armor1RatioError << std::endl;
    // std::cout << "armor2DiffAngleError=" << car.armor2DiffAngleError << std::endl;
    // std::cout << "armor2CenterAngleError=" << car.armor2CenterAngleError << std::endl;
    // std::cout << "armor2HeightError=" << car.armor2HeightError << std::endl;
    // std::cout << "armor2LenRatioError=" << car.armor2LenRatioError << std::endl;
    // std::cout << "armor2RatioError=" << car.armor2RatioError << std::endl;
    // std::cout << "noArmor1CenterAngleError=" << car.noArmor1CenterAngleError << std::endl;
    // std::cout << "noArmor1HeightError=" << car.noArmor1HeightError << std::endl;
    // std::cout << "noArmor1LenRatioError=" << car.noArmor1LenRatioError << std::endl;
    // std::cout << "noArmor1RatioError=" << car.noArmor1RatioError << std::endl;
    // std::cout << "noArmor2CenterAngleError=" << car.noArmor2CenterAngleError << std::endl;
    // std::cout << "noArmor2HeightError=" << car.noArmor2HeightError << std::endl;
    // std::cout << "noArmor2LenRatioError=" << car.noArmor2LenRatioError << std::endl;
    // std::cout << "noArmor2RatioError=" << car.noArmor2RatioError << std::endl;
    // std::cout << "solidError=" << car.solidError << std::endl;
    // std::cout << "biasError=" << car.biasError << std::endl;
    // std::cout << "threeLightAngleRatio=" << car.threeLightAngleRatio << std::endl;
    // std::cout << "fourLightAngleRatio=" << car.fourLightAngleRatio << std::endl;
    // std::cout << "likeFourLight=" << car.likeFourLight << std::endl;
    // float car1Left = car.lightPossibles[0].box.center.x,
    //       car1Right = car.lightPossibles.back().box.center.x;
    // std::cout << "carLeft=" << car1Left << ", carRight=" << car1Right << std::endl;
    // float car1MeanHeight = 0.0, car1MeanLen = 0.0;
    // for (const aim_deps::Light &light : car.lightPossibles)
    // {
    //     car1MeanHeight += light.box.center.y;
    //     car1MeanLen += light.box.length;
    // }
    // car1MeanLen /= car.lightPossibles.size();
    // car1MeanHeight /= car.lightPossibles.size();
    // std::cout << "carMeanHeight=" << car1MeanHeight << std::endl;
    // std::cout << "carMeanLen=" << car1MeanLen << std::endl;
    std::cout << "isTarArmor1=" << car.isTarArmor1 << std::endl;
    std::cout << "isTarArmor2=" << car.isTarArmor2 << std::endl;
}

void CarMatch::printCarsError(const CarsPossible &cars)
{
    std::cout << "cars.nCar=" << cars.nCar << std::endl;
    std::cout << "cars.carsErrorValue=" << cars.carsErrorValue << std::endl;
    std::cout << "cars.betweenError=" << cars.betweenError << std::endl;
    for (size_t j = 0; j < cars.carsPossible.size(); j++)
    {
        if (cars.carsPossible[j].nLight > 1)
        {
            std::cout << "========================car " << j << "===========================" << std::endl;
            printCarError(cars.carsPossible[j]);
        }
    }
}

void CarMatch::printDivision()
{
    for (size_t i = 0; i < division.size(); i++)
    {
        std::cout << "divide " << i << std::endl;
        for (size_t j = 0; j < division[i].carsPossible.size(); j++)
        {
            std::cout << "car " << j << std::endl;
            for (size_t k = 0; k < division[i].carsPossible[j].lightPossibles.size(); k++)
            {
                std::cout << "light " << k << " pointx: " << division[i].carsPossible[j].lightPossibles[k].box.center.x << std::endl;
            }
        }
    }
}
