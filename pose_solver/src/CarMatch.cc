#include "../include/CarMatch.hpp"
#include "PoseSolver.hpp"

float getPointDist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void CarMatch::clear()
{
    lightStack.clear();
    divideClass.clear();
    division.clear();
}

void CarMatch::runMatch(std::vector<aim_deps::Light> &Lights, cv::Mat img)
{
    std::cout << "match.possibles.size: " << Lights.size() << std::endl;
    src = img;
    clear();
    backTrack(0, Lights);
    sortLight();
    divisionLoad();
    calError();
    sortErrors();
    // printError();
}

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v)
{
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);
    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    stable_sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) { return v[i1].carsErrorValue < v[i2].carsErrorValue; });
    return idx;
}

void CarMatch::sortErrors()
{
    int cnt = 0;
    for (auto i : sort_indexes(division))
    {
        if (cnt > 0)
            break;
        std::cout << "division[i].carsErrorValue=" << division[i].carsErrorValue << std::endl;
        std::cout << "division[i].betweenError=" << division[i].betweenError << std::endl;
        for (size_t j = 0; j < division[i].carsPossible.size(); j++)
        {
            std::cout << "===============division[i][j]=================="
                      << "cnt=" << cnt << ", i=" << i << ", j=" << j << std::endl;
            std::cout << "sumError=" << division[i].carsPossible[j].sumError << std::endl;
            std::cout << "nLight=" << division[i].carsPossible[j].nLight << std::endl;
            std::cout << "first=" << division[i].carsPossible[j].first << std::endl;
            std::cout << "isOneLight=" << division[i].carsPossible[j].isOneLight << std::endl;
            std::cout << "armor1DiffAngleError=" << division[i].carsPossible[j].armor1DiffAngleError << std::endl;
            std::cout << "armor1CenterAngleError=" << division[i].carsPossible[j].armor1CenterAngleError << std::endl;
            std::cout << "armor1HeightError=" << division[i].carsPossible[j].armor1HeightError << std::endl;
            std::cout << "armor1LenRatioError=" << division[i].carsPossible[j].armor1LenRatioError << std::endl;
            std::cout << "armor1RatioError=" << division[i].carsPossible[j].armor1RatioError << std::endl;
            std::cout << "armor2DiffAngleError=" << division[i].carsPossible[j].armor2DiffAngleError << std::endl;
            std::cout << "armor2CenterAngleError=" << division[i].carsPossible[j].armor2CenterAngleError << std::endl;
            std::cout << "armor2HeightError=" << division[i].carsPossible[j].armor2HeightError << std::endl;
            std::cout << "armor2LenRatioError=" << division[i].carsPossible[j].armor2LenRatioError << std::endl;
            std::cout << "armor2RatioError=" << division[i].carsPossible[j].armor2RatioError << std::endl;
            std::cout << "noArmor1CenterAngleError=" << division[i].carsPossible[j].noArmor1CenterAngleError << std::endl;
            std::cout << "noArmor1HeightError=" << division[i].carsPossible[j].noArmor1HeightError << std::endl;
            std::cout << "noArmor1LenRatioError=" << division[i].carsPossible[j].noArmor1LenRatioError << std::endl;
            std::cout << "noArmor1RatioError=" << division[i].carsPossible[j].noArmor1RatioError << std::endl;
            std::cout << "noArmor2CenterAngleError=" << division[i].carsPossible[j].noArmor2CenterAngleError << std::endl;
            std::cout << "noArmor2HeightError=" << division[i].carsPossible[j].noArmor2HeightError << std::endl;
            std::cout << "noArmor2LenRatioError=" << division[i].carsPossible[j].noArmor2LenRatioError << std::endl;
            std::cout << "noArmor2RatioError=" << division[i].carsPossible[j].noArmor2RatioError << std::endl;
            std::cout << "solidError=" << division[i].carsPossible[j].solidError << std::endl;
            std::cout << "threeLightAngleRatio=" << division[i].carsPossible[j].threeLightAngleRatio << std::endl;
            std::cout << "fourLightAngleRatio=" << division[i].carsPossible[j].fourLightAngleRatio << std::endl;
            float car1Left = division[i].carsPossible[j].lightPossibles[0].box.center.x,
                  car1Right = division[i].carsPossible[j].lightPossibles.back().box.center.x;
            std::cout << "carLeft=" << car1Left << ", carRight=" << car1Right << std::endl;
            float car1MeanHeight = 0.0, car1MeanLen = 0.0;
            for (const aim_deps::Light &light : division[i].carsPossible[j].lightPossibles)
            {
                car1MeanHeight += light.box.center.y;
                car1MeanLen += light.box.length;
            }
            car1MeanLen /= division[i].carsPossible[j].lightPossibles.size();
            car1MeanHeight /= division[i].carsPossible[j].lightPossibles.size();
            std::cout << "carMeanHeight=" << car1MeanHeight << std::endl;
            std::cout << "carMeanLen=" << car1MeanLen << std::endl;
            drawCar(division[i].carsPossible[j].lightPossibles);
        }
        cnt++;
    }
}

void CarMatch::drawCar(const std::vector<aim_deps::Light> &lightPossibles)
{
    if (lightPossibles.size() > 1)
    {
        cv::Mat src2 = src.clone();
        for (size_t k = 0; k < lightPossibles.size(); ++k)
        {
            cv::line(src2, lightPossibles[k].box.vex[0], lightPossibles[k].box.vex[1], cv::Scalar(0, 0, 255), 3);
        }
        cv::imshow("carPossible", src2);
        cv::waitKey(0);
        // if (carPossibles.back()<500 && lightPossibles.size() == 4) cv::waitKey(0);
        // else cv::waitKey(1);
    }
}

void CarMatch::calError()
{
    float carError = 0.0;
    for (size_t i = 0; i < division.size(); i++)
    {
        // std::cout<<"division "<<i<<std::endl;
        for (size_t j = 0; j < division[i].carsPossible.size(); j++)
        {
            switch (division[i].carsPossible[j].lightPossibles.size())
            {
            case 1:
                carError = oneLight(division[i].carsPossible[j]);
                break;
            case 2:
                carError = twoLight(division[i].carsPossible[j]);
                break;
            case 3:
                carError = threeLight(division[i].carsPossible[j]);
                break;
            case 4:
                carError = fourLight(division[i].carsPossible[j]);
                break;
            default:
                std::cout << "======================================\n";
            }
            division[i].carsErrorValue += carError;
        }
        division[i].betweenError = betweenError(division[i]);
        division[i].carsErrorValue += division[i].betweenError;
        division[i].nCar = division[i].carsPossible.size();
    }
}

bool CarMatch::overlap(const CarPossible &carPossible1, const CarPossible &carPossible2)
{
    float car1MeanHeight = 0.0, car1MeanLen = 0.0, car2MeanHeight = 0.0, car2MeanLen = 0.0;
    for (const aim_deps::Light &light : carPossible1.lightPossibles)
    {
        car1MeanHeight += light.box.center.y;
        car1MeanLen += light.box.length;
    }
    car1MeanHeight /= carPossible1.lightPossibles.size();
    car1MeanLen /= carPossible1.lightPossibles.size();
    for (const aim_deps::Light &light : carPossible2.lightPossibles)
    {
        car2MeanHeight += light.box.center.y;
        car2MeanLen += light.box.length;
    }
    car2MeanHeight /= carPossible2.lightPossibles.size();
    car2MeanLen /= carPossible2.lightPossibles.size();
    float car1Left, car1Right, car2Left, car2Right;
    if (carPossible1.lightPossibles.size() == 1)
    {
        car1Left = std::min(carPossible1.lightPossibles[0].box.vex[0].x, carPossible1.lightPossibles[0].box.vex[1].x);
        car1Right = std::max(carPossible1.lightPossibles.back().box.vex[0].x, carPossible1.lightPossibles.back().box.vex[1].x);
        car2Left = carPossible2.lightPossibles[1].box.center.x;
        car2Right = carPossible2.lightPossibles.end()[-2].box.center.x;
    }
    else if (carPossible2.lightPossibles.size() == 1)
    {
        car1Left = carPossible1.lightPossibles[1].box.center.x;
        car1Right = carPossible1.lightPossibles.end()[-2].box.center.x;
        car2Left = std::min(carPossible2.lightPossibles[0].box.vex[0].x, carPossible2.lightPossibles[0].box.vex[1].x);
        car2Right = std::max(carPossible2.lightPossibles.back().box.vex[0].x, carPossible2.lightPossibles.back().box.vex[1].x);
    }
    else
    {
        car1Left = carPossible1.lightPossibles[0].box.center.x;
        car1Right = carPossible1.lightPossibles.back().box.center.x;
        car2Left = carPossible2.lightPossibles[0].box.center.x;
        car2Right = carPossible2.lightPossibles.back().box.center.x;
    }
    // std::cout<<"car1Left="<<car1Left<<", car1Right="<<car1Right<<", car2Left="<<car2Left<<", car2Right="<<car2Right<<std::endl;
    if (carPossible1.lightPossibles.size() == 1 || carPossible2.lightPossibles.size() == 1)
    {
        float diffH = fabs(car1MeanHeight - car2MeanHeight) / ((car1MeanLen < car2MeanLen) ? car1MeanLen : car2MeanLen);
        if (diffH * diffH * 10 < 5)
        {
            if ((car1Left < car2Right && car1Left > car2Left && car1Right > car2Right) || (car1Right > car2Left && car1Left < car2Left && car1Right < car2Right))
            {
                return true;
            }
            if ((car1Left < car2Left && car1Right > car2Right) || (car2Left < car1Left && car2Right > car1Right))
            {
                return true;
            }
        }
    }
    else if (fabs(car1MeanHeight - car2MeanHeight) < 20)
    {
        if ((car1Left < car2Right && car1Left > car2Left && car1Right > car2Right) || (car1Right > car2Left && car1Left < car2Left && car1Right < car2Right))
        {
            return true;
        }
        if ((car1Left < car2Left && car1Right > car2Right) || (car2Left < car1Left && car2Right > car1Right))
        {
            return true;
        }
    }
    return false;
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
            if (carsPossible.carsPossible[i].lightPossibles.size() == 1 && carsPossible.carsPossible[j].lightPossibles.size() == 1)
            {
                if (isArmor(carsPossible.carsPossible[i].lightPossibles[0], carsPossible.carsPossible[j].lightPossibles[0]))
                    return 1500000;
            }
            if (overlap(carsPossible.carsPossible[i], carsPossible.carsPossible[j]))
            {
                return 1400000;
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

void CarMatch::armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarPossible &carPossible, bool firstArmor, int nlgt)
{

    float diffAngleError = fabs(b1.angle - b2.angle);
    diffAngleError = diffAngleError * diffAngleError / 5;
    cv::Point2f vertical = (b1.center - b2.center);
    float centerAngleError;
    float angle = fabs(atan(vertical.y / vertical.x)) / 3.1415926 * 180;
    if (angle > 40)
        centerAngleError = 1200000;
    else
        centerAngleError = 0;
    float heightError = fabs(b1.center.y - b2.center.y);
    heightError = heightError * heightError * 10;
    if (heightError > 9000)
        heightError = 1100000;

    float lenRatioError = (b1.length < b2.length ? b2.length / b1.length : b1.length / b2.length);
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
    float ratioError = 0.0;
    if (ratio > 20)
        ratioError = 18000000;
    // if (ratio < 19.36 && ratio > 14.44)
    //     ratioError = fabs(ratio - 0.0);
    // else
    // ratioError = (ratio - 3.0) * (ratio - 3.0) * 0;
    if (firstArmor)
    {
        carPossible.armor1DiffAngleError = diffAngleError;
        carPossible.armor1HeightError = heightError;
        carPossible.armor1LenRatioError = lenRatioError;
        carPossible.armor1RatioError = ratioError;
        carPossible.armor1CenterAngleError = centerAngleError;
    }
    else
    {
        carPossible.armor2DiffAngleError = diffAngleError;
        carPossible.armor2HeightError = heightError;
        carPossible.armor2LenRatioError = lenRatioError;
        carPossible.armor2RatioError = ratioError;
        carPossible.armor2CenterAngleError = centerAngleError;
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
    if (angle > 40)
        centerAngleError = 1300000;
    else
        centerAngleError = 0;
    float heightError = 0.0;
    heightError = fabs(b1.center.y - b2.center.y);
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
    heightError = heightError * heightError;
    if (heightError > 1500)
        heightError = 1700000;

    float ratio = getRatio(b1, b2);
    float ratioError = 0.0;
    if (ratio > 35)
        ratioError = 18000000;
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
        errorValue = errorValue / 100;
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
    carPossible.sumError = e.sumError;
    carPossible.first = first;
}

float CarMatch::oneLight(CarPossible &carPossible)
{
    carPossible.nLight = 1;
    carPossible.isOneLight = true;
    carPossible.solidError = carPossible.lightPossibles[0].box.length * carPossible.lightPossibles[0].box.length / 5;
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
    for (size_t i = 0; i < carPossible.lightPossibles.size(); i++)
    {
        for (size_t j = i + 1; j < carPossible.lightPossibles.size(); j++)
        {
            if ((carPossible.lightPossibles[i].box.center.y - carPossible.lightPossibles[j].box.center.y) > 40)
            {
                e.fourLightAngleRatio = 105;
                return;
            }
        }
    }

    if (ne == 1)
    {
        float h1 = fabs(carPossible.lightPossibles[0].box.center.y - carPossible.lightPossibles[1].box.center.y);
        float h2 = fabs(carPossible.lightPossibles[2].box.center.y - carPossible.lightPossibles[3].box.center.y);
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
        if (h1 > 30 || h2 > 30)
        {
            e.fourLightAngleRatio = 106;
            return;
        }
        float diffAngle1 = fabs(carPossible.lightPossibles[0].box.angle - carPossible.lightPossibles[1].box.angle);
        float diffAngle2 = fabs(carPossible.lightPossibles[2].box.angle - carPossible.lightPossibles[3].box.angle);
        if ((diffAngle1 > 20 || diffAngle2 > 20) || (diffAngle1 > 15 && diffAngle2 > 15))
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

        float diffAngle1 = carPossible.lightPossibles[0].box.angle - carPossible.lightPossibles[1].box.angle;
        float diffAngle2 = carPossible.lightPossibles[2].box.angle - carPossible.lightPossibles[3].box.angle;
        if (diffAngle1 > 10 || diffAngle2 > 10)
        {
            e.fourLightAngleRatio = 103;
        }
        else if (diffAngle1 < -12 || diffAngle2 < -12)
        {
            e.fourLightAngleRatio = 0.12;
        }
    }
}

void CarMatch::betweenFourLightError(const CarPossible &carPossible, CarPossible &e)
{
    float diffH1 = carPossible.lightPossibles[0].box.center.y - carPossible.lightPossibles[1].box.center.y;
    float diffH2 = carPossible.lightPossibles[3].box.center.y - carPossible.lightPossibles[2].box.center.y;
    if (fabs(diffH1) > 15 || fabs(diffH2) > 15)
        if (diffH1 * diffH2 < 0)
            e.betweenFourError = 1600000;
}

bool CarMatch::constraint(int i)
{
    // 过滤掉灯条数大于4的车
    int maxlight[80] = {0};
    for (size_t j = 0; j < lightStack.size(); j++)
    {
        maxlight[lightStack[j].carNum]++;
    }
    for (int j = 0; j < 80; j++)
    {
        if (maxlight[j] > 4)
            if (i == j)
                return false;
    }
    return true;
}

int CarMatch::findMaxCar()
{
    int m = -1;
    for (size_t i = 0; i < lightStack.size(); i++)
    {
        if (lightStack[i].carNum > m)
            m = lightStack[i].carNum;
    }
    return m;
}

void CarMatch::backTrack(int t, std::vector<aim_deps::Light> &Lights)
{
    if (t > 10)
        return;
    if (lightStack.size() == Lights.size())
    {
        divideClass.push_back(lightStack);
        // std::cout<<"========push=========\n";
    }
    else
    {
        for (int i = 0; i <= nCar; i++)
        {
            Lights[t].carNum = i;
            if (fabs(Lights[t].box.angle) > 50)
                continue;
            lightStack.push_back(Lights[t]);
            if (lightStack.size() < Lights.size() && findMaxCar() == nCar)
                nCar++;
            // std::cout<<"break1: lightStack.size: "<<lightStack.size()<<", ncar="<<nCar<<", i="<<i<<", t="<<t<<std::endl;
            // for (size_t j=0;j<lightStack.size();j++)
            //     std::cout<<"lightNum="<<j<<", carNum="<<lightStack[j].carNum<<std::endl;
            if (constraint(i))
            {
                backTrack(t + 1, Lights);
            }
            lightStack.pop_back();
            if (nCar - 2 == findMaxCar())
                nCar--;
            // std::cout<<"break2: lightStack.size: "<<lightStack.size()<<", ncar="<<nCar<<", i="<<i<<", t="<<t<<std::endl;
            // for (size_t j=0;j<lightStack.size();j++)
            //     std::cout<<"lightNum="<<j<<", carNum="<<lightStack[j].carNum<<std::endl;
        }
    }
}

bool compareX(aim_deps::Light l1, aim_deps::Light l2)
{
    if (l1.carNum == l2.carNum)
        return l1.box.center.x < l2.box.center.x;
    else
        return l1.carNum < l2.carNum;
}

void CarMatch::sortLight()
{
    // 给灯条排序
    for (size_t i = 0; i < divideClass.size(); i++)
    {
        std::sort(divideClass[i].begin(), divideClass[i].end(), compareX);
    }
}

void CarMatch::divisionLoad()
{
    // 将divideClass转为division
    CarPossible empty;
    CarsPossible emptyCars;
    for (size_t i = 0; i < divideClass.size(); i++)
    {
        for (size_t j = 0; j < divideClass[i].size() - 1; j++)
        {
            if (j == 0)
                empty.lightPossibles.push_back(divideClass[i][j]);
            if (divideClass[i][j + 1].carNum == divideClass[i][j].carNum)
            {
                empty.lightPossibles.push_back(divideClass[i][j + 1]);
                if (j == divideClass[i].size() - 2)
                {
                    emptyCars.carsPossible.push_back(empty);
                    empty.lightPossibles.clear();
                }
            }
            else
            {
                emptyCars.carsPossible.push_back(empty);
                empty.lightPossibles.clear();
                empty.lightPossibles.push_back(divideClass[i][j + 1]);
                if (j == divideClass[i].size() - 2)
                {
                    emptyCars.carsPossible.push_back(empty);
                    empty.lightPossibles.clear();
                }
            }
        }
        division.push_back(emptyCars);
        emptyCars.carsPossible.clear();
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

void CarMatch::printCars()
{
    std::cout << "divideClass.size: " << divideClass.size() << std::endl;
    for (size_t i = 0; i < divideClass.size(); i++)
    {
        std::cout << "divide " << i << std::endl;
        for (size_t j = 0; j < divideClass[i].size(); j++)
            std::cout << divideClass[i][j].carNum << ", " << divideClass[i][j].index << " pointx: " << divideClass[i][j].box.center.x << std::endl;
    }
}
