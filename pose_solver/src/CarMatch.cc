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
            division[i].betweenError = betweenError(division[i]);
            carError += division[i].betweenError;
            division[i].carsErrorValue += carError;
        }
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

    float car1Left = carPossible1.lightPossibles[0].box.center.x,
          car1Right = carPossible1.lightPossibles.back().box.center.x,
          car2Left = carPossible2.lightPossibles[0].box.center.x,
          car2Right = carPossible2.lightPossibles.back().box.center.x;
    // std::cout<<"car1Left="<<car1Left<<", car1Right="<<car1Right<<", car2Left="<<car2Left<<", car2Right="<<car2Right<<std::endl;

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
                    return 1000000;
            }
            if (overlap(carsPossible.carsPossible[i], carsPossible.carsPossible[j]))
            {
                return 1000000;
            }
        }
    }
    return 0;
}

float CarMatch::getRatio(float l1, float l2)
{ //对边中点连线的长度平方比值是否合适
    float len1 = aim_deps::getPointDist((points[0] + points[1]) / 2, (points[2] + points[3]) / 2),
          len2 = (l1 + l2) / 2,
          ratio = len1 / (len2 * len2);
    return ratio;
}

float CarMatch::getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2)
{
    //lightCompensate(b1, b2);
    points[0] = b1.vex[0];
    points[1] = b1.vex[1];
    points[2] = b2.vex[1];
    points[3] = b2.vex[0];
    cv::Point2f diff = points[0] - points[1];
    /// 这个地方的意思是：需要灯条有合适的角度（cot值必须小于1.5）
    return diff.x / diff.y;
}

void CarMatch::armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarPossible &carPossible, bool firstArmor)
{
    float diffAngleError = (b1.angle - b2.angle);
    if (diffAngleError < 15)
    {
        diffAngleError = 0.0;
    }
    else
    {
        diffAngleError = diffAngleError * diffAngleError * 10;
    }

    cv::Point2f vertical = (b1.center - b2.center);
    float centerAngleError;
    float angle = fabs(atan(vertical.y / vertical.x)) / 3.1415926 * 180;
    if (angle > 70)
        centerAngleError = 1000000;
    else
        centerAngleError = 0;
    float heightError = fabs(b1.center.y - b2.center.y);
    heightError = heightError * heightError * 10;
    if (heightError > 10000)
        heightError = 1000000;

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
    getArmorPlate(b1, b2);
    float ratio = getRatio(b1.length, b2.length);
    float ratioError;
    // if (ratio < 19.36 && ratio > 14.44)
    //     ratioError = fabs(ratio - 16.9);
    // else
    ratioError = (ratio - 16.9) * (ratio - 16.9) * 10;
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

void CarMatch::noArmorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarPossible &carPossible, bool firstNoArmor, int lefthigh)
{
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
    cv::Point2f vertical = (b1.center - b2.center);
    float centerAngleError;
    float angle = fabs(atan(vertical.y / vertical.x)) / 3.1415926 * 180;
    if (angle > 70)
        centerAngleError = 1000000;
    else
        centerAngleError = 0;

    float heightError = 0.0;
    switch (lefthigh)
    {
    case -1:
        heightError = fabs(b1.center.y - b2.center.y);
        ;
        break;
    case 0:
        heightError = fabs(b1.vex[0].y - b2.vex[1].y);
        break;
    case 1:
        heightError = fabs(b1.vex[1].y - b2.vex[0].y);
        break;
    }
    heightError = heightError * heightError * 5;
    if (heightError > 5000)
        heightError = 1000000;

    getArmorPlate(b1, b2);
    float ratio = getRatio(b1.length, b2.length);
    float ratioError;
    // if (ratio < 19.36 && ratio > 14.44)
    //     ratioError = fabs(ratio - 16.9);
    // else
    ratioError = (ratio - 18) * (ratio - 18) * 10;

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
    if (error.nLight == 2)
        errorValue = errorValue / 8;
    if (error.nLight == 3)
        errorValue = errorValue / 50;
    if (error.nLight == 4)
        errorValue = errorValue / 64;
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
    carPossible.nLight = e.nLight;
    carPossible.sumError = e.sumError;
    carPossible.first = first;
}

float CarMatch::oneLight(CarPossible &carPossible)
{
    carPossible.nLight = 1;
    carPossible.isOneLight = true;
    carPossible.solidError = carPossible.lightPossibles[0].box.length * carPossible.lightPossibles[0].box.length / 2;
    carPossible.sumError = carPossible.solidError;
    return carPossible.sumError;
}

float CarMatch::twoLight(CarPossible &carPossible)
{
    CarPossible e1, e2;
    e1.nLight = 2;
    e2.nLight = 2;
    armorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e1, true);
    noArmorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e2, true, -1);
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
    armorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e1, true);
    noArmorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e1, true, 0);
    noArmorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e2, true, 1);
    armorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e2, true);
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
    armorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e1, true);
    noArmorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e1, true, -1);
    armorError(carPossible.lightPossibles[2].box, carPossible.lightPossibles[3].box, e1, false);
    noArmorError(carPossible.lightPossibles[0].box, carPossible.lightPossibles[1].box, e2, true, 1);
    armorError(carPossible.lightPossibles[1].box, carPossible.lightPossibles[2].box, e2, true);
    noArmorError(carPossible.lightPossibles[2].box, carPossible.lightPossibles[3].box, e2, false, 0);
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
