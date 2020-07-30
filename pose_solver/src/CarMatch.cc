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
    divisionError.clear();
}

void CarMatch::runMatch(std::vector<aim_deps::Light> &Lights)
{
    std::cout << "match.possibles.size: " << Lights.size() << std::endl;
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
    for (auto i : sort_indexes(divisionError))
    {
        if (cnt > 0)
            break;
        std::cout << "divisionError[i].carsErrorValue=" << divisionError[i].carsErrorValue << std::endl;
        for (size_t j = 0; j < division[i].size(); j++)
        {
            std::cout << "===============divisionError[i][j]=================="
                      << "cnt=" << cnt << ", i=" << i << ", j=" << j << std::endl;
            std::cout << "sumError=" << divisionError[i].carsError[j].sumError << std::endl;
            std::cout << "nLight=" << divisionError[i].carsError[j].nLight << std::endl;
            std::cout << "isOneLight=" << divisionError[i].carsError[j].isOneLight << std::endl;
            std::cout << "armor1DiffAngleError=" << divisionError[i].carsError[j].armor1DiffAngleError << std::endl;
            std::cout << "armor1HeightError=" << divisionError[i].carsError[j].armor1HeightError << std::endl;
            std::cout << "armor1LenRatioError=" << divisionError[i].carsError[j].armor1LenRatioError << std::endl;
            std::cout << "armor1RatioError=" << divisionError[i].carsError[j].armor1RatioError << std::endl;
            std::cout << "armor2DiffAngleError=" << divisionError[i].carsError[j].armor2DiffAngleError << std::endl;
            std::cout << "armor2HeightError=" << divisionError[i].carsError[j].armor2HeightError << std::endl;
            std::cout << "armor2LenRatioError=" << divisionError[i].carsError[j].armor2LenRatioError << std::endl;
            std::cout << "armor2RatioError=" << divisionError[i].carsError[j].armor2RatioError << std::endl;
            std::cout << "noArmor1HeightError=" << divisionError[i].carsError[j].noArmor1HeightError << std::endl;
            std::cout << "noArmor1LenRatioError=" << divisionError[i].carsError[j].noArmor1LenRatioError << std::endl;
            std::cout << "noArmor1RatioError=" << divisionError[i].carsError[j].noArmor1RatioError << std::endl;
            std::cout << "noArmor2HeightError=" << divisionError[i].carsError[j].noArmor2HeightError << std::endl;
            std::cout << "noArmor2LenRatioError=" << divisionError[i].carsError[j].noArmor2LenRatioError << std::endl;
            std::cout << "noArmor2RatioError=" << divisionError[i].carsError[j].noArmor2RatioError << std::endl;
            std::cout << "solidError=" << divisionError[i].carsError[j].solidError << std::endl;
            drawCar(division[i][j].lightPossibles);
        }
        cnt++;
    }
}

void CarMatch::getImage(cv::Mat img)
{
    src = img;
}

void CarMatch::drawCar(const std::vector<aim_deps::Light> &lightPossibles)
{
    cv::Mat src2 = src.clone();
    for (size_t k = 0; k < lightPossibles.size(); ++k)
    {
        cv::line(src2, lightPossibles[k].box.vex[0], lightPossibles[k].box.vex[1], cv::Scalar(0, 255, 0), 3);
    }
    cv::imshow("carError", src2);
    cv::waitKey(0);
    // if (carErrors.back()<500 && lightPossibles.size() == 4) cv::waitKey(0);
    // else cv::waitKey(1);
}

void CarMatch::calError()
{
    CarError carError;
    CarsError carsError;
    for (size_t i = 0; i < division.size(); i++)
    {
        // std::cout<<"division "<<i<<std::endl;
        for (size_t j = 0; j < division[i].size(); j++)
        {
            switch (division[i][j].lightPossibles.size())
            {
            case 1:
                carError = oneLight(division[i][j].lightPossibles);
                break;
            case 2:
                carError = twoLight(division[i][j].lightPossibles);
                break;
            case 3:
                carError = threeLight(division[i][j].lightPossibles);
                break;
            case 4:
                carError = fourLight(division[i][j].lightPossibles);
                break;
            default:
                std::cout << "======================================\n";
            }
            // drawCar(division[i][j].lightPossibles);
            carsError.carsErrorValue += carError.sumError;
            carsError.carsError.push_back(carError);
        }
        divisionError.push_back(carsError);
        carsError.carsError.clear();
        carsError.carsErrorValue = 0.0;
    }
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

void CarMatch::armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarError &carError, bool firstArmor)
{
    float diffAngleError = (b1.angle - b2.angle) * (b1.angle - b2.angle) * 10;
    // cv::Point2f vertical = (b1.center - b2.center);
    // cv::Point2f direction1 = b1.vex[0] - b1.vex[1];
    // cv::Point2f direction2 = b2.vex[0] - b2.vex[1];
    // cv::Point2f direction = (direction1 + direction2) / 2;
    // float dot = vertical.x * direction.x + vertical.y * direction.y;
    // float det = fabs(vertical.x * direction.y - vertical.y * direction.x);
    // float angle = fabs(3.1415926 / 2 - atan2(det, dot));
    // float heightError = angle * angle * 100000;
    float heightError = fabs(b1.center.y - b2.center.y);
    heightError = heightError * heightError * 10;
    if (heightError > 10000) heightError = 1000000;
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
        carError.armor1DiffAngleError = diffAngleError;
        carError.armor1HeightError = heightError;
        carError.armor1LenRatioError = lenRatioError;
        carError.armor1RatioError = ratioError;
    }
    else
    {
        carError.armor2DiffAngleError = diffAngleError;
        carError.armor2HeightError = heightError;
        carError.armor2LenRatioError = lenRatioError;
        carError.armor2RatioError = ratioError;
    }
    // std::cout<<"armorError: "<<"ratioError="<<ratioError<<", diffAngleError="<<diffAngleError<<", heightError="<<heightError<<", lenRatioError="<<lenRatioError<<std::endl;
}

void CarMatch::noArmorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2, CarError &carError, bool firstNoArmor, int lefthigh)
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
    // cv::Point2f vertical = (b1.center - b2.center);
    // cv::Point2f direction1 = b1.vex[0] - b1.vex[1];
    // cv::Point2f direction2 = b2.vex[0] - b2.vex[1];
    // cv::Point2f direction = (direction1 + direction2) / 2;
    // float dot = vertical.x * direction.x + vertical.y * direction.y;
    // float det = fabs(vertical.x * direction.y - vertical.y * direction.x);
    // float angle = fabs(3.1415926 / 2 - atan2(det, dot));
    // float heightError = angle * angle * 100000;
    float heightError = 0.0;
    switch (lefthigh)
    {
    case -1:
        heightError = fabs(b1.center.y - b2.center.y);;
        break;
    case 0:
        heightError = fabs(b1.vex[0].y - b2.vex[1].y);
        break;
    case 1:
        heightError = fabs(b1.vex[1].y - b2.vex[0].y);
        break;
    }
    heightError = heightError * heightError * 10;
    if (heightError > 10000) heightError = 1000000;
    getArmorPlate(b1, b2);
    float ratio = getRatio(b1.length, b2.length);
    float ratioError;
    // if (ratio < 19.36 && ratio > 14.44)
    //     ratioError = fabs(ratio - 16.9);
    // else
    ratioError = (ratio - 18) * (ratio - 18) * 10;
    if (firstNoArmor)
    {
        carError.noArmor1HeightError = heightError;
        carError.noArmor1LenRatioError = lenRatioError;
        carError.noArmor1RatioError = ratioError;
    }
    else
    {
        carError.noArmor2HeightError = heightError;
        carError.noArmor2LenRatioError = lenRatioError;
        carError.noArmor2RatioError = ratioError;
    }
    // std::cout<<"noArmorError: "<<"ratioError="<<ratioError<<", heightError="<<heightError<<", lenRatioError="<<lenRatioError<<std::endl;
}

float CarMatch::sumError(CarError &error)
{
    float errorValue = 0.0;
    errorValue += error.armor1DiffAngleError;
    errorValue += error.armor1HeightError;
    errorValue += error.armor1LenRatioError;
    errorValue += error.armor1RatioError;
    errorValue += error.armor2DiffAngleError;
    errorValue += error.armor2HeightError;
    errorValue += error.armor2LenRatioError;
    errorValue += error.armor2RatioError;
    errorValue += error.noArmor1HeightError;
    errorValue += error.noArmor1LenRatioError;
    errorValue += error.noArmor1RatioError;
    errorValue += error.noArmor2HeightError;
    errorValue += error.noArmor2LenRatioError;
    errorValue += error.noArmor2RatioError;
    errorValue += error.solidError;
    if (error.nLight > 2)
        errorValue = errorValue / ((error.nLight) * (error.nLight) * (error.nLight));
    error.sumError = errorValue;
    return errorValue;
}

bool CarMatch::isE1more(CarError &e1, CarError &e2)
{
    float error1, error2;
    error1 = sumError(e1);
    error2 = sumError(e2);
    return error1 > error2 ? true : false;
}

CarError CarMatch::oneLight(std::vector<aim_deps::Light> lightPossibles)
{
    CarError e1;
    e1.nLight = 1;
    e1.isOneLight = true;
    e1.solidError = lightPossibles[0].box.length * lightPossibles[0].box.length * 10;
    e1.sumError = e1.solidError;
    return e1;
}

CarError CarMatch::twoLight(std::vector<aim_deps::Light> lightPossibles)
{
    CarError e1, e2;
    e1.nLight = 2;
    e2.nLight = 2;
    armorError(lightPossibles[0].box, lightPossibles[1].box, e1, true);
    noArmorError(lightPossibles[0].box, lightPossibles[1].box, e2, true, -1);
    return isE1more(e1, e2) ? e2 : e1;
}

CarError CarMatch::threeLight(std::vector<aim_deps::Light> lightPossibles)
{
    CarError e1, e2;
    e1.nLight = 3;
    e2.nLight = 3;
    armorError(lightPossibles[0].box, lightPossibles[1].box, e1, true);
    noArmorError(lightPossibles[1].box, lightPossibles[2].box, e1, true, 0);
    noArmorError(lightPossibles[0].box, lightPossibles[1].box, e2, true, 1);
    armorError(lightPossibles[1].box, lightPossibles[2].box, e2, true);
    return isE1more(e1, e2) ? e2 : e1;
}

CarError CarMatch::fourLight(std::vector<aim_deps::Light> lightPossibles)
{
    CarError e1, e2;
    e1.nLight = 4;
    e2.nLight = 4;
    armorError(lightPossibles[0].box, lightPossibles[1].box, e1, true);
    noArmorError(lightPossibles[1].box, lightPossibles[2].box, e1, true, -1);
    armorError(lightPossibles[2].box, lightPossibles[3].box, e1, false);
    noArmorError(lightPossibles[0].box, lightPossibles[1].box, e2, true, 1);
    armorError(lightPossibles[1].box, lightPossibles[2].box, e2, true);
    noArmorError(lightPossibles[2].box, lightPossibles[3].box, e2, false, 0);
    return isE1more(e1, e2) ? e2 : e1;
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
    std::vector<CarPossible> emptyCars;
    for (size_t i = 0; i < divideClass.size(); i++)
    {
        for (size_t j = 0; j < divideClass[i].size() - 1; j++)
        {
            // std::cout<<"i="<<i<<", j="<<j<<std::endl;
            // std::cout<<"carNum: cur="<<divideClass[i][j].carNum<<", last="<<divideClass[i][j-1].carNum<<std::endl;
            if (j == 0)
                empty.lightPossibles.push_back(divideClass[i][j]);
            if (divideClass[i][j + 1].carNum == divideClass[i][j].carNum)
            {
                empty.lightPossibles.push_back(divideClass[i][j + 1]);
                if (j == divideClass[i].size() - 2)
                {
                    emptyCars.push_back(empty);
                    empty.lightPossibles.clear();
                }
            }
            else
            {
                emptyCars.push_back(empty);
                empty.lightPossibles.clear();
                empty.lightPossibles.push_back(divideClass[i][j + 1]);
                if (j == divideClass[i].size() - 2)
                {
                    emptyCars.push_back(empty);
                    empty.lightPossibles.clear();
                }
            }
        }
        division.push_back(emptyCars);
        emptyCars.clear();
    }
}

void CarMatch::printDivision()
{
    for (size_t i = 0; i < division.size(); i++)
    {
        std::cout << "divide " << i << std::endl;
        for (size_t j = 0; j < division[i].size(); j++)
        {
            std::cout << "car " << j << std::endl;
            for (size_t k = 0; k < division[i][j].lightPossibles.size(); k++)
            {
                std::cout << "light " << k << " pointx: " << division[i][j].lightPossibles[k].box.center.x << std::endl;
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
