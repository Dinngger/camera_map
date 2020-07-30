#include "../include/CarMatch.hpp"
#include "PoseSolver.hpp"

float getPointDist(const cv::Point2f &p1, const cv::Point2f &p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

void CarMatch::clear(){
    lightStack.clear();
    divideClass.clear();
    division.clear();
    errors.clear();
}

void CarMatch::runMatch(std::vector<aim_deps::Light>& Lights){
    std::cout<<"match.possibles.size: "<<Lights.size()<<std::endl;
    clear();
    backTrack(0, Lights);
    sortLight();
    divisionLoad();
    calError();
    printError();
}

void CarMatch::getImage(cv::Mat img){
    src = img;
}

void CarMatch::drawCar(const std::vector<aim_deps::Light> &lightPossibles){
    cv::Mat src2 = src.clone();
    for (size_t k = 0; k < lightPossibles.size(); ++k){
        cv::line(src2, lightPossibles[k].box.vex[0], lightPossibles[k].box.vex[1], cv::Scalar(0, 255, 0), 3);
    }
    cv::imshow("carError", src2);
    char key = cv::waitKey(0);
}

void CarMatch::calError(){
    float ierror, carError;
    for (size_t i=0;i<division.size();i++){
        ierror = 0.0;
        for (size_t j=0;j<division[i].size();j++){
            carError = 0.0;
            switch(division[i][j].lightPossibles.size()){
                case 1:
                    carError = oneLight();
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
                default :
                    std::cout<<"======================================\n";
            }
            drawCar(division[i][j].lightPossibles);
            std::cout<<"carError: "<<carError<<std::endl;
            carErrors.push_back(carError);
            ierror += carError;
        }
        errors.push_back(ierror);
    }
}

void CarMatch::printError(){
    for (size_t i=0;i<errors.size();i++){
        std::cout<<"i="<<i<<", errors="<<errors[i]<<std::endl;
    }
}

float CarMatch::getRatio(float l1, float l2) {                    //对边中点连线的长度平方比值是否合适
    float len1 = aim_deps::getPointDist((points[0]+points[1])/2, (points[2]+points[3])/2),
         len2 = (l1 + l2) / 2,
         ratio = len1/(len2 * len2);
    return ratio;
}

float CarMatch::getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2){
    //lightCompensate(b1, b2);
    points[0] = b1.vex[0];
    points[1] = b1.vex[1];
    points[2] = b2.vex[1];
    points[3] = b2.vex[0];
    cv::Point2f diff = points[0] - points[1];
    /// 这个地方的意思是：需要灯条有合适的角度（cot值必须小于1.5）
    return diff.x/diff.y;
}

float CarMatch::armorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2){
    float diffAngleError = fabs(b1.angle - b2.angle);
    getArmorPlate(b1, b2);
    float ratio = getRatio(b1.length, b2.length);
    float ratioError;
    if (ratio < 19.36 && ratio > 14.44)
        ratioError = fabs(ratio - 16.9);
    else
        ratioError = (ratio - 16.9)*(ratio - 16.9);
    return diffAngleError + ratioError;
}

float CarMatch::noArmorError(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2){
    getArmorPlate(b1, b2);
    float ratio = getRatio(b1.length, b2.length);
    float ratioError;
    if (ratio < 19.36 && ratio > 14.44)
        ratioError = fabs(ratio - 16.9);
    else
        ratioError = (ratio - 16.9)*(ratio - 16.9);
    return ratioError;
}

float CarMatch::oneLight(){
    return 100.0;
}

float CarMatch::twoLight(std::vector<aim_deps::Light> lightPossibles){
    float e1, e2;
    e1 = armorError(lightPossibles[0].box, lightPossibles[1].box);
    e2 = noArmorError(lightPossibles[0].box, lightPossibles[1].box);
    return e1 < e2 ? e1 : e2;
}

float CarMatch::threeLight(std::vector<aim_deps::Light> lightPossibles){
    float e1, e2;
    e1 = armorError(lightPossibles[0].box, lightPossibles[1].box);
    e1 += noArmorError(lightPossibles[1].box, lightPossibles[2].box);
    e2 = armorError(lightPossibles[1].box, lightPossibles[2].box);
    e2 += noArmorError(lightPossibles[0].box, lightPossibles[1].box);

    return e1 < e2 ? e1 : e2;
}

float CarMatch::fourLight(std::vector<aim_deps::Light> lightPossibles){
    float e1, e2;
    e1 = armorError(lightPossibles[0].box, lightPossibles[1].box);
    e1 += noArmorError(lightPossibles[1].box, lightPossibles[2].box);
    e1 += armorError(lightPossibles[2].box, lightPossibles[3].box);
    e2 = armorError(lightPossibles[0].box, lightPossibles[1].box);
    e2 += noArmorError(lightPossibles[1].box, lightPossibles[2].box);
    e2 += armorError(lightPossibles[2].box, lightPossibles[3].box);
    return e1 < e2 ? e1 : e2;
}

bool CarMatch::constraint(int i){
    // 过滤掉灯条数大于4的车
    int maxlight[80] = {0};
    for (size_t j=0;j<lightStack.size();j++){
        maxlight[lightStack[j].carNum]++;
    }
    for (int j=0;j<80;j++){
        if (maxlight[j] > 4)
            if (i == j)
                return false;
    }
    return true;
}

int CarMatch::findMaxCar(){
    int m=-1;
    for (size_t i=0;i<lightStack.size();i++){
        if (lightStack[i].carNum > m)
            m = lightStack[i].carNum;
    }
    return m;
}

void CarMatch::backTrack(int t, std::vector<aim_deps::Light>& Lights){
    if(t > 10)
        return;
    if (lightStack.size() == Lights.size()){
        divideClass.push_back(lightStack);
        // std::cout<<"========push=========\n";
    }
    else{
        for (int i=0;i<=nCar;i++){
            Lights[t].carNum = i;
            lightStack.push_back(Lights[t]);
            if (lightStack.size() < Lights.size() && findMaxCar() == nCar)
                nCar++;
            // std::cout<<"break1: lightStack.size: "<<lightStack.size()<<", ncar="<<nCar<<", i="<<i<<", t="<<t<<std::endl;
            // for (size_t j=0;j<lightStack.size();j++)
            //     std::cout<<"lightNum="<<j<<", carNum="<<lightStack[j].carNum<<std::endl;
            if (constraint(i)){
                backTrack(t+1, Lights);
            }
            lightStack.pop_back();
            if (nCar-2 == findMaxCar())
                nCar--;
            // std::cout<<"break2: lightStack.size: "<<lightStack.size()<<", ncar="<<nCar<<", i="<<i<<", t="<<t<<std::endl;
            // for (size_t j=0;j<lightStack.size();j++)
            //     std::cout<<"lightNum="<<j<<", carNum="<<lightStack[j].carNum<<std::endl;
        }
    }
}

bool compareX(aim_deps::Light l1, aim_deps::Light l2){
    if (l1.carNum == l2.carNum)
        return l1.box.center.x < l2.box.center.x;
    else
        return l1.carNum < l2.carNum;
}

void CarMatch::sortLight(){
    // 给灯条排序
    for (size_t i=0;i<divideClass.size();i++){
        std::sort(divideClass[i].begin(), divideClass[i].end(), compareX);
    }
}

void CarMatch::divisionLoad(){
    // 将divideClass转为division
    CarPossible empty;
    std::vector<CarPossible> emptyCars;
    for (size_t i=0;i<divideClass.size();i++){
        for (size_t j=0;j<divideClass[i].size()-1;j++){
            // std::cout<<"i="<<i<<", j="<<j<<std::endl;
            // std::cout<<"carNum: cur="<<divideClass[i][j].carNum<<", last="<<divideClass[i][j-1].carNum<<std::endl;
            if (j == 0) empty.lightPossibles.push_back(divideClass[i][j]);
            if (divideClass[i][j+1].carNum == divideClass[i][j].carNum){
                empty.lightPossibles.push_back(divideClass[i][j+1]);
                if (j == divideClass[i].size()-2){
                    emptyCars.push_back(empty);
                    empty.lightPossibles.clear();
                }
            }
            else{
                emptyCars.push_back(empty);
                empty.lightPossibles.clear();
                empty.lightPossibles.push_back(divideClass[i][j+1]);
                if (j == divideClass[i].size()-2){
                    emptyCars.push_back(empty);
                    empty.lightPossibles.clear();
                }
            }
        }
        division.push_back(emptyCars);
        emptyCars.clear();
    }
}

void CarMatch::printDivision(){
    for (size_t i=0;i<division.size();i++){
        std::cout<<"divide "<<i<<std::endl;
        for (size_t j=0;j<division[i].size();j++){
            std::cout<<"car "<<j<<std::endl;
            for (size_t k=0;k<division[i][j].lightPossibles.size();k++){
                std::cout<<"light "<<k<<" pointx: "<<division[i][j].lightPossibles[k].box.center.x<<std::endl;
            }
        }
    }
}

void CarMatch::printCars(){
    std::cout<<"divideClass.size: "<<divideClass.size()<<std::endl;
    for (size_t i=0;i<divideClass.size();i++){
        std::cout<<"divide "<<i<<std::endl;
        for (size_t j=0;j<divideClass[i].size();j++)
            std::cout<<divideClass[i][j].carNum<<", "<<divideClass[i][j].index<<" pointx: "<<divideClass[i][j].box.center.x<<std::endl;
    }
}

