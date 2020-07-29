#include "../include/CarMatch.hpp"
#include "PoseSolver.hpp"

void CarMatch::clear(){
    lightStack.clear();
    divideClass.clear();
}

void CarMatch::runMatch(std::vector<aim_deps::Light>& Lights){
    std::cout<<"match.possibles.size: "<<Lights.size()<<std::endl;
    clear();
    backTrack(0, Lights);
    sortLight();
    divisionLoad();
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

