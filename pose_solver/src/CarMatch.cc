#include "../include/CarMatch.hpp"
#include "PoseSolver.hpp"

bool CarMatch::constraint(std::vector<CarPossible> tempCars){
    // 约束函数。判断tempCars是否满足约束。
    return true;
}

void CarMatch::runMatch(const std::vector<aim_deps::Light>& Lights){
    for (size_t i=0;i<Lights.size();i++){
        tempCars.push_back(emptyCar);
        for (size_t j=0;j<i+1;j++){
            tempCars[j].lightPossibles.push_back(Lights[i]);
            if (constraint(tempCars)){
                carsPossible.push_back(tempCars[j]);
            }
        }
    }
}

