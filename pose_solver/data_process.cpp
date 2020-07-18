#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {
    ifstream fi("../../data.csv", ios::in);
    for (int i=0; i<54022; i++) {
        int rgb;
        for (int j=0; j<27; j++)
            fi >> rgb;
    }
    return 0;
}