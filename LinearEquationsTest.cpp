/*
 * LinearEquationsTest.cpp
 * 
 * Created Date: Saturday, September 28th 2019, 12:28:06 am
 * by Dinger
 */

#include <iostream>
#include <string>
#include <sstream>
#include "Matrix.hpp"
#include "GaussEliminationWithPivoting.hpp"

int main()
{
    uint32_t n = 8;
    Matrix<double> Ab(n, n + 1);
    std::cout << "The Matrix A|b is \n";
    std::string sMatrix = "784 557 1 0 0 0 -398969.76 -283451.73 508.89\n"
                          "0 0 0 784 557 1 163769.76 116351.73 -208.89\n"
                          "218 687 1 0 0 0 -75574.06 -238162.29 346.67\n"
                          "0 0 0 218 687 1 45780 144270 -210\n"
                          "603 871 1 0 0 0 -242876.34 -350821.38 402.78\n"
                          "0 0 0 603 871 1 253597.68 366307.76 -420.56\n"
                          "1344 688 1 0 0 0 -697388.16 -356996.32 518.89\n"
                          "0 0 0 1344 688 1 624207.36 319534.72 -464.44\n";
    std::cout << sMatrix;
    std::stringstream ss;
    ss << sMatrix;
    for (int i=0; i<n; i++) {
        for (int j=0; j<n+1; j++) {
            ss >> Ab.access(i, j);
        }
    }
    GaussEliminationWithPivoting<double> GEP(Ab);
    std::vector<double> result(n);
    if (!GEP.solve(result)) {
        std::cout << "Gauss error!\n";
    } else {
        std::cout << "Gauss result: \n";
        for (int i=0; i<n; i++) {
            std::cout << result[i] << std::endl;
        }
    }
    return 0;
}
