#include "spatial_transformation.h"
#include <cstdlib>
#include <iostream>

int main() {
    Eigen::Matrix4f hx;
    hrx(hx, 1.5);
    std::cout << hx << std::endl;

    auto hh = hx * hx;
    std::cout << hh << std::endl;

    std::cout << hh(0);

    return 0;
}