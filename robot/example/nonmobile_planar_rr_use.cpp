#include "nonmobile_planar_rr.h"
#include <array>
#include <iostream>

int main(int argc, char const *argv[]) {
    double l1 = 2.0;
    double l2 = 2.0;

    PlanarRR robot(l1, l2);

    double theta1 = 0.0;
    double theta2 = 0.0;

    printf("------forward kinematic------\n");
    std::array<double, 2> xy = robot.forward_kinematic(theta1, theta2);
    for (size_t i = 0; i < xy.size(); i++) {
        printf("%f, ", xy[i]);
    }

    printf("\n");
    printf("------forward link------\n");
    std::array<double, 6> bee = robot.forward_link(theta1, theta2);
    for (size_t i = 0; i < bee.size(); i++) {
        printf("%f, ", bee[i]);
    }
    printf("\n");

    return 0;
}
