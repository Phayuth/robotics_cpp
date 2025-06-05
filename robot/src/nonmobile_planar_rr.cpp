#include "nonmobile_planar_rr.h"
#include <array>
#include <cmath>

PlanarRR::PlanarRR(double link1, double link2) {
    this->link1 = link1;
    this->link2 = link2;
}

PlanarRR::~PlanarRR() {
}

std::array<double, 2> PlanarRR::forward_kinematic(double theta1, double theta2) {
    double x = this->link1 * cos(theta1) + this->link2 * cos(theta1 + theta2);
    double y = this->link1 * sin(theta1) + this->link2 * sin(theta1 + theta2);

    std::array<double, 2> xy = {x, y};

    return xy;
}

std::array<double, 6> PlanarRR::forward_link(double theta1, double theta2) {
    double elbowx = this->link1 * cos(theta1);
    double elbowy = this->link1 * sin(theta1);

    double eex = this->link1 * cos(theta1) + this->link2 * cos(theta1 + theta2);
    double eey = this->link1 * sin(theta1) + this->link2 * sin(theta1 + theta2);

    std::array<double, 6> bee = {0.0, 0.0, elbowx, elbowy, eex, eey};

    return bee;
}
