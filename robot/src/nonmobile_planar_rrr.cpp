#include "nonmobile_planar_rrr.h"
#include <array>
#include <cmath>

PlanarRRR::PlanarRRR(double link1, double link2, double link3) {
    this->link1 = link1;
    this->link2 = link2;
    this->link3 = link3;
}

PlanarRRR::~PlanarRRR() {
}

std::array<double, 3> PlanarRRR::forward_kinematic(double theta1, double theta2,
                                                   double theta3) {
    double x = link1 * cos(theta1) + link2 * cos(theta1 + theta2) +
               link3 * cos(theta1 + theta2 + theta3);
    double y = link1 * sin(theta1) + link2 * sin(theta1 + theta2) +
               link3 * sin(theta1 + theta2 + theta3);
    double p = theta1 + theta2 + theta3;
    std::array<double, 3> xyp = {x, y, p};

    return xyp;
}

std::array<double, 8> PlanarRRR::forward_link(double theta1, double theta2,
                                              double theta3) {
    double x1 = link1 * cos(theta1);
    double y1 = link1 * sin(theta1);

    double x2 = x1 + link2 * cos(theta1 + theta2);
    double y2 = y1 + link2 * sin(theta1 + theta2);

    double x3 = x2 + link3 * cos(theta1 + theta2 + theta3);
    double y3 = y2 + link3 * sin(theta1 + theta2 + theta3);

    std::array<double, 8> bee = {0.0, 0.0, x1, y1, x2, y2, x3, y3};

    return bee;
}

std::array<double, 3> PlanarRRR::forward_kinematic_with_gripper(double theta1,
                                                                double theta2,
                                                                double theta3) {
    double x =
        link1 * cos(theta1) + link2 * cos(theta1 + theta2) +
        (link3 + gripperLength - gripperOffset) * cos(theta1 + theta2 + theta3);
    double y =
        link1 * sin(theta1) + link2 * sin(theta1 + theta2) +
        (link3 + gripperLength - gripperOffset) * sin(theta1 + theta2 + theta3);
    double p = theta1 + theta2 + theta3;
    std::array<double, 3> xyp = {x, y, p};

    return xyp;
}

std::array<double, 20>
PlanarRRR::forward_link_with_gripper(double theta1, double theta2, double theta3) {
    double x1 = link1 * cos(theta1);
    double y1 = link1 * sin(theta1);

    double x2 = x1 + link2 * cos(theta1 + theta2);
    double y2 = y1 + link2 * sin(theta1 + theta2);

    double x3 = x2 + link3 * cos(theta1 + theta2 + theta3);
    double y3 = y2 + link3 * sin(theta1 + theta2 + theta3);

    double x4 = x3 + gripperLength * cos(theta1 + theta2 + theta3);
    double y4 = y3 + gripperLength * sin(theta1 + theta2 + theta3);

    double xtcp =
        x3 + (gripperLength - gripperOffset) * cos(theta1 + theta2 + theta3);
    double ytcp =
        y3 + (gripperLength - gripperOffset) * sin(theta1 + theta2 + theta3);

    double xblj =
        x3 + (gripperWidth / 2) * cos(theta1 + theta2 + theta3 + M_PI / 2);
    double yblj =
        y3 + (gripperWidth / 2) * sin(theta1 + theta2 + theta3 + M_PI / 2);

    double xbrj =
        x3 + (gripperWidth / 2) * cos(theta1 + theta2 + theta3 - M_PI / 2);
    double ybrj =
        y3 + (gripperWidth / 2) * sin(theta1 + theta2 + theta3 - M_PI / 2);

    double xtlj = xblj + gripperLength * cos(theta1 + theta2 + theta3);
    double ytlj = yblj + gripperLength * sin(theta1 + theta2 + theta3);

    double xtrj = xbrj + gripperLength * cos(theta1 + theta2 + theta3);
    double ytrj = ybrj + gripperLength * sin(theta1 + theta2 + theta3);

    std::array<double, 20> bee = {0.0,  0.0,  x1,   y1,   x2,   y2,   x3,
                                  y3,   x4,   y4,   xtcp, ytcp, xblj, yblj,
                                  xbrj, ybrj, xtlj, ytlj, xtrj, ytrj};

    return bee;
}