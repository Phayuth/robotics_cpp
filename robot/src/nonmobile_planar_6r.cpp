#include "nonmobile_planar_6r.h"
#include <array>
#include <cmath>

Planar6R::Planar6R(double link1, double link2, double link3, double link4,
                   double link5, double link6) {
    this->link1 = link1;
    this->link2 = link2;
    this->link3 = link3;
    this->link4 = link4;
    this->link5 = link5;
    this->link6 = link6;
}

Planar6R::~Planar6R() {
}

std::array<double, 3> Planar6R::forward_kinematic(double theta1, double theta2,
                                                  double theta3, double theta4,
                                                  double theta5, double theta6) {
    double x = link1 * cos(theta1) + link2 * cos(theta1 + theta2) +
               link3 * cos(theta1 + theta2 + theta3) +
               link4 * cos(theta1 + theta2 + theta3 + theta4) +
               link5 * cos(theta1 + theta2 + theta3 + theta4 + theta5) +
               link6 * cos(theta1 + theta2 + theta3 + theta4 + theta5 + theta6);

    double y = link1 * sin(theta1) + link2 * sin(theta1 + theta2) +
               link3 * sin(theta1 + theta2 + theta3) +
               link4 * sin(theta1 + theta2 + theta3 + theta4) +
               link5 * sin(theta1 + theta2 + theta3 + theta4 + theta5) +
               link6 * sin(theta1 + theta2 + theta3 + theta4 + theta5 + theta6);

    double p = theta1 + theta2 + theta3 + theta4 + theta5 + theta6;

    std::array<double, 3> xyp = {x, y, p};
    return xyp;
}

std::array<double, 14> Planar6R::forward_link(double theta1, double theta2,
                                              double theta3, double theta4,
                                              double theta5, double theta6) {
    double x1 = link1 * cos(theta1);
    double y1 = link1 * sin(theta1);

    double x2 = x1 + link2 * cos(theta1 + theta2);
    double y2 = y1 + link2 * sin(theta1 + theta2);

    double x3 = x2 + link3 * cos(theta1 + theta2 + theta3);
    double y3 = y2 + link3 * sin(theta1 + theta2 + theta3);

    double x4 = x3 + link4 * cos(theta1 + theta2 + theta3 + theta4);
    double y4 = y3 + link4 * sin(theta1 + theta2 + theta3 + theta4);

    double x5 = x4 + link5 * cos(theta1 + theta2 + theta3 + theta4 + theta5);
    double y5 = y4 + link5 * sin(theta1 + theta2 + theta3 + theta4 + theta5);

    double x6 =
        x5 + link6 * cos(theta1 + theta2 + theta3 + theta4 + theta5 + theta6);
    double y6 =
        y5 + link6 * sin(theta1 + theta2 + theta3 + theta4 + theta5 + theta6);

    std::array<double, 14> bee = {
        0.0, 0.0, x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6};
    return bee;
}

std::array<std::array<double, 6>, 3>
Planar6R::get_jacobian(double theta1, double theta2, double theta3, double theta4,
                       double theta5, double theta6) {

    std::array<double, 6> cumsum_thetas = {
        theta1,
        theta1 + theta2,
        theta1 + theta2 + theta3,
        theta1 + theta2 + theta3 + theta4,
        theta1 + theta2 + theta3 + theta4 + theta5,
        theta1 + theta2 + theta3 + theta4 + theta5 + theta6};

    std::array<double, 6> cos_thetas = {cos(cumsum_thetas[0]),
                                        cos(cumsum_thetas[1]),
                                        cos(cumsum_thetas[2]),
                                        cos(cumsum_thetas[3]),
                                        cos(cumsum_thetas[4]),
                                        cos(cumsum_thetas[5])};
    std::array<double, 6> sin_thetas = {sin(cumsum_thetas[0]),
                                        sin(cumsum_thetas[1]),
                                        sin(cumsum_thetas[2]),
                                        sin(cumsum_thetas[3]),
                                        sin(cumsum_thetas[4]),
                                        sin(cumsum_thetas[5])};
    std::array<double, 6> cumsum_links = {link1,
                                          link1 + link2,
                                          link1 + link2 + link3,
                                          link1 + link2 + link3 + link4,
                                          link1 + link2 + link3 + link4 + link5,
                                          link1 + link2 + link3 + link4 + link5 +
                                              link6};

    double s1 = link1 * sin_thetas[0];
    double s2 = link2 * sin_thetas[1];
    double s3 = link3 * sin_thetas[2];
    double s4 = link4 * sin_thetas[3];
    double s5 = link5 * sin_thetas[4];
    double s6 = link6 * sin_thetas[5];
    double c1 = link1 * cos_thetas[0];
    double c2 = link2 * cos_thetas[1];
    double c3 = link3 * cos_thetas[2];
    double c4 = link4 * cos_thetas[3];
    double c5 = link5 * cos_thetas[4];
    double c6 = link6 * cos_thetas[5];

    double sum_s = s1 + s2 + s3 + s4 + s5 + s6;
    double sum_c = c1 + c2 + c3 + c4 + c5 + c6;

    std::array<std::array<double, 6>, 3> jacobian = {};
    jacobian[0][0] = -sum_s;
    jacobian[0][1] = -(sum_s - s1);
    jacobian[0][2] = -(sum_s - s1 - s2);
    jacobian[0][3] = -(sum_s - s1 - s2 - s3);
    jacobian[0][4] = -(sum_s - s1 - s2 - s3 - s4);
    jacobian[0][5] = -(sum_s - s1 - s2 - s3 - s4 - s5);

    jacobian[1][0] = sum_c;
    jacobian[1][1] = sum_c - c1;
    jacobian[1][2] = sum_c - c1 - c2;
    jacobian[1][3] = sum_c - c1 - c2 - c3;
    jacobian[1][4] = sum_c - c1 - c2 - c3 - c4;
    jacobian[1][5] = sum_c - c1 - c2 - c3 - c4 - c5;

    jacobian[2][0] = 1.0;
    jacobian[2][1] = 1.0;
    jacobian[2][2] = 1.0;
    jacobian[2][3] = 1.0;
    jacobian[2][4] = 1.0;
    jacobian[2][5] = 1.0;

    return jacobian;
}