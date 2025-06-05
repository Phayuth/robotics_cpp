#ifndef NONMOBILE_PLANAR_6R_H
#define NONMOBILE_PLANAR_6R_H
#include <array>

class Planar6R {
    private:
        double link1;
        double link2;
        double link3;
        double link4;
        double link5;
        double link6;

    public:
        Planar6R(double link1, double link2, double link3, double link4,
                 double link5, double link6);
        ~Planar6R();

        std::array<double, 3> forward_kinematic(double theta1, double theta2,
                                                double theta3, double theta4,
                                                double theta5, double theta6);

        std::array<double, 14> forward_link(double theta1, double theta2,
                                            double theta3, double theta4,
                                            double theta5, double theta6);

        std::array<std::array<double, 6>, 3>
        get_jacobian(double theta1, double theta2, double theta3, double theta4,
                     double theta5, double theta6);
};

#endif