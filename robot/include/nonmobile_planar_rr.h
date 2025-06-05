#ifndef NONMOBILE_PLANAR_RR_H
#define NONMOBILE_PLANAR_RR_H
#include <array>

class PlanarRR {
    private:
        double link1;
        double link2;

    public:
        PlanarRR(double link1, double link2);
        ~PlanarRR();

        std::array<double, 2> forward_kinematic(double theta1, double theta2);
        std::array<double, 6> forward_link(double theta1, double theta2);
};

#endif