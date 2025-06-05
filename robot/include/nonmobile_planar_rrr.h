#ifndef NONMOBILE_PLANAR_RRR_H
#define NONMOBILE_PLANAR_RRR_H
#include <array>

class PlanarRRR {
    private:
        double link1;
        double link2;
        double link3;

        double gripperLength;
        double gripperWidth;
        double gripperOffset;

    public:
        PlanarRRR(double link1, double link2, double link3);
        ~PlanarRRR();

        std::array<double, 3> forward_kinematic(double theta1, double theta2,
                                                double theta3);
        std::array<double, 8> forward_link(double theta1, double theta2,
                                           double theta3);

        std::array<double, 3> forward_kinematic_with_gripper(double theta1,
                                                             double theta2,
                                                             double theta3);
        std::array<double, 20>
        forward_link_with_gripper(double theta1, double theta2, double theta3);
};

#endif