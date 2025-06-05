#ifndef SIM_PLANAR_RRR_H
#define SIM_PLANAR_RRR_H
#include "collision.h"
#include "nonmobile_planar_rrr.h"
#include <vector>

class PlanarRRRSIM {
    public:
        PlanarRRR robot;
        std::vector<Rectangle> env;

        PlanarRRRSIM(const PlanarRRR &robot, const std::vector<Rectangle> &env);
        ~PlanarRRRSIM();

        bool check_collision(double theta1, double theta2, double theta3);
};

#endif