#ifndef SIM_PLANAR_RR_H
#define SIM_PLANAR_RR_H
#include "collision.h"
#include "nonmobile_planar_rr.h"
#include <vector>

class PlanarRRSIM {
    public:
        PlanarRR robot;
        std::vector<Rectangle> env;

        PlanarRRSIM(const PlanarRR &robot, const std::vector<Rectangle> &env);
        ~PlanarRRSIM();

        bool check_collision(double theta1, double theta2);
};
#endif