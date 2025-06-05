#ifndef SIM_PLANAR_6R_H
#define SIM_PLANAR_6R_H
#include "collision.h"
#include "nonmobile_planar_6r.h"
#include <vector>

class Planar6RSIM {
    public:
        Planar6R robot;
        std::vector<Rectangle> env;

        Planar6RSIM(const Planar6R &robot, const std::vector<Rectangle> &env);
        ~Planar6RSIM();

        bool check_collision(double theta1, double theta2, double theta3,
                             double theta4, double theta5, double theta6);
};

#endif