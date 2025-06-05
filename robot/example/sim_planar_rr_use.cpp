#include "nonmobile_planar_rr.h"
#include "sim_planar_rr.h"
#include <iostream>
#include <vector>

int main() {
    double l1 = 2.0;
    double l2 = 2.0;
    PlanarRR robot(l1, l2);

    std::vector<Rectangle> env;
    env.push_back(Rectangle(-2.75, 1.0, 2.0, 1.0));
    env.push_back(Rectangle(1.5, 2.0, 2.0, 1.0));

    PlanarRRSIM sim(robot, env);

    double theta1 = 0.0;
    double theta2 = 0.0;

    bool c = sim.check_collision(theta1, theta2);
    printf("The collision is %d \n", c);

    return 0;
}
