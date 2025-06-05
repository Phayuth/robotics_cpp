#include "sim_planar_rr.h"
#include "collision.h"
#include <iostream>
#include <vector>

PlanarRRSIM::PlanarRRSIM(const PlanarRR &robot, const std::vector<Rectangle> &env)
    : robot(robot), env(env) {
}

PlanarRRSIM::~PlanarRRSIM() {
}

bool PlanarRRSIM::check_collision(double theta1, double theta2) {
    // constuct line from link
    std::array<double, 6> linkpose = robot.forward_link(theta1, theta2);
    std::vector<Line> links;
    links.push_back(Line(linkpose[0], linkpose[1], linkpose[2], linkpose[3]));
    links.push_back(Line(linkpose[2], linkpose[3], linkpose[4], linkpose[5]));

    // loop through env(rectangle) and line for collision
    for (size_t i = 0; i < env.size(); i++) {
        for (size_t j = 0; j < links.size(); j++) {
            if (check_link_v_rectangle(links[j], env[i])) {
                return true;
            }
        }
    }
    return false;
}