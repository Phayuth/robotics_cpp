#include "sim_planar_rrr.h"
#include "collision.h"
#include <iostream>
#include <vector>

PlanarRRRSIM::PlanarRRRSIM(const PlanarRRR &robot,
                           const std::vector<Rectangle> &env)
    : robot(robot), env(env) {
}

PlanarRRRSIM::~PlanarRRRSIM() {
}

bool PlanarRRRSIM::check_collision(double theta1, double theta2, double theta3) {
    std::array<double, 8> linkpose = robot.forward_link(theta1, theta2, theta3);
    std::vector<Line> links;
    links.push_back(Line(linkpose[0], linkpose[1], linkpose[2], linkpose[3]));
    links.push_back(Line(linkpose[2], linkpose[3], linkpose[4], linkpose[5]));
    links.push_back(Line(linkpose[4], linkpose[5], linkpose[6], linkpose[7]));

    for (size_t i = 0; i < env.size(); i++) {
        for (size_t j = 0; j < links.size(); j++) {
            if (check_link_v_rectangle(links[j], env[i])) {
                return true;
            }
        }
    }
    return false;
}