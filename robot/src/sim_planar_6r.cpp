#include "sim_planar_6r.h"
#include "collision.h"
#include <iostream>
#include <vector>

Planar6RSIM::Planar6RSIM(const Planar6R &robot, const std::vector<Rectangle> &env)
    : robot(robot), env(env) {
}

Planar6RSIM::~Planar6RSIM() {
}

bool Planar6RSIM::check_collision(double theta1, double theta2, double theta3,
                                  double theta4, double theta5, double theta6) {
    std::array<double, 14> linkpose =
        robot.forward_link(theta1, theta2, theta3, theta4, theta5, theta6);
    std::vector<Line> links;
    links.push_back(Line(linkpose[0], linkpose[1], linkpose[2], linkpose[3]));
    links.push_back(Line(linkpose[2], linkpose[3], linkpose[4], linkpose[5]));
    links.push_back(Line(linkpose[4], linkpose[5], linkpose[6], linkpose[7]));
    links.push_back(Line(linkpose[6], linkpose[7], linkpose[8], linkpose[9]));
    links.push_back(Line(linkpose[8], linkpose[9], linkpose[10], linkpose[11]));
    links.push_back(Line(linkpose[10], linkpose[11], linkpose[12], linkpose[13]));

    for (size_t i = 0; i < env.size(); i++) {
        for (size_t j = 0; j < links.size(); j++) {
            if (check_link_v_rectangle(links[j], env[i])) {
                return true;
            }
        }
    }
    return false;
}
