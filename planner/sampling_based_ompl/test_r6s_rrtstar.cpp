#include <iostream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
    // This is where you'd add your custom validity checks, like collisions or
    // joint limits.
    return true;
}

int main() {
    ompl::RNG::setSeed(9);

    auto space = std::make_shared<ob::RealVectorStateSpace>(6);
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-2 * M_PI); // set all joint bound to 2pi
    bounds.setHigh(2 * M_PI);
    bounds.setLow(2, -M_PI); // set elbow joint (index 2) bound to pi
    bounds.setHigh(2, M_PI);
    space->setBounds(bounds);
    og::SimpleSetup ss(space);

    ss.setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.0;
    start[3] = 0.0;
    start[4] = 0.0;
    start[5] = 0.0;
    ob::ScopedState<> goal(space);
    goal[0] = M_PI;
    goal[1] = 0.0;
    goal[2] = 0.0;
    goal[3] = 0.0;
    goal[4] = 0.0;
    goal[5] = 0.0;
    ss.setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    planner->setRange(0.3);
    planner->setGoalBias(0.05);
    ss.setPlanner(planner);
    planner->getProblemDefinition();
    ob::PlannerStatus solved = ss.solve(2.0);

    if (solved) {
        std::cout << "Found solution!" << std::endl;

        // Print the path
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    } else {
        std::cout << "No solution found." << std::endl;
    }

    return 0;
}
