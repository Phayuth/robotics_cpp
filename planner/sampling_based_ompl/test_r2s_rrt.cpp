#include <iostream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
    // This is where you'd add your custom validity checks, like collisions or
    // joint limits.
    return true;
}

int main() {
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-3.14159);
    bounds.setHigh(3.14159);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    start[0] = 0.0;
    start[1] = 0.0;
    ob::ScopedState<> goal(space);
    goal[0] = 1.57;
    goal[1] = 1.57;
    ss.setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRT>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved) {
        std::cout << "Found solution!" << std::endl;

        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    } else {
        std::cout << "No solution found." << std::endl;
    }

    return 0;
}
