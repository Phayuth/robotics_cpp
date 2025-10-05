#include <fstream>
#include <iostream>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
    return true;
}

void save_to_storage(ob::PlannerDataStorage &datastorage, ob::PlannerData &data,
                     std::string &filename) {
    datastorage.store(data, "stored_with_storage");
}

void load_from_storage(ob::PlannerDataStorage &datastorage, ob::PlannerData &data,
                       std::string &filename) {
    datastorage.load("stored_with_sorage", data);
}

void build_roadmap() {
}

void save_to_graphml(ob::PlannerData &data, std::string &filename) {
    std::ofstream file(filename + ".graphml");
    data.printGraphML(file);
    file.close();
}

int main() {

    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-2 * M_PI);
    bounds.setHigh(2 * M_PI);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    ss.setStateValidityChecker(isStateValid);

    ob::ScopedState<> qs(space);
    qs[0] = 0.0;
    qs[1] = 0.0;
    ob::ScopedState<> qg(space);
    qg[0] = 1.57;
    qg[1] = 1.57;

    ss.setStartAndGoalStates(qs, qg);

    // initialize PRM clean
    auto planner = std::make_shared<og::PRM>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // storage
    ob::PlannerDataStorage datastorage;

    ob::PlannerStatus solved = ss.solve(10.0);
    if (solved) {
        std::cout << "Found!" << std::endl;

        ss.getSolutionPath().print(std::cout);

        ob::PlannerData data(ss.getSpaceInformation());
        ss.getPlannerData(data);

    } else {
        std::cout << "No Solution!" << std::endl;
    }
    return 0;
}
