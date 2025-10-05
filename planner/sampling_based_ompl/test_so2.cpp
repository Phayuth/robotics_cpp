#include <iostream>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main() {
    // auto space = std::make_shared<ob::CompoundStateSpace>();

    // // Add two SO2 spaces to represent two joints
    // space->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0); // First
    // joint space->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0); //
    // Second joint

    // ob::ScopedState<ob::CompoundStateSpace> start(space);
    // start[0] = 0.0;
    // start[1] = 0.0;

    // ob::ScopedState<ob::CompoundStateSpace> goal(space);
    // goal[0] = M_PI_2;
    // goal[1] = M_PI_2;

    // double d = start.distance(goal);
    // printf("\ndistance is %f\n", d);

    // ob::State *interpolatedState = space->allocState();
    // double eta = 0.3;
    // space->interpolate(start.get(), goal.get(), eta, interpolatedState);
    // std::cout << interpolatedState << std::endl;
    // const auto *compoundState = goal->as<ob::CompoundStateSpace>();

    // Extract values from the interpolated state
    // const auto *compoundState = interpolatedState->as<ob::CompoundState>();
    // const auto *so2State1 =
    // compoundState->components[0]->as<ob::SO2StateSpace>(); const auto *so2State2
    // = compoundState->components[1]->as<ob::SO2StateSpace>(); Get the values of
    // the angles auto angle1 = so2State1->as<ob::SO2StateSpace>(); auto angle2 =
    // so2State1->as<ob::SO2StateSpace>(); std::cout << "Interpolated angles: " <<
    // so2State1 << ", " << so2State2 << std::endl;
    // space->freeState(interpolatedState);

    // Define a compound state space for a 2-joint SO2 robot
    ob::StateSpacePtr space(new ob::CompoundStateSpace());
    space->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0); // 1st SO2 joint
    space->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0); // 2nd SO2 joint

    // Allocate space for a state
    ob::State *state = space->allocState();
    // state[0] = M_PI_2;
    // state[1] = M_PI_4;
    // state->as<ob::SO2StateSpace>();

    // Assuming the state is already set (during planning), you can access the SO2
    // values like this:
    auto *compound_state = state->as<ob::CompoundState>();
    compound_state->as<ob::SO2StateSpace::StateType>(0)->value =
        1.57; // Joint 1: 90 degrees (π/2 radians)
    compound_state->as<ob::SO2StateSpace::StateType>(1)->value =
        3.14; // Joint 2: 180 degrees (π radians)

    // Access the values of the first and second SO2 subspaces
    double joint1_value =
        compound_state->as<ob::SO2StateSpace::StateType>(0)->value;
    double joint2_value =
        compound_state->as<ob::SO2StateSpace::StateType>(1)->value;

    std::cout << "Joint 1 value: " << joint1_value << std::endl;
    std::cout << "Joint 2 value: " << joint2_value << std::endl;

    // Free the allocated state when done
    space->freeState(state);

    return 0;
}
