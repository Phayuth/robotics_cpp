#include <eigen3/Eigen/Dense>
#include <iostream>
#include <nlopt.hpp>
#include <vector>

double dt = 0.01;
double xinit = 1.0;
double yinit = 5.0;
double tinit = 2.0;

double xfinal = 5.0;
double yfinal = 1.0;

int N = 100;
int num_states = 2;

double ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad,
                         void *data) {
    std::vector<double> state{xinit, yinit, tinit};
    for (int i = 0; i < N; ++i) {
        // Update state based on control inputs
        double u1 = x[i * num_states];     // Control input for x
        double u2 = x[i * num_states + 1]; // Control input for y

        // Update state using a simple Euler integration
        state[0] += u1 * dt; // Update x position
        state[1] += u2 * dt; // Update y position
        state[2] += dt;      // Update time

        // Check if we reached the final state
        if (state[0] >= xfinal && state[1] <= yfinal) {
            break;
        }
    }
    return cost;
}
int main(int argc, char const *argv[]) {
    nlopt::opt opt(nlopt::LD_SLSQP, N * num_states);

    return 0;
}
