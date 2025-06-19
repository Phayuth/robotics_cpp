#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <nlopt.hpp>
#include <vector>

double dt = 0.01;

double xinit = 1.0;
double yinit = 5.0;
double thinit = 2.0;

double xfinal = 5.0;
double yfinal = 10.0;
double thfinal = 0.0;

int N = 100;
int num_states = 2;

void diffdrive_model(double &x, double &y, double &th, double u1, double u2) {
    x += u1 * cos(th) * dt;
    y += u1 * sin(th) * dt;
    th += u2 * dt;
}

void final_state(const std::vector<double> &x, double &xfinal, double &yfinal,
                 double &thfinal) {

    xfinal = xinit;
    yfinal = yinit;
    thfinal = thinit;

    for (int i = 0; i < N; i++) {
        double u1 = x[i * num_states];
        double u2 = x[i * num_states + 1];
        diffdrive_model(xfinal, yfinal, thfinal, u1, u2);
    }
}
double ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad,
                         void *data) {
    double xfinal_opt, yfinal_opt, thfinal_opt;
    final_state(x, xfinal_opt, yfinal_opt, thfinal_opt);
    double cost = sqrt(pow(xfinal_opt - xfinal, 2) + pow(yfinal_opt - yfinal, 2));
    return cost;
}

int main(int argc, char const *argv[]) {
    nlopt::opt opt(nlopt::LN_COBYLA, N * num_states);

    // options
    opt.set_min_objective(ObjectiveFunction, nullptr);
    opt.set_xtol_rel(1e-6);
    // opt.set_maxeval(1000);
    // std::vector<double> lb(N * num_states, -1.0); // lower bounds
    // std::vector<double> ub(N * num_states, 1.0);  // upper bounds
    // opt.set_lower_bounds(lb);
    // opt.set_upper_bounds(ub);

    // Initial guess
    std::vector<double> x(N * num_states);
    for (int i = 0; i < N; i++) {
        x[i * num_states] = 0.0;     // u1
        x[i * num_states + 1] = 0.0; // u2
    }

    double minf;
    nlopt::result result = opt.optimize(x, minf);

    std::cout << "Found minimum with cost = " << minf << std::endl;

    // Save optimized input vector to file
    std::ofstream fout("nlopt_shooting_diffdrive.txt");
    for (int i = 0; i < N; i++) {
        fout << x[i * num_states] << "," << x[i * num_states + 1] << std::endl;
    }
    fout.close();

    return 0;
}
