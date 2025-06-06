#include <iostream>
#include <nlopt.hpp>

// Objective function: (x - 2)^2
double objectiveFunction(const std::vector<double> &x, std::vector<double> &grad,
                         void *data) {
    if (!grad.empty()) {
        grad[0] = 2 * (x[0] - 2);
    }
    return (x[0] - 2) * (x[0] - 2);
}

void opt_single_variable() {
    nlopt::opt opt(nlopt::LD_MMA, 1); // 1D optimization

    // options
    opt.set_min_objective(objectiveFunction, nullptr);
    opt.set_xtol_rel(1e-6);

    // Initial guess
    std::vector<double> x(1);
    x[0] = 5.0;

    double minf; // Store the minimum value
    nlopt::result result = opt.optimize(x, minf);

    std::cout << "Found minimum at x = " << x[0] << ", f(x) = " << minf
              << std::endl;
}

double multiObjectiveFunction(const std::vector<double> &x,
                              std::vector<double> &grad, void *data) {
    if (!grad.empty()) {
        grad[0] = 2 * (x[0] - 1); // ∂f/∂x
        grad[1] = 2 * (x[1] + 2); // ∂f/∂y
    }
    return (x[0] - 1) * (x[0] - 1) + (x[1] + 2) * (x[1] + 2);
}

void opt_multi_variable() {
    nlopt::opt opt(nlopt::LD_MMA, 2); // 2D optimization

    // options
    opt.set_min_objective(multiObjectiveFunction, nullptr);
    opt.set_xtol_rel(1e-6);

    // Initial guess
    std::vector<double> x(2);
    x[0] = 5.0;
    x[1] = 5.0;
    double minf; // Store the minimum value
    nlopt::result result = opt.optimize(x, minf);

    std::cout << "Found minimum at x = " << x[0] << ", y = " << x[1]
              << ", f(x, y) = " << minf << std::endl;
}
int main() {
    while (true) {
        std::cout << "\n\n\n\n---- Optimization Example ----\n"
                  << "Choose an option:\n"
                  << "1. Optimize single variable function\n"
                  << "2. Optimize multi-variable function\n"
                  << "0. Exit\n"
                  << "------------------------------\n"
                  << "Enter your choice: ";
        std::string choice;
        std::cin >> choice;
        std::cout << std::endl;

        if (choice == "1") {
            opt_single_variable();
        } else if (choice == "2") {
            opt_multi_variable();
        } else if (choice == "0") {
            std::cout << "Exiting." << std::endl;
            break;
        } else {
            std::cout << "Invalid choice. Exiting." << std::endl;
            break;
        }
    }
    return 0;
}
