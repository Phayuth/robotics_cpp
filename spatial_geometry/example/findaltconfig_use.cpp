#include "findaltconfig.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    Eigen::RowVectorXd qlimit(6);
    qlimit << 2 * M_PI, 2 * M_PI, M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI;
    std::cout << qlimit << std::endl;

    Eigen::RowVectorXd qgoal(6);
    qgoal << -1.12 + 2 * M_PI, -1.86, 1.87, 0.0, M_PI / 2, 0.0;

    Eigen::MatrixXd qalt = find_alt_config(qgoal, qlimit);
    printf("\nqalt \n");
    std::cout << qalt << std::endl;

    return 0;
}
