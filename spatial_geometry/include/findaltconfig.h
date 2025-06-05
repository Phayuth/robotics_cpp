#ifndef FINDALTCONFIG_H
#define FINDALTCONFIG_H

#include <Eigen/Dense>
#include <vector>
double degrees_to_radians(double degrees);
double wrap_to_pi(double q);
Eigen::MatrixXd cartesian_product(const std::vector<double> &vec, int k);
Eigen::MatrixXd find_alt_config(Eigen::RowVectorXd q, Eigen::RowVectorXd qbound);

#endif