#ifndef TOOLS_H_
#define TOOLS_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "../src/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd calculate_RMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truths);
double normalize(const double a);

#endif /* TOOLS_H_ */
