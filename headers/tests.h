#ifndef TESTS_H_
#define TESTS_H_

#include <stdlib.h>
#include "radarpredictor.h"
#include "statepredictor.h"
#include "../src/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool test_compute_augmented_sigma();
bool test_predict_sigma_x();
bool test_predict_x_P();
bool radar_predictor_process_test();


#endif /* TESTS_H_ */
