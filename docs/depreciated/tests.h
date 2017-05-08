#ifndef TESTS_H_
#define TESTS_H_

#include <stdlib.h>
#include "radarpredictor.h"
#include "measurementpredictor.h"
#include "statepredictor.h"
#include "stateupdater.h"
#include "settings.h"
#include "../src/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool test_compute_augmented_sigma();
bool test_predict_sigma_x();
bool test_predict_x_P();
bool radar_predictor_process_test();
bool measurement_predictor_process_test();
bool state_updater_test();
void all_tests();

#endif /* TESTS_H_ */
