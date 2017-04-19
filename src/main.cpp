#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "settings.h"
#include "radarpredictor.h"
#include "statepredictor.h"
#include "tests.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(){
  cout << "ALL TESTS SHOULD RETURN 1" << endl;
  cout << "- StatePredictor Tests" << endl;
  cout << "--- compute_augmented_sigma(): "<< test_compute_augmented_sigma() << endl;
  cout << "--- predict_sigma(): "<< test_predict_sigma_x() << endl;
  cout << "--- predict_x_P(): "<< test_predict_x_P() << endl;
  cout << "- RadarPredictor Tests" << endl;
  cout << "--- radar_predictor_process_test(): "<< radar_predictor_process_test() << endl;

  return 0;
}
