#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "radarpredictor.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(){

/*********************************************************************************
 * TEST: Predicting Radar Measurements
 *********************************************************************************/

  int n_x = 5;
  int n_augmented = 7;
  int n_sigma = 2 * n_augmented + 1;
  double lambda = 3 - n_augmented;

  VectorXd w = VectorXd(n_sigma);
  w(0) = lambda / (lambda + double(n_augmented));
  double weight = 0.5 / (lambda + double(n_augmented));

  for( int i = 1; i < n_sigma; i++) {
    w(i) = weight;
  }

  MatrixXd predicted_sigma_x = MatrixXd(n_x, n_sigma);
  predicted_sigma_x <<
  5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
   1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
 0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  RadarPredictor radarPredictor(w);
  cout << "before radarPredictor processing " << endl;

  radarPredictor.process(predicted_sigma_x);
  cout << "predicted radar measurement vector z:" << endl;
  cout << radarPredictor.getz() << endl;
  cout << "predicted measurement covariance matrix S:" << endl;
  cout << radarPredictor.getS() << endl;

  return 0;
}
