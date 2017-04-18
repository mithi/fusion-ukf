#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "settings.h"
#include "radarpredictor.h"
#include "statepredictor.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void tests();

int main(){

  tests();
  return 0;
}

void tests(){
  VectorXd W = VectorXd(NSIGMA);
  W(0) = LAMBDA / (LAMBDA + double(NAUGMENTED));
  const double WEIGHT = 0.5 / (LAMBDA + double(NAUGMENTED));

  for( int i = 1; i < NSIGMA; i++) {
    W(i) = WEIGHT;
  }

   cout << "----------------------------------------------------------" << endl;
   cout << " STATE PREDICTOR TEST" << endl;
   cout << "----------------------------------------------------------" << endl;

   VectorXd x = VectorXd(NX);
   x << 5.7441,
        1.3800,
        2.2049,
        0.5015,
        0.3528;

  MatrixXd P = MatrixXd(NX, NX);
  P <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
       -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
        0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
       -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
       -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  cout << W << endl;
  StatePredictor statePredictor(W);
  //statePredictor.process(x, P, 0.1);
  MatrixXd predicted_sigma_x = MatrixXd(NX, NSIGMA);
  predicted_sigma_x <<
  5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
   1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
 0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  cout << "x" << endl;
  VectorXd new_x = statePredictor.predict_x(predicted_sigma_x);
  cout << new_x << endl;
  cout << "P" << endl;
  cout << statePredictor.predict_P(predicted_sigma_x, new_x) << endl;

  cout << "----------------------------------------------------------" << endl;
  cout << " RADAR PREDICTOR TEST" << endl;
  cout << "----------------------------------------------------------" << endl;


  RadarPredictor radarPredictor(W);

  radarPredictor.process(predicted_sigma_x);
  cout << "predicted radar measurement vector z:" << endl;
  cout << radarPredictor.getz() << endl;
  cout << "predicted measurement covariance matrix S:" << endl;
  cout << radarPredictor.getS() << endl;

}
