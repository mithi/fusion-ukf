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

  VectorXd WEIGHTS = VectorXd(NSIGMA);
  WEIGHTS << w0, w, w, w, w, w, w, w, w, w, w, w, w, w, w;

  StatePredictor statePredictor(WEIGHTS);
  RadarPredictor radarPredictor(WEIGHTS);
}
