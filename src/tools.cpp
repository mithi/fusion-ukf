#include "tools.h"

double normalize(const double a){

  double b = a;

  while (b > M_PI){
    b -= 2. * M_PI;
  }

  while (b < -M_PI){
    b += 2. * M_PI;
  }

  return b;
}

VectorXd calculate_RMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths){

  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  for (int k = 0; k < estimations.size(); k++){

    VectorXd diff = estimations[k] - ground_truths[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}
