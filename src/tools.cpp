#include "tools.h"

using namespace std;

double normalize(const double a){
  return (fabs(a) > M_PI) ? remainder(a, 2. * M_PI) : a;
}

VectorXd calculate_RMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths){

  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  for(int k = 0; k < estimations.size(); ++k){
    VectorXd diff = estimations[k] - ground_truths[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}
