#ifndef RADARPREDICTOR_H_
#define RADARPREDICTOR_H_

#include "../src/Eigen/Dense"
#include "tools.h"
#include <stdlib.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class RadarPredictor{
   /******************************
    RadarPredictor is a class responsible for
    calculating the predicted_z and the measurement covariance matrix S
    based on a given predicted sigma points matrix sigma_x

    After calling the process() please call getS() and getz()
    to get the corresponding calculated values
   *******************************/

   private:
     const int nz = 3; // number of measurements
     const int nx = 5; // number of states
     const int nsigma = (nx + 2) * 2 + 1; // number of sigma points

     //measurements noise standard deviations
     const double std_rho = 0.3; // meters
     const double std_phi = 0.0175; // radians
     const double std_rhodot = 0.1; // meters / second

     //measurements noise variances
     const double var_rho = std_rho * std_rho;
     const double var_phi = std_phi * std_phi;
     const double var_rhodot = std_rhodot * std_rhodot;

     VectorXd w = VectorXd(nsigma); // weights for the mean measurement prediction
     MatrixXd R = MatrixXd(nz, nz); // noise covariance matrix
     VectorXd z = VectorXd(nz); // mean predicted measurement
     MatrixXd S = MatrixXd(nz, nz); // measurement covariance matrix

     // PRIVATE FUNCTIONS
     MatrixXd compute_sigma_z(const MatrixXd sigma_x);
     MatrixXd compute_z(const MatrixXd sigma_z);
     MatrixXd compute_S(const MatrixXd sigma_z, const MatrixXd predicted_z);

  public:
    RadarPredictor(const VectorXd w);
    void process(const MatrixXd sigma_x);
    MatrixXd getS() const;
    MatrixXd getz() const;
};

#endif //RADARPREDICTOR_H_
