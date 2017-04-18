#ifndef RADARPREDICTOR_H_
#define RADARPREDICTOR_H_

#include <stdlib.h>
#include "../src/Eigen/Dense"
#include "tools.h"
#include "settings.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/******************************
 RadarPredictor is a class responsible for
 calculating the predicted measurement vector z
 and measurement covariance matrix S
 based on a given predicted sigma points matrix sigma_x

 After calling the process() giving the predicted_sigma
 please call getS() and getz() to get the corresponding calculated values
*******************************/

class RadarPredictor{

   private:
     VectorXd w = VectorXd(NSIGMA); // weights for the mean measurement prediction
     MatrixXd R = MatrixXd(NZ_RADAR, NZ_RADAR); // noise covariance matrix
     VectorXd z = VectorXd(NZ_RADAR); // mean predicted measurement
     MatrixXd S = MatrixXd(NZ_RADAR, NZ_RADAR); // measurement covariance matrix

     // PRIVATE FUNCTIONS
     MatrixXd compute_sigma_z(const MatrixXd sigma_x);
     MatrixXd compute_z(const MatrixXd sigma_z);
     MatrixXd compute_S(const MatrixXd sigma_z, const MatrixXd predicted_z);

  public:
    RadarPredictor(const VectorXd W);
    void process(const MatrixXd sigma_x);
    MatrixXd getS() const;
    VectorXd getz() const;
};

#endif //RADARPREDICTOR_H_
