#ifndef RADARPREDICTOR_H_
#define RADARPREDICTOR_H_

#include "../src/Eigen/Dense"
#include "tools.h"
#include "settings.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/******************************
 RadarPredictor is a class responsible for
 calculating the predicted measurement vector z, measurement covariance matrix S
 and sigma point matrix transformed to measurement space sigma_z
 based on a given predicted sigma points matrix sigma_x

 After calling the process() please call getS(), getz() and get_sigma()
 to get the corresponding calculated values
*******************************/

class RadarPredictor{

  private:
    MatrixXd R = MatrixXd(NZ_RADAR, NZ_RADAR);
    VectorXd z = VectorXd(NZ_RADAR);
    MatrixXd S = MatrixXd(NZ_RADAR, NZ_RADAR);
    MatrixXd sigma_z = MatrixXd(NZ_RADAR, NSIGMA);

  public:
    RadarPredictor();
    void process(const MatrixXd sigma_x);
    MatrixXd get_sigma() const;
    MatrixXd getS() const;
    VectorXd getz() const;

    // PRIVATE FUNCTIONS
    MatrixXd compute_sigma_z(const MatrixXd sigma_x);
    MatrixXd compute_z(const MatrixXd sigma_z);
    MatrixXd compute_S(const MatrixXd sigma_z, const MatrixXd predicted_z);
};

#endif //RADARPREDICTOR_H_
