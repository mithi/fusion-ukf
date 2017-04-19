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
     VectorXd w = VectorXd(NSIGMA);
     MatrixXd R = MatrixXd(NZ_RADAR, NZ_RADAR);
     VectorXd z = VectorXd(NZ_RADAR);
     MatrixXd S = MatrixXd(NZ_RADAR, NZ_RADAR);
     MatrixXd sigma_z = MatrixXd(NZ_RADAR, NSIGMA);

  public:
    RadarPredictor(const VectorXd W);
    void process(const MatrixXd sigma_x);
    MatrixXd getS() const;
    VectorXd getz() const;
};

#endif //RADARPREDICTOR_H_
