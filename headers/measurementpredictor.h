#ifndef MEASUREMENTPREDICTOR_H_
#define MEASUREMENTPREDICTOR_H_

#include "../src/Eigen/Dense"
#include "tools.h"
#include "settings.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/******************************
 MeasurementPredictor is a class responsible for
 calculating the predicted measurement vector z, measurement covariance matrix S
 and sigma point matrix transformed to measurement space sigma_z
 based on a given predicted sigma points matrix sigma_x

 After calling the process() please call getS(), getz() and get_sigma()
 to get the corresponding calculated values
*******************************/

class MeasurementPredictor{

  private:
    int nz;
    DataPointType current_type;
    MatrixXd R;
    VectorXd z;
    MatrixXd S;
    MatrixXd sigma_z;

    // PRIVATE FUNCTIONS
    void initialize(const DataPointType sensor_type);
    MatrixXd compute_sigma_z(const MatrixXd& sigma_x);
    MatrixXd compute_z(const MatrixXd& sigma_z);
    MatrixXd compute_S(const MatrixXd& sigma_z, const MatrixXd& predicted_z);

  public:
    MeasurementPredictor();
    void process(const MatrixXd& sigma_x, const DataPointType sensor_type);
    MatrixXd get_sigma() const;
    MatrixXd getS() const;
    VectorXd getz() const;
};

#endif //MEASUREMENTPREDICTOR_H_
