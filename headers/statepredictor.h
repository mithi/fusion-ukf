#ifndef STATEPREDICTOR_H_
#define STATEPREDICTOR_H_

#include "../src/Eigen/Dense"
#include "tools.h"
#include "settings.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/******************************
 StatePredictor is a class responsible for calculating the predicted sigma,
 predicted state vector x, and predicted state covariance matrix P
 based the current state vector x, current state covariance matrix P
 and time passed dt

 After calling the process(),
 please call getP(), getx() and get_sigma()
 to get the corresponding calculated values
*******************************/

class StatePredictor{

  private:
    MatrixXd sigma = MatrixXd(NX, NSIGMA); //predicted sigma points
    VectorXd x = VectorXd(NX); // predicted state vector
    MatrixXd P = MatrixXd(NX, NX); // predicted state covariance matrix

    //PRIVATE FUNCTIONS
    MatrixXd compute_augmented_sigma(const VectorXd& current_x, const MatrixXd& current_P);
    MatrixXd predict_sigma(const MatrixXd& augmented_sigma, double dt);
    VectorXd predict_x(const MatrixXd& predicted_sigma);
    MatrixXd predict_P(const MatrixXd& predicted_sigma, const VectorXd& predicted_x);

  public:
    StatePredictor();
    void process(VectorXd& current_x, MatrixXd& current_P, double dt);
    MatrixXd get_sigma() const;
    VectorXd getx() const;
    MatrixXd getP() const;
};

#endif //STATEPREDICTOR_H_
