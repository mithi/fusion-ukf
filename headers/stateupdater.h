#ifndef STATEUPDATER_H_
#define STATEUPDATER_H_

#include "../src/Eigen/Dense"
#include "settings.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/******************************
 StateUpdater is a class responsible for updating the
 state vector x, and state covariance matrix P (and the current nis)
 based on the following:
 - the current incoming measurement vector z
 - the measurement covariance matrix S
 - the predicted state covariance P
 - the predicted state vector x
 - the predicted measurement vector z
 - the predicted sigma points in state space sigma_x
 - the predicted sigma point in measurement space sigma_z

 After calling the process()
 please call getx() and getP() (and get_nis())
 to get the corresponding calculated values
*******************************/

class StateUpdater{

  private:
    MatrixXd x;
    MatrixXd P;
    double nis;

    // PRIVATE FUNCTIONS
    MatrixXd compute_Tc(const VectorXd& predicted_x,
                        const VectorXd& predicted_z,
                        const MatrixXd& sigma_x,
                        const MatrixXd& sigma_z);
    void update(const VectorXd& z,
                const MatrixXd& S,
                const MatrixXd& Tc,
                const VectorXd& predicted_z,
                const VectorXd& predicted_x,
                const MatrixXd& predicted_P);

  public:
    StateUpdater();
    void process(const VectorXd& predicted_x,
                 const VectorXd& predicted_z,
                 const VectorXd& z,
                 const MatrixXd& S,
                 const MatrixXd& predicted_P,
                 const MatrixXd& sigma_x,
                 const MatrixXd& sigma_z);
    VectorXd getx() const;
    MatrixXd getP() const;
    double get_nis() const;
};

#endif /* STATEUPDATER_H_ */
