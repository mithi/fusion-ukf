#ifndef STATEUPDATER_H_
#define STATEUPDATER_H_

#include "../src/Eigen/Dense"
#include "settings.h"
#include "tools.h"
#include <stdlib.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class StateUpdater{

  private:
    VectorXd w = VectorXd(NSIGMA);
    MatrixXd x;
    MatrixXd P;

  public:
    StateUpdater(const VectorXd W);
    void process(const VectorXd z,
                 const MatrixXd S,
                 const MatrixXd predicted_P,
                 const VectorXd predicted_x,
                 const VectorXd predicted_z,
                 const MatrixXd sigma_x,
                 const MatrixXd sigma_z);
    VectorXd getx() const;
    MatrixXd getP() const;

    // PRIVATE FUNCTIONS
    MatrixXd compute_Tc(const VectorXd predicted_x,
                        const VectorXd predicted_z,
                        const MatrixXd sigma_x,
                        const MatrixXd sigma_z);
    void update(const VectorXd z,
                const MatrixXd S,
                const MatrixXd Tc,
                const VectorXd predicted_z,
                const VectorXd predicted_x,
                const MatrixXd predicted_P);
};

#endif /* STATEUPDATER_H_ */
