#include "stateupdater.h"

StateUpdater::StateUpdater(){}

MatrixXd StateUpdater::compute_Tc(const VectorXd& predicted_x,
                                  const VectorXd& predicted_z,
                                  const MatrixXd& sigma_x,
                                  const MatrixXd& sigma_z){

  int NZ = predicted_z.size();
  VectorXd dz;
  VectorXd dx;
  MatrixXd Tc = MatrixXd::Zero(NX, NZ);

  for(int c = 0; c < NSIGMA; c++){

    dx = sigma_x.col(c) - predicted_x;
    dx(3) = normalize(dx(3));

    dz = sigma_z.col(c) - predicted_z;
    
    if(NZ == NZ_RADAR){
      dz(1) = normalize(dz(1));
    }
    
    Tc += WEIGHTS[c] * dx * dz.transpose();
  }

  return Tc;
}

void StateUpdater::update(const VectorXd& z,
                          const MatrixXd& S, 
                          const MatrixXd& Tc,
                          const VectorXd& predicted_z, 
                          const VectorXd& predicted_x, 
                          const MatrixXd& predicted_P){

  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  VectorXd dz = z - predicted_z;
  if(dz.size() == NZ_RADAR){
    // yaw/phi in radians
    dz(1) = normalize(dz(1));
  }
  
  this->x = predicted_x + K * dz;
  this->P = predicted_P - K * S * K.transpose();
  this->nis = dz.transpose() * Si * dz;
}

void StateUpdater::process(const VectorXd& predicted_x,
                           const VectorXd& predicted_z, 
                           const VectorXd& z, 
                           const MatrixXd& S, 
                           const MatrixXd& predicted_P,
                           const MatrixXd& sigma_x, 
                           const MatrixXd& sigma_z){

  MatrixXd Tc = this->compute_Tc(predicted_x, predicted_z, sigma_x, sigma_z);
  this->update(z, S, Tc, predicted_z, predicted_x, predicted_P);
}

VectorXd StateUpdater::getx() const{
  return this->x;
}

MatrixXd StateUpdater::getP() const{
  return this->P;
}

double StateUpdater::get_nis() const{
  return this->nis;
}
