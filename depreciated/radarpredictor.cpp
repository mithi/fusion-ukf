#include "radarpredictor.h"

RadarPredictor::RadarPredictor(){

  this->R << VAR_RHO, 0, 0,
             0, VAR_PHI, 0,
             0, 0, VAR_RHODOT;
}

MatrixXd RadarPredictor::compute_sigma_z(const MatrixXd sigma_x){

  double THRESH = 1e-4;
  double px, py, v, yaw, vx, vy, rho, phi, rhodot;

  MatrixXd sigma = MatrixXd(NZ_RADAR, NSIGMA);
  sigma.fill(0.0);

  for (int c = 0; c < NSIGMA; c++){

    px = sigma_x(0, c);
    py = sigma_x(1, c);
    v = sigma_x(2, c);
    yaw = sigma_x(3, c);

    vx = cos(yaw) * v;
    vy = sin(yaw) * v;

    rho = sqrt(px * px + py * py);
    phi = atan2(py, px);
    rhodot = (rho > THRESH) ? ((px * vx + py * vy) / rho) : 0.0; // avoid division by zero

    sigma(0, c) = rho;
    sigma(1, c) = phi;
    sigma(2, c) = rhodot;
  }

  return sigma;
}

MatrixXd RadarPredictor::compute_z(const MatrixXd sigma){

  VectorXd z = VectorXd(NZ_RADAR);
  z.fill(0.0);

  for(int c = 0; c < NSIGMA; c++){
    z += WEIGHTS[c] * sigma.col(c);
  }

  return z;
}

MatrixXd RadarPredictor::compute_S(const MatrixXd sigma, const MatrixXd z){

  VectorXd dz = VectorXd(NZ_RADAR);
  MatrixXd S = MatrixXd(NZ_RADAR, NZ_RADAR);
  S.fill(0.0);

  for (int c = 0; c < NSIGMA; c++){

    dz = sigma.col(c) - z;
    dz(1) = normalize(dz(1));

    S += WEIGHTS[c] * dz * dz.transpose();
  }

  S += this->R;
  return S;
}

void RadarPredictor::process(const MatrixXd sigma_x){

  // transform predicted sigma_x into measurement space
  this->sigma_z = this->compute_sigma_z(sigma_x);
  // get the mean predicted measurement vector z
  this->z = this->compute_z(this->sigma_z);
  // get the measurement covariance matrix S
  this->S = this->compute_S(this->sigma_z, this->z);
}

VectorXd RadarPredictor::getz() const {
  return this->z;
}

MatrixXd RadarPredictor::getS() const {
  return this->S;
}

MatrixXd RadarPredictor::get_sigma() const {
  return this->sigma_z;
}
