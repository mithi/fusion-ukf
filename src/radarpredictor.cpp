#include "radarpredictor.h"

RadarPredictor::RadarPredictor(const VectorXd w){

  this->R << this->var_rho, 0, 0,
             0, this->var_phi, 0,
             0, 0, this->var_rhodot;

  this->w = w;

}

MatrixXd RadarPredictor::calculate_sigma_z(const MatrixXd sigma_x){

  double px, py, v, yaw, vx, vy, rho, phi, rho_dot;
  MatrixXd sigma_z = MatrixXd(this->nz, this->nsigma);
  sigma_z.fill(0.0);

  for (int c = 0; c < this->nsigma; c++){

    px = sigma_x(0, c);
    py = sigma_x(1, c);
    v = sigma_x(2, c);
    yaw = sigma_x(3, c);

    vx = cos(yaw) * v;
    vy = sin(yaw) * v;

    rho = sqrt(px * px + py * py);
    phi = atan2(py, px);
    rho_dot = (px * vx + py * vy) / rho;

    sigma_z(0, c) = rho;
    sigma_z(1, c) = phi;
    sigma_z(2, c) = rho_dot;
  }

  return sigma_z;
}

MatrixXd RadarPredictor::calculate_z(const MatrixXd sigma_z){

  VectorXd z = VectorXd(this->nz);
  z.fill(0.0);

  for(int c = 0; c < this->nsigma; c++){
    z += this->w(c) * sigma_z.col(c);
    cout << z << endl;
  }

  return z;
}

MatrixXd RadarPredictor::calculate_S(const MatrixXd sigma_z, const MatrixXd z){

  VectorXd dz = VectorXd(this->nz);
  MatrixXd S = MatrixXd(this->nz, this->nz);
  S.fill(0.0);

  for (int c = 0; c < this->nsigma; c++){

    dz = sigma_z.col(c) - z;
    dz(1) = normalize(dz(1));

    S += this->w(c) * dz * dz.transpose();
  }

  S += this->R;
  return S;
}

void RadarPredictor::process(const MatrixXd sigma_x){

  // transform predicted sigma_x into measurement space
  MatrixXd sigma_z = this->calculate_sigma_z(sigma_x);

  // get the mean predicted measurement vector z
  this->z = this->calculate_z(sigma_z);

  // get the measurement covariance matrix S
  this->S = this->calculate_S(sigma_z, this->z);
}

MatrixXd RadarPredictor::getz() const {
  return this->z;
}
MatrixXd RadarPredictor::getS() const {
  return this->S;
}
