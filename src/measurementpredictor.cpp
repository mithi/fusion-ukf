#include "measurementpredictor.h"

MeasurementPredictor::MeasurementPredictor(){}

void MeasurementPredictor::initialize(const DataPointType sensor_type){

  this->current_type = sensor_type;

  if (this->current_type == DataPointType::RADAR){

    this->nz = NZ_RADAR;
    this->R = MatrixXd(this->nz, this->nz);
    this->R << VAR_RHO, 0, 0,
               0, VAR_PHI, 0,
               0, 0, VAR_RHODOT;

  } else if (this->current_type == DataPointType::LIDAR){

    this->nz = NZ_LIDAR;
    this->R = MatrixXd(this->nz, this->nz);
    this->R << VAR_PX, 0,
               0, VAR_PY;
  }
}

MatrixXd MeasurementPredictor::compute_sigma_z(const MatrixXd sigma_x){

  const double THRESH = 1e-4;
  double px, py, v, yaw, vx, vy, rho, phi, rhodot;

  MatrixXd sigma = MatrixXd(this->nz, NSIGMA);
  sigma.fill(0.0);

  for (int c = 0; c < NSIGMA; c++){

    if (this->current_type == DataPointType::RADAR){

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

    } else if (this->current_type == DataPointType::LIDAR){

      sigma(0, c) = sigma_x(0, c); //px
      sigma(1, c) = sigma_x(1, c); //py
    }
  }

  return sigma;
}

MatrixXd MeasurementPredictor::compute_z(const MatrixXd sigma){

  VectorXd z = VectorXd(this->nz);
  z.fill(0.0);

  for (int c = 0; c < NSIGMA; c++)
    cout << "WEIGHTS:" << WEIGHTS[c] << endl;

  for(int c = 0; c < NSIGMA; c++){
    z += WEIGHTS[c] * sigma.col(c);
  }

  return z;
}

MatrixXd MeasurementPredictor::compute_S(const MatrixXd sigma, const MatrixXd z){

  VectorXd dz;
  MatrixXd S = MatrixXd(this->nz, this->nz);
  S.fill(0.0);

  for (int c = 0; c < NSIGMA; c++){

    dz = sigma.col(c) - z;
    dz(1) = normalize(dz(1));

    S += WEIGHTS[c] * dz * dz.transpose();
  }

  S += this->R;
  return S;
}

void MeasurementPredictor::process(const MatrixXd sigma_x, const DataPointType sensor_type){

  // let the processor know whether it's processing a LIDAR or RADAR measurement
  this->initialize(sensor_type);

  // transform predicted sigma_x into measurement space
  this->sigma_z = this->compute_sigma_z(sigma_x);

  // get the mean predicted measurement vector z
  this->z = this->compute_z(this->sigma_z);

  // get the measurement covariance matrix S
  this->S = this->compute_S(this->sigma_z, this->z);
}

VectorXd MeasurementPredictor::getz() const {
  return this->z;
}

MatrixXd MeasurementPredictor::getS() const {
  return this->S;
}

MatrixXd MeasurementPredictor::get_sigma() const {
  return this->sigma_z;
}
