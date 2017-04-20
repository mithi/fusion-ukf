#include "fusionukf.h"

FusionUKF::FusionUKF(){
  this->initialized = false;
}

void FusionUKF::initialize(const DataPoint data){
  this->x = data.get_state();
  this->P = MatrixXd::Identity(NX, NX);
  this->initialized = true;
}

void FusionUKF::update(const DataPoint data){

  VectorXd predicted_z;
  MatrixXd sigma_x;
  MatrixXd sigma_z;
  MatrixXd S;

  // get the time difference in seconds
  double dt = (double(data.get_timestamp()) - double(this->timestamp)) / 1000000.0;

  // state prediction
  // get predicted state and covariance of predicted state, predicted sigma points in state space
  this->statePredictor.process(this->x, this->P, dt);
  this->x = this->statePredictor.getx();
  this->P = this->statePredictor.getP();
  sigma_x = this->statePredictor.get_sigma();

  // measurement prediction
  // get predicted measurement, covariance of predicted measurement, predicted sigma points in measurement space
  this->measurementPredictor.process(sigma_x, data.get_type());
  predicted_z = this->measurementPredictor.getz();
  S = this->measurementPredictor.getS();
  sigma_z = this->measurementPredictor.get_sigma();

  // state update
  // updated the state and covariance of state... also get the nis
  this->stateUpdater.process(this->x, predicted_z, data.get(), S, P, sigma_x, sigma_z);
  this->x = this->stateUpdater.getx();
  this->P  = this->stateUpdater.getP();
  this->nis = this->stateUpdater.get_nis();

  // update timestamp
  this->timestamp = data.get_timestamp();
}

void FusionUKF::process(const DataPoint data){
  this->initialized ? this->update(data) : this->initialize(data);
}

VectorXd FusionUKF::get() const {
  return this->x;
}

double FusionUKF::get_nis() const {
  return this->nis;
}
