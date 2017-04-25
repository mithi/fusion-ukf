#include "statepredictor.h"

StatePredictor::StatePredictor(){}

MatrixXd StatePredictor::compute_augmented_sigma(const VectorXd current_x, const MatrixXd current_P){

  MatrixXd augmented_sigma = MatrixXd::Zero(NAUGMENTED, NSIGMA);
  VectorXd augmented_x = VectorXd::Zero(NAUGMENTED);
  MatrixXd augmented_P = MatrixXd::Zero(NAUGMENTED, NAUGMENTED);

  augmented_x.head(NX) = current_x;

  augmented_P.topLeftCorner(NX, NX) = current_P;
  augmented_P(NX, NX) = VAR_SPEED_NOISE;
  augmented_P(NX + 1, NX + 1) = VAR_YAWRATE_NOISE;

  MatrixXd L = augmented_P.llt().matrixL();
  augmented_sigma.col(0) = augmented_x;

  for (int c = 0; c < NAUGMENTED; c++){
    int i = c + 1;
    augmented_sigma.col(i)  = augmented_x + SCALE * L.col(c);
    augmented_sigma.col(i + NAUGMENTED)  = augmented_x - SCALE * L.col(c);
  }

  return augmented_sigma;
}

MatrixXd StatePredictor::predict_sigma(const MatrixXd augmented_sigma, double dt){

  double THRESH = 0.001;
  double px, py, speed, yaw, yawrate, speed_noise, yawrate_noise;
  double p_px, p_py, p_speed, p_yaw, p_yawrate, p_speed_noise, p_yawrate_noise;

  MatrixXd predicted_sigma = MatrixXd(NX, NSIGMA);

  for (int c = 0; c < NSIGMA; c++){

   /*************************************
    * Get the current state
    *************************************/
    px = augmented_sigma(0, c);
    py = augmented_sigma(1, c);
    speed = augmented_sigma(2, c);
    yaw = augmented_sigma(3, c);
    yawrate = augmented_sigma(4, c);
    speed_noise = augmented_sigma(5, c);
    yawrate_noise = augmented_sigma(6, c);

   /*************************************
    * predict the next state
    *************************************/
    p_speed = speed; // constant
    p_yaw = yaw + yawrate * dt;
    p_yawrate = yawrate; // constant

    if (fabs(yawrate) <= THRESH){
      // moving straight
      p_px = px + speed * dt * cos(yaw);
      p_py = py + speed * dt * sin(yaw);

    } else {

      double theta = yaw + yawrate * dt;
      p_px = px + speed / yawrate * (sin(theta) - sin(yaw));
      p_py = py + speed / yawrate * (cos(yaw) - cos(theta));
    }

   /*************************************
    * Add noise to the predicted state
    *************************************/
    double dt2 = dt * dt;
    double p_noise = 0.5 * speed_noise * dt2;
    double y_noise = 0.5 * yawrate_noise * dt2;

    p_px += p_noise * cos(yaw);
    p_py += p_noise * sin(yaw);
    p_speed += speed_noise * dt;
    p_yaw += y_noise;
    p_yawrate += yawrate_noise * dt;

   /*************************************
    * Write the prediction to the appropriate column
    *************************************/
    predicted_sigma(0, c) = p_px;
    predicted_sigma(1, c) = p_py;
    predicted_sigma(2, c) = p_speed;
    predicted_sigma(3, c) = p_yaw;
    predicted_sigma(4, c) = p_yawrate;
  }

  return predicted_sigma;
}

VectorXd StatePredictor::predict_x(const MatrixXd predicted_sigma){

  VectorXd predicted_x = VectorXd::Zero(NX);

  for (int c = 0; c < NSIGMA; c++){
    predicted_x += WEIGHTS[c] * predicted_sigma.col(c);
  }

  return predicted_x;
}

MatrixXd StatePredictor::predict_P(const MatrixXd predicted_sigma, const VectorXd predicted_x){


  MatrixXd predicted_P = MatrixXd::Zero(NX, NX);

  VectorXd dx = VectorXd(NX);

  for (int c = 0; c < NSIGMA; c++){

    dx = predicted_sigma.col(c) - predicted_x;
    dx(3) = normalize(dx(3));
    predicted_P += WEIGHTS[c] * dx * dx.transpose();
  }

  return predicted_P;
}

void StatePredictor::process(VectorXd current_x, MatrixXd current_P, double dt){

  MatrixXd augmented_sigma = compute_augmented_sigma(current_x, current_P);
  this->sigma = predict_sigma(augmented_sigma, dt);
  this->x = predict_x(this->sigma);
  this->P = predict_P(this->sigma, this->x);
}

MatrixXd StatePredictor::getP() const{
  return this->P;
}

MatrixXd StatePredictor::get_sigma() const{
  return this->sigma;
}

VectorXd StatePredictor::getx() const{
  return this->x;
}
