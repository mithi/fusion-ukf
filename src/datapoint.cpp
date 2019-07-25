#include "datapoint.h"

using namespace std;

DataPoint::DataPoint(){
  this->initialized = false;
}

DataPoint:: DataPoint(const long long timestamp, const DataPointType& data_type, const VectorXd& raw){
  this->set(timestamp, data_type, raw);
}

void DataPoint::set(const long long timestamp, const DataPointType& data_type, const VectorXd& raw){
  this->timestamp = timestamp;
  this->data_type = data_type;
  this->raw = raw;
  this->initialized = true;
}

VectorXd DataPoint::get() const{
  return this->raw;
}

VectorXd DataPoint::get_state() const{

  VectorXd state(NX);

  if(this->data_type == DataPointType::LIDAR){

    double px = this->raw(0);
    double py = this->raw(1);
    state << px, py, 0.0, 0.0, 0.0;

  }else if(this->data_type == DataPointType::RADAR){

    double rho = this->raw(0);
    double phi = this->raw(1);

    double px = rho * cos(phi);
    double py = rho * sin(phi);
    state << px, py, 0.0, 0.0, 0.0;

  }else if(this->data_type == DataPointType::STATE){
    state = this->raw;

  }else if(this->data_type == DataPointType::TRUTH){

    double px = this->raw(0);
    double py = this->raw(1);
    double vx = this->raw(2);
    double vy = this->raw(3);

    double v = sqrt(vx * vx + vy * vy);
    double yaw = atan2(vy, vx);
    state << px, py, v, yaw, 0.0;
  }

  return state;
}

VectorXd DataPoint::get_vec() const{

  VectorXd vec(NX - 1);

  if(this->data_type == DataPointType::LIDAR){

    double px = this->raw(0);
    double py = this->raw(1);
    vec << px, py, 0.0, 0.0;

  }else if(this->data_type == DataPointType::RADAR){

    double rho = this->raw(0);
    double phi = this->raw(1);
    double px = rho * cos(phi);
    double py = rho * sin(phi);

    vec << px, py, 0.0, 0.0;

  }else if(this->data_type == DataPointType::STATE){

    double px = this->raw(0);
    double py = this->raw(1);
    double v = this->raw(3);
    double yaw = this->raw(4);

    double vx = v * cos(yaw);
    double vy = v * sin(yaw);

    vec << px, py, vx, vy;

  }else if(this->data_type == DataPointType::TRUTH){

    vec = this->raw;
  }

  return vec;
}


long long DataPoint::get_timestamp() const{
  return this->timestamp;
}

DataPointType DataPoint::get_type() const{
  return this->data_type;
}

void DataPoint::print() const{

  if(this->initialized){

    cout << "Timestamp: " << this->timestamp << endl;
    cout << "Sensor ID: " << static_cast<int>(this->data_type) << " (LIDAR = 0 | RADAR = 1 | STATE = 2) " << endl;
    cout << "Raw Data: " << endl;
    cout << this->raw << endl;

  }else{

    cout << "DataPoint is not initialized." << endl;
  }
}
