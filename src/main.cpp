#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>

#include "Eigen/Dense"
#include "settings.h"
#include "tools.h"
#include "usagecheck.h"
#include "datapoint.h"
#include "fusionukf.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {

  /*******************************************************************
   * CHECK IF CORRECTLY EXECUTED BY USER
   *******************************************************************/
  check_arguments(argc, argv);

  string in_filename = argv[1];
  string out_filename = argv[2];

  ifstream in_file(in_filename.c_str(), ifstream::in);
  ofstream out_file(out_filename.c_str(), ofstream::out);

  check_files(in_file, in_filename, out_file, out_filename);

  /*******************************************************************
   * READ DATA FROM FILE AND STORE IN MEMORY
   *******************************************************************/
  vector<DataPoint> all_sensor_data;
  vector<DataPoint> all_truth_data;

  double val1, val2, val3;
  double x, y, vx, vy, v, yaw, yawrate;
  long long timestamp;
  string sensor_id;

  string line;

  while(getline(in_file, line)){

    istringstream iss(line);
    DataPoint sensor_data;
    DataPoint truth_data;

    iss >> sensor_id;

    if(sensor_id.compare("L") == 0){

      iss >> val1;
      iss >> val2;
      iss >> timestamp;

      VectorXd lidar_vec(NZ_LIDAR);
      lidar_vec << val1, val2;
      sensor_data.set(timestamp, DataPointType::LIDAR, lidar_vec);

    }else if(sensor_id.compare("R") == 0){

      iss >> val1;
      iss >> val2;
      iss >> val3;
      iss >> timestamp;

      VectorXd radar_vec(NZ_RADAR);
      radar_vec << val1, val2, val3;
      sensor_data.set(timestamp, DataPointType::RADAR, radar_vec);
    }

    iss >> x;
    iss >> y;
    iss >> vx;
    iss >> vy;
    iss >> yaw;
    iss >> yawrate;

    v = sqrt(vx * vx + vy * vy);

    VectorXd truth_vec(NX);
    truth_vec << x, y, v, yaw, yawrate;
    truth_data.set(timestamp, DataPointType::STATE, truth_vec);

    all_sensor_data.push_back(sensor_data);
    all_truth_data.push_back(truth_data);
  }

  /*******************************************************************
   * column names for output file
   *******************************************************************/
  out_file << "time_stamp" << "\t";
  out_file << "px_state" << "\t";
  out_file << "py_state" << "\t";
  out_file << "v_state" << "\t";
  out_file << "yaw_angle_state" << "\t";
  out_file << "yaw_rate_state" << "\t";
  out_file << "sensor_type" << "\t";
  out_file << "NIS" << "\t";
  out_file << "px_measured" << "\t";
  out_file << "py_measured" << "\t";
  out_file << "px_ground_truth" << "\t";
  out_file << "py_ground_truth" << "\t";
  out_file << "vx_ground_truth" << "\t";
  out_file << "vy_ground_truth" << "\n";

  /*******************************************************************
   * USE DATA AND FUSIONUKF FOR STATE ESTIMATIONS
   *******************************************************************/

   FusionUKF fusionUKF;

   vector<VectorXd> predictions;
   vector<VectorXd> ground_truths;
   vector<VectorXd> estimations_vec;
   vector<VectorXd> ground_truths_vec;

   VectorXd prediction;
   VectorXd measurement;
   VectorXd truth;
   DataPointType sensor_type;
   DataPoint estimation;
   DataPoint sensor_data;
   string sensor_name;
   double nis;

  for(int k = 0; k < all_sensor_data.size(); ++k){

   /*******************************************************************
    * STORE ALL DATA FROM SENSOR AND GROUND TRUTH TO MEMORY
    *******************************************************************/
    truth =  all_truth_data[k].get_vec();
    sensor_data = all_sensor_data[k];
    timestamp = sensor_data.get_timestamp();

    sensor_type = sensor_data.get_type();
    sensor_name = ((sensor_type == DataPointType::RADAR) ? "radar" : "lidar");
    measurement = sensor_data.get_state();

   /*******************************************************************
    * PREDICT NEXT STATE USING FUSIONUKF
    *******************************************************************/
    fusionUKF.process(sensor_data);
    prediction = fusionUKF.get();
    nis = fusionUKF.get_nis();

   /*******************************************************************
    * WRITE ALL INFO IN OUTPUT FILE
    *******************************************************************/
    out_file << timestamp << "\t";
    out_file << prediction(0) << "\t";
    out_file << prediction(1) << "\t";
    out_file << prediction(2) << "\t";
    out_file << prediction(3) << "\t";
    out_file << prediction(4) << "\t";

    out_file << sensor_name << "\t";
    out_file << nis << "\t";

    out_file << measurement(0) << "\t";
    out_file << measurement(1) << "\t";

    out_file << truth(0) << "\t";
    out_file << truth(1) << "\t";
    out_file << truth(2) << "\t";
    out_file << truth(3) << "\n";

   /*******************************************************************
    * STORE ALL DATA IN APPROPRIATE VECTOR FOR RMSE CALCULATION LATER
    *******************************************************************/
    estimation.set(timestamp, DataPointType::STATE, prediction);
    estimations_vec.push_back(estimation.get_vec());
    predictions.push_back(prediction);

    ground_truths_vec.push_back(truth);
    ground_truths.push_back(all_truth_data[k].get_state());
  }

  /*******************************************************************
   * CALCULATE ROOT MEAN SQUARE ERROR
   *******************************************************************/
   VectorXd RMSE;

   RMSE = calculate_RMSE(estimations_vec, ground_truths_vec);
   cout << "RMSE:" << endl << RMSE << endl;

  /*******************************************************************
   * PRINT TO CONSOLE IN A NICE FORMAT FOR DEBUGGING
   *******************************************************************/
   //print_EKF_data(RMSE, predictions, ground_truths, all_sensor_data);

  /*******************************************************************
   * CLOSE FILES
   *******************************************************************/
  if(out_file.is_open()) { 
    out_file.close();
  }
  if(in_file.is_open()) {
    in_file.close();
  }

  cout << "Done!" << endl;
  return 0;
}
