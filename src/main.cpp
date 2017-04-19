#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "settings.h"
#include "radarpredictor.h"
#include "statepredictor.h"
#include "tests.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(){
  all_tests();
  return 0;
}
