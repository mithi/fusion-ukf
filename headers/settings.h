#ifndef SETTINGS_H_
#define SETTINGS_H_

// process noise standard deviation longitudinal acceleration in m/s^2
const double STD_SPEED_NOISE = 0.2;
// process noise standard deviation yaw acceleration in rad/s^2
const double STD_YAWRATE_NOISE = 0.2;

const double VAR_SPEED_NOISE = STD_SPEED_NOISE * STD_SPEED_NOISE;
const double VAR_YAWRATE_NOISE = STD_YAWRATE_NOISE * STD_YAWRATE_NOISE;

//measurements noise standard deviations
const double STD_RHO = 0.3; // meters
const double STD_PHI = 0.0175; // radians
const double STD_RHODOT = 0.1; // meters / second
const double VAR_RHO = STD_RHO * STD_RHO;
const double VAR_PHI = STD_PHI * STD_PHI;
const double VAR_RHODOT = STD_RHODOT * STD_RHODOT;

const int NZ_RADAR  = 3; // number of measurements
const int NX = 5; // number of states
const int NAUGMENTED = NX + 2; // number of states plus two noise values
const int NSIGMA = NAUGMENTED * 2 + 1; // number of sigma points
const int LAMBDA = 3 - NAUGMENTED;
const double SCALE = sqrt(LAMBDA + NAUGMENTED); // coefficient used to create augmented sigma points

#endif /* SETTINGS_H_ */
