#include <system_parameters.h>

#include <Eigen/Dense>
#include <string>

// system parameters 
int RUN_MODE;
double STATIONARY_THRESHOLD;
double ERROR_VELOCITY_VARIANCE_X;
double ERROR_VELOCITY_VARIANCE_Y;
double ERROR_VELOCITY_VARIANCE_Z;
double INITIAL_POSITION_VARIANCE_PARAMETER_LAT;
double INITIAL_POSITION_VARIANCE_PARAMETER_LON;
double INITIAL_POSITION_VARIANCE_PARAMETER_ALT;
double INITIAL_ATTITUDE_VARIANCE_PARAMETER_YAW;
double GNSS_VARIANCE_PARAMETER_LAT;
double GNSS_VARIANCE_PARAMETER_LON;
double GNSS_VARIANCE_PARAMETER_ALT;
double GNSS_VARIANCE_PARAMETER_YAW;
double ANGULAR_VEL_THRESHOLD_FOR_STRAIGHT_MOTION;
double INITIAL_ENCODER_SCALE;  // initial encoder scale
double PEBG_STANDARD_DEVIATION_COEFFICIENT;
double PWEBG_STANDARD_DEVIATION_COEFFICIENT;
double INITIAL_LATITUDE;  // the latitude of the map coordinate origin
double INITIAL_LONGITUDE;
int DURATION_THRESHOLD_FOR_STRAIGHT_MOTION;
bool GNSS_DATA_FLUCTUATION_RESISTANCE;  // Is the GNSS data fluctuation
                                        // resistance mechanism enabled
bool GNSS_UPDATE;                       // Whether to use GNSS correction
bool LANE_UPDATE;                       // Whether to use lane correction
bool LONGITUDINAL_UPDATE;               // Whether to use longitudinal correction
bool GNSS_POS_CALI;                     // Is GNSS used to correct the position
bool GNSS_YAW_CALI;                     // Is GNSS used to correct the yaw
bool LANE_POS_CALI;                     // Is lane used to correct the position
bool LANE_YAW_CALI;                     // Is lane used to correct the yaw
bool IMU_NOISE;                         // Whether add IMU noise
int IMU_NOISE_MODE;
double IMU_NOISE_BOUND;  // The boundary value of added IMU noise, measured in
                         // degrees
bool GNSS_NOISE;                        // if add noise to GNSS
double GNSS_NOISE_BOUND;                // GNSS noise bound, the unit is degree
bool LAT_MEAS_NOISE;       // if add noise to lateral measurement
int LAT_MEAS_NOISE_MODE;
int LAT_MEAS_NOISE_LEVEL;  // the noise level, unit is %
double LAT_MEAS_NOISE_MEAN;
double LAT_MEAS_NOISE_STD;
bool LON_MEAS_NOISE;       // if add noise to longitudinal measurement
int LON_MEAS_NOISE_MODE;
int LON_MEAS_NOISE_LEVEL;  // the noise level, unit is %
double LON_MEAS_NOISE_MEAN;
double LON_MEAS_NOISE_STD;

// vehicle parameters
double VEHICLE_LENGTH;
double VEHICLE_WIDTH;

// road parameters
double ROAD_WIDTH;
