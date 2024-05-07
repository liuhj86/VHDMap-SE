#ifndef VHDMAPSE_INCLUDE_SYSTEM_PARAMETERS_H_
#define VHDMAPSE_INCLUDE_SYSTEM_PARAMETERS_H_

#define G0 9.7803267714
#define deg M_PI / 180.0         // degree
#define rad 180.0 / M_PI         // radian
#define dph deg / 3600.0         // degree per hour
#define dpsh deg / sqrt(3600.0)  // degree per square-root hour
#define mg G0 / 1000.0           // mili-gravity force
#define ug mg / 1000.0           // micro-gravity force
#define mgpsHz mg / sqrt(1.0)    // mili-gravity force per second
#define ugpsHz ug / sqrt(1.0)    // micro-gravity force per second

#include <Eigen/Dense>

// system parameters
extern int RUN_MODE;
extern double STATIONARY_THRESHOLD;
extern double ERROR_VELOCITY_VARIANCE_X;
extern double ERROR_VELOCITY_VARIANCE_Y;
extern double ERROR_VELOCITY_VARIANCE_Z;
extern double INITIAL_POSITION_VARIANCE_PARAMETER_LAT;
extern double INITIAL_POSITION_VARIANCE_PARAMETER_LON;
extern double INITIAL_POSITION_VARIANCE_PARAMETER_ALT;
extern double INITIAL_ATTITUDE_VARIANCE_PARAMETER_YAW;
extern double GNSS_VARIANCE_PARAMETER_LAT;
extern double GNSS_VARIANCE_PARAMETER_LON;
extern double GNSS_VARIANCE_PARAMETER_ALT;
extern double GNSS_VARIANCE_PARAMETER_YAW;
extern double ANGULAR_VEL_THRESHOLD_FOR_STRAIGHT_MOTION;
extern double INITIAL_ENCODER_SCALE;
extern double PEBG_STANDARD_DEVIATION_COEFFICIENT;
extern double PWEBG_STANDARD_DEVIATION_COEFFICIENT;
extern double INITIAL_LATITUDE;
extern double INITIAL_LONGITUDE;
extern int DURATION_THRESHOLD_FOR_STRAIGHT_MOTION;
extern bool GNSS_DATA_FLUCTUATION_RESISTANCE;
extern bool GNSS_UPDATE;
extern bool LANE_UPDATE;
extern bool LONGITUDINAL_UPDATE;
extern bool GNSS_POS_CALI;
extern bool GNSS_YAW_CALI;
extern bool LANE_POS_CALI;
extern bool LANE_YAW_CALI;
extern bool IMU_NOISE;
extern int IMU_NOISE_MODE;
extern double IMU_NOISE_BOUND;
extern bool GNSS_NOISE;           // if add noise to GNSS
extern double GNSS_NOISE_BOUND;   // GNSS noise bound, the unit is degree
extern bool LAT_MEAS_NOISE;       // if add noise to lateral measurement
extern int LAT_MEAS_NOISE_LEVEL;  // the noise level, unit is %
extern bool LON_MEAS_NOISE;       // if add noise to longitudinal measurement
extern int LON_MEAS_NOISE_LEVEL;  // the noise level, unit is %
static int COOR_BIAS_X = -754334;
static int COOR_BIAS_Y = -2533881;

// earth parameters
static double Re = 6378137.0;  // WGS84 Equatorial radius in meters
static double Rp = 6356752.31425;
static double Ef = 1.0 / 298.257223563;
static double Wie = 7.2921151467e-5;
static double Ee = 0.0818191908425;       // First eccentricity
static double EeEe = Ee * Ee;  // The square of the first eccentricity
static double Ee2Ee2 =
    0.00673949674227;  // The square of the second eccentricity
static double g0 = 9.7803267714;
static Eigen::Vector3d weie = Eigen::Vector3d(0., 0., Wie);

// vehicle parameters
extern double VEHICLE_LENGTH;
extern double VEHICLE_WIDTH;

// road parameters
extern double ROAD_WIDTH;

#endif  // VHDMAPSE_INCLUDE_SYSTEM_PARAMETERS_H_
