#ifndef VHDMAPSE_INCLUDE_NODE_PARAMETERS_H_
#define VHDMAPSE_INCLUDE_NODE_PARAMETERS_H_

#include <string>

extern std::string GNSS_TOPIC;
extern std::string IMU_TOPIC;
extern std::string ENCODER_TOPIC;
extern std::string YAW_TOPIC;
extern std::string POSITION_TOPIC;
extern std::string CAMERA_LANE_TOPIC;
extern std::string LC_MEASUREMENT_TOPIC;
extern std::string CAMERA_SIGN_TOPIC;
extern std::string MAP_RESPONSE_LANE_TOPIC;
extern std::string MAP_RESPONSE_LC_TOPIC;
extern std::string MAP_RESPONSE_SIGN_TOPIC;
extern std::string MAP_REQUEST_LANE_TOPIC;
extern std::string MAP_REQUEST_LC_TOPIC;
extern std::string MAP_REQUEST_SIGN_TOPIC;
extern int GNSS_TOPIC_FREQUENCY;
extern int ENCODER_TOPIC_FREQUENCY;
extern int IMU_TOPIC_FREQUENCY;

#endif  // VHDMAPSE_INCLUDE_NODE_PARAMETERS_H_
