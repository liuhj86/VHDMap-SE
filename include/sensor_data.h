#ifndef VHDMAPSE_INCLUDE_SENSOR_DATA_H_
#define VHDMAPSE_INCLUDE_SENSOR_DATA_H_

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>

namespace sensor {
class SensorData {
 public:
  SensorData() {}
  virtual ~SensorData() {}
  double timestamp_;
};
class ImuData : public SensorData {
 public:
  ImuData() {}
  ~ImuData() {}

  void operator=(ImuData imu_data);

  Eigen::Vector3d angular_velocity_;     
  tf2::Quaternion attitude_quaternion_;  
  Eigen::Vector3d linear_acceleration_;  
};
class GNSSData : public SensorData {
 public:
  GNSSData() {}
  ~GNSSData() {}

  void operator=(GNSSData gnss_data);

  Eigen::Vector3d geodetic_coordinates_;  // LLA
  Eigen::Vector3d covariance_;
  double yaw_;
  bool yaw_available_flag_;
  bool pos_available_flag_;
};
class EncoderData : public SensorData {
 public:
  EncoderData() {}
  ~EncoderData() {}

  void operator=(EncoderData encoder_data);
  double rear_beam_center;

  double twist_linear_x;
  double twist_linear_y;
  double twist_linear_z;

};
class LaneData : public SensorData {
 public:
  LaneData() {}
  ~LaneData() {}

  void operator=(LaneData image_data);
  double left_offset_;
  double right_offset_;
  double middle_offset_;
  double left_angle_rad_;  // Left lane inclination angle in radians
  double right_angle_rad_;
};
class LongitudinalConstraintData : public SensorData {
 public:
  LongitudinalConstraintData() {}
  ~LongitudinalConstraintData() {}

  void operator=(LongitudinalConstraintData lc_data);
  double x_;
  double y_;
  double z_;

  double point_x_;
  double point_y_;
  double point_z_;

  double gt_pos_x_;
  double gt_pos_y_;
  double gt_pos_z_;

  Eigen::Matrix3d meas_cov_;
};
}  // namespace sensor
#endif  // VHDMAPSE_INCLUDE_SENSOR_DATA_H_
