#include <sensor_data.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace sensor;

void ImuData::operator=(ImuData imu_data) {
  angular_velocity_ = imu_data.angular_velocity_;
  linear_acceleration_ = imu_data.linear_acceleration_;
  attitude_quaternion_ = imu_data.attitude_quaternion_;
  timestamp_ = imu_data.timestamp_;
}
void GNSSData::operator=(GNSSData gnss_data) {
  yaw_ = gnss_data.yaw_;
  geodetic_coordinates_ = gnss_data.geodetic_coordinates_;
  covariance_ = gnss_data.covariance_;
  timestamp_ = gnss_data.timestamp_;
}
void EncoderData::operator=(EncoderData encoder_data) {
  rear_beam_center = encoder_data.rear_beam_center;
  twist_linear_x = encoder_data.twist_linear_x;
  twist_linear_y = encoder_data.twist_linear_y;
  twist_linear_z = encoder_data.twist_linear_z;
  timestamp_ = encoder_data.timestamp_;
}
void LaneData::operator=(LaneData another_data) {
  timestamp_ = another_data.timestamp_;
  left_offset_ = another_data.left_offset_;
  right_offset_ = another_data.right_offset_;
  middle_offset_ = another_data.middle_offset_;
  left_angle_rad_ = another_data.left_angle_rad_;   
  right_angle_rad_ = another_data.right_angle_rad_;
}
void LongitudinalConstraintData::operator=(
    LongitudinalConstraintData another_data) {
  timestamp_ = another_data.timestamp_;
  x_ = another_data.x_;
  y_ = another_data.y_;
  z_ = another_data.z_;

  point_x_ = another_data.point_x_;
  point_y_ = another_data.point_y_;
  point_z_ = another_data.point_z_;

  gt_pos_x_ = another_data.gt_pos_x_;
  gt_pos_y_ = another_data.gt_pos_y_;
  gt_pos_z_ = another_data.gt_pos_z_;
}
