
#include <common_utils.h>
#include <error_state_kalman_filter.h>
#include <math_utils.h>
#include <mercator.h>
#include <ros/ros.h>
#include <system_parameters.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <iostream>
#include <lane_info_handler.hpp>
#include <numeric>
#include <state.hpp>
#include <vector>
void ErrorStateKalmanFilter::Initialize() {
  Xk_.resize(9);  // p phi webg
  Fk_.resize(9, 9);
  Gk_.resize(9, 9);
  Pk_.resize(9, 9);
  Qk_.resize(9, 9);
  K_.resize(9, 9);

  Fk_.setZero();
  Gk_.setZero();
  Pk_.setZero();
  Qk_.setZero();
  Zk_.setZero();
  Hk_.setZero();
  Rk_.setZero();
  K_.setZero();
  Xk_.setZero();

  for (int i = 0; i <= 8; i++) {
    Xk_(i) = 1e-20;
  }

  double pbg = pow(100 * dph, 2);
  double pebg = pow(PEBG_STANDARD_DEVIATION_COEFFICIENT * dpsh, 2);
  double pwebg = pow(PWEBG_STANDARD_DEVIATION_COEFFICIENT * dpsh, 2);

  Pk_.block<3, 3>(0, 0) =
      Eigen::Vector3d(pow(INITIAL_POSITION_VARIANCE_PARAMETER_LAT / Re, 2),
                      pow(INITIAL_POSITION_VARIANCE_PARAMETER_LON / Re, 2),
                      pow(INITIAL_POSITION_VARIANCE_PARAMETER_ALT, 2))
          .asDiagonal();  // pos
  Pk_.block<3, 3>(3, 3) =
      Eigen::Vector3d(0, 0,
                      pow(math_utils::DegreesToRadians(
                              INITIAL_ATTITUDE_VARIANCE_PARAMETER_YAW),
                          2))
          .asDiagonal();
  Pk_.block<3, 3>(6, 6) = Eigen::Vector3d(pbg, pbg, 4 * pbg).asDiagonal();
  Qk_.block<3, 3>(0, 0) =
      Eigen::Vector3d(ERROR_VELOCITY_VARIANCE_X, ERROR_VELOCITY_VARIANCE_Y,
                      ERROR_VELOCITY_VARIANCE_Z)
          .asDiagonal();
  Qk_.block<3, 3>(3, 3) = Eigen::Vector3d(pebg, pebg, 4 * pebg).asDiagonal();
  Qk_.block<3, 3>(6, 6) = Eigen::Vector3d(pwebg, pwebg, 4 * pwebg).asDiagonal();

  GNSS_pos_cali_cnt_ = 0;
  map_pos_cali_cnt_ = 0;
}
void ErrorStateKalmanFilter::set_Fk(const State& state, double dt) {
  Eigen::MatrixXd Ft;
  Ft.resize(9, 9);
  Ft.setZero();
  Eigen::Matrix3d Fpp, Fqp, Fqq, Fqwebg, M1, M3;

  double lat = state.p_(0);
  double vE = state.v_(0);
  double vN = state.v_(1);
  Eigen::Matrix3d Cbn = state.q_.toRotationMatrix();
  const EarthParameterState& eparams = state.earth_parameter_state_;

  double sinLat = sin(lat);
  double cosLat = cos(lat);
  double tanLat = sinLat / cosLat;
  double RNhcl = eparams.RNh * cosLat;
  double RNh2 = eparams.RNh * eparams.RNh;
  double RMh2 = eparams.RMh * eparams.RMh;
  M1 << 0, 0, 0, -Wie * sinLat, 0, 0, Wie * cosLat, 0, 0;

  M3 << 0, 0, vN / RMh2, 0, 0, -vE / RNh2, vE / (eparams.RNh * cosLat * cosLat),
      0, -vE * tanLat / RNh2;

  Fpp << 0, 0, -vN / RMh2, vE * sinLat / (RNhcl * cosLat), 0,
      -vE / (RNh2 * cosLat), 0, 0, 0;

  Fqp = M1 + M3;
  Fqq = -math_utils::skew(eparams.wnin);
  Fqwebg = -Cbn;

  Ft.block<3, 3>(0, 0) = Fpp;
  Ft.block<3, 3>(3, 0) = Fqp;
  Ft.block<3, 3>(3, 3) = Fqq;
  Ft.block<3, 3>(3, 6) = Fqwebg;

  Fk_ = Eigen::Matrix<double, 9, 9>::Identity() + Ft * dt;
}
void ErrorStateKalmanFilter::set_Gk(const State& state, double dt) {
  Gk_.setZero();

  double lat = state.p_(0);
  const EarthParameterState& eparams = state.earth_parameter_state_;
  Eigen::Matrix3d Cbn = state.q_.toRotationMatrix();

  double sinLat = sin(lat);
  double cosLat = cos(lat);
  double tanLat = sinLat / cosLat;
  double RNhcl = eparams.RNh * cosLat;
  Eigen::Matrix3d Fpv, Fqv, M2;

  M2 << 0, -1.0 / eparams.RMh, 0, 1.0 / eparams.RNh, 0, 0, tanLat / eparams.RNh,
      0, 0;

  Fpv << 0, 1.0 / eparams.RMh, 0, 1.0 / RNhcl, 0, 0, 0, 0, 1;

  Fqv = M2;

  Gk_.block<3, 3>(0, 0) = Fpv * sqrt(dt);
  Gk_.block<3, 3>(3, 0) = Fqv * sqrt(dt);
  Gk_.block<3, 3>(3, 3) = Cbn * sqrt(dt);
  Gk_.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity() * sqrt(dt);
}
void ErrorStateKalmanFilter::Predict() {
  Xk_ = Fk_ * Xk_;
  Pk_ = Fk_ * Pk_ * Fk_.transpose() + Gk_ * Qk_ * Gk_.transpose();  // 3.5*1e-17
  Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();
}
void ErrorStateKalmanFilter::GNSSUpdate(const State& state,
                                        const sensor::GNSSData& gnss_data,
                                        const double north_angle, bool pos_cali,
                                        bool yaw_cali) {
  if (pos_cali && yaw_cali && gnss_data.pos_available_flag_ &&
      gnss_data.yaw_available_flag_) {
    GNSS_pos_cali_cnt_++;
    std::cout << "GNSS correction count： " << GNSS_pos_cali_cnt_ << std::endl;
    std::cout
        << BLUE
        << "Using GNSS to simultaneously correct heading angle and position"
        << COLOR_RESET << std::endl;
    Eigen::Vector3d rpy = math_utils::R2rpy(state.q_.toRotationMatrix());
    Zk_.resize(6);
    Zk_.setZero();
    Zk_.segment<3>(0) = state.p_ - gnss_data.geodetic_coordinates_;

    double delta_yaw;
    if (RUN_MODE == 0) {
      double north_bias = rpy.z() - north_angle;
      if (north_bias < 0) {
        north_bias += 2 * M_PI;
      }
      delta_yaw = (math_utils::DegreesToRadians(gnss_data.yaw_) - north_bias);
    } else if (RUN_MODE == 2) {
      double corrected_state_yaw = rpy.z() + 0.5 * M_PI;
      if (corrected_state_yaw > M_PI) {
        corrected_state_yaw -= 2 * M_PI;
      }
      double yaw_earth;
      common_utils::CalcYawEarthFromYawENU(corrected_state_yaw, &yaw_earth);
      delta_yaw = math_utils::DegreesToRadians(gnss_data.yaw_) - yaw_earth;
      std::cout << BLUE << gnss_data.yaw_ << ' '
                << math_utils::RadiansToDegrees(yaw_earth) << COLOR_RESET
                << std::endl;
    }

    Eigen::Vector3d ea;  // The Euler angle vector, roll, and pitch are all 0,
                         // only yaw is of interest
    if (delta_yaw > M_PI) {  // Turning 180 degrees or more in a certain
                             // direction is equivalent to turning 180 degrees
                             // or less in the opposite direction
      // Initialize Euler angle (Z-Y-X, RPY, first roll around the x-axis, then
      // pitch around the y-axis, and finally yaw around the z-axis)
      ea << 0, 0, delta_yaw - 2 * M_PI;
    } else if (delta_yaw < -M_PI) {
      ea << 0, 0, delta_yaw + 2 * M_PI;
    } else {
      ea << 0, 0, delta_yaw;
    }

    Eigen::AngleAxisd rotation_vector;
    rotation_vector = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    Eigen::Vector3d tmp;
    if (RUN_MODE == 0) {
      tmp << 0, 0, delta_yaw;
    } else if (RUN_MODE == 2) {
      tmp << 0, 0, -delta_yaw;
    }
    // Eigen::Vector3d tmp(0, 0, 0);
    Zk_.segment<3>(3) = tmp;
    Hk_.resize(6, 9);
    Hk_.setZero();
    Hk_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Hk_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    // Hk_.block<1, 3>(3, 3) = Eigen::Vector3d(Hehex, Hehey, Hehez);
    Rk_.resize(6, 6);
    Rk_.setZero();
    Rk_.block<3, 3>(0, 0) = gnss_data.covariance_.asDiagonal();
    Rk_.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
    Rk_(3, 3) = std::max(gnss_data.covariance_(0), gnss_data.covariance_(1));
    Rk_(4, 4) = std::max(gnss_data.covariance_(0), gnss_data.covariance_(1));
    Rk_(5, 5) = std::max(gnss_data.covariance_(0), gnss_data.covariance_(1));

    Eigen::MatrixXd Py, Pyinv;
    Py.resize(6, 6);
    Pyinv.resize(6, 6);
    Py = Hk_ * Pk_ * Hk_.transpose() + Rk_;  // S = H * P * H.transpose() + R;
    Pyinv.setIdentity();                     // solve Ax=B
    Py.llt().solveInPlace(Pyinv);
    K_.resize(9, 6);
    K_ = Pk_ * Hk_.transpose() * Pyinv;  // K = P*H.transpose()*S.inverse()
    Xk_ = Xk_ + K_ * (Zk_ - Hk_ * Xk_);
    Eigen::MatrixXd IKH_ = Eigen::Matrix<double, 9, 9>::Identity() - K_ * Hk_;
    Pk_ = IKH_ * Pk_ * IKH_.transpose() + K_ * Rk_ * K_.transpose();
    Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();
  } else if (gnss_data.pos_available_flag_ && pos_cali) {
    GNSS_pos_cali_cnt_++;
    std::cout << "GNSS correction count： " << GNSS_pos_cali_cnt_ << std::endl;
    std::cout << BLUE << "GNSS position correction" << COLOR_RESET << std::endl;
    Eigen::Vector3d rpy = math_utils::R2rpy(state.q_.toRotationMatrix());
    Zk_.resize(3);
    Zk_.segment<3>(0) = state.p_ - gnss_data.geodetic_coordinates_;
    // cout << "zk: " << Zk_ << endl;
    Hk_.resize(3, 9);
    Hk_.setZero();
    Hk_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Rk_ = gnss_data.covariance_.asDiagonal();
    // Eigen::Vector3d a(1e-40, 1e-40, 1e-40);
    // Rk_ = a.asDiagonal();
    // Rk_ << 1e-20, 0, 0, 0, 1e-20, 0, 0, 0, 1e-20;
    Eigen::Matrix3d Py, Pyinv;
    Py = Hk_ * Pk_ * Hk_.transpose() + Rk_;  // S = H * P * H.transpose() + R;
    Pyinv.setIdentity();                     // solve Ax=B
    Py.llt().solveInPlace(Pyinv);
    K_.resize(9, 3);
    K_ = Pk_ * Hk_.transpose() * Pyinv;  // K = P*H.transpose()*S.inverse()
    Xk_ = Xk_ + K_ * (Zk_ - Hk_ * Xk_);
    Eigen::MatrixXd IKH_ = Eigen::Matrix<double, 9, 9>::Identity() - K_ * Hk_;
    Pk_ = IKH_ * Pk_ * IKH_.transpose() + K_ * Rk_ * K_.transpose();
    Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();
  } else if (yaw_cali && gnss_data.yaw_available_flag_) {
    std::cout << BLUE << "GNSS yaw correction" << COLOR_RESET << std::endl;
    Eigen::Vector3d rpy = math_utils::R2rpy(state.q_.toRotationMatrix());
    Zk_.resize(3);
    Zk_.setZero();
    double delta_yaw;
    if (RUN_MODE == 0) {
      double north_bias = rpy.z() - north_angle;
      if (north_bias < 0) {
        north_bias += 2 * M_PI;
      }
      delta_yaw = (math_utils::DegreesToRadians(gnss_data.yaw_) - north_bias);
    } else if (RUN_MODE == 2) {
      double corrected_state_yaw = rpy.z() + 0.5 * M_PI;
      if (corrected_state_yaw > M_PI) {
        corrected_state_yaw -= 2 * M_PI;
      }
      double yaw_earth;
      common_utils::CalcYawEarthFromYawENU(corrected_state_yaw, &yaw_earth);
      delta_yaw = math_utils::DegreesToRadians(gnss_data.yaw_) - yaw_earth;
      std::cout << BLUE << gnss_data.yaw_ << ' '
                << math_utils::RadiansToDegrees(yaw_earth) << COLOR_RESET
                << std::endl;
    }

    double gnss_yaw_in_rviz =
        gnss_data.yaw_ + math_utils::RadiansToDegrees(north_angle);
    if (gnss_yaw_in_rviz > 180) {
      gnss_yaw_in_rviz -= 360;
    } else if (gnss_yaw_in_rviz < -180) {
      gnss_yaw_in_rviz += 360;
    }
    std::cout << BLUE << gnss_yaw_in_rviz << ' '
              << math_utils::RadiansToDegrees(rpy.z()) << COLOR_RESET
              << std::endl;
    Eigen::Vector3d ea;
    if (delta_yaw > M_PI) {
      ea << 0, 0, delta_yaw - 2 * M_PI;
    } else if (delta_yaw < -M_PI) {
      ea << 0, 0, delta_yaw + 2 * M_PI;
    } else {
      ea << 0, 0, delta_yaw;
    }

    Eigen::AngleAxisd rotation_vector;
    rotation_vector = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    // Eigen::Vector3d tmp = rotation_vector.angle() * rotation_vector.axis();
    Eigen::Vector3d tmp;
    if (RUN_MODE == 0) {
      tmp << 0, 0, delta_yaw;
    } else if (RUN_MODE == 2) {
      tmp << 0, 0, -delta_yaw;
    }
    std::cout << tmp << std::endl;
    Zk_.segment<3>(0) = tmp;
    Hk_.resize(3, 9);
    Hk_.setZero();
    Hk_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    // Hk_.block<1, 3>(3, 3) = Eigen::Vector3d(Hehex, Hehey, Hehez);
    Rk_ = gnss_data.covariance_.asDiagonal();
    Rk_(0, 0) = std::max(gnss_data.covariance_(0), gnss_data.covariance_(1));
    Rk_(1, 1) = std::max(gnss_data.covariance_(0), gnss_data.covariance_(1));
    Rk_(2, 2) = std::max(gnss_data.covariance_(0), gnss_data.covariance_(1));
    Eigen::MatrixXd Py, Pyinv;
    Py.resize(3, 3);
    Pyinv.resize(3, 3);
    Py = Hk_ * Pk_ * Hk_.transpose() + Rk_;  // S = H * P * H.transpose() + R;
    Pyinv.setIdentity();                     // solve Ax=B
    Py.llt().solveInPlace(Pyinv);
    K_.resize(9, 3);
    K_ = Pk_ * Hk_.transpose() * Pyinv;  // K = P*H.transpose()*S.inverse()
    Xk_ = Xk_ + K_ * (Zk_ - Hk_ * Xk_);
    Eigen::MatrixXd IKH_ = Eigen::Matrix<double, 9, 9>::Identity() - K_ * Hk_;
    Pk_ = IKH_ * Pk_ * IKH_.transpose() + K_ * Rk_ * K_.transpose();
    Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();
  }
}
void ErrorStateKalmanFilter::LaneUpdate(
    const State& state, sensor::LaneData& lane_data,
    const bool& straight_motion_flag, double& north_angle,
    std::vector<MapPoint>* map_lane_points,
    std::vector<uint16_t>* map_lane_points_flags, MercatorProj& mercator,
    const double& bias_x, const double& bias_y, int* msg_index,
    const bool& pos_cali, const bool& yaw_cali,
    int* left_lane_min_dist_point_index, int* right_lane_min_dist_point_index,
    double* car_x, double* car_y, double* k, double* b) {
  bool enable_pos_update_flag = true, enable_yaw_update_flag = true;
  Eigen::Vector3d rpy = math_utils::R2rpy(state.q_.toRotationMatrix());
  double yaw_earth = 0;
  if (RUN_MODE == 0) {
    common_utils::CalcYawEarthFromYaw(rpy.z(), true, north_angle, &yaw_earth);
    std::cout << "state q: " << math_utils::RadiansToDegrees(rpy.z()) << ' '
              << "north angle: " << math_utils::RadiansToDegrees(north_angle)
              << ' ' << "yaw earth: " << math_utils::RadiansToDegrees(yaw_earth)
              << std::endl;
  } else if (RUN_MODE == 2) {
    common_utils::CalcYawEarthFromYawENU(rpy.z() + 0.5 * M_PI, &yaw_earth);
    std::cout << "state q: " << math_utils::RadiansToDegrees(rpy.z()) << ' '
              << "north angle: " << math_utils::RadiansToDegrees(north_angle)
              << ' ' << "yaw earth: " << math_utils::RadiansToDegrees(yaw_earth)
              << std::endl;
  }
  double local_pos_x, local_pos_y;
  if (RUN_MODE == 0) {
    mercator.ToProj(state.p_(0), state.p_(1), local_pos_x, local_pos_y);
    *car_y =
        (local_pos_x + bias_x);  // + VEHICLE_LENGTH * sin(M_PI - yaw_rviz);
    *car_x =
        -(local_pos_y + bias_y);  // + VEHICLE_LENGTH * cos(M_PI - yaw_rviz);
  } else if (RUN_MODE == 2) {
    UTMCoor odom_coor;
    common_utils::LL2UTM(state.p_[1], state.p_[0], &odom_coor);
    *car_x = odom_coor.x + COOR_BIAS_X;
    *car_y = odom_coor.y + COOR_BIAS_Y;
  }

  std::cout << "car_y: " << *car_y << std::endl
            << "car_x: " << *car_x << std::endl;
  Eigen::Vector3d image_offset, map_offset;
  double yaw_earth_obs = 0, lane_yaw_obs = 0;
  LaneInfoHandler lane_info_handler(yaw_earth, north_angle, *car_x, *car_y,
                                    map_lane_points, map_lane_points_flags,
                                    msg_index);
  lane_info_handler.ProcessLaneInfo(
      lane_data, &image_offset, &lane_yaw_obs, &map_offset, &yaw_earth_obs,
      pos_cali, yaw_cali, &enable_pos_update_flag, &enable_yaw_update_flag,
      left_lane_min_dist_point_index, right_lane_min_dist_point_index, k, b);
  std::cout << "pos cali: " << pos_cali << ' '
            << "enable pos update flag: " << enable_pos_update_flag
            << std::endl;
  if (pos_cali && yaw_cali && enable_pos_update_flag &&
      enable_yaw_update_flag && straight_motion_flag) {
    std::cout << GREEN << "Lane correction for both pos and yaw" << COLOR_RESET
              << std::endl;
    map_pos_cali_cnt_++;
    std::cout << "map pos correction count： " << map_pos_cali_cnt_
              << std::endl;
    Eigen::Vector3d rpy = math_utils::R2rpy(state.q_.toRotationMatrix());
    Zk_.resize(6);
    Zk_.setZero();
    if (RUN_MODE == 0) {
      Zk_.segment<3>(0) =
          -state.earth_parameter_state_.Mrp * (image_offset - map_offset);
    } else if (RUN_MODE == 2) {
      Zk_.segment<3>(0) =
          state.earth_parameter_state_.Mrp * (image_offset - map_offset);
    }
    double yaw_earth_pred = 0;
    if (RUN_MODE == 0) {
      common_utils::CalcYawEarthFromYaw(rpy.z(), true, north_angle,
                                        &yaw_earth_pred);
    } else if (RUN_MODE == 2) {
      // This is because there is a 90 degree deviation in the orientation of
      // our sensor installation orientation, which needs to be corrected when
      // calculating the deviation. Users choose whether they need it or not
      // based on their own needs.
      double corrected_state_yaw = rpy.z() + 0.5 * M_PI;
      if (corrected_state_yaw > M_PI) {
        corrected_state_yaw -= 2 * M_PI;
      }
      if (corrected_state_yaw < -M_PI) {
        corrected_state_yaw += 2 * M_PI;
      }
      common_utils::CalcYawEarthFromYawENU(corrected_state_yaw,
                                           &yaw_earth_pred);
    }
    double delta_yaw = yaw_earth_obs - yaw_earth_pred;
    if (delta_yaw < -M_PI) {
      delta_yaw += 2 * M_PI;
    } else if (delta_yaw > M_PI) {
      delta_yaw -= 2 * M_PI;
    }
    std::cout << "yaw earth obs: " << yaw_earth_obs << std::endl;
    std::cout << BLUE << math_utils::RadiansToDegrees(yaw_earth_obs) << ' '
              << math_utils::RadiansToDegrees(yaw_earth_pred) << COLOR_RESET
              << std::endl;

    Eigen::Vector3d tmp;
    if (RUN_MODE == 0) {
      tmp << 0, 0, delta_yaw;
    } else if (RUN_MODE == 2) {
      tmp << 0, 0, -delta_yaw;
    }
    std::cout << tmp << std::endl;
    Zk_.segment<3>(3) = tmp;
    Hk_.resize(6, 9);
    Hk_.setZero();
    Hk_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Hk_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    Rk_.resize(6, 6);
    Rk_.setZero();
    Eigen::Vector3d pos_rk_vector, yaw_rk_vector;
    if (LAT_MEAS_NOISE) {
      pos_rk_vector = Eigen::Vector3d(
          pow(10, 6 * fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1) *
              1e-20,
          pow(10, 6 * fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1) *
              1e-20,
          pow(10, 6 * fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1) *
              1e-20);
      yaw_rk_vector = Eigen::Vector3d(
          pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
          pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
          pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20);
    } else {
      if (RUN_MODE == 0) {
        pos_rk_vector = Eigen::Vector3d(
            pow(10, 6 *
                        fabs(fabs(lane_data.left_offset_) +
                             fabs(lane_data.right_offset_) - ROAD_WIDTH) /
                        0.1 / 2) *
                1e-20,
            pow(10, 6 *
                        fabs(fabs(lane_data.left_offset_) +
                             fabs(lane_data.right_offset_) - ROAD_WIDTH) /
                        0.1 / 2) *
                1e-20,
            pow(10, 6 *
                        fabs(fabs(lane_data.left_offset_) +
                             fabs(lane_data.right_offset_) - ROAD_WIDTH) /
                        0.1 / 2) *
                1e-20);
        yaw_rk_vector = Eigen::Vector3d(
            pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
            pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
            pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20);
      } else if (RUN_MODE == 2) {
        pos_rk_vector = Eigen::Vector3d(
            pow(10, 6 *
                        fabs(fabs(lane_data.left_offset_) +
                             fabs(lane_data.right_offset_) - ROAD_WIDTH) /
                        0.1 / 2) *
                1e-20,
            pow(10, 6 *
                        fabs(fabs(lane_data.left_offset_) +
                             fabs(lane_data.right_offset_) - ROAD_WIDTH) /
                        0.1 / 2) *
                1e-20,
            pow(10, 6 *
                        fabs(fabs(lane_data.left_offset_) +
                             fabs(lane_data.right_offset_) - ROAD_WIDTH) /
                        0.1 / 2) *
                1e-20);
        yaw_rk_vector = Eigen::Vector3d(
            pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
            pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
            pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20);
      }
    }
    std::cout << "lane pos correction variance: " << pos_rk_vector << std::endl;
    std::cout << "lane yaw correction variance: " << yaw_rk_vector << std::endl;
    Rk_.block<3, 3>(0, 0) = pos_rk_vector.asDiagonal();
    Rk_.block<3, 3>(3, 3) = yaw_rk_vector.asDiagonal();

    // Eigen::Vector3d rk_vector(
    //     pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
    //     pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20,
    //     pow(10, 7 * fabs(lane_yaw_obs - 0.5 * M_PI) / 0.01) * 1e-20);
    Eigen::MatrixXd Py, Pyinv;
    Py.resize(6, 6);
    Pyinv.resize(6, 6);
    Py = Hk_ * Pk_ * Hk_.transpose() + Rk_;  // S = H * P * H.transpose() + R;
    // std::cout << "Py: " << Py << std::endl;
    Pyinv.setIdentity();  // solve Ax=B
    Py.llt().solveInPlace(Pyinv);
    K_.resize(9, 6);
    K_ = Pk_ * Hk_.transpose() * Pyinv;  // K = P*H.transpose()*S.inverse()
    Xk_ = Xk_ + K_ * (Zk_ - Hk_ * Xk_);
    Eigen::MatrixXd IKH_ = Eigen::Matrix<double, 9, 9>::Identity() - K_ * Hk_;
    Pk_ = IKH_ * Pk_ * IKH_.transpose() + K_ * Rk_ * K_.transpose();
    Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();  
  } else if (pos_cali && enable_pos_update_flag) {  
    std::cout << GREEN << "lane pos correction" << COLOR_RESET << std::endl;
    map_pos_cali_cnt_++;
    std::cout << "map pos correction count： " << map_pos_cali_cnt_
              << std::endl;
    Zk_.resize(3);
    if (RUN_MODE == 0) {
      Zk_.segment<3>(0) =
          -state.earth_parameter_state_.Mrp * (image_offset - map_offset);
    } else if (RUN_MODE == 2) {
      Zk_.segment<3>(0) =
          state.earth_parameter_state_.Mrp * (image_offset - map_offset);
    }

    Hk_.resize(3, 9);
    Hk_.setZero();
    Hk_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Rk_.resize(3, 3);
    Eigen::Vector3d rk_vector;
    // Eigen::Vector3d rk_vector(1e-14, 1e-14, 1e-14);
   
    rk_vector = Eigen::Vector3d(
        pow(10, 6 * (fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1)) *
            1e-20,
        pow(10, 6 * (fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1)) *
            1e-20,
        pow(10, 6 * (fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1)) *
            1e-20);
    
    std::cout << "lane pos correction variance: " << rk_vector << std::endl;
    Rk_ = rk_vector.asDiagonal();
    Eigen::Matrix3d Py, Pyinv;
    Py = Hk_ * Pk_ * Hk_.transpose() + Rk_;  // S = H * P * H.transpose() + R;
    Pyinv.setIdentity();                     // solve Ax=B
    Py.llt().solveInPlace(Pyinv);
    K_.resize(9, 3);
    K_ = Pk_ * Hk_.transpose() * Pyinv;  // K = P*H.transpose()*S.inverse()
    Xk_ = Xk_ + K_ * (Zk_ - Hk_ * Xk_);
    Eigen::MatrixXd IKH_ = Eigen::Matrix<double, 9, 9>::Identity() - K_ * Hk_;
    Pk_ = IKH_ * Pk_ * IKH_.transpose() + K_ * Rk_ * K_.transpose();
    Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();
  } else if (yaw_cali && straight_motion_flag && enable_yaw_update_flag) {
    std::cout << GREEN << "lane yaw correction" << COLOR_RESET << std::endl;
    Eigen::Vector3d rpy = math_utils::R2rpy(state.q_.toRotationMatrix());
    Zk_.resize(3);
    Zk_.setZero();
    double yaw_earth_pred = 0;
    double delta_yaw;
    if (RUN_MODE == 0) {
      common_utils::CalcYawEarthFromYaw(rpy.z(), true, north_angle,
                                        &yaw_earth_pred);
      delta_yaw = yaw_earth_obs - yaw_earth_pred;
    } else if (RUN_MODE == 2) {
      double corrected_state_yaw = rpy.z() + 0.5 * M_PI;
      if (corrected_state_yaw > M_PI) {
        corrected_state_yaw -= 2 * M_PI;
      }
      if (corrected_state_yaw < -M_PI) {
        corrected_state_yaw += 2 * M_PI;
      }
      common_utils::CalcYawEarthFromYawENU(corrected_state_yaw,
                                           &yaw_earth_pred);
      delta_yaw = yaw_earth_obs - yaw_earth_pred;
    }

    if (delta_yaw < -M_PI) {
      delta_yaw += 2 * M_PI;
    } else if (delta_yaw > M_PI) {
      delta_yaw -= 2 * M_PI;
    }
    std::cout << "yaw earth obs: " << yaw_earth_obs << std::endl;
    std::cout << BLUE << math_utils::RadiansToDegrees(yaw_earth_obs) << ' '
              << math_utils::RadiansToDegrees(yaw_earth_pred) << COLOR_RESET
              << std::endl;

    Eigen::Vector3d tmp;
    if (RUN_MODE == 0) {
      tmp << 0, 0, delta_yaw;
    } else if (RUN_MODE == 2) {
      tmp << 0, 0, -delta_yaw;
    }
    std::cout << tmp << std::endl;
    Zk_.segment<3>(0) = tmp;
    Hk_.resize(3, 9);
    Hk_.setZero();
    Hk_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Rk_.resize(3, 3);
    Eigen::Vector3d rk_vector;
    
    rk_vector = Eigen::Vector3d(
        pow(10, 10 * (fabs(lane_yaw_obs - 0.5 * M_PI) / 0.005)) * 1e-20,
        pow(10, 10 * (fabs(lane_yaw_obs - 0.5 * M_PI) / 0.005)) * 1e-20,
        pow(10, 10 * (fabs(lane_yaw_obs - 0.5 * M_PI) / 0.005)) * 1e-20);
    
    std::cout << "lane yaw correction variance " << rk_vector << std::endl;
    Rk_ = rk_vector.asDiagonal();
    Eigen::Matrix3d Py, Pyinv;
    Py = Hk_ * Pk_ * Hk_.transpose() + Rk_;  // S = H * P * H.transpose() + R;
    Pyinv.setIdentity();                     // solve Ax=B
    Py.llt().solveInPlace(Pyinv);
    K_.resize(9, 3);
    K_ = Pk_ * Hk_.transpose() * Pyinv;  // K = P*H.transpose()*S.inverse()
    Xk_ = Xk_ + K_ * (Zk_ - Hk_ * Xk_);
    Eigen::MatrixXd IKH_ = Eigen::Matrix<double, 9, 9>::Identity() - K_ * Hk_;
    Pk_ = IKH_ * Pk_ * IKH_.transpose() + K_ * Rk_ * K_.transpose();
    Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();
  }
}
void ErrorStateKalmanFilter::LongitudinalConstraintUpdate(
    const State& state, const sensor::LongitudinalConstraintData& lc_data,
    const double north_angle, std::vector<MapPoint>* map_lc_points,
    MercatorProj mercator, const double bias_x, const double bias_y) {
  std::cout << YELLOW << "longitudinal pos correction" << COLOR_RESET
            << std::endl;
  bool enable_flag = true;
  Eigen::Vector3d map_offset;
  Eigen::Vector3d measurement_offset;

  if (map_lc_points->size() == 0) {
    std::cout << "Exception: No longitudinal constraint points responsed."
              << std::endl;
  } else {
    if (RUN_MODE == 0) {
      Eigen::Vector3d rpy = math_utils::R2rpy(state.q_.toRotationMatrix());
      double yaw_earth = 0;
      common_utils::CalcYawEarthFromYaw(rpy.z(), 1, north_angle, &yaw_earth);
      double local_pos_x, local_pos_y;
      double car_x, car_y;
      mercator.ToProj(state.p_(0), state.p_(1), local_pos_x, local_pos_y);
      // Assuming that the y-axis in the map points due north and the x-axis
      // points due east
      car_y = local_pos_x + bias_x;     // + VEHICLE_LENGTH * sin(yaw_earth);
      car_x = -(local_pos_y + bias_y);  // + VEHICLE_LENGTH * cos(yaw_earth);
      /**
       * The negative sign is added to the northward deviation vector because
       * the coordinate systems of Carla and ROS are different, with one
       * pointing north and the other pointing south on the y-axis. When
       * converting error vectors from the x-y coordinate systems to
       * latitude and longitude error vectors, it is required that the x-axis
       * and y-axis point east and north respectively. Therefore, the difference
       * in y-direction coordinates on the map is not the difference in the
       * north direction, it is only after adding a negative sign.
       */
      Eigen::Vector3d map_offset_tmp((*map_lc_points)[0].x - car_x,
                                     -((*map_lc_points)[0].y - car_y), 0.0);
      measurement_offset << lc_data.x_, -lc_data.y_, lc_data.z_;
      std::cout << "map target: " << (*map_lc_points)[0].x << ' '
                << (*map_lc_points)[0].y << std::endl;
      std::cout << "measurement target: " << lc_data.point_x_ << ' '
                << lc_data.point_y_ << std::endl;
      std::cout << "map pos: " << car_x << ' ' << car_y << std::endl;
      std::cout << "measurement_pos: " << lc_data.gt_pos_x_ << " "
                << lc_data.gt_pos_y_ << std::endl;
      map_offset = map_offset_tmp;
      if (std::fabs((*map_lc_points)[0].x - lc_data.point_x_) > 1 ||
          std::fabs((*map_lc_points)[0].y - lc_data.point_y_) > 1) {
        ROS_ERROR("update target error too much");
        enable_flag = false;
      }

      std::cout << "measurement: " << measurement_offset << std::endl;
      std::cout << "map: " << map_offset << std::endl;
    } else if (RUN_MODE == 2) {
      UTMCoor odom_coor;
      double car_x, car_y;
      common_utils::LL2UTM(state.p_[1], state.p_[0], &odom_coor);

      car_x = odom_coor.x + COOR_BIAS_X;
      car_y = odom_coor.y + COOR_BIAS_Y;

      Eigen::Vector3d map_offset_tmp((*map_lc_points)[0].x - car_x,
                                     (*map_lc_points)[0].y - car_y, 0.0);
      measurement_offset << lc_data.x_, lc_data.y_, lc_data.z_;
      std::cout << "map target: " << (*map_lc_points)[0].x << ' '
                << (*map_lc_points)[0].y << std::endl;
      std::cout << "measurement target: " << lc_data.point_x_ << ' '
                << lc_data.point_y_ << std::endl;
      std::cout << "map pos: " << car_x << ' ' << car_y << std::endl;
      std::cout << "measurement_pos: " << lc_data.gt_pos_x_ << " "
                << lc_data.gt_pos_y_ << std::endl;
      map_offset = map_offset_tmp;
      if (std::fabs((*map_lc_points)[0].x - lc_data.point_x_) > 1 ||
          std::fabs((*map_lc_points)[0].y - lc_data.point_y_) > 1) {
        ROS_ERROR("update target error too much");
        enable_flag = false;
      }

      std::cout << "measurement: " << measurement_offset << std::endl;
      std::cout << "map: " << map_offset << std::endl;
    }

    if (enable_flag) {
      // ROS_ERROR("update successfully");
      map_pos_cali_cnt_++;
      std::cout << "map pos correction count： " << map_pos_cali_cnt_
                << std::endl;
      Zk_.resize(3);
      Zk_.setZero();
      if (RUN_MODE == 0) {
        Zk_.segment<3>(0) = -state.earth_parameter_state_.Mrp *
                            (measurement_offset - map_offset);
      } else if (RUN_MODE == 2) {
        Zk_.segment<3>(0) = state.earth_parameter_state_.Mrp *
                            (measurement_offset - map_offset);
      }

      Hk_.resize(3, 9);
      Hk_.setZero();
      Hk_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      Rk_.resize(3, 3);
      Rk_.setZero();
      // Eigen::Vector3d rk_vector;
      // if (LON_MEAS_NOISE) {
      //   rk_vector = Eigen::Vector3d(
      //       pow(10, 6 * fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1) *
      //           1e-20,
      //       pow(10, 6 * fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1) *
      //           1e-20,
      //       pow(10, 6 * fabs(image_offset.norm() - 0.5 * ROAD_WIDTH) / 0.1) *
      //           1e-20);
      // } else {
      //   rk_vector = Eigen::Vector3d(1e-20, 1e-20, 1e-20);
      // }

      // Rk_ = rk_vector.asDiagonal();
      Rk_ = lc_data.meas_cov_;
      std::cout << "longitudinal pos correction variance: " << Rk_ << std::endl;
      Eigen::Matrix3d Py, Pyinv;
      Py = Hk_ * Pk_ * Hk_.transpose() + Rk_;  // S = H * P * H.transpose() + R;
      Pyinv.setIdentity();                     // solve Ax=B
      Py.llt().solveInPlace(Pyinv);
      K_.resize(9, 3);
      K_ = Pk_ * Hk_.transpose() * Pyinv;  // K = P*H.transpose()*S.inverse()
      Xk_ = Xk_ + K_ * (Zk_ - Hk_ * Xk_);
      Eigen::MatrixXd IKH_ = Eigen::Matrix<double, 9, 9>::Identity() - K_ * Hk_;
      Pk_ = IKH_ * Pk_ * IKH_.transpose() + K_ * Rk_ * K_.transpose();
      Pk_ = 0.5 * (Pk_ + Pk_.transpose()).eval();
    }
    map_lc_points->clear();
  }
}
void ErrorStateKalmanFilter::Reset() {
  for (int i = 0; i <= 8; i++) {
    Xk_(i) = 1e-20;
  }
}
void ErrorStateKalmanFilter::CalcMeanStandardDeviation(
    const std::vector<double>& target_vec, double* mean, double* deviation) {
  if (target_vec.size() != 0) {
    double sum = std::accumulate(target_vec.begin(), target_vec.end(), 0.0);
    *mean = sum / target_vec.size();
    double accum = 0.0;
    for (int i = 0; i <= target_vec.size() - 1; ++i) {
      accum += (target_vec[i] - *mean) * (target_vec[i] - *mean);
    }
    *deviation = sqrt(accum / target_vec.size());
  }
}