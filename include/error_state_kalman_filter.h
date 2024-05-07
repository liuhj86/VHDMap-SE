#ifndef VHDMAPSE_INCLUDE_ERROR_STATE_KALMAN_FILTER_H_
#define VHDMAPSE_INCLUDE_ERROR_STATE_KALMAN_FILTER_H_

#include <Eigen/Dense>
#include <vector>

#include "common_utils.h"
#include "mercator.h"
#include "sensor_data.h"
#include "state.hpp"
class ErrorStateKalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
  ErrorStateKalmanFilter() {}
  ~ErrorStateKalmanFilter() {}
  void Initialize();
  void set_Fk(const State& state,
              double dt);  
  void set_Gk(const State& state, double dt);
  void Predict();
  void GNSSUpdate(const State& state, const sensor::GNSSData& gnss_data,
                  const double north_angle, bool pos_cali, bool yaw_cali);
  void LaneUpdate(const State& state, sensor::LaneData& lane_data,
                  const bool& straight_motion_flag, double& north_angle,
                  std::vector<MapPoint>* map_lane_points,
                  std::vector<uint16_t>* map_lane_points_flags,
                  MercatorProj& mercator, const double& bias_x,
                  const double& bias_y, int* msg_index, const bool& pos_cali,
                  const bool& yaw_cali, int* left_lane_min_dist_point_index,
                  int* right_lane_min_dist_point_index, double* car_x,
                  double* car_y, double* k, double* b);
  void LongitudinalConstraintUpdate(
      const State& state, const sensor::LongitudinalConstraintData& lc_data,
      const double north_angle, std::vector<MapPoint>* lc_points,
      const MercatorProj mercator, const double bias_x, const double bias_y);

  void Reset();
  void CalcMeanStandardDeviation(const std::vector<double>& target_vec,
                                 double* mean, double* deviation);

  Eigen::MatrixXd Fk_;
  Eigen::MatrixXd Gk_;
  Eigen::MatrixXd Pk_;
  Eigen::MatrixXd Qk_;

  Eigen::VectorXd Zk_;
  Eigen::MatrixXd Hk_;
  Eigen::MatrixXd Rk_;
  Eigen::MatrixXd K_;

  Eigen::VectorXd Xk_;


  int GNSS_pos_cali_cnt_;
  int map_pos_cali_cnt_;
};
#endif  // VHDMAPSE_INCLUDE_ERROR_STATE_KALMAN_FILTER_H_
