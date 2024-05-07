#include <geometry_msgs/PoseStamped.h>
#include <math_utils.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <navigation_system.h>
#include <node_parameters.h>
#include <sensor_data.h>
#include <system_parameters.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>

#include "common_utils.h"
std::string state_estimation_time_file =
    "/home/liuhj/state_estimation_time.csv";

vhdmap_se::NavigationSystem::NavigationSystem(
    ros::Publisher* system_pos_pub, ros::Publisher* system_odom_pub,
    ros::Publisher* request_lane_map_pub, ros::Publisher* request_lc_map_pub,
    ros::Publisher* request_sign_map_pub, ros::Publisher* map_lane_points_pub,
    ros::Publisher* map_lane_min_dist_points_pub,
    ros::Publisher* perpendicular_pub) {
  system_positon_publisher_ = system_pos_pub;
  system_odom_publisher_ = system_odom_pub;
  system_request_map_publisher_lane_ = request_lane_map_pub;
  system_request_map_publisher_lc_ = request_lc_map_pub;
  system_request_map_publisher_sign_ = request_sign_map_pub;
  system_map_lane_points_publisher_ = map_lane_points_pub;
  system_map_lane_min_dist_points_publisher_ = map_lane_min_dist_points_pub;
  system_perpendicular_publisher_ = perpendicular_pub;

  filter_a_ = new filters::ParallelFilterA();
  filter_b_ = new filters::ParallelFilterB();

  filter_a_->integrator_ = new Integrator(north_angle_);
  filter_b_->integrator_ = new Integrator(north_angle_);

  imu_data_buffer_.Allocate(10000);
  gnss_data_buffer_.Allocate(50);
  encoder_data_buffer_.Allocate(10000);
  lane_data_buffer_.Allocate(500);
  lc_data_buffer_.Allocate(500);
  // sign_data_buffer_.Allocate(30);
}
vhdmap_se::NavigationSystem::~NavigationSystem() {
  delete filter_a_;
  delete filter_b_;
}
bool vhdmap_se::NavigationSystem::InitializeSystem() {
  // System variable initialization
  system_status_ = INIT;
  straight_motion_flag_ = 0;
  straight_motion_flag_cnt_ = 0;
  postion_confirmation_flag_ = false;
  accept_lane_points_flag_ = true;

  encoder_scale_ = INITIAL_ENCODER_SCALE;
  north_angle_ = 3.316;  // Set an meaningless value to indicate that
                         // north angle has not been calibrated yet.
                         // north angle: 0-180 degrees. 3.316rad = 190 degrees.
  system_local_pos_bias_x_ = 0;
  system_local_pos_bias_y_ = 0;
  system_local_pos_x_ = 0;
  system_local_pos_y_ = 0;
  // System state initialization
  system_state_.p_ << math_utils::DegreesToRadians(INITIAL_LATITUDE),
      math_utils::DegreesToRadians(INITIAL_LONGITUDE),
      0;  // Fake LLA data for initialization
  sensor::ImuData first_imu_data;
  if (!imu_data_buffer_.GetFirstMeasurementData(&first_imu_data)) {
    ROS_INFO_STREAM("Wait for imu data to initialize the system.");
    return false;
  } else {
    system_state_.time_ = first_imu_data.timestamp_;
    system_state_.ori_time_ = first_imu_data.timestamp_;
    // system_state_.bg_ = first_imu_data.angular_velocity_;
    system_state_.bg_ << 0, 0, 0;
    system_state_.q_ =
        Eigen::Quaterniond(first_imu_data.attitude_quaternion_.w(),
                           first_imu_data.attitude_quaternion_.x(),
                           first_imu_data.attitude_quaternion_.y(),
                           first_imu_data.attitude_quaternion_.z());
  }
  system_initial_state_ = system_state_;
  filter_a_->Initialize(system_state_);
  filter_b_->Initialize(system_state_);

  // Mercator initialization
  system_proj_.SetAB(Re, Rp);
  system_proj_.SetB0(system_initial_state_.p_[0]);
  system_proj_.SetL0(system_initial_state_.p_[1]);

  // System mode configuration
  GNSS_data_fluctuation_resistance_ = GNSS_DATA_FLUCTUATION_RESISTANCE;
  GNSS_update_ = GNSS_UPDATE;
  lane_update_ = LANE_UPDATE;
  longitudinal_update_ = LONGITUDINAL_UPDATE;
  GNSS_pos_cali_ = GNSS_POS_CALI;
  GNSS_yaw_cali_ = GNSS_YAW_CALI;
  lane_pos_cali_ = LANE_POS_CALI;
  lane_yaw_cali_ = LANE_YAW_CALI;

  return true;
}
bool vhdmap_se::NavigationSystem::RunTheSystem() {
  if (!encoder_data_buffer_.Empty()) {
    sensor::EncoderData encoder_data_tmp;
    double last_gnss_data_time;
    double last_lane_data_time;
    double last_lc_data_time;
    double last_sign_data_time;
    auto main_start = std::chrono::high_resolution_clock::now();
    if (postion_confirmation_flag_) {
      if (GNSS_update_) {
        if (gnss_data_buffer_.GetLastTime(&last_gnss_data_time)) {
          while (!gnss_data_buffer_.Empty() &&
                 filter_a_->filter_state_.time_ < last_gnss_data_time) {
            gnss_data_buffer_.measurement_it_ =
                gnss_data_buffer_.measurement_map_.upper_bound(
                    filter_a_->filter_state_.time_);
            sensor::GNSSData gnss_data_for_update =
                gnss_data_buffer_.measurement_it_->second;
            double target_time = gnss_data_buffer_.measurement_it_->first;
            double last_encoder_data_time;
            encoder_data_buffer_.GetLastTime(&last_encoder_data_time);
            if (last_encoder_data_time < target_time) {
              ROS_WARN(
                  "Not enough encoder data to pursue gnss time, wait for "
                  "more...");
              break;
            }
            while (filter_a_->filter_state_.time_ < target_time) {
              encoder_data_buffer_.measurement_it_ =
                  encoder_data_buffer_.measurement_map_.upper_bound(
                      filter_a_->filter_state_.time_);
              double dt = std::min(encoder_data_buffer_.measurement_it_->first,
                                   target_time) -
                          filter_a_->filter_state_.time_;
              if (target_time < encoder_data_buffer_.measurement_it_->first) {
                CreateSuitableEncoderData(&encoder_data_tmp, target_time);
                filter_a_->integrator_->PosIntegrate(
                    encoder_data_tmp, &(filter_a_->filter_state_), dt,
                    north_angle_, encoder_scale_);
              } else {
                filter_a_->integrator_->PosIntegrate(
                    encoder_data_buffer_.measurement_it_->second,
                    &(filter_a_->filter_state_), dt, north_angle_,
                    encoder_scale_);
              }
              while (filter_a_->filter_state_.ori_time_ <
                     filter_a_->filter_state_.time_) {
                imu_data_buffer_.measurement_it_ =
                    imu_data_buffer_.measurement_map_.upper_bound(
                        filter_a_->filter_state_.ori_time_);
                if (imu_data_buffer_.measurement_it_ ==
                        imu_data_buffer_.measurement_map_.end() ||
                    imu_data_buffer_.measurement_it_->first >
                        filter_a_->filter_state_.time_) {
                  break;
                }
                double ori_dt =
                    std::min(imu_data_buffer_.measurement_it_->first,
                             filter_a_->filter_state_.time_) -
                    filter_a_->filter_state_.ori_time_;
                filter_a_->integrator_->OriIntegrate(
                    imu_data_buffer_.measurement_it_->second,
                    filter_a_->filter_state_, ori_dt);
              }
              filter_a_->eskf_->set_Fk(filter_a_->filter_state_, dt);
              filter_a_->eskf_->set_Gk(filter_a_->filter_state_, dt);
              filter_a_->eskf_->Predict();
            }
            filter_a_->eskf_->GNSSUpdate(
                filter_a_->filter_state_,
                gnss_data_buffer_.measurement_it_->second, north_angle_,
                GNSS_pos_cali_, GNSS_yaw_cali_);
            InjectErrorState();
            filter_a_->eskf_->Reset();
            if (filter_a_->filter_state_.DataValidationCheck()) {
              filter_b_->StateSynchronize(filter_a_);
            } else {
              ROS_WARN("State data invalid!");
              return false;
            }
          }
        }
      }
      // longitudinal correction
      if (longitudinal_update_) {
        if (lc_data_buffer_.GetLastTime(&last_lc_data_time)) {
          while (!lc_data_buffer_.Empty() &&
                 filter_a_->filter_state_.time_ < last_lc_data_time) {
            // std::cout << filter_a_->filter_state_.time_ << ' '
            //           << last_lc_data_time << std::endl;
            lc_data_buffer_.measurement_it_ =
                lc_data_buffer_.measurement_map_.upper_bound(
                    filter_a_->filter_state_.time_);
            sensor::LongitudinalConstraintData lc_data_for_update =
                lc_data_buffer_.measurement_it_->second;
            double target_time = lc_data_buffer_.measurement_it_->first;
            double last_encoder_data_time;
            encoder_data_buffer_.GetLastTime(&last_encoder_data_time);
            if (last_encoder_data_time < target_time) {
              ROS_WARN(
                  "Not enough encoder data to pursue longitudinal data time, "
                  "wait "
                  "for more...");
              break;
            }
            while (filter_a_->filter_state_.time_ < target_time) {
              encoder_data_buffer_.measurement_it_ =
                  encoder_data_buffer_.measurement_map_.upper_bound(
                      filter_a_->filter_state_.time_);
              double dt = std::min(encoder_data_buffer_.measurement_it_->first,
                                   target_time) -
                          filter_a_->filter_state_.time_;
              if (target_time < encoder_data_buffer_.measurement_it_->first) {
                std::cout << "create suitable encoder data." << std::endl;
                CreateSuitableEncoderData(&encoder_data_tmp, target_time);
                filter_a_->integrator_->PosIntegrate(
                    encoder_data_tmp, &(filter_a_->filter_state_), dt,
                    north_angle_, encoder_scale_);
              } else {
                filter_a_->integrator_->PosIntegrate(
                    encoder_data_buffer_.measurement_it_->second,
                    &(filter_a_->filter_state_), dt, north_angle_,
                    encoder_scale_);
              }
              std::cout << "pos integration finished" << std::endl;
              while (filter_a_->filter_state_.ori_time_ <
                     filter_a_->filter_state_.time_) {
                imu_data_buffer_.measurement_it_ =
                    imu_data_buffer_.measurement_map_.upper_bound(
                        filter_a_->filter_state_.ori_time_);
                if (imu_data_buffer_.measurement_it_ ==
                        imu_data_buffer_.measurement_map_.end() ||
                    imu_data_buffer_.measurement_it_->first >
                        filter_a_->filter_state_.time_) {
                  break;
                }
                double ori_dt =
                    std::min(imu_data_buffer_.measurement_it_->first,
                             filter_a_->filter_state_.time_) -
                    filter_a_->filter_state_.ori_time_;
                filter_a_->integrator_->OriIntegrate(
                    imu_data_buffer_.measurement_it_->second,
                    filter_a_->filter_state_, ori_dt);
              }
              std::cout << "ori integration finished" << std::endl;
              filter_a_->eskf_->set_Fk(filter_a_->filter_state_, dt);
              filter_a_->eskf_->set_Gk(filter_a_->filter_state_, dt);
              filter_a_->eskf_->Predict();
            }
            if (map_longitudinal_constraint_points_.size() == 0) {
              accept_longitudinal_constraint_flag_ = true;
              std::cout << "Requesting LC map" << std::endl;
              RequestMap(*system_request_map_publisher_lc_);
              lon_request_time_ = std::chrono::high_resolution_clock::now();
            } else {
              std::cout << "longitudinal correction" << std::endl;
              filter_a_->eskf_->LongitudinalConstraintUpdate(
                  filter_a_->filter_state_,
                  lc_data_buffer_.measurement_it_->second, north_angle_,
                  &map_longitudinal_constraint_points_, system_proj_,
                  system_local_pos_bias_x_, system_local_pos_bias_y_);
              std::cout << "longitudinal correction ends" << std::endl;
            }
            InjectErrorState();
            filter_a_->eskf_->Reset();
            if (filter_a_->filter_state_.DataValidationCheck()) {
              filter_b_->StateSynchronize(filter_a_);
            } else {
              ROS_WARN("State data invalid!");
              return false;
            }
          }
        }
      }
      // lane_update
      if (lane_update_) {
        if (lane_data_buffer_.GetLastTime(&last_lane_data_time)) {
          std::cout << "lane info data buffer not empty." << std::endl;
          // std::cout << std::fixed << std::setprecision(10)
          //           << filter_a_->filter_state_.time_ << ' '
          //           << last_lane_data_time << std::endl;
          while (!lane_data_buffer_.Empty() &&
                 filter_a_->filter_state_.time_ < last_lane_data_time) {
            // std::cout << filter_a_->filter_state_.time_ << ' '
            //           << last_lane_data_time << std::endl;
            lane_data_buffer_.measurement_it_ =
                lane_data_buffer_.measurement_map_.upper_bound(
                    filter_a_->filter_state_.time_);
            sensor::LaneData lane_data_for_update =
                lane_data_buffer_.measurement_it_->second;
            double target_time = lane_data_buffer_.measurement_it_->first;
            double last_encoder_data_time;
            encoder_data_buffer_.GetLastTime(&last_encoder_data_time);
            if (last_encoder_data_time < target_time) {
              ROS_WARN(
                  "Not enough encoder data to pursue lane data time, wait "
                  "for more...");
              break;
            }
            while (filter_a_->filter_state_.time_ < target_time) {
              encoder_data_buffer_.measurement_it_ =
                  encoder_data_buffer_.measurement_map_.upper_bound(
                      filter_a_->filter_state_.time_);
              double dt = std::min(encoder_data_buffer_.measurement_it_->first,
                                   target_time) -
                          filter_a_->filter_state_.time_;
              if (target_time < encoder_data_buffer_.measurement_it_->first) {
                std::cout << "create suitable encoder data." << std::endl;
                CreateSuitableEncoderData(&encoder_data_tmp, target_time);
                // UTMCoor odom_coor;
                // common_utils::LL2UTM(filter_a_->filter_state_.p_[1],
                //                      filter_a_->filter_state_.p_[0],
                //                      &odom_coor);
                // std::cout << "filter a pos: " << odom_coor.x + COOR_BIAS_X
                //           << ' ' << odom_coor.y + COOR_BIAS_Y << std::endl;
                filter_a_->integrator_->PosIntegrate(
                    encoder_data_tmp, &(filter_a_->filter_state_), dt,
                    north_angle_, encoder_scale_);

                // common_utils::LL2UTM(filter_a_->filter_state_.p_[1],
                //                      filter_a_->filter_state_.p_[0],
                //                      &odom_coor);
                // std::cout << "filter a pos: " << odom_coor.x + COOR_BIAS_X
                //           << ' ' << odom_coor.y + COOR_BIAS_Y << std::endl;
              } else {
                // UTMCoor odom_coor;
                // common_utils::LL2UTM(filter_a_->filter_state_.p_[1],
                //                      filter_a_->filter_state_.p_[0],
                //                      &odom_coor);
                // std::cout << "filter a pos: " << odom_coor.x + COOR_BIAS_X
                //           << ' ' << odom_coor.y + COOR_BIAS_Y << std::endl;
                filter_a_->integrator_->PosIntegrate(
                    encoder_data_buffer_.measurement_it_->second,
                    &(filter_a_->filter_state_), dt, north_angle_,
                    encoder_scale_);

                // common_utils::LL2UTM(filter_a_->filter_state_.p_[1],
                //                      filter_a_->filter_state_.p_[0],
                //                      &odom_coor);
                // std::cout << "filter a pos: " << odom_coor.x + COOR_BIAS_X
                //           << ' ' << odom_coor.y + COOR_BIAS_Y << std::endl;
              }
              while (filter_a_->filter_state_.ori_time_ <
                     filter_a_->filter_state_.time_) {
                imu_data_buffer_.measurement_it_ =
                    imu_data_buffer_.measurement_map_.upper_bound(
                        filter_a_->filter_state_.ori_time_);
                if (imu_data_buffer_.measurement_it_ ==
                        imu_data_buffer_.measurement_map_.end() ||
                    imu_data_buffer_.measurement_it_->first >
                        filter_a_->filter_state_.time_) {
                  break;
                }
                double ori_dt =
                    std::min(imu_data_buffer_.measurement_it_->first,
                             filter_a_->filter_state_.time_) -
                    filter_a_->filter_state_.ori_time_;
                filter_a_->integrator_->OriIntegrate(
                    imu_data_buffer_.measurement_it_->second,
                    filter_a_->filter_state_, ori_dt);
              }
              filter_a_->eskf_->set_Fk(filter_a_->filter_state_, dt);
              filter_a_->eskf_->set_Gk(filter_a_->filter_state_, dt);
              filter_a_->eskf_->Predict();
            }
            if (map_lane_points_.size() == 0) {
              accept_lane_points_flag_ = true;
              std::cout << "request lane map" << std::endl;
              RequestMap(*system_request_map_publisher_lane_);
              lane_request_time_ = std::chrono::high_resolution_clock::now();
            } else {
              int left_lane_min_dist_point_index,
                  right_lane_min_dist_point_index;
              double car_x, car_y, k, b;
              filter_a_->eskf_->LaneUpdate(
                  filter_a_->filter_state_,
                  lane_data_buffer_.measurement_it_->second,
                  straight_motion_flag_, north_angle_, &map_lane_points_,
                  &map_lane_points_flags_, system_proj_,
                  system_local_pos_bias_x_, system_local_pos_bias_y_,
                  &map_response_lane_index_, lane_pos_cali_, lane_yaw_cali_,
                  &left_lane_min_dist_point_index,
                  &right_lane_min_dist_point_index, &car_x, &car_y, &k, &b);
              PubMapLanePoints();
              PubMapLaneMinDistPoints(left_lane_min_dist_point_index,
                                      right_lane_min_dist_point_index);
              VisualizePerpendicular(car_x, car_y, k, b,
                                     left_lane_min_dist_point_index,
                                     right_lane_min_dist_point_index);
            }
            InjectErrorState();
            filter_a_->eskf_->Reset();
            if (filter_a_->filter_state_.DataValidationCheck()) {
              filter_b_->StateSynchronize(filter_a_);
            } else {
              ROS_WARN("State data invalid!");
              return false;
            }
          }
        }
      }
    }
    while ((encoder_data_buffer_.measurement_it_ =
                encoder_data_buffer_.measurement_map_.upper_bound(
                    filter_b_->filter_state_.time_)) !=
           encoder_data_buffer_.measurement_map_.end()) {
      double dt = encoder_data_buffer_.measurement_it_->first -
                  filter_b_->filter_state_.time_;
      // UTMCoor odom_coor;
      // common_utils::LL2UTM(filter_b_->filter_state_.p_[1],
      //                      filter_b_->filter_state_.p_[0], &odom_coor);
      // std::cout << "filter b pos: " << odom_coor.x + COOR_BIAS_X << ' '
      //           << odom_coor.y + COOR_BIAS_Y << std::endl;
      filter_b_->integrator_->PosIntegrate(
          encoder_data_buffer_.measurement_it_->second,
          &(filter_b_->filter_state_), dt, north_angle_, encoder_scale_);

      // common_utils::LL2UTM(filter_b_->filter_state_.p_[1],
      //                      filter_b_->filter_state_.p_[0], &odom_coor);
      // std::cout << "filter b pos: " << odom_coor.x + COOR_BIAS_X << ' '
      //           << odom_coor.y + COOR_BIAS_Y << std::endl;
      while (filter_b_->filter_state_.ori_time_ <
             filter_b_->filter_state_.time_) {
        imu_data_buffer_.measurement_it_ =
            imu_data_buffer_.measurement_map_.upper_bound(
                filter_b_->filter_state_.ori_time_);
        if (imu_data_buffer_.measurement_it_ ==
                imu_data_buffer_.measurement_map_.end() ||
            imu_data_buffer_.measurement_it_->first >
                filter_b_->filter_state_.time_) {
          break;
        }
        double ori_dt = std::min(imu_data_buffer_.measurement_it_->first,
                                 filter_b_->filter_state_.time_) -
                        filter_b_->filter_state_.ori_time_;
        filter_b_->integrator_->OriIntegrate(
            imu_data_buffer_.measurement_it_->second, filter_b_->filter_state_,
            ori_dt);
      }
    }
    system_state_ = filter_b_->filter_state_;
    PublishMsg();
    auto main_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> main_duration = main_end - main_start;
    std::cout << BLUE << "The main part took " << main_duration.count()
              << " seconds to execute." << COLOR_RESET << std::endl;
    std::ofstream out_file(state_estimation_time_file, std::ios::app);
    if (out_file.is_open()) {
      out_file << std::to_string(main_duration.count()) << std::endl;
      out_file.close();
    } else {
      std::cout << "file not open" << std::endl;
    }
    return true;
  } else {
    ROS_WARN("No Encoder data!");
  }
}
bool vhdmap_se::NavigationSystem::IsSystemStationary(
    const sensor::GNSSData& last_data,
    const sensor::GNSSData& penultimate_data) {
  Eigen::Vector3d tn = system_state_.earth_parameter_state_.Mpr *
                       (last_data.geodetic_coordinates_ -
                        penultimate_data.geodetic_coordinates_);
  if (sqrt(tn.x() * tn.x() + tn.y() * tn.y()) < STATIONARY_THRESHOLD) {
    std::cout << RED << "system stationary" << COLOR_RESET << std::endl;
    return true;
  } else {
    return false;
  }
}
void vhdmap_se::NavigationSystem::CreateSuitableEncoderData(
    sensor::EncoderData* encoder_data, const double time) {
  std::map<double, sensor::EncoderData>::iterator it =
      encoder_data_buffer_.measurement_map_.upper_bound(time);
  float next_rear_beam_center_tmp;
  next_rear_beam_center_tmp = it->second.rear_beam_center;
  double next_time = it->first;
  --it;
  float previous_rear_beam_tmp;
  previous_rear_beam_tmp = it->second.rear_beam_center;
  double previous_time = it->first;
  encoder_data->rear_beam_center =
      (time - previous_time) *
          ((next_rear_beam_center_tmp - previous_rear_beam_tmp) /
           (next_time - previous_time)) +
      previous_rear_beam_tmp;
}
bool vhdmap_se::NavigationSystem::AlignImu(const double& target_time,
                                              State* state) {
  std::map<double, sensor::ImuData>::iterator it, itr, it_tmp;
  // The yaw angle in IMU is the angle of rotation from the initial position
  // north_bias is the clockwise rotation angle relative to due north direction
  if (imu_data_buffer_.measurement_map_.empty()) {
    ROS_WARN("No imu data available, if this continues, system may crash");
    return false;
  } else {
    it = imu_data_buffer_.measurement_map_.begin();
    itr = it;
    while ((it->first - target_time) * (itr->first - target_time) > 0 &&
           it != imu_data_buffer_.measurement_map_.end()) {
      itr = it;
      it++;
    }
    if (it != imu_data_buffer_.measurement_map_.end()) {
      if ((it->first - target_time) * (itr->first - target_time) == 0) itr = it;
    }
    sensor::ImuData imu_data = itr->second;
    tf2::Quaternion tf2_q(
        imu_data.attitude_quaternion_.x(), imu_data.attitude_quaternion_.y(),
        imu_data.attitude_quaternion_.z(), imu_data.attitude_quaternion_.w());
    tf2_q.normalize();
    Eigen::Quaterniond q_tmp(tf2_q.w(), tf2_q.x(), tf2_q.y(), tf2_q.z());
    q_tmp.norm();
    state->q_ = q_tmp;
    return true;
  }
}
void vhdmap_se::NavigationSystem::InjectErrorState() {
  std::cout << RED << "Pos Before: " << std::fixed << std::setprecision(10)
            << filter_a_->filter_state_.p_ << COLOR_RESET << std::endl;
  filter_a_->filter_state_.p_ -= filter_a_->eskf_->Xk_.segment<3>(0);
  std::cout << GREEN << "Pos After: " << std::fixed << std::setprecision(10)
            << filter_a_->filter_state_.p_ << COLOR_RESET << std::endl;
  tf2::Quaternion tf2_q1(
      filter_a_->filter_state_.q_.x(), filter_a_->filter_state_.q_.y(),
      filter_a_->filter_state_.q_.z(), filter_a_->filter_state_.q_.w());
  double yaw, pitch, roll;
  tf2::Matrix3x3(tf2_q1).getRPY(roll, pitch, yaw);
  filter_a_->filter_state_.q_ =
      math_utils::axis2Quat(filter_a_->eskf_->Xk_.segment<3>(3)) *
      filter_a_->filter_state_.q_;
  tf2::Quaternion tf2_q2(
      filter_a_->filter_state_.q_.x(), filter_a_->filter_state_.q_.y(),
      filter_a_->filter_state_.q_.z(), filter_a_->filter_state_.q_.w());
  double yaw1, pitch1, roll1;
  tf2::Matrix3x3(tf2_q2).getRPY(roll1, pitch1, yaw1);
  if (RUN_MODE == 0) {
    common_utils::CalcYawEarthFromYaw(yaw, 1, north_angle_, &yaw);
  } else if (RUN_MODE == 2) {
    common_utils::CalcYawEarthFromYawENU(yaw, &yaw);
  }
  std::cout << RED << "Yaw Before: " << std::fixed << std::setprecision(10)
            << math_utils::RadiansToDegrees(yaw) << COLOR_RESET << std::endl;
  if (RUN_MODE == 0) {
    common_utils::CalcYawEarthFromYaw(yaw1, 1, north_angle_, &yaw1);
  } else if (RUN_MODE == 2) {
    common_utils::CalcYawEarthFromYawENU(yaw1, &yaw1);
  }
  std::cout << GREEN << "Yaw After: " << std::fixed << std::setprecision(10)
            << math_utils::RadiansToDegrees(yaw1) << COLOR_RESET << std::endl;
  filter_a_->filter_state_.bg_ = filter_a_->eskf_->Xk_.segment<3>(6);
  filter_a_->filter_state_.earth_parameter_state_.UpdateEarthParams(
      filter_a_->filter_state_.p_, filter_a_->filter_state_.v_);
}
void vhdmap_se::NavigationSystem::PublishMsg() {
  sensor_msgs::NavSatFix position_msg;
  nav_msgs::Odometry odom_msg;
  position_msg.latitude = math_utils::RadiansToDegrees(system_state_.p_[0]);
  position_msg.longitude = math_utils::RadiansToDegrees(system_state_.p_[1]);
  position_msg.altitude = system_state_.p_[2];
  position_msg.header.frame_id = "map";
  // position_msg.header.stamp = system_state_.time_;
  system_positon_publisher_->publish(position_msg);

  // path_pos_stamped_msg.pose.position.y =
  //     100000 * (position_msg.latitude -
  //               math_utils::RadiansToDegrees(system_initial_state_.p_[0]));
  // path_pos_stamped_msg.pose.position.x =
  //     100000 * (position_msg.longitude -
  //               math_utils::RadiansToDegrees(system_initial_state_.p_[1]));

  odom_msg.header.frame_id = "map";
  // odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.stamp = ros::Time().fromSec(system_state_.time_);

  if (RUN_MODE == 0) {
    system_proj_.ToProj(system_state_.p_[0], system_state_.p_[1],
                        system_local_pos_x_, system_local_pos_y_);
    odom_msg.pose.pose.position.x =
        -(system_local_pos_y_ + system_local_pos_bias_y_);
    odom_msg.pose.pose.position.y =
        (system_local_pos_x_ + system_local_pos_bias_x_);

    tf2::Quaternion tf2_q1(system_state_.q_.x(), system_state_.q_.y(),
                           system_state_.q_.z(), system_state_.q_.w());
    double yaw1, pitch1, roll1;
    tf2::Matrix3x3(tf2_q1).getRPY(roll1, pitch1, yaw1);
    double yaw_earth;
    common_utils::CalcYawEarthFromYaw(
        yaw1, filter_b_->integrator_->north_angle_cali_flag_, north_angle_,
        &yaw_earth);
    // if (filter_b_->integrator_->north_angle_cali_flag_) {
    //   north_bias = yaw1 - north_angle_;
    //   if (north_bias < 0) {
    //     north_bias += 2 * M_PI;
    //   }
    // } else {
    //   north_bias = yaw1;
    //   if (north_bias < 0) {
    //     north_bias += 2 * M_PI;
    //   }
    // }
    double yaw_rviz = 0;
    common_utils::CalcYawRVizFromYawEarth(yaw_earth, &yaw_rviz);

    tf2::Quaternion tf2_q2;
    // tf2_q2.setRPY(roll1, pitch1, M_PI - yaw_rviz);
    tf2_q2.setRPY(roll1, pitch1, yaw_rviz);
    tf2_q2.normalize();
    odom_msg.pose.pose.orientation.w = tf2_q2.w();
    odom_msg.pose.pose.orientation.x = tf2_q2.x();
    odom_msg.pose.pose.orientation.y = tf2_q2.y();
    odom_msg.pose.pose.orientation.z = tf2_q2.z();
  } else if (RUN_MODE == 1) {
    UTMCoor odom_coor;
    common_utils::LL2UTM(system_state_.p_[1], system_state_.p_[0], &odom_coor);
    odom_msg.pose.pose.position.x = odom_coor.x;
    odom_msg.pose.pose.position.y = odom_coor.y;
    tf2::Quaternion tf2_q1(system_state_.q_.x(), system_state_.q_.y(),
                           system_state_.q_.z(), system_state_.q_.w());
    double yaw1, pitch1, roll1;
    tf2::Matrix3x3(tf2_q1).getRPY(roll1, pitch1, yaw1);
    double yaw_earth;
    common_utils::CalcYawEarthFromYaw(
        yaw1, filter_b_->integrator_->north_angle_cali_flag_, north_angle_,
        &yaw_earth);
    double yaw_enu = 0;
    common_utils::CalcYawENUFromYawEarth(yaw_earth, &yaw_enu);
    // double yaw_rviz = 0;
    // common_utils::CalcYawRVizFromYawEarth(yaw_earth, &yaw_rviz);

    tf2::Quaternion tf2_q2;
    tf2_q2.setRPY(roll1, pitch1, yaw_enu);
    tf2_q2.normalize();
    // odom_msg.pose.pose.orientation.w = tf2_q2.w();
    // odom_msg.pose.pose.orientation.x = tf2_q2.x();
    // odom_msg.pose.pose.orientation.y = tf2_q2.y();
    // odom_msg.pose.pose.orientation.z = tf2_q2.z();
    odom_msg.pose.pose.orientation.w = system_state_.q_.w();
    odom_msg.pose.pose.orientation.x = system_state_.q_.x();
    odom_msg.pose.pose.orientation.y = system_state_.q_.y();
    odom_msg.pose.pose.orientation.z = system_state_.q_.z();
  } else {
    UTMCoor odom_coor;
    common_utils::LL2UTM(system_state_.p_[1], system_state_.p_[0], &odom_coor);
    odom_msg.pose.pose.position.x = odom_coor.x + COOR_BIAS_X;
    odom_msg.pose.pose.position.y = odom_coor.y + COOR_BIAS_Y;
    tf2::Quaternion tf2_q1(system_state_.q_.x(), system_state_.q_.y(),
                           system_state_.q_.z(), system_state_.q_.w());
    double yaw1, pitch1, roll1;
    tf2::Matrix3x3(tf2_q1).getRPY(roll1, pitch1, yaw1);
    std::cout << "yaw enu from imu: " << math_utils::RadiansToDegrees(yaw1)
              << std::endl;
    double yaw_earth;
    common_utils::CalcYawEarthFromYaw(
        yaw1, filter_b_->integrator_->north_angle_cali_flag_, north_angle_,
        &yaw_earth);
    double yaw_enu = 0;
    common_utils::CalcYawENUFromYawEarth(yaw_earth, &yaw_enu);

    // tf2::Quaternion tf2_q2;
    // tf2_q2.setRPY(roll1, pitch1, yaw_enu);
    // tf2_q2.normalize();
    tf2::Quaternion tf2_q2;
    tf2_q2.setRPY(roll1, pitch1, yaw1+0.5*M_PI);
    tf2_q2.normalize();
    // odom_msg.pose.pose.orientation.w = system_state_.q_.w();
    // odom_msg.pose.pose.orientation.x = system_state_.q_.x();
    // odom_msg.pose.pose.orientation.y = system_state_.q_.y();
    // odom_msg.pose.pose.orientation.z = system_state_.q_.z();
    odom_msg.pose.pose.orientation.w = tf2_q2.w();
    odom_msg.pose.pose.orientation.x = tf2_q2.x();
    odom_msg.pose.pose.orientation.y = tf2_q2.y();
    odom_msg.pose.pose.orientation.z = tf2_q2.z();
  }

  system_odom_publisher_->publish(odom_msg);
}
void vhdmap_se::NavigationSystem::CalibrateEncoderScale(double gnss_speed,
                                                           double time) {
  double sum = 0;
  double encoder_average_speed = 0;
  std::map<double, sensor::EncoderData>::iterator it, itr, it_tmp;
  if (encoder_data_buffer_.measurement_map_.empty()) {
    ROS_WARN("No encoder data available, calibration will be abandoned");
  } else {
    it = encoder_data_buffer_.measurement_map_.begin();
    itr = it;
    it_tmp = it;
    double min = abs(it->first - time);
    while ((it->first - time) * (itr->first - time) > 0 &&
           it != encoder_data_buffer_.measurement_map_.end()) {
      if (abs(it->first - time) <= min) {
        it_tmp = it;
        min = abs(it->first - time);
      }
      itr = it;
      it++;
    }
    if (abs(it->first - time) <= min &&
        it != encoder_data_buffer_.measurement_map_.end()) {
      it_tmp = it;
    }
  }
  int a = 0;
  int num = static_cast<int>(ENCODER_TOPIC_FREQUENCY / GNSS_TOPIC_FREQUENCY);
  for (int i = 0; i <= num; i++) {
    sum += it_tmp->second.rear_beam_center;
    if (it_tmp == encoder_data_buffer_.measurement_map_.begin()) {
      a = i;
      break;
    } else {
      it_tmp--;
    }
    a = i;
  }
  encoder_average_speed = sum / (a + 1);
  double encoder_scale_tmp;
  if (encoder_average_speed != 0)
    encoder_scale_tmp = gnss_speed / encoder_average_speed;
  encoder_scale_vec_.push_back(encoder_scale_tmp);
  uint16_t vec_size = encoder_scale_vec_.size();
  if (vec_size > 10) {
    encoder_scale_vec_.erase(encoder_scale_vec_.begin());
    double encoder_scale_sum =
        accumulate(encoder_scale_vec_.begin(), encoder_scale_vec_.end(), 0.0);
    encoder_scale_ = encoder_scale_sum / 10.0;
  } else if (vec_size != 0) {
    double encoder_scale_sum =
        accumulate(encoder_scale_vec_.begin(), encoder_scale_vec_.end(), 0.0);
    encoder_scale_ = encoder_scale_sum / vec_size;
  } else {
  }
}
void vhdmap_se::NavigationSystem::RequestMap(
    const ros::Publisher& request_pub) {
  nav_msgs::Odometry request_msg;
  if (RUN_MODE == 0) {
    tf2::Quaternion tf2_q1(
        filter_a_->filter_state_.q_.x(), filter_a_->filter_state_.q_.y(),
        filter_a_->filter_state_.q_.z(), filter_a_->filter_state_.q_.w());
    double yaw1, pitch1, roll1;
    tf2::Matrix3x3(tf2_q1).getRPY(roll1, pitch1, yaw1);
    double north_bias;
    if (filter_b_->integrator_->north_angle_cali_flag_) {
      north_bias = yaw1 - north_angle_;
      if (north_bias < 0) {
        north_bias += 2 * M_PI;
      }
    } else {
      north_bias = yaw1;
      if (north_bias < 0) {
        north_bias += 2 * M_PI;
      }
    }
    double yaw_rviz = 0;
    /*
     * Yaw in the rviz coordinate system is referenced to the x-axis, and
     * rotating in the positive direction of the y-axis is a positive value.
     * Therefore, the orientation needs to be converted.
     */
    if (north_bias >= 0 && north_bias <= 1.5 * M_PI) {
      yaw_rviz = 0.5 * M_PI - north_bias;
    } else {
      yaw_rviz = 2.5 * M_PI - north_bias;
    }
    double current_x, current_y;
    system_proj_.ToProj(filter_a_->filter_state_.p_[0],
                        filter_a_->filter_state_.p_[1], current_x, current_y);

    request_msg.pose.pose.position.y =
        (current_x +
         system_local_pos_bias_x_);  //+ VEHICLE_LENGTH * cos(north_bias);
    request_msg.pose.pose.position.x =
        -(current_y +
          system_local_pos_bias_y_);  //+ VEHICLE_LENGTH * sin(north_bias);
    request_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion tf2_q2;
    tf2_q2.setRPY(roll1, pitch1, M_PI - yaw_rviz);
    tf2_q2.normalize();

    request_msg.pose.pose.orientation.x = tf2_q2.x();
    request_msg.pose.pose.orientation.y = tf2_q2.y();
    request_msg.pose.pose.orientation.z = tf2_q2.z();
    request_msg.pose.pose.orientation.w = tf2_q2.w();
  } else if (RUN_MODE == 1) {
    UTMCoor odom_coor;
    common_utils::LL2UTM(filter_a_->filter_state_.p_[1],
                         filter_a_->filter_state_.p_[0], &odom_coor);
    request_msg.pose.pose.position.x = odom_coor.x;
    request_msg.pose.pose.position.y = odom_coor.y;

    request_msg.pose.pose.orientation.w = filter_a_->filter_state_.q_.w();
    request_msg.pose.pose.orientation.x = filter_a_->filter_state_.q_.x();
    request_msg.pose.pose.orientation.y = filter_a_->filter_state_.q_.y();
    request_msg.pose.pose.orientation.z = filter_a_->filter_state_.q_.z();
  } else {
    UTMCoor odom_coor;
    common_utils::LL2UTM(filter_a_->filter_state_.p_[1],
                         filter_a_->filter_state_.p_[0], &odom_coor);
    request_msg.pose.pose.position.x = odom_coor.x + COOR_BIAS_X;
    request_msg.pose.pose.position.y = odom_coor.y + COOR_BIAS_Y;

    request_msg.pose.pose.orientation.w = filter_a_->filter_state_.q_.w();
    request_msg.pose.pose.orientation.x = filter_a_->filter_state_.q_.x();
    request_msg.pose.pose.orientation.y = filter_a_->filter_state_.q_.y();
    request_msg.pose.pose.orientation.z = filter_a_->filter_state_.q_.z();
  }

  request_msg.header.frame_id = "map";
  request_msg.header.stamp = ros::Time::now();
  request_pub.publish(request_msg);
  std::cout << "request for map published." << std::endl;
}
double vhdmap_se::NavigationSystem::CalcYawEarthFromGNSS(
    const sensor::GNSSData current, const sensor::GNSSData previous) {
  double Ec = Rp + (Re - Rp) *
                       (90. - math_utils::RadiansToDegrees(
                                  previous.geodetic_coordinates_.x())) /
                       90.;
  double Ed = Ec * cos(previous.geodetic_coordinates_.x());
  double dx =
      (current.geodetic_coordinates_.y() - previous.geodetic_coordinates_.y()) *
      Ed;
  double dy =
      (current.geodetic_coordinates_.x() - previous.geodetic_coordinates_.x()) *
      Ec;
  double angle = 0.0;
  angle = math_utils::RadiansToDegrees(atan(fabs(dx / dy)));
  double dLo =
      current.geodetic_coordinates_.y() - previous.geodetic_coordinates_.y();
  double dLa =
      current.geodetic_coordinates_.x() - previous.geodetic_coordinates_.x();
  if (dLo > 0 && dLa <= 0) {
    angle = (90. - angle) + 90;
  } else if (dLo <= 0 && dLa < 0) {
    angle = angle + 180.;
  } else if (dLo < 0 && dLa >= 0) {
    angle = (90. - angle) + 270;
  }
  return angle;
}
void vhdmap_se::NavigationSystem::PubMapLanePoints() {
  sensor_msgs::PointCloud map_lane_pc_msg;
  int size = map_lane_points_.size();
  for (int i = 0; i < size; i++) {
    geometry_msgs::Point32 point;
    point.x = map_lane_points_[i].x;
    point.y = map_lane_points_[i].y;
    point.z = map_lane_points_[i].z;
    map_lane_pc_msg.points.emplace_back(point);
  }
  map_lane_pc_msg.header.frame_id = "map";
  map_lane_pc_msg.header.stamp = ros::Time().now();
  system_map_lane_points_publisher_->publish(map_lane_pc_msg);
}
void vhdmap_se::NavigationSystem::PubMapLaneMinDistPoints(
    const int& left_point_index, const int& right_point_index) {
  sensor_msgs::PointCloud map_lane_min_dist_pc_msg;

  geometry_msgs::Point32 left_lane_point, right_lane_point;
  // left_lane_point.x = map_lane_points_[left_point_index].x;
  // left_lane_point.y = map_lane_points_[left_point_index].y;
  // left_lane_point.z = map_lane_points_[left_point_index].z;
  // map_lane_min_dist_pc_msg.points.emplace_back(left_lane_point);
  right_lane_point.x = map_lane_points_[right_point_index].x;
  right_lane_point.y = map_lane_points_[right_point_index].y;
  right_lane_point.z = map_lane_points_[right_point_index].z;
  map_lane_min_dist_pc_msg.points.emplace_back(right_lane_point);

  map_lane_min_dist_pc_msg.header.frame_id = "map";
  map_lane_min_dist_pc_msg.header.stamp = ros::Time().now();
  system_map_lane_min_dist_points_publisher_->publish(map_lane_min_dist_pc_msg);
}
void vhdmap_se::NavigationSystem::VisualizePerpendicular(
    const double& car_x, const double& car_y, const double& k, const double& b,
    const int& left_point_index, const int& right_point_index) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "/VHDMap_SE/perpendicular";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.scale.x = 0.3;  // 0.15
  marker.scale.y = 0.3;  // 0.15
  std_msgs::ColorRGBA green, yellow, gray;
  green.a = gray.a = yellow.a = 0.65;
  green.g = 1;
  yellow.r = yellow.g = 1;
  marker.color = green;
  marker.pose.orientation.w = 1;
  visualization_msgs::MarkerArray perpendicular;
  geometry_msgs::Point geo_point_car, geo_point_left, geo_point_right;

  marker.id = 1;
  int count = 1;
  geo_point_car.x = car_x;
  geo_point_car.y = car_y;
  marker.points.emplace_back(geo_point_car);

  geo_point_left.x = map_lane_points_[left_point_index].x;
  geo_point_left.y = k * geo_point_left.x + b;
  marker.points.emplace_back(geo_point_left);
  geo_point_right.x = map_lane_points_[right_point_index].x;
  geo_point_right.y = k * geo_point_right.x + b;
  marker.points.emplace_back(geo_point_right);
  // std::cout<<"now is "<<count<<" line"<<std::endl;
  // count ++;
  perpendicular.markers.push_back(marker);
  marker.id = marker.id + 1;
  marker.points.clear();

  system_perpendicular_publisher_->publish(perpendicular);
}