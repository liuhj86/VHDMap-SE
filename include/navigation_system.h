#ifndef VHDMAPSE_INCLUDE_NAVIGATION_SYSTEM_H_
#define VHDMAPSE_INCLUDE_NAVIGATION_SYSTEM_H_

#include <nav_msgs/Path.h>
#include <parallel_filters.h>
#include <ros/ros.h>
#include <sensor_data.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>

#include <buffer.hpp>
#include <chrono>
#include <state.hpp>
#include <vector>

#include "mercator.h"

namespace vhdmap_se {
class NavigationSystem {
 public:
  enum SystemStatus { INIT = 0, RUNNING = 1, RESET = 2 };
  NavigationSystem(ros::Publisher* system_pos_pub,
                      ros::Publisher* system_odom_pub,
                      ros::Publisher* request_lane_map_pub,
                      ros::Publisher* request_lc_map_pub,
                      ros::Publisher* request_sign_map_pub,
                      ros::Publisher* map_lane_points_pub,
                      ros::Publisher* map_lane_min_dist_points_pub,
                      ros::Publisher* perpendicular_pub);
  ~NavigationSystem();
  bool InitializeSystem();
  bool RunTheSystem();
  bool IsSystemStationary(const sensor::GNSSData& last_data,
                          const sensor::GNSSData& penultimate_data);
  void CreateSuitableEncoderData(sensor::EncoderData* encoder_data,
                                 const double time);
  bool AlignImu(const double& current_time, State* state);
  void InjectErrorState();
  void PublishMsg();
  void CalibrateEncoderScale(double gnss_speed, double time);
  double CalcYawEarthFromGNSS(const sensor::GNSSData current,
                              const sensor::GNSSData previous);
  void RequestMap(const ros::Publisher& request_pub);
  void PubMapLanePoints();
  void PubMapLaneMinDistPoints(const int& left_point_index,
                               const int& right_point_index);
  void VisualizePerpendicular(const double& car_x, const double& car_y,
                              const double& k, const double& b,
                              const int& left_point_index,
                              const int& right_point_index);
  // void RequestMapForSign(const ros::Publisher& request_pub,
  //                        const sensor::SignData& new_sign_data);

  ros::Publisher* system_positon_publisher_;
  ros::Publisher* system_odom_publisher_;
  ros::Publisher* system_request_map_publisher_lane_;
  ros::Publisher* system_request_map_publisher_lc_;
  ros::Publisher* system_request_map_publisher_sign_;
  ros::Publisher* system_map_lane_points_publisher_;
  ros::Publisher* system_map_lane_min_dist_points_publisher_;
  ros::Publisher* system_perpendicular_publisher_;

  // system status
  State system_state_;
  State system_initial_state_;
  double system_local_pos_bias_x_;  // Cartesian coordinates before confirming
                                    // the true position
  double system_local_pos_bias_y_;
  double system_local_pos_x_;  // X and y in Cartesian coordinate systems
  double system_local_pos_y_;

  // sensor data buffer
  MapRingBuffer<sensor::ImuData> imu_data_buffer_;
  MapRingBuffer<sensor::GNSSData> gnss_data_buffer_;
  MapRingBuffer<sensor::EncoderData> encoder_data_buffer_;
  MapRingBuffer<sensor::LaneData> lane_data_buffer_;
  MapRingBuffer<sensor::LongitudinalConstraintData> lc_data_buffer_;
  // MapRingBuffer<sensor::SignData> sign_data_buffer_;

  // system state machine
  SystemStatus system_status_;

  // parallel filters
  filters::ParallelFilterA* filter_a_;
  filters::ParallelFilterB* filter_b_;

  // Linear motion flag, used for heading calibration during motion
  bool straight_motion_flag_;
  int straight_motion_flag_cnt_;

  // Local yaw pointing due north
  double north_angle_;

  double encoder_scale_;                   // encoder scale
  std::vector<double> encoder_scale_vec_;  // encoder scale buffer

  // Mercator projection
  MercatorProj system_proj_;

  // GNSS correction related
  bool postion_confirmation_flag_;
  bool GNSS_data_fluctuation_resistance_;  //  Is the GNSS data fluctuation
                                           //  resistance mechanism enabled
  bool GNSS_update_;                       // Is GNSS correction enabled
  bool GNSS_pos_cali_;  // Is GNSS used to correct the position
  bool GNSS_yaw_cali_;  // Is GNSS used to correct the yaw

  
  MapPoint sign_point;

  // Lane correction related
  std::vector<MapPoint> map_lane_points_;
  bool lane_update_;    // Is lane correction enabled
  bool lane_pos_cali_;  // Is lane used to correct the position
  bool lane_yaw_cali_;  // Is GNSS used to correct the yaw
  std::chrono::time_point<std::chrono::high_resolution_clock>
      lane_request_time_;
  std::chrono::time_point<std::chrono::high_resolution_clock> lane_accept_time_;

  // The effectiveness of each lane marking point
  std::vector<uint16_t> map_lane_points_flags_;
  int map_response_lane_index_;
  bool accept_lane_points_flag_;

  
  std::vector<MapSign> map_signs_;
  bool accept_signs_flag_;

  // Longitudinal correction related
  std::vector<MapPoint> map_longitudinal_constraint_points_;
  bool longitudinal_update_;  
  bool accept_longitudinal_constraint_flag_;
  std::chrono::time_point<std::chrono::high_resolution_clock>
      lon_request_time_;
  std::chrono::time_point<std::chrono::high_resolution_clock> lon_accept_time_;
};
}  // namespace vhdmap_se
#endif  // VHDMAPSE_INCLUDE_NAVIGATION_SYSTEM_H_
