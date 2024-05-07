#ifndef VHDMAPSE_INCLUDE_NAVIGATIONNODE_H_
#define VHDMAPSE_INCLUDE_NAVIGATIONNODE_H_

#include <geometry_msgs/PoseArray.h>
#include <navigation_system.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <node_parameters.h>
#include <ros/ros.h>
#include <sensor_data.h>
#include <sensor_msgs/PointCloud.h>
#include <system_parameters.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <vector>

class NavigationSystemNode {
 public:
  NavigationSystemNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~NavigationSystemNode();
  void Run();
  void ReadParameters(const ros::NodeHandle& pnh);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg);
  void EncoderCallback(const nav_msgs::Odometry::ConstPtr& encoder_msg);
  void CameraLaneCallback(const sensor_msgs::NavSatFix::ConstPtr& lane_msg);
  void LongitudinalConstraintCallback(const geometry_msgs::PoseArray::ConstPtr&
                                          longitudinal_constraint_msg_msg);
  // void SignCallback(const);
  void MapLaneCallback(const sensor_msgs::PointCloud::ConstPtr& map_msg);
  void MapLCCallback(const geometry_msgs::PoseArray::ConstPtr& map_msg);

  void MapSignCallback(const sensor_msgs::PointCloud::ConstPtr& map_msg);

  vhdmap_se::NavigationSystem* navigation_system_;
  bool system_initialization_done_flag_;
  std::vector<double> gnss_lat_delta_vector_;
  std::vector<double> gnss_lon_delta_vector_;
  nav_msgs::Path gnss_path_pos_msg_;  // gnss path

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_gnss_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_encoder_;
  ros::Subscriber sub_camera_sign_;
  ros::Subscriber sub_camera_lane_;
  ros::Subscriber sub_lc_measurement_;
  ros::Subscriber sub_map_lane_;
  ros::Subscriber sub_map_longitudinal_constraint_;
  ros::Subscriber sub_map_sign_;
  ros::Publisher pub_position_;
  ros::Publisher pub_gnss_path_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_map_request_lane_;
  ros::Publisher pub_map_request_lc_;
  ros::Publisher pub_map_request_sign_;
  ros::Publisher pub_gnss_odom_;
  ros::Publisher pub_map_lane_points_debug_;
  ros::Publisher pub_map_lane_min_dist_point_debug_;
  ros::Publisher pub_perpendicular_;
  std::string map_lon_interaction_response_record_file_name_;
  std::string map_lat_interaction_response_record_file_name_;
};

#endif  // VHDMAPSE_INCLUDE_NAVIGATIONNODE_H_
