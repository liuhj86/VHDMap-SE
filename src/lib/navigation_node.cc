#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <navigation_node.h>
#include <navigation_system.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

#include <iomanip>
#include <opencv2/opencv.hpp>

#include "common_utils.h"

NavigationSystemNode::NavigationSystemNode(const ros::NodeHandle& nh,
                                           const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  navigation_system_ = new vhdmap_se::NavigationSystem(
      &pub_position_, &pub_odom_, &pub_map_request_lane_, &pub_map_request_lc_,
      &pub_map_request_sign_, &pub_map_lane_points_debug_,
      &pub_map_lane_min_dist_point_debug_, &pub_perpendicular_);
  system_initialization_done_flag_ = false;
  gnss_path_pos_msg_.header.stamp = ros::Time::now();
  gnss_path_pos_msg_.header.frame_id = "map";
  map_lat_interaction_response_record_file_name_ = "/home/liuhj/lat_inter.csv";
  map_lon_interaction_response_record_file_name_ = "/home/liuhj/lon_inter.csv";
}
NavigationSystemNode::~NavigationSystemNode() { delete navigation_system_; }
void NavigationSystemNode::Run() {
  srand(time(NULL));
  ReadParameters(pnh_);
  sub_gnss_ = nh_.subscribe<sensor_msgs::NavSatFix>(
      GNSS_TOPIC, 5, &NavigationSystemNode::GNSSCallback, this);
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(
      IMU_TOPIC, 1000, &NavigationSystemNode::ImuCallback, this);
  sub_encoder_ = nh_.subscribe<nav_msgs::Odometry>(
      ENCODER_TOPIC, 1000, &NavigationSystemNode::EncoderCallback, this);
  sub_camera_lane_ = nh_.subscribe<sensor_msgs::NavSatFix>(
      CAMERA_LANE_TOPIC, 50, &NavigationSystemNode::CameraLaneCallback, this);
  sub_lc_measurement_ = nh_.subscribe<geometry_msgs::PoseArray>(
      LC_MEASUREMENT_TOPIC, 50,
      &NavigationSystemNode::LongitudinalConstraintCallback, this);
  sub_map_lane_ = nh_.subscribe<sensor_msgs::PointCloud>(
      MAP_RESPONSE_LANE_TOPIC, 50, &NavigationSystemNode::MapLaneCallback,
      this);
  sub_map_longitudinal_constraint_ = nh_.subscribe<geometry_msgs::PoseArray>(
      MAP_RESPONSE_LC_TOPIC, 50, &NavigationSystemNode::MapLCCallback, this);
  pub_position_ = nh_.advertise<sensor_msgs::NavSatFix>(POSITION_TOPIC, 5);
  pub_gnss_path_ = nh_.advertise<nav_msgs::Path>("/VHDMap_SE/gnss/path", 5);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/VHDMap_SE/odom", 1);
  pub_gnss_odom_ = nh_.advertise<nav_msgs::Odometry>("/VHDMap_SE/gnss/odom", 5);
  pub_map_request_lane_ =
      nh_.advertise<nav_msgs::Odometry>(MAP_REQUEST_LANE_TOPIC, 5);
  pub_map_request_lc_ =
      nh_.advertise<nav_msgs::Odometry>(MAP_REQUEST_LC_TOPIC, 5);
  pub_map_lane_points_debug_ =
      nh_.advertise<sensor_msgs::PointCloud>("/VHDMap_SE/map_lane_points", 5);
  pub_map_lane_min_dist_point_debug_ = nh_.advertise<sensor_msgs::PointCloud>(
      "/VHDMap_SE/map_lane_min_dist_point", 5);
  pub_perpendicular_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/VHDMap_SE/visualize/perpendicular", 10);
  while (ros::ok) {
    ros::spinOnce();
  }
}
void NavigationSystemNode::ReadParameters(const ros::NodeHandle& pnh) {
  std::string config_file;
  if (pnh_.getParam("config_file", config_file)) {
    ROS_INFO_STREAM("loading "
                    << "config_file : " << config_file);
  } else {
    ROS_ERROR_STREAM("failed to load config_file");
    pnh_.shutdown();
  }
  cv::FileStorage fs_settings(config_file, cv::FileStorage::READ);
  if (!fs_settings.isOpened())
    std::cerr << "ERROR: Wrong path of settings file." << std::endl;
  RUN_MODE = fs_settings["run_mode"];
  STATIONARY_THRESHOLD = fs_settings["stationary_threshold"];
  ERROR_VELOCITY_VARIANCE_X = fs_settings["error_velocity_variance_x"];
  ERROR_VELOCITY_VARIANCE_Y = fs_settings["error_velocity_variance_y"];
  ERROR_VELOCITY_VARIANCE_Z = fs_settings["error_velocity_variance_z"];
  INITIAL_POSITION_VARIANCE_PARAMETER_LAT =
      fs_settings["initial_position_variance_parameter_lat"];
  INITIAL_POSITION_VARIANCE_PARAMETER_LON =
      fs_settings["initial_position_variance_parameter_lon"];
  INITIAL_POSITION_VARIANCE_PARAMETER_ALT =
      fs_settings["initial_position_variance_parameter_alt"];
  INITIAL_ATTITUDE_VARIANCE_PARAMETER_YAW =
      fs_settings["initial_attitude_variance_parameter_yaw"];
  GNSS_VARIANCE_PARAMETER_LAT = fs_settings["gnss_variance_parameter_lat"];
  GNSS_VARIANCE_PARAMETER_LON = fs_settings["gnss_variance_parameter_lon"];
  GNSS_VARIANCE_PARAMETER_ALT = fs_settings["gnss_variance_parameter_alt"];
  GNSS_VARIANCE_PARAMETER_YAW = fs_settings["gnss_variance_parameter_yaw"];
  ANGULAR_VEL_THRESHOLD_FOR_STRAIGHT_MOTION =
      fs_settings["angular_vel_threshold_for_straight_motion"];
  DURATION_THRESHOLD_FOR_STRAIGHT_MOTION =
      fs_settings["duration_threshold_for_straight_motion"];
  INITIAL_ENCODER_SCALE = fs_settings["initial_encoder_scale"];
  PEBG_STANDARD_DEVIATION_COEFFICIENT =
      fs_settings["pebg_standard_deviation_coefficient"];
  PWEBG_STANDARD_DEVIATION_COEFFICIENT =
      fs_settings["pwebg_standard_deviation_coefficient"];
  VEHICLE_LENGTH = fs_settings["vehicle_length"];
  VEHICLE_WIDTH = fs_settings["vehicle_width"];
  ROAD_WIDTH = fs_settings["road_width"];
  INITIAL_LATITUDE = fs_settings["initial_latitude"];
  INITIAL_LONGITUDE = fs_settings["initial_longitude"];
  GNSS_DATA_FLUCTUATION_RESISTANCE =
      (int)fs_settings["GNSS_data_fluctuation_resistance"];
  GNSS_UPDATE = (int)fs_settings["GNSS_update"];
  LANE_UPDATE = (int)fs_settings["lane_update"];
  LONGITUDINAL_UPDATE = (int)fs_settings["longitudinal_update"];
  GNSS_POS_CALI = (int)fs_settings["GNSS_pos_cali"];
  GNSS_YAW_CALI = (int)fs_settings["GNSS_yaw_cali"];
  LANE_POS_CALI = (int)fs_settings["lane_pos_cali"];
  LANE_YAW_CALI = (int)fs_settings["lane_yaw_cali"];
  IMU_NOISE = (int)fs_settings["imu_noise"];
  IMU_NOISE_MODE = fs_settings["imu_noise_mode"];
  IMU_NOISE_BOUND = (double)fs_settings["imu_noise_bound"];
  GNSS_NOISE = (int)fs_settings["GNSS_noise"];
  GNSS_NOISE_BOUND = (double)fs_settings["GNSS_noise_bound"];
  LAT_MEAS_NOISE = (int)fs_settings["lat_meas_noise"];
  LAT_MEAS_NOISE_LEVEL = (int)fs_settings["lat_meas_noise_level"];
  LON_MEAS_NOISE = (int)fs_settings["lon_meas_noise"];
  LON_MEAS_NOISE_LEVEL = (int)fs_settings["lon_meas_noise_level"];
  fs_settings["gnss_topic"] >> GNSS_TOPIC;
  fs_settings["imu_topic"] >> IMU_TOPIC;
  fs_settings["encoder_topic"] >> ENCODER_TOPIC;
  fs_settings["camera_lane_topic"] >> CAMERA_LANE_TOPIC;
  fs_settings["lc_measurement_topic"] >> LC_MEASUREMENT_TOPIC;
  fs_settings["camera_sign_topic"] >> CAMERA_SIGN_TOPIC;
  fs_settings["map_response_lane_topic"] >> MAP_RESPONSE_LANE_TOPIC;
  fs_settings["map_response_lc_topic"] >> MAP_RESPONSE_LC_TOPIC;
  fs_settings["map_response_sign_topic"] >> MAP_RESPONSE_SIGN_TOPIC;
  fs_settings["yaw_topic"] >> YAW_TOPIC;
  fs_settings["position_topic"] >> POSITION_TOPIC;
  fs_settings["map_request_lane_topic"] >> MAP_REQUEST_LANE_TOPIC;
  fs_settings["map_request_lc_topic"] >> MAP_REQUEST_LC_TOPIC;
  fs_settings["map_request_sign_topic"] >> MAP_REQUEST_SIGN_TOPIC;
  ROS_INFO("Read parameters completed");
}
void NavigationSystemNode::ImuCallback(
    const sensor_msgs::Imu::ConstPtr& imu_msg) {
  sensor::ImuData imu_data_raw, imu_data_corrected, imu_data_revised;
  sensor::ImuData last_imu_data;
  imu_data_raw.timestamp_ = imu_msg->header.stamp.toSec();
  fromMsg(imu_msg->orientation, imu_data_raw.attitude_quaternion_);
  std::cout << "IMU data received." << std::endl;
  // Directly reading the angular velocity of IMU (drift can be significant)
  /*
  imu_data_raw.angular_velocity_ << imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
  */

  // Calculate the angular velocity by subtracting the previous IMU angle from
  // the current angle (in case of abnormal power outage, directly read the
  // angular velocity data provided by the IMU)
  if (navigation_system_->imu_data_buffer_.GetLastMeasurementData(
          &last_imu_data)) {
    double roll1, pitch1, yaw1;
    tf2::Matrix3x3(imu_data_raw.attitude_quaternion_)
        .getRPY(roll1, pitch1, yaw1);
    double roll2, pitch2, yaw2;
    tf2::Matrix3x3(last_imu_data.attitude_quaternion_)
        .getRPY(roll2, pitch2, yaw2);
    double delta_t = imu_data_raw.timestamp_ - last_imu_data.timestamp_;
    if (delta_t <= static_cast<double>(5.0 / IMU_TOPIC_FREQUENCY) &&
        delta_t != 0.0) {
      // After the IMU is powered on again, the reading will reset to 0 degrees,
      // and at this time, the angular velocity cannot be calculated by
      // subtracting. We tolerate a delay of 5 normal IMU data cycles here, and
      // exceeding this cycle is considered a prolonged power outage.
      imu_data_raw.angular_velocity_ << (roll1 - roll2) / delta_t,
          (pitch1 - pitch2) / delta_t, (yaw1 - yaw2) / delta_t;
    } else {
      imu_data_raw.angular_velocity_ << imu_msg->angular_velocity.x,
          imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
    }
  } else {
    imu_data_raw.angular_velocity_ << imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
  }
  imu_data_raw.linear_acceleration_ << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
  if (std::fabs(imu_data_raw.angular_velocity_.z()) <
      ANGULAR_VEL_THRESHOLD_FOR_STRAIGHT_MOTION) {
    navigation_system_->straight_motion_flag_cnt_++;
    if (navigation_system_->straight_motion_flag_cnt_ >=
        DURATION_THRESHOLD_FOR_STRAIGHT_MOTION) {
      navigation_system_->straight_motion_flag_ = 1;
    }
  } else {  // If data with absolute values exceeding the threshold is found,
            // the linear motion condition is not valid and needs to be
            // recounted
    navigation_system_->straight_motion_flag_ = 0;
    navigation_system_->straight_motion_flag_cnt_ = 0;
  }
  fromMsg(imu_msg->orientation, imu_data_raw.attitude_quaternion_);

  imu_data_corrected = imu_data_raw;
  navigation_system_->imu_data_buffer_.AddMeasurementData(
      imu_data_corrected, imu_data_corrected.timestamp_);
}
void NavigationSystemNode::GNSSCallback(
    const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
  bool yaw_available_flag = false;
  bool pos_flag = true;
  std::cout << "GNSS data received." << std::endl;

  if (std::isnan(gnss_msg->latitude) || std::isnan(gnss_msg->longitude) ||
      std::isnan(gnss_msg->altitude)) {
    ROS_WARN("pos invalid, GNSS signal not applicable !");
  } else {
    double msg_latitude, msg_longitude;
    double lat_ramdom_noise = common_utils::random(GNSS_NOISE_BOUND);
    double lon_ramdom_noise = common_utils::random(GNSS_NOISE_BOUND);
    int lat_sign = (lat_ramdom_noise > 0) - (lat_ramdom_noise < 0);
    int lon_sign = (lon_ramdom_noise > 0) - (lon_ramdom_noise < 0);
    if (GNSS_NOISE) {
      msg_latitude = gnss_msg->latitude +
                     lat_sign * 0.5 * std::sqrt(fabs(lat_ramdom_noise)) *
                         5e-5;  // 1 degree = 111 km
      msg_longitude = gnss_msg->longitude +
                      lon_sign * 0.5 * std::sqrt(fabs(lon_ramdom_noise)) * 5e-5;
    } else {
      msg_latitude = gnss_msg->latitude;
      msg_longitude = gnss_msg->longitude;
    }
    if (RUN_MODE == 0) {
      nav_msgs::Odometry gnss_odom_msg;
      geometry_msgs::PoseStamped gnss_path_pos_stamped_msg;
      double gnss_x, gnss_y;
      navigation_system_->system_proj_.ToProj(
          math_utils::DegreesToRadians(msg_latitude),
          math_utils::DegreesToRadians(msg_longitude), gnss_x, gnss_y);

      gnss_path_pos_stamped_msg.pose.position.x = -gnss_y;
      gnss_path_pos_stamped_msg.pose.position.y = gnss_x;

      gnss_path_pos_stamped_msg.header.frame_id = "map";
      gnss_path_pos_stamped_msg.header.stamp = ros::Time::now();
      gnss_path_pos_msg_.poses.push_back(gnss_path_pos_stamped_msg);
      pub_gnss_path_.publish(gnss_path_pos_msg_);

      gnss_odom_msg.pose.pose.position.x = -gnss_y;
      gnss_odom_msg.pose.pose.position.y = gnss_x;
      tf2::Quaternion tf2_q1(navigation_system_->system_state_.q_.x(),
                             navigation_system_->system_state_.q_.y(),
                             navigation_system_->system_state_.q_.z(),
                             navigation_system_->system_state_.q_.w());
      double yaw1, pitch1, roll1;
      tf2::Matrix3x3(tf2_q1).getRPY(roll1, pitch1, yaw1);
      double yaw_earth;
      common_utils::CalcYawEarthFromYaw(
          yaw1,
          navigation_system_->filter_b_->integrator_->north_angle_cali_flag_,
          navigation_system_->north_angle_, &yaw_earth);
      double yaw_rviz = 0;
      common_utils::CalcYawRVizFromYawEarth(yaw_earth, &yaw_rviz);
      tf2::Quaternion tf2_q2;
      tf2_q2.setRPY(roll1, pitch1, yaw_rviz);
      tf2_q2.normalize();
      gnss_odom_msg.pose.pose.orientation.w = tf2_q2.w();
      gnss_odom_msg.pose.pose.orientation.x = tf2_q2.x();
      gnss_odom_msg.pose.pose.orientation.y = tf2_q2.y();
      gnss_odom_msg.pose.pose.orientation.z = tf2_q2.z();
      gnss_odom_msg.header.frame_id = "map";
      gnss_odom_msg.header.stamp = gnss_msg->header.stamp;
      pub_gnss_odom_.publish(gnss_odom_msg);
    } else if (RUN_MODE == 2) {
      nav_msgs::Odometry gnss_odom_msg;
      double gnss_x, gnss_y;

      UTMCoor odom_coor;
      common_utils::LL2UTM(math_utils::DegreesToRadians(msg_longitude),
                           math_utils::DegreesToRadians(msg_latitude),
                           &odom_coor);

      gnss_odom_msg.pose.pose.position.x = odom_coor.x + COOR_BIAS_X;
      gnss_odom_msg.pose.pose.position.y = odom_coor.y + COOR_BIAS_Y;
      tf2::Quaternion tf2_q1(navigation_system_->system_state_.q_.x(),
                             navigation_system_->system_state_.q_.y(),
                             navigation_system_->system_state_.q_.z(),
                             navigation_system_->system_state_.q_.w());
      double yaw1, pitch1, roll1;
      tf2::Matrix3x3(tf2_q1).getRPY(roll1, pitch1, yaw1);
      double yaw_earth;
      common_utils::CalcYawEarthFromYaw(
          yaw1,
          navigation_system_->filter_b_->integrator_->north_angle_cali_flag_,
          navigation_system_->north_angle_, &yaw_earth);
      double yaw_rviz = 0;
      common_utils::CalcYawRVizFromYawEarth(yaw_earth, &yaw_rviz);
      tf2::Quaternion tf2_q2;
      tf2_q2.setRPY(roll1, pitch1, yaw_rviz);
      tf2_q2.normalize();
      gnss_odom_msg.pose.pose.orientation.w = tf2_q2.w();
      gnss_odom_msg.pose.pose.orientation.x = tf2_q2.x();
      gnss_odom_msg.pose.pose.orientation.y = tf2_q2.y();
      gnss_odom_msg.pose.pose.orientation.z = tf2_q2.z();
      gnss_odom_msg.header.frame_id = "map";
      gnss_odom_msg.header.stamp = gnss_msg->header.stamp;
      pub_gnss_odom_.publish(gnss_odom_msg);
    }

    if (navigation_system_->filter_b_->integrator_->north_angle_cali_flag_ ==
        false) {
      std::cout << YELLOW << "north yaw hasn't been calibrated" << COLOR_RESET
                << std::endl;
      pos_flag = false;
    }
    sensor::GNSSData gnss_data_current, gnss_data_previous;
    gnss_data_current.geodetic_coordinates_
        << math_utils::DegreesToRadians(msg_latitude),
        math_utils::DegreesToRadians(msg_longitude), gnss_msg->altitude;
    gnss_data_current.covariance_
        << gnss_msg->position_covariance[0] *
               2.458172257647332e-14,  // Convert the variance in meters to
                                       // degrees for updating, as shown in
                                       // equations 5.2.29b and 4.1.60 in the
                                       // ESKF book
        gnss_msg->position_covariance[4] * 2.458172257647332e-14,
        gnss_msg->position_covariance[8] * 2.458172257647332e-14;
    // GNSS variance compensation calculation
    double lat_delta_mean, lon_delta_mean, lat_delta_stdev, lon_delta_stdev;
    navigation_system_->filter_a_->eskf_->CalcMeanStandardDeviation(
        gnss_lat_delta_vector_, &lat_delta_mean, &lat_delta_stdev);
    navigation_system_->filter_a_->eskf_->CalcMeanStandardDeviation(
        gnss_lon_delta_vector_, &lon_delta_mean, &lon_delta_stdev);
    if (navigation_system_->GNSS_data_fluctuation_resistance_) {
      gnss_data_current.covariance_(0) += lat_delta_stdev * lat_delta_stdev *
                                          lat_delta_stdev * lat_delta_stdev /
                                          1e-16;
      gnss_data_current.covariance_(1) += lon_delta_stdev * lon_delta_stdev *
                                          lon_delta_stdev * lon_delta_stdev /
                                          1e-16;
    }
    gnss_data_current.timestamp_ = gnss_msg->header.stamp.toSec();
    // if (navigation_system_->filter_b_->integrator_
    //        ->north_angle_cali_flag_ ==
    //    false) {  // The orientation of the system cannot be corrected without
    //    calibration of the north angle
    //  yaw_available_flag = false;
    // }
    if (navigation_system_->gnss_data_buffer_.GetLastMeasurementData(
            &gnss_data_previous) &&
        !std::isnan(gnss_data_current.yaw_)) {
      gnss_data_current.yaw_ = navigation_system_->CalcYawEarthFromGNSS(
          gnss_data_current, gnss_data_previous);
      yaw_available_flag = true;
    } else {
      yaw_available_flag = false;
    }
    /**
     * Determine the north angle and current LLA under sufficient
     * conditions, which means that the system is not stationary, maintains a
     * straight line motion, can calculate the heading angle through GNSS data,
     * and the variance of the current GNSS data is small enough. Another
     * condition is that the current northbound angle of the system has not been
     * calibrated yet.
     */
    if (navigation_system_->straight_motion_flag_ && yaw_available_flag &&
        navigation_system_->north_angle_ ==
            3.316 &&  // Set an meaningless value to indicate that
                      // north angle has not been calibrated yet
        !navigation_system_->postion_confirmation_flag_ &&
        gnss_data_current.covariance_(0) < 1e-13 &&
        gnss_data_current.covariance_(1) < 1e-13 &&
        !navigation_system_->IsSystemStationary(gnss_data_current,
                                                gnss_data_previous)) {
      if (navigation_system_->AlignImu(
              gnss_data_current.timestamp_,
              &navigation_system_->filter_a_->filter_state_)) {
        // determine the north angle
        tf2::Quaternion tf2_q(
            navigation_system_->filter_a_->filter_state_.q_.x(),
            navigation_system_->filter_a_->filter_state_.q_.y(),
            navigation_system_->filter_a_->filter_state_.q_.z(),
            navigation_system_->filter_a_->filter_state_.q_.w());
        double yaw, pitch, roll;
        tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
        navigation_system_->filter_b_->integrator_->CaliNorthAngle(
            gnss_data_current.yaw_, yaw);
        std::cout << "gnss_yaw2: " << gnss_data_current.yaw_
                  << "yaw: " << math_utils::RadiansToDegrees(yaw) << std::endl;
        // confirm LLA
        navigation_system_->filter_a_->filter_state_.p_ =
            gnss_data_current.geodetic_coordinates_;
        navigation_system_->filter_a_->filter_state_.earth_parameter_state_
            .UpdateEarthParams(gnss_data_current.geodetic_coordinates_,
                               navigation_system_->system_state_.v_);
        navigation_system_->filter_a_->filter_state_.time_ =
            gnss_data_current.timestamp_;
        // State synchronization
        navigation_system_->filter_b_->StateSynchronize(
            navigation_system_->filter_a_);
        navigation_system_->system_state_ =
            navigation_system_->filter_a_->filter_state_;
        // Reset Mercator Projection

        // navigation_system_->system_proj_.SetB0(
        //     navigation_system_->system_state_.p_[0]);
        // navigation_system_->system_proj_.SetL0(
        //     navigation_system_->system_state_.p_[1]);
        // navigation_system_->system_local_pos_bias_x_ =
        //     navigation_system_->system_local_pos_x_;
        // navigation_system_->system_local_pos_bias_y_ =
        //     navigation_system_->system_local_pos_y_;

        // release flags
        navigation_system_->filter_b_->integrator_->north_angle_cali_flag_ =
            true;
        navigation_system_->filter_a_->integrator_->north_angle_cali_flag_ =
            true;
        navigation_system_->postion_confirmation_flag_ = true;
      }
    }

    // Calibration encoder scale
    /*
    if (navigation_system_->postion_confirmation_flag_ &&
        navigation_system_->straight_motion_flag_ &&
        gnss_data_current.covariance_(0) < pow(10, -13) &&
        gnss_data_current.covariance_(1) < pow(10, -13)) {
      Vector3d tn =
          navigation_system_->system_state_.earth_parameter_state_.Mpr *
          (gnss_data_current.geodetic_coordinates_ -
           gnss_data_previous.geodetic_coordinates_);
      tn /= (gnss_data_current.timestamp_ - gnss_data_previous.timestamp_);
      navigation_system_->CalibrateEncoderScale(
          sqrt(tn(0) * tn(0) + tn(1) * tn(1)), gnss_data_current.timestamp_);
    }
    */
    if (navigation_system_->straight_motion_flag_ && yaw_available_flag &&
        gnss_data_current.covariance_(0) < 1e-13 &&
        gnss_data_current.covariance_(1) < 1e-13 &&
        navigation_system_->postion_confirmation_flag_ &&
        !navigation_system_->IsSystemStationary(gnss_data_current,
                                                gnss_data_previous)) {
      gnss_data_current.yaw_available_flag_ = true;
    } else {
      gnss_data_current.yaw_available_flag_ = false;
    }
    gnss_data_current.pos_available_flag_ = pos_flag;
    navigation_system_->gnss_data_buffer_.AddMeasurementData(
        gnss_data_current, gnss_data_current.timestamp_);
    gnss_lat_delta_vector_.push_back(
        gnss_data_current.geodetic_coordinates_(0) -
        gnss_data_previous.geodetic_coordinates_(0));
    gnss_lon_delta_vector_.push_back(
        gnss_data_current.geodetic_coordinates_(1) -
        gnss_data_previous.geodetic_coordinates_(1));
    if (gnss_lat_delta_vector_.size() > 5) {
      gnss_lat_delta_vector_.erase(gnss_lat_delta_vector_.begin());
      gnss_lon_delta_vector_.erase(gnss_lon_delta_vector_.begin());
    }
  }
}
void NavigationSystemNode::EncoderCallback(
    const nav_msgs::Odometry::ConstPtr& encoder_msg) {
  sensor::EncoderData new_encoder_data;
  std::cout << "encoder data received." << std::endl;
  if (RUN_MODE == 0) {
    new_encoder_data.rear_beam_center = encoder_msg->twist.twist.linear.x;
  } else {
    new_encoder_data.rear_beam_center = std::sqrt(
        encoder_msg->twist.twist.linear.x * encoder_msg->twist.twist.linear.x +
        encoder_msg->twist.twist.linear.y * encoder_msg->twist.twist.linear.y +
        encoder_msg->twist.twist.linear.z * encoder_msg->twist.twist.linear.z);
    new_encoder_data.twist_linear_x = encoder_msg->twist.twist.linear.x;
    new_encoder_data.twist_linear_y = encoder_msg->twist.twist.linear.y;
    new_encoder_data.twist_linear_z = encoder_msg->twist.twist.linear.z;
  }

  if (new_encoder_data.rear_beam_center <
      0.1) {  // If the speed is 0 or less than 0, it indicates that there is no
              // movement or reversing, and it is not considered a straight
              // line motion.
    navigation_system_->straight_motion_flag_ = 0;
    navigation_system_->straight_motion_flag_cnt_ = 0;
  }
  new_encoder_data.timestamp_ = encoder_msg->header.stamp.toSec();
  double last_time;
  navigation_system_->encoder_data_buffer_.GetLastTime(&last_time);
  navigation_system_->encoder_data_buffer_.AddMeasurementData(
      new_encoder_data, new_encoder_data.timestamp_);
  bool execution_result = false;
  switch (navigation_system_->system_status_) {
    case vhdmap_se::NavigationSystem::INIT:
      if (execution_result = navigation_system_->InitializeSystem()) {
        system_initialization_done_flag_ = true;
        navigation_system_->system_status_ =
            vhdmap_se::NavigationSystem::RUNNING;
      } else {
        ROS_WARN("Initialization failed");
      }
      break;
    case vhdmap_se::NavigationSystem::RUNNING:
      if (execution_result = navigation_system_->RunTheSystem()) {
        navigation_system_->system_status_ =
            vhdmap_se::NavigationSystem::RUNNING;
      } else {
        ROS_WARN("System can not run");
        navigation_system_->system_status_ = vhdmap_se::NavigationSystem::RESET;
      }
      break;
    case vhdmap_se::NavigationSystem::RESET:
      navigation_system_->system_state_.Reset();
      navigation_system_->filter_a_->filter_state_.Reset();
      navigation_system_->filter_b_->filter_state_.Reset();
      navigation_system_->system_status_ = vhdmap_se::NavigationSystem::INIT;
      system_initialization_done_flag_ = false;
      break;
    default:
      ROS_ERROR("Wrong system status");
      break;
  }
}
void NavigationSystemNode::CameraLaneCallback(
    const sensor_msgs::NavSatFix::ConstPtr& lane_msg) {
  std::cout << "camera lane info received." << std::endl;
  sensor::LaneData new_lane_data;
  new_lane_data.timestamp_ = lane_msg->header.stamp.toSec();
  if (LAT_MEAS_NOISE) {
    new_lane_data.left_offset_ = common_utils::AddNoise(
        lane_msg->position_covariance[0], LAT_MEAS_NOISE_LEVEL);
    new_lane_data.right_offset_ = common_utils::AddNoise(
        lane_msg->position_covariance[2], LAT_MEAS_NOISE_LEVEL);
    new_lane_data.left_angle_rad_ = common_utils::AddNoise(
        lane_msg->position_covariance[1], LAT_MEAS_NOISE_LEVEL);
    new_lane_data.right_angle_rad_ = common_utils::AddNoise(
        lane_msg->position_covariance[3], LAT_MEAS_NOISE_LEVEL);
  } else {
    new_lane_data.left_offset_ = lane_msg->position_covariance[0];
    new_lane_data.right_offset_ = lane_msg->position_covariance[2];
    new_lane_data.left_angle_rad_ = lane_msg->position_covariance[1];
    new_lane_data.right_angle_rad_ = lane_msg->position_covariance[3];
  }
  // std::cout << "left offset before adding noise: "
  //           << lane_msg->position_covariance[0] << ' '
  //           << new_lane_data.left_offset_ << std::endl;
  // std::cout << "right offset before adding noise: "
  //           << lane_msg->position_covariance[2] << ' '
  //           << new_lane_data.right_offset_ << std::endl;
  // std::cout << "left angle rad before adding noise: "
  //           << lane_msg->position_covariance[1] << ' '
  //           << new_lane_data.left_angle_rad_ << std::endl;
  // std::cout << "right offset before adding noise: "
  //           << lane_msg->position_covariance[3] << ' '
  //           << new_lane_data.right_angle_rad_ << std::endl;

  navigation_system_->lane_data_buffer_.AddMeasurementData(
      new_lane_data, new_lane_data.timestamp_);
}
void NavigationSystemNode::LongitudinalConstraintCallback(
    const geometry_msgs::PoseArray::ConstPtr& longitudinal_constraint_msg) {
  std::cout << "longitudinal constraint msg received." << std::endl;

  sensor::LongitudinalConstraintData new_lc_data;
  new_lc_data.timestamp_ = longitudinal_constraint_msg->header.stamp.toSec();

  
  new_lc_data.x_ = longitudinal_constraint_msg->poses[0].position.x;
  new_lc_data.y_ = longitudinal_constraint_msg->poses[0].position.y;
  new_lc_data.z_ = longitudinal_constraint_msg->poses[0].position.z;
  Eigen::Vector3d rk_vector =
      Eigen::Vector3d(longitudinal_constraint_msg->poses[3].position.x,
                      longitudinal_constraint_msg->poses[3].position.x, 1e-20);
  new_lc_data.meas_cov_ = rk_vector.asDiagonal();
    
  // // target point
  // new_lc_data.point_x_ = longitudinal_constraint_msg->poses[1].position.x;
  // new_lc_data.point_y_ = longitudinal_constraint_msg->poses[1].position.y;
  // new_lc_data.point_z_ = longitudinal_constraint_msg->poses[1].position.z;

  // new_lc_data.gt_pos_x_ = longitudinal_constraint_msg->poses[2].position.x;
  // new_lc_data.gt_pos_y_ = longitudinal_constraint_msg->poses[2].position.y;
  // new_lc_data.gt_pos_z_ = longitudinal_constraint_msg->poses[2].position.z;

  navigation_system_->lc_data_buffer_.AddMeasurementData(
      new_lc_data, new_lc_data.timestamp_);
}

void NavigationSystemNode::MapLaneCallback(
    const sensor_msgs::PointCloud::ConstPtr& map_msg) {
  if (navigation_system_
          ->accept_lane_points_flag_) {  // Prevent receiving delayed points
                                         // after receiving all points
    std::cout << "Prepare to accept lane responses" << std::endl;
    if (map_msg->channels[0].values[0]) {  // There will be points sent over
      navigation_system_->map_response_lane_index_ =
          map_msg->channels[1].values[0];  // Index of separation point for
                                           // left and right roads
      MapPoint map_point;
      for (int i = 0; i < map_msg->points.size(); i++) {
        map_point.x = map_msg->points[i].x;
        map_point.y = map_msg->points[i].y;
        map_point.z = map_msg->points[i].z;
        navigation_system_->map_lane_points_.push_back(map_point);
        if (RUN_MODE == 0) {
          navigation_system_->map_lane_points_flags_.push_back(
              map_msg->channels[2].values[i]);
        } else if (RUN_MODE == 2) {
          navigation_system_->map_lane_points_flags_.push_back(1);
        }
      }
      navigation_system_->accept_lane_points_flag_ =
          false;  // Prohibit receiving new points
    }

  } else {
    std::cout << "flag not released" << std::endl;
  }
}
void NavigationSystemNode::MapLCCallback(
    const geometry_msgs::PoseArray::ConstPtr& map_msg) {
  MapPoint map_point;
  for (int i = 0; i < map_msg->poses.size(); i++) {
    map_point.x = map_msg->poses[i].position.x;
    map_point.y = map_msg->poses[i].position.y;
    map_point.z = map_msg->poses[i].position.z;
    navigation_system_->map_longitudinal_constraint_points_.emplace_back(
        map_point);
  }
  navigation_system_->lon_accept_time_ =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> lc_response_duration =
      navigation_system_->lon_accept_time_ -
      navigation_system_->lon_request_time_;
  std::ofstream file(map_lon_interaction_response_record_file_name_,
                     std::ios::app);
  if (file.is_open()) {
    file << lc_response_duration.count() << std::endl;
    file.close();
  } else {
    std::cout << "could not open file" << std::endl;
  }
  navigation_system_->accept_longitudinal_constraint_flag_ =
      false;  // Prohibit receiving new points
}
void NavigationSystemNode::MapSignCallback(
    const sensor_msgs::PointCloud::ConstPtr& map_msg) {
  if (navigation_system_->accept_signs_flag_) {
    if (map_msg->channels[0].values[0]) {
      MapPoint map_point;
      MapSign map_sign;
      for (int i = 0; i < map_msg->points.size(); i++) {
        map_sign.position.x = map_msg->points[i].x;
        map_sign.position.y = map_msg->points[i].y;
        map_sign.position.z = map_msg->points[i].z;
        map_sign.type = map_msg->channels[1].values[i];
        navigation_system_->map_signs_.push_back(map_sign);
      }
      navigation_system_->accept_signs_flag_ = false;
    }
  }
}
