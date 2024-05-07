#ifndef VHDMAPSE_INCLUDE_STATE_HPP_
#define VHDMAPSE_INCLUDE_STATE_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <earth_parameter_state.hpp>

class State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  State() {
    p_.setZero();
    v_.setZero();
    q_.setIdentity();
    bg_.setZero();  
    time_ = 0.0;
    ori_time_ = 0.0;
  }
  ~State() {}

  void operator=(const State state) {
    p_ = state.p_;
    v_ = state.v_;
    q_ = state.q_;
    bg_ = state.bg_;
    time_ = state.time_;
    ori_time_ = state.ori_time_;
  }
  bool DataValidationCheck() {
    if (std::isnan(p_.x()) || std::isnan(p_.y()) || std::isnan(p_.z())) {
      ROS_WARN("position state is nan");
      return false;
    }

    if (std::isnan(v_.x()) || std::isnan(v_.y()) || std::isnan(v_.z())) {
      ROS_WARN("velocity state is nan");
      return false;
    }

    if (std::isnan(q_.x()) || std::isnan(q_.y()) || std::isnan(q_.z()) ||
        std::isnan(q_.w())) {
      ROS_WARN("attitude state is nan");
      return false;
    }

    if (std::isnan(bg_.x()) || std::isnan(bg_.y()) || std::isnan(bg_.z())) {
      ROS_WARN("angular velocity state is nan");
      return false;
    }

    return true;
  }
  void Reset() {
    p_.setZero();
    v_.setZero();
    q_.setIdentity();
    bg_.setZero();
  }
  // Earth related parameter state
  EarthParameterState earth_parameter_state_;

  // system state
  Eigen::Vector3d p_;     // LLA
  Eigen::Vector3d v_;     // velocity
  Eigen::Quaterniond q_;  
  Eigen::Vector3d bg_;    
  double time_;    // system time
  double
      ori_time_;  // orientation integration time
};

#endif  // VHDMAPSE_INCLUDE_STATE_HPP_
