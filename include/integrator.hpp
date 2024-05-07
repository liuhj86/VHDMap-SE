#ifndef VHDMAPSE_INCLUDE_INTEGRATOR_HPP_
#define VHDMAPSE_INCLUDE_INTEGRATOR_HPP_

#include <common_utils.h>
#include <math_utils.h>
#include <mercator.h>
#include <ros/ros.h>
#include <sensor_data.h>
#include <system_parameters.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <queue>
#include <state.hpp>
#include <vector>

// void ConeRowingErrorCompensate(const double& wm, const double& vm){
//   cs = []
// }

class Integrator {
 public:
  Integrator(double& north_angle) {
    first_time_integrate_ = true;
    north_angle_in_integrator_ = &north_angle;
    north_angle_cali_flag_ = false;
  }
  ~Integrator() { delete north_angle_in_integrator_; }
  void CaliNorthAngle(const double& north_yaw,
                      const double& corresponding_yaw) {
    double diff = math_utils::RadiansToDegrees(corresponding_yaw) - north_yaw;
    if (diff > 180) {
      *north_angle_in_integrator_ = math_utils::DegreesToRadians(diff - 360);
    } else if (diff < -180) {
      *north_angle_in_integrator_ = math_utils::DegreesToRadians(diff + 360);
    } else {
      *north_angle_in_integrator_ = math_utils::DegreesToRadians(diff);
    }
  }
  void OriIntegrate(const sensor::ImuData imu_data, State& state,
                    const double dt) {
    double& time = state.ori_time_;
    Eigen::Vector3d& bg = state.bg_;
    Eigen::Quaterniond& q = state.q_;  
                                      
    EarthParameterState& earth = state.earth_parameter_state_;

    // orientation update (common integration method) uses the attitude data of
    // current IMU to calculate delta with the attitude data of the previous
    // IMU.

    // if (first_time_integrate_ == true) {
    //   last_imu_data_ = imu_data;
    //   first_time_integrate_ = false;
    //   return;
    // }
    // tf2::Quaternion cur_imu_q_tf2 = imu_data.attitude_quaternion_;
    // tf2::Quaternion last_imu_q_tf2 = last_imu_data_.attitude_quaternion_;
    // Eigen::Quaterniond cur_imu_q_eigen(cur_imu_q_tf2.w(), cur_imu_q_tf2.x(),
    //                                    cur_imu_q_tf2.y(), cur_imu_q_tf2.z());
    // Eigen::Quaterniond last_imu_q_eigen(last_imu_q_tf2.w(),
    // last_imu_q_tf2.x(),
    //                                    last_imu_q_tf2.y(),
    //                                    last_imu_q_tf2.z());
    // Eigen::Quaterniond delta_q_eigen;
    // delta_q_eigen = cur_imu_q_eigen * last_imu_q_eigen.inverse();
    // tf2::Quaternion tf2_q(q.x(), q.y(), q.z(), q.w());
    // double cur_yaw, res_yaw, cur_pitch, res_pitch, cur_roll, res_roll;
    // tf2::Matrix3x3(tf2_q).getRPY(cur_roll, cur_pitch, cur_yaw);
    // std::cout << "cur angle: " << math_utils::RadiansToDegrees(cur_roll)
    //           << ' ' << math_utils::RadiansToDegrees(cur_pitch) << ' '
    //           << math_utils::RadiansToDegrees(cur_yaw) << std::endl;
    // q = delta_q_eigen * q;
    // q.normalize();
    // last_imu_data_ = imu_data;
    // time+=dt;

    // Orientation update (common integration method) directly uses IMU angular
    // velocity for integration.

    Eigen::Vector3d delta_angle;
    if (IMU_NOISE) {
      Eigen::Vector3d noised_angular_vel;
      if (IMU_NOISE_MODE == 0) {  // random mode
        noised_angular_vel << imu_data.angular_velocity_(0),
            imu_data.angular_velocity_(1),
            imu_data.angular_velocity_(2) +
                common_utils::random(
                    math_utils::DegreesToRadians(IMU_NOISE_BOUND));
      } else if (IMU_NOISE_MODE == 1) {  // fixed bias mode
        noised_angular_vel << imu_data.angular_velocity_(0),
            imu_data.angular_velocity_(1),
            imu_data.angular_velocity_(2) +
                math_utils::DegreesToRadians(IMU_NOISE_BOUND);
      }
      delta_angle = (noised_angular_vel - bg) * dt;
      // std::cout << noised_angular_vel << std::endl
      //           << bg << ' ' << "dt: " << dt << std::endl;
    } else {
      delta_angle = (imu_data.angular_velocity_ - bg) * dt;
    }

    // std::cout << "delta angle: " <<
    // math_utils::RadiansToDegrees(delta_angle(0))
    //           << ' ' << math_utils::RadiansToDegrees(delta_angle(1)) << '
    // '
    //           << math_utils::RadiansToDegrees(delta_angle(2)) <<
    // std::endl;
    tf2::Quaternion tf2_q(q.x(), q.y(), q.z(), q.w());
    double cur_yaw, res_yaw, cur_pitch, res_pitch, cur_roll, res_roll;
    tf2::Matrix3x3(tf2_q).getRPY(cur_roll, cur_pitch, cur_yaw);
    Eigen::Vector3d cur_angle(cur_pitch, cur_roll, cur_yaw);
    // std::cout << "cur angle: " << math_utils::RadiansToDegrees(cur_pitch)
    // <<
    // ' '
    //           << math_utils::RadiansToDegrees(cur_roll) << ' '
    //           << math_utils::RadiansToDegrees(cur_yaw) << std::endl;
    Eigen::Vector3d res_angle = cur_angle + delta_angle;
    tf2::Quaternion res_q_tf2;
    res_q_tf2.setRPY(res_angle(1), res_angle(0), res_angle(2));
    Eigen::Quaterniond res_q_eigen(res_q_tf2.w(), res_q_tf2.x(), res_q_tf2.y(),
                                   res_q_tf2.z());
    q = res_q_eigen.normalized();
    tf2::Quaternion tf2_q1(q.x(), q.y(), q.z(), q.w());
    double res_yaw1, res_pitch1, res_roll1;
    tf2::Matrix3x3(tf2_q1).getRPY(res_roll1, res_pitch1, res_yaw1);
    // std::cout << "res angle: " << math_utils::RadiansToDegrees(res_pitch1)
    //           << ' ' << math_utils::RadiansToDegrees(res_roll1) << ' '
    //           << math_utils::RadiansToDegrees(res_yaw1) << std::endl;
    time += dt;

    // orientation update (integration method of strapdown inertial navigation)

    // if (first_time_integrate_ == true) {
    //   da_k_ = (imu_data.angular_velocity_ - bg) * dt;
    //   first_time_integrate_ = false;
    // }
    // std::cout << math_utils::RadiansToDegrees(imu_data.angular_velocity_(2))
    // *
    //                  dt
    //           << std::endl;
    // da_k1_ = (imu_data.angular_velocity_ - bg) * dt;
    // Eigen::Vector3d phi_k1 =
    //     da_k1_ + 1.0 / 12 * da_k_.cross(da_k1_);  // Single sample plus previous
                                                     // cycle
    // // wm = qmulv(qconf(qnb), eth.wnin)*ts
    // // phim = (sum(wm, 1) + zeros(1,3))'
    // //
    // // Vector3d phi_k1 =
    // //    da_k1_ + 1. / 6. * da_k1_.cross(da_k1_);  // Two sample 
                                                       // approximation
    // da_k_ = da_k1_;
    // Eigen::Vector3d varsigma_k1 = -earth.wnin * dt;
    // Eigen::Quaterniond q_k1 =
    //     math_utils::axis2Quat(varsigma_k1) * q *
    //     math_utils::axis2Quat(phi_k1);
    // q_k1.norm();
    // q = q_k1;
    // time += dt;

    // Orientation update (direct reading sensor data)
    // tf2::Quaternion tf2_q(
    //     imu_data.attitude_quaternion_.x(), imu_data.attitude_quaternion_.y(),
    //     imu_data.attitude_quaternion_.z(),
    //     imu_data.attitude_quaternion_.w());
    // tf2_q.normalize();
    // Eigen::Quaterniond q_tmp(tf2_q.w(), tf2_q.x(), tf2_q.y(), tf2_q.z());
    // q_tmp.normalize();
    // q = q_tmp;
    // time += dt;

    // tf2::Quaternion tf2_q1(q.x(), q.y(), q.z(), q.w());
    // double yaw, pitch, roll;
    // tf2::Matrix3x3(tf2_q1).getRPY(roll, pitch, yaw);

    // tf2::Quaternion tf2_q2(
    //     imu_data.attitude_quaternion_.x(), imu_data.attitude_quaternion_.y(),
    //     imu_data.attitude_quaternion_.z(),
    //     imu_data.attitude_quaternion_.w());
    // double yaw1, pitch1, roll1;
    // tf2::Matrix3x3(tf2_q2).getRPY(roll1, pitch1, yaw1);
    // std::cout << "yaw: " << math_utils::RadiansToDegrees(yaw) << std::endl;
    // std::cout << "imu_yaw: " << math_utils::RadiansToDegrees(yaw1) <<
    // std::endl; std::cout << "orientation: " << q.x() << ' ' << q.y() << ' '
    // << q.z() << ' '
    //           << q.w() << std::endl;
    // std::cout << "imu orientation: " << imu_data.attitude_quaternion_.x() <<
    // ' '
    //           << imu_data.attitude_quaternion_.y() << ' '
    //           << imu_data.attitude_quaternion_.z() << ' '
    //           << imu_data.attitude_quaternion_.w() << std::endl;
  }
  void PosIntegrate(const sensor::EncoderData& encoder_data, State* state,
                    const double& dt, const double& north_angle,
                    const double& encoder_scale) {
    double& time = state->time_;
    Eigen::Vector3d& pos = state->p_;
    Eigen::Vector3d& vel = state->v_;
    Eigen::Vector3d& bg = state->bg_;
    Eigen::Quaterniond& q = state->q_;
    EarthParameterState& earth = state->earth_parameter_state_;
    tf2::Quaternion tf2_q10(q.x(), q.y(), q.z(), q.w());
    double yaw10, pitch10, roll10;
    tf2::Matrix3x3(tf2_q10).getRPY(roll10, pitch10, yaw10);
    tf2::Quaternion tf2_q1(q.x(), q.y(), q.z(), q.w());
    double yaw, pitch, roll;
    tf2::Matrix3x3(tf2_q1).getRPY(roll, pitch, yaw);
    // position update
    double north_bias;  // heading angle relative to north
    if (RUN_MODE == 0) {
      if (north_angle_cali_flag_) {
        north_bias = yaw - north_angle;
        if (north_bias < 0) {
          north_bias += 2 * M_PI;
        }
      } else {
        north_bias = yaw;
        if (north_bias < 0) {
          north_bias += 2 * M_PI;
        }
      }
    } else if (RUN_MODE == 2) {
      double corrected_state_yaw = yaw + 0.5 * M_PI;
      if (corrected_state_yaw > M_PI) {
        corrected_state_yaw -= 2 * M_PI;
      }
      if (corrected_state_yaw < -M_PI) {
        corrected_state_yaw += 2 * M_PI;
      }
      common_utils::CalcYawEarthFromYawENU(corrected_state_yaw, &north_bias);
    }

    /*
    double tire_slip_angle;                  //slip angle
    double rear_beam_center_turning_radius;  //Rear beam center turning radius
    double mass_center_turning_radius;       //Geometric center turning radius
    double mass_center_speed;                //Geometric center velocity
    double front_wheel_average_deflection;   //Average steering angle of front
                                             //wheels
    */
    Eigen::Vector3d vel_new;

    /* Ackermann motion model, bicycle model (if needed)
    if (north_angle_cali_flag_) {
      if (imu_data.angular_velocity_.z() < 0 ) { // turning right
        rear_beam_center_turning_radius =
            encoder_data.rear_beam_center * encoder_scale /
            (-imu_data.angular_velocity_.z());
        mass_center_turning_radius = sqrt(
            rear_beam_center_turning_radius * rear_beam_center_turning_radius +
            0.25 * VEHICLE_LENGTH * VEHICLE_LENGTH);
        mass_center_speed = (-imu_data.angular_velocity_.z()) *
                            mass_center_turning_radius;
        front_wheel_average_deflection =
            VEHICLE_LENGTH / mass_center_turning_radius;
        tire_slip_angle = atan(0.5 * tan(front_wheel_average_deflection));
        Vector3d vel_tmp(mass_center_speed * sin(north_bias + tire_slip_angle),
                         mass_center_speed * cos(north_bias + tire_slip_angle),
                         0);
        vel_new = vel_tmp;

      } else if (imu_data.angular_velocity_.z() > 0 ) { // turning left
        rear_beam_center_turning_radius =
            encoder_data.rear_beam_center * encoder_scale /
            imu_data.angular_velocity_.z();
        mass_center_turning_radius = sqrt(
            rear_beam_center_turning_radius * rear_beam_center_turning_radius +
            0.25 * VEHICLE_LENGTH * VEHICLE_LENGTH);
        mass_center_speed = imu_data.angular_velocity_.z() *
                            mass_center_turning_radius;
        front_wheel_average_deflection =
            VEHICLE_LENGTH / mass_center_turning_radius;
        tire_slip_angle = atan(0.5 * tan(front_wheel_average_deflection));
        Vector3d vel_tmp(mass_center_speed * sin(north_bias - tire_slip_angle),
                         mass_center_speed * cos(north_bias - tire_slip_angle),
                         0);
        vel_new = vel_tmp;

      } else {
        Vector3d vel_tmp(encoder_data.rear_beam_center * sin(north_bias),
                         encoder_data.rear_beam_center * cos(north_bias), 0);
        vel_new = vel_tmp * encoder_scale;
      }
    } else {
      if (imu_data.angular_velocity_.z() < 0 ) {  // turning right
        rear_beam_center_turning_radius =
            encoder_data.rear_beam_center * encoder_scale /
            (-imu_data.angular_velocity_.z());
        mass_center_turning_radius = sqrt(
            rear_beam_center_turning_radius * rear_beam_center_turning_radius +
            0.25 * VEHICLE_LENGTH * VEHICLE_LENGTH);
        mass_center_speed = (-imu_data.angular_velocity_.z()) *
                            mass_center_turning_radius;
        front_wheel_average_deflection =
            VEHICLE_LENGTH / mass_center_turning_radius;
        tire_slip_angle = atan(0.5 * tan(front_wheel_average_deflection));
        Vector3d vel_tmp(mass_center_speed * cos(north_bias - tire_slip_angle),
                         mass_center_speed * sin(north_bias + tire_slip_angle),
                         0);
        vel_new = vel_tmp;

      } else if (imu_data.angular_velocity_.z() > 0 ) {  // turning left
        rear_beam_center_turning_radius =
            encoder_data.rear_beam_center * encoder_scale /
            imu_data.angular_velocity_.z();
        mass_center_turning_radius = sqrt(
            rear_beam_center_turning_radius * rear_beam_center_turning_radius +
            0.25 * VEHICLE_LENGTH * VEHICLE_LENGTH);
        mass_center_speed = imu_data.angular_velocity_.z() *
                            mass_center_turning_radius;
        front_wheel_average_deflection =
            VEHICLE_LENGTH / mass_center_turning_radius;
        tire_slip_angle = atan(0.5 * tan(front_wheel_average_deflection));
        Vector3d vel_tmp(mass_center_speed * cos(north_bias + tire_slip_angle),
                         mass_center_speed * sin(north_bias - tire_slip_angle),
                         0);
        vel_new = vel_tmp;
      }
      else {
        Vector3d vel_tmp(encoder_data.rear_beam_center * cos(north_bias),
                         encoder_data.rear_beam_center * sin(north_bias), 0);
        vel_new = vel_tmp * encoder_scale;
      }
    }
    */
    // velocity update
    if (RUN_MODE == 0) {
      Eigen::Vector3d vel_tmp(encoder_data.rear_beam_center * sin(north_bias),
                              encoder_data.rear_beam_center * cos(north_bias),
                              0);
      vel_new = vel_tmp * encoder_scale;
    } else {
      Eigen::Vector3d vel_tmp(encoder_data.twist_linear_x,
                              encoder_data.twist_linear_y,
                              encoder_data.twist_linear_z);
      // Eigen::Vector3d vel_tmp(encoder_data.rear_beam_center *
      // sin(north_bias),
      //                         encoder_data.rear_beam_center *
      //                         cos(north_bias), 0);
      // std::cout << "integrator orientation： "
      //           << math_utils::RadiansToDegrees(north_bias) << ' '
      //           << "speed: " << encoder_data.rear_beam_center << std::endl;
      // std::cout << "vel compare east: " << vel_tmp(0) << ' '
      //           << encoder_data.twist_linear_x << std::endl;
      // std::cout << "vel compare north: " << vel_tmp(1) << ' '
      //           << encoder_data.twist_linear_y << std::endl;
      // std::cout << "vel compare up: " << vel_tmp(2) << ' '
      //           << encoder_data.twist_linear_z << std::endl;
      vel_new = vel_tmp * encoder_scale;
    }
    // std::cout << "vel new: " << vel_new << std::endl;
    // cout<<encoder_data.rear_beam_center<<endl;
    Eigen::Vector3d pos_kh,
        vel_kh;  // Position and velocity at intermediate moments
    Eigen::Vector3d varsigma_kh = earth.wnin * dt / 2.;
    Eigen::Vector3d xi_kh = weie * dt / 2.;
    Eigen::Quaterniond Cne_kh = math_utils::axis2Quat(xi_kh).inverse() *
                                earth.Cne * math_utils::axis2Quat(varsigma_kh);
    Cne_kh.norm();
    Eigen::Matrix3d Cne_kh_DCM =
        Cne_kh.toRotationMatrix();  // Rotation matrix at intermediate moments
    double lat_kh = atan(Cne_kh_DCM(2, 2) /
                         Cne_kh_DCM(2, 1));  // latitude at intermediate moments
    double lon_kh =
        atan2(-Cne_kh_DCM(0, 0),
              Cne_kh_DCM(1, 0));             // longitude at intermediate moments
    double h_kh =
        pos(2) + vel(2) * dt / 2;  // altitude at intermediate moments
    pos_kh << lat_kh, lon_kh, h_kh;  // LLA at intermediate moments
    vel_kh = 0.5 * (vel_new + vel);
    EarthParameterState eparams_kh(pos_kh, vel_kh);
    Eigen::Vector3d pos_k1 = pos + eparams_kh.Mrp * vel_kh * dt;
    pos = pos_k1;
    vel = vel_new;
    earth.UpdateEarthParams(pos, vel);
    // time update
    time += dt;
  }
  Eigen::Vector3d da_k_;
  Eigen::Vector3d da_k1_;
  bool first_time_integrate_;
  double* north_angle_in_integrator_;
  bool north_angle_cali_flag_;
  double last_yaw;
  sensor::ImuData last_imu_data_;
};
#endif  // VHDMAPSE_INCLUDE_INTEGRATOR_HPP_
