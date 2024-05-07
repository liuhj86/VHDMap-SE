#ifndef VHDMAPSE_INCLUDE_LANEINFOHANDLER_HPP_
#define VHDMAPSE_INCLUDE_LANEINFOHANDLER_HPP_

#include <ros/ros.h>
#include <sensor_data.h>
#include <sensor_msgs/PointCloud.h>

#include <Eigen/Dense>
#include <iostream>

#include "common_utils.h"
#include "system_parameters.h"

class LaneInfoHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
  LaneInfoHandler(const double& yaw_earth, const double& north_angle,
                  const double& car_x, const double& car_y,
                  std::vector<MapPoint>* map_lane_points,
                  std::vector<uint16_t>* map_lane_points_flags, int* msg_index)
      : yaw_earth_(yaw_earth),
        north_angle_(north_angle),
        car_x_(car_x),
        car_y_(car_y),
        map_lane_points_(map_lane_points),
        map_lane_points_flags_(map_lane_points_flags),
        msg_index_(msg_index) {
    yaw_rviz_ = 0;
  };
  ~LaneInfoHandler(){};
  double CalcP2LDist(const bool& slope_flag, const int& index) {
    if (slope_flag) {
      return fabs(k_ * (map_lane_points_->at(index).x) -
                  (map_lane_points_->at(index).y) + b_) /
             sqrt(k_ * k_ + 1);
    } else {
      return fabs(map_lane_points_->at(index).x - car_x_);
    }
  }
  // (car_x_ - map_lane_points_->at(min_dist_point_index_left).x) *
  //                    (car_x_ -
  //                     map_lane_points_->at(min_dist_point_index_left).x) +
  //                (car_y_ - map_lane_points_->at(min_dist_point_index_left).y)
  //                *
  //                    (car_y_ -
  //                     map_lane_points_->at(min_dist_point_index_left).y) >
  //            1 + ROAD_WIDTH * ROAD_WIDTH)
  int SelectSidePos(const sensor::LaneData& lane_data,
                    const int& min_dist_point_index_left,
                    const int& min_dist_point_index_right) {
    if ((map_lane_points_flags_->at(min_dist_point_index_left) ==
             0 ||                               // Left points invalid
         fabs(lane_data.left_offset_ >= 10) ||  // Left lane line not detected
         fabs(lane_data.left_offset_ < 1) ||    // Left lane detection error
         fabs(lane_data.left_offset_) <=
             fabs(lane_data.right_offset_)) &&  // The distance from the left
                                                // lane line is less than the
                                                // right lane line
        fabs(lane_data.right_offset_) <
            10 &&  // The right lane line is detected and valid
        fabs(lane_data.right_offset_) > 1 &&
        map_lane_points_flags_->at(min_dist_point_index_right) != 0) {
      return 1;
    } else if (fabs(lane_data.left_offset_) <
                   10 &&  // The left lane line must be detected and valid,
                          // and the distance should not be too large
               fabs(lane_data.left_offset_) > 1 &&
               map_lane_points_flags_->at(min_dist_point_index_left) != 0) {
      return 0;
    } else {
      std::cout << RED
                << "The lane lines on both sides do not meet the "
                   "application conditions, ineffective filtering"
                << COLOR_RESET << std::endl;
      return 2;
    }
  }
  
  int SelectSideYaw(const sensor::LaneData& lane_data,
                    const int& min_dist_point_index_left,
                    const int& min_dist_point_index_right) {
    if ((map_lane_points_flags_->at(min_dist_point_index_left) == 0 ||
         map_lane_points_flags_->at(min_dist_point_index_left + 1) ==
             0 ||  // The point on the left line used to calculate the slope of the
                   // lane line is invalid (the point after the minimum distance
                   // point and the minimum distance point)
         fabs(lane_data.right_angle_rad_ - 0.5 * M_PI) <=
             fabs(lane_data.left_angle_rad_ -
                  0.5 * M_PI)) &&  
        map_lane_points_flags_->at(min_dist_point_index_right) != 0 &&
        map_lane_points_flags_->at(min_dist_point_index_right + 1) != 0) {
      return 1;

    } else if (map_lane_points_flags_->at(min_dist_point_index_left) != 0 &&
               map_lane_points_flags_->at(min_dist_point_index_left + 1) != 0) {
      return 0;
    } else {
      std::cout << RED
                << "The lane lines on both sides do not meet the application "
                   "conditions, ineffective filtering"
                << COLOR_RESET << std::endl;
      std::cout << "The specific observation values are： "
                << map_lane_points_flags_->at(min_dist_point_index_left) << ' '
                << map_lane_points_flags_->at(min_dist_point_index_right) << ' '
                << map_lane_points_flags_->at(min_dist_point_index_left + 1)
                << ' '
                << map_lane_points_flags_->at(min_dist_point_index_right + 1)
                << ' ' << fabs(lane_data.left_angle_rad_) << ' '
                << fabs(lane_data.right_angle_rad_) << std::endl;
      return 2;
    }
  }
  
  bool IfCurLaneRangeValid(const sensor::LaneData& lane_data,
                           const int& min_dist_point_index_left,
                           const int& min_dist_point_index_right) {
    if (map_lane_points_flags_->at(min_dist_point_index_left) == 0 &&
        map_lane_points_flags_->at(min_dist_point_index_right) == 0) {
      std::cout << "The lane lines cannot be used to correct the position. "
                   "The specific observation values are： "
                << map_lane_points_flags_->at(min_dist_point_index_left) << ' '
                << map_lane_points_flags_->at(min_dist_point_index_right) << ' '
                << fabs(lane_data.left_offset_) << ' '
                << fabs(lane_data.right_offset_) << std::endl;
      return false;
    } else {
      return true;
    }
  }
  bool IfCurLaneYawObsValid(const sensor::LaneData& lane_data,
                            const int& min_dist_point_index_left,
                            const int& min_dist_point_index_right) {
    /**
     * Limit the measurement value, or rather the value will make the variance
     * too large to succeed inf
     */
    if ((lane_data.right_angle_rad_ >= 1.675 ||
         lane_data.right_angle_rad_ <= 1.465 ||
         map_lane_points_flags_->at(min_dist_point_index_right) == 0 ||
         map_lane_points_flags_->at(min_dist_point_index_right + 1) == 0) &&
            lane_data.left_angle_rad_ >=
                1.675 || 
        lane_data.left_angle_rad_ <= 1.465 ||
        map_lane_points_flags_->at(min_dist_point_index_left) == 0 ||
        map_lane_points_flags_->at(min_dist_point_index_left + 1) == 0) {
      std::cout << "The lane lines cannot be used to correct the heading "
                   "angle. The specific observation values are： "
                << map_lane_points_flags_->at(min_dist_point_index_left) << ' '
                << map_lane_points_flags_->at(min_dist_point_index_left + 1)
                << ' ' << map_lane_points_flags_->at(min_dist_point_index_right)
                << ' '
                << map_lane_points_flags_->at(min_dist_point_index_right + 1)
                << ' ' << fabs(lane_data.right_angle_rad_) << ' '
                << fabs(lane_data.left_angle_rad_) << std::endl;
      return false;
    } else {
      return true;
    }
  }
  void FindMinDistPoint(const int& left_right_flag, const bool& slope_flag,
                        int* min_dist_point_index, double* min_dist) {
    int start_index, end_index;
    if (left_right_flag == 0) {  
      start_index = 0;
      end_index = *msg_index_;
    } else {  
      start_index = *msg_index_;
      end_index = map_lane_points_->size();
    }
    for (int i = start_index; i < end_index - 1; i++) {
      // Two adjacent points are taken into the line equation, and the
      // multiplication result is either a different sign or equal to zero,
      // which is the boundary point
      double adj_product = 0;
      if (slope_flag) {  // The straight line has a slope
        adj_product = (k_ * (map_lane_points_->at(i).x) -
                       (map_lane_points_->at(i).y) + b_) *
                      (k_ * (map_lane_points_->at(i + 1).x) -
                       (map_lane_points_->at(i + 1).y) + b_);
      } else {
        adj_product = (map_lane_points_->at(i).x - car_x_) *
                      (map_lane_points_->at(i + 1).x - car_x_);
      }

      // Found the boundary point
      if (adj_product <= 0) {
        if (adj_product ==
            0) {  // If one of these two points is on the straight line
          *min_dist = 0;
          // Determine if the point indexed by i+1 is on the straight line
          if (CalcP2LDist(slope_flag, i + 1) == 0) {
            *min_dist_point_index = i + 1;
          } else {
            *min_dist_point_index = i;
          }
        } else {  // Substitute these two points into the line equation, and if
                  // the calculation results have different signs, consider i as
                  // the closest point
          *min_dist_point_index = i;
          *min_dist = CalcP2LDist(slope_flag, i);
        }
        break;
      }
      // If no adjacent points with different signs are found until the last
      // point, it is necessary to determine whether the first point is closer
      // or the last point is closer
      if (i == end_index - 2) {
        double start_point_dist = CalcP2LDist(slope_flag, start_index);
        double end_point_dist = CalcP2LDist(slope_flag, end_index - 1);
        if (start_point_dist < end_point_dist) {
          *min_dist_point_index = start_index;
          *min_dist = start_point_dist;
        } else {
          *min_dist_point_index = end_index - 1;
          *min_dist = end_point_dist;
        }
      }
    }
  }
  void ProcessLaneInfo(sensor::LaneData& lane_data,
                       Eigen::Vector3d* image_offset, double* lane_yaw_obs,
                       Eigen::Vector3d* map_offset, double* yaw_earth_obs,
                       const bool& pos_update_flag, const bool& yaw_update_flag,
                       bool* pos_update_enable_flag,
                       bool* yaw_update_enable_flag,
                       int* left_lane_min_dist_point_index,
                       int* right_lane_min_dist_point_index, double* k,
                       double* b) {
    if (RUN_MODE == 0) {
      std::cout << "yaw earth: " << math_utils::RadiansToDegrees(yaw_earth_)
                << std::endl;
      common_utils::CalcYawRVizFromYawEarth(yaw_earth_, &yaw_rviz_);
      
      std::cout << "rviz yaw: " << math_utils::RadiansToDegrees(yaw_rviz_)
                << std::endl;
      
      k_ = (-1) / tan(yaw_rviz_);
    } else if (RUN_MODE == 2) {
      std::cout << "yaw earth: " << math_utils::RadiansToDegrees(yaw_earth_)
                << std::endl;
      common_utils::CalcYawENUFromYawEarth(yaw_earth_, &yaw_rviz_);
      std::cout << "rviz yaw from earth: "
                << math_utils::RadiansToDegrees(yaw_rviz_) << std::endl;
      k_ = (-1) / tan(yaw_rviz_);
    }
    b_ = car_y_ - k_ * car_x_;
    *k = k_;
    *b = b_;
    std::cout << "k: " << k_ << ' ' << "b: " << b_ << std::endl;
    double ref_offset;  // Observation values of Lane Yaw
    double dist_in_map_metric;
    uint16_t left_right_flag = 0;  // left 0 right 1
    bool slope_flag;
    // A straight line perpendicular to the direction of travel has a slope
    if (yaw_rviz_ != 0 && yaw_rviz_ != M_PI && yaw_rviz_ != -M_PI) {
      slope_flag = true;
    } else {
      slope_flag = false;
    }
    // Initialize the minimum distance value between the lane line point and
    // this line as the distance from the first point to the line
    int min_dist_point_index_left = 0;
    double min_dist_left = CalcP2LDist(slope_flag, 0);
    int min_dist_point_index_right = *msg_index_;
    double min_dist_right = CalcP2LDist(slope_flag, *msg_index_);
    FindMinDistPoint(0, slope_flag, &min_dist_point_index_left,
                     &min_dist_left);
    *left_lane_min_dist_point_index = min_dist_point_index_left;
    std::cout << "min index left: " << min_dist_point_index_left
              << " min dist left: " << min_dist_left << std::endl;
    FindMinDistPoint(1, slope_flag, &min_dist_point_index_right,
                     &min_dist_right);
    *right_lane_min_dist_point_index = min_dist_point_index_right;
    std::cout << "min index right: " << min_dist_point_index_right
              << " min dist right: " << min_dist_right << std::endl;
    // If min_dist_point the last point, it means we need to request a new section of the
    // lane lines again
    if (min_dist_point_index_left == *msg_index_ - 1 ||
        min_dist_point_index_right == map_lane_points_->size() - 1 ||
        common_utils::CalcP2PDist(
            car_x_, car_y_, map_lane_points_->at(min_dist_point_index_left).x,
            map_lane_points_->at(min_dist_point_index_left).y) > 3 &&
            common_utils::CalcP2PDist(
                car_x_, car_y_,
                map_lane_points_->at(min_dist_point_index_right).x,
                map_lane_points_->at(min_dist_point_index_right).y) > 3) {
      std::cout << RED << "Request new lane lines again" << COLOR_RESET << std::endl;
      std::cout << min_dist_point_index_left << ' ' << *msg_index_ - 1 << ' '
                << min_dist_point_index_right << ' '
                << map_lane_points_->size() - 1 << ' '
                << common_utils::CalcP2PDist(
                       car_x_, car_y_,
                       map_lane_points_->at(min_dist_point_index_left).x,
                       map_lane_points_->at(min_dist_point_index_left).y)
                << ' '
                << common_utils::CalcP2PDist(
                       car_x_, car_y_,
                       map_lane_points_->at(min_dist_point_index_right).x,
                       map_lane_points_->at(min_dist_point_index_right).y)
                << ' ' << lane_data.left_offset_ << ' '
                << lane_data.right_offset_ << std::endl;
      *pos_update_enable_flag = false;
      *yaw_update_enable_flag = false;
      map_lane_points_->clear();
      map_lane_points_flags_->clear();
    } else {
      // Lane line points do not need to be requested again
      // Then first, determine whether the lane information can be used to update the
      // vehicle's position
      std::cout << "Decide which side of the lane line to use" << std::endl;
      if (pos_update_flag) {
        if (!IfCurLaneRangeValid(lane_data, min_dist_point_index_left,
                                 min_dist_point_index_right)) {
          *pos_update_enable_flag = false;
        } else {
          int side = SelectSidePos(lane_data, min_dist_point_index_left,
                                   min_dist_point_index_right);
          std::cout << "left offset obs: " << lane_data.left_offset_
                    << std::endl;
          std::cout << "right offset obs: " << lane_data.right_offset_
                    << std::endl;
          if (side == 1) {
            // use the right side
            std::cout << "Using the right point, the coordinates of the right "
                         "point are： x: "
                      << map_lane_points_->at(min_dist_point_index_right).x
                      << " y: "
                      << map_lane_points_->at(min_dist_point_index_right).y
                      << std::endl;
            ref_offset = lane_data.right_offset_;
            dist_in_map_metric = sqrt(
                (car_x_ - map_lane_points_->at(min_dist_point_index_right).x) *
                    (car_x_ -
                     map_lane_points_->at(min_dist_point_index_right).x) +
                (car_y_ - map_lane_points_->at(min_dist_point_index_right).y) *
                    (car_y_ -
                     map_lane_points_->at(min_dist_point_index_right).y) -
                min_dist_right * min_dist_right);
            left_right_flag = 1;
            double ref_line_yaw_earth = yaw_earth_ + 0.5 * M_PI;
            if (ref_line_yaw_earth > 2 * M_PI) {
              ref_line_yaw_earth -= 2 * M_PI;
            }
            double ref_line_yaw_ENU;
            common_utils::CalcYawENUFromYawEarth(ref_line_yaw_earth,
                                                 &ref_line_yaw_ENU);
            std::cout << "ref line yaw earth: " << ref_line_yaw_earth
                      << " ref line yaw ENU: " << ref_line_yaw_ENU << std::endl;
            Eigen::Vector3d image_offset_tmp(
                fabs(ref_offset) * cos(ref_line_yaw_ENU),
                fabs(ref_offset) * sin(ref_line_yaw_ENU), 0.0);
            *image_offset = image_offset_tmp;
          } else if (side == 0) {  // use the left side
            std::cout << "Using the left point, the coordinates of the left "
                         "point are： x: "
                      << map_lane_points_->at(min_dist_point_index_left).x
                      << " y: "
                      << map_lane_points_->at(min_dist_point_index_left).y
                      << std::endl;
            ref_offset = lane_data.left_offset_;
            dist_in_map_metric = sqrt(
                (car_x_ - map_lane_points_->at(min_dist_point_index_left).x) *
                    (car_x_ -
                     map_lane_points_->at(min_dist_point_index_left).x) +
                (car_y_ - map_lane_points_->at(min_dist_point_index_left).y) *
                    (car_y_ -
                     map_lane_points_->at(min_dist_point_index_left).y) -
                min_dist_left * min_dist_left);
            double ref_line_yaw_earth = yaw_earth_ - 0.5 * M_PI;
            if (ref_line_yaw_earth < 0) {
              ref_line_yaw_earth += 2 * M_PI;
            }
            double ref_line_yaw_ENU;
            common_utils::CalcYawENUFromYawEarth(ref_line_yaw_earth,
                                                 &ref_line_yaw_ENU);
            std::cout << "ref line yaw earth: " << ref_line_yaw_earth
                      << " ref line yaw ENU: " << ref_line_yaw_ENU << std::endl;
            Eigen::Vector3d image_offset_tmp(
                fabs(ref_offset) * cos(ref_line_yaw_ENU),
                fabs(ref_offset) * sin(ref_line_yaw_ENU), 0.0);
            *image_offset = image_offset_tmp;
          }
        }
        if (left_right_flag) {  // right
          double ref_line_yaw_rviz, ref_line_yaw_ENU;
          if (RUN_MODE == 0) {
            ref_line_yaw_rviz =
                yaw_rviz_ +
                0.5 * M_PI;  // Note that due to the opposite chirality between
                             // the rviz coordinate system and the East North Up
                             // coordinate system, the rotation direction is
                             // also opposite. The clockwise rotation of the
                             // right-hand system is actually the
                             // counterclockwise rotation of the left-hand
                             // system. RViz is a left-hand style.
            if (ref_line_yaw_rviz > M_PI) {
              ref_line_yaw_rviz -= 2 * M_PI;
            }
            common_utils::TransBetXENUAndRViz(ref_line_yaw_rviz,
                                              &ref_line_yaw_ENU);
          } else if (RUN_MODE == 2) {
            ref_line_yaw_rviz = yaw_rviz_ - 0.5 * M_PI;
            if (ref_line_yaw_rviz < -M_PI) {
              ref_line_yaw_rviz += 2 * M_PI;
            }
            ref_line_yaw_ENU = ref_line_yaw_rviz;
          }

          std::cout << "ref line yaw rviz: " << ref_line_yaw_rviz
                    << " ref line yaw ENU: " << ref_line_yaw_ENU << std::endl;
          
          Eigen::Vector3d map_offset_tmp(
              dist_in_map_metric * cos(ref_line_yaw_ENU),
              dist_in_map_metric * sin(ref_line_yaw_ENU),
              0.0);  // M_PI - yaw_rviz - 0.5 * M_PI
          *map_offset = map_offset_tmp;
        } else {  // left
          
          double ref_line_yaw_rviz, ref_line_yaw_ENU;
          if (RUN_MODE == 0) {
            ref_line_yaw_rviz = yaw_rviz_ - 0.5 * M_PI;
            if (ref_line_yaw_rviz < -M_PI) {
              ref_line_yaw_rviz += 2 * M_PI;
            }

            common_utils::TransBetXENUAndRViz(ref_line_yaw_rviz,
                                              &ref_line_yaw_ENU);
          } else if (RUN_MODE == 2) {
            ref_line_yaw_rviz = yaw_rviz_ + 0.5 * M_PI;
            if (ref_line_yaw_rviz > M_PI) {
              ref_line_yaw_rviz -= 2 * M_PI;
            }
            ref_line_yaw_ENU = ref_line_yaw_rviz;
          }

          std::cout << "ref line yaw rviz: " << ref_line_yaw_rviz
                    << " ref line yaw ENU: " << ref_line_yaw_ENU << std::endl;
          Eigen::Vector3d map_offset_tmp(
              dist_in_map_metric *
                  cos(ref_line_yaw_ENU),  // M_PI - yaw_rviz - 0.5 * M_PI
              dist_in_map_metric * sin(ref_line_yaw_ENU), 0.0);
          *map_offset = map_offset_tmp;
        }
        std::cout << "dist: " << dist_in_map_metric << std::endl;
        std::cout << "measurement: " << *image_offset << std::endl;
        std::cout << "map: " << *map_offset << std::endl;
      }
      if (yaw_update_flag) {
        if (!IfCurLaneYawObsValid(lane_data, min_dist_point_index_left,
                                  min_dist_point_index_right)) {
          *yaw_update_enable_flag = false;
          std::cout << "can not update yaw" << std::endl;
        } else {
          int side = SelectSideYaw(lane_data, min_dist_point_index_left,
                                   min_dist_point_index_right);
          if (side == 1) {
            // right
            std::cout << "Using the right point, the coordinates of the right "
                         "point are： x: "
                      << map_lane_points_->at(min_dist_point_index_right).x
                      << " y: "
                      << map_lane_points_->at(min_dist_point_index_right).y
                      << std::endl;
            *lane_yaw_obs = lane_data.right_angle_rad_;
            /**
             * Here, you can directly access the location with an index of
             * min_index+1 without any access errors because if min_index is
             * already the last point, the lane marking point will be requested
             * again in the previous step.
             */
            double lane_yaw_map =
                atan2(map_lane_points_->at(min_dist_point_index_right + 1).y -
                          map_lane_points_->at(min_dist_point_index_right).y,
                      map_lane_points_->at(min_dist_point_index_right + 1).x -
                          map_lane_points_->at(min_dist_point_index_right).x);
            double lane_yaw_earth;
            if (RUN_MODE == 0) {
              common_utils::CalcYawEarthFromYawRViz(lane_yaw_map,
                                                    &lane_yaw_earth);
            } else if (RUN_MODE == 2) {
              common_utils::CalcYawEarthFromYawENU(lane_yaw_map,
                                                   &lane_yaw_earth);
            }

            // double yaw_obs_in_rviz = 0.5 * M_PI - lane_yaw_obs +
            // lane_yaw_map;
            std::cout << "lane yaw obs: " << *lane_yaw_obs << ' '
                      << "lane_yaw_map: " << lane_yaw_map << ' '
                      << "lane_yaw_earth: " << lane_yaw_earth << std::endl;

            *yaw_earth_obs = lane_yaw_earth - (0.5 * M_PI - *lane_yaw_obs);
            if (*yaw_earth_obs < 0) {
              *yaw_earth_obs += 2 * M_PI;
            }

            left_right_flag = 1;
          } else if (side == 0) {  // left
            std::cout << "Using the left point, the coordinates of the left "
                         "point are： x: "
                      << map_lane_points_->at(min_dist_point_index_left).x
                      << " y: "
                      << map_lane_points_->at(min_dist_point_index_left).y
                      << std::endl;
            *lane_yaw_obs = lane_data.left_angle_rad_;
            double lane_yaw_map =
                atan2(map_lane_points_->at(min_dist_point_index_left + 1).y -
                          map_lane_points_->at(min_dist_point_index_left).y,
                      map_lane_points_->at(min_dist_point_index_left + 1).x -
                          map_lane_points_->at(min_dist_point_index_left).x);
            /**
             * The yaw in the rviz coordinate system is based on the x-axis as a
             * reference, and rotation towards the y-axis is a positive value.
             * Also, because Carla is a left-hand system, the x-axis points to
             * the west, which is opposite to the setting of the x-axis pointing
             * to the east in a right-hand system. Therefore, turning
             * 0.5*PI-lane_yaw_obs counterclockwise in the map is equivalent to
             * turning-(0.5*PI-lane_yaw_obs) counterclockwise in rviz. Map and rviz
             * are in the same coordinate system, with x increasing in the
             * direction of the west side, which is the same as Carla's setting.
             */
            double lane_yaw_earth;
            if (RUN_MODE == 0) {
              common_utils::CalcYawEarthFromYawRViz(lane_yaw_map,
                                                    &lane_yaw_earth);
            } else if (RUN_MODE == 2) {
              common_utils::CalcYawEarthFromYawENU(lane_yaw_map,
                                                   &lane_yaw_earth);
            }

            std::cout << "lane yaw obs: " << *lane_yaw_obs << ' '
                      << "lane_yaw_map: " << lane_yaw_map << ' '
                      << "lane_yaw_earth: " << lane_yaw_earth << std::endl;

            *yaw_earth_obs = lane_yaw_earth - (0.5 * M_PI - *lane_yaw_obs);

            if (*yaw_earth_obs < 0) {
              *yaw_earth_obs += 2 * M_PI;
            }

            left_right_flag = 0;
          } else {
            std::cout << RED << "Error" << side << COLOR_RESET << std::endl;
          }
        }
      }

      // Clearing abandoned points to save computational complexity
      std::cout
          << "Clear abandoned points, the number of poins on lane lins before "
             "clearing is"
          << map_lane_points_->size()
          << "The index of the boundary points on both sides is" << *msg_index_
          << std::endl;
      auto clear_iter1 = map_lane_points_->erase(
          std::begin(*map_lane_points_),
          std::begin(*map_lane_points_) + min_dist_point_index_left);
      auto flags_clear_iter1 = map_lane_points_flags_->erase(
          std::begin(*map_lane_points_flags_),
          std::begin(*map_lane_points_flags_) + min_dist_point_index_left);
      *msg_index_ -= min_dist_point_index_left;
      min_dist_point_index_right -= min_dist_point_index_left;
      min_dist_point_index_left -= min_dist_point_index_left;
      auto clear_iter2 = map_lane_points_->erase(
          std::begin(*map_lane_points_) + *msg_index_,
          std::begin(*map_lane_points_) + min_dist_point_index_right);
      auto flags_clear_iter2 = map_lane_points_flags_->erase(
          std::begin(*map_lane_points_flags_) + *msg_index_,
          std::begin(*map_lane_points_flags_) + min_dist_point_index_right);
      min_dist_point_index_right -= (min_dist_point_index_right - *msg_index_);
      *left_lane_min_dist_point_index = min_dist_point_index_left;
      *right_lane_min_dist_point_index = min_dist_point_index_right;
      std::cout << "the number of poins on lane lins after clearing is"
                << map_lane_points_->size()
                << "The index of the boundary points on both sides is"
                << *msg_index_ << std::endl;
    }
  }

  double yaw_earth_;
  double north_angle_;
  double yaw_rviz_;
  double car_x_;
  double car_y_;
  std::vector<MapPoint>* map_lane_points_;
  std::vector<uint16_t>* map_lane_points_flags_;
  int* msg_index_;
  double k_;
  double b_;
};

#endif  // VHDMAPSE_INCLUDE_LANEINFOHANDLER_HPP_