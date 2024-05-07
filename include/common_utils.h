#ifndef VHDMAPSE_INCLUDE_COMMON_UTILS_H_
#define VHDMAPSE_INCLUDE_COMMON_UTILS_H_

#include <math.h>
#include <proj.h>  
#include <proj_api.h>
#include <stdint.h>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <random>

#include "absl/strings/str_cat.h"

#define RED "\033[0;31m"
#define GREEN "\033[0;32m"
#define BLUE "\033[0;34m"
#define YELLOW "\033[1;33m"
#define PURPLE "\033[0;35m"
#define CYAN "\033[0;36m"
#define WHITE "\033[1;37m"
#define COLOR_RESET "\033[0m"

struct MapPoint {
  float x;
  float y;
  float z;
};
struct MapSign {
  MapPoint position;
  uint16_t type;
};
struct UTMCoor {
  UTMCoor() : x(0.0), y(0.0) {}
  double x;
  double y;
};

namespace common_utils {

static bool LL2UTM(double lon_rad, double lat_rad, UTMCoor* utm_xy) {
  projPJ pj_latlon;
  projPJ pj_utm;
  int zone = 0;
  zone = static_cast<int>((lon_rad * RAD_TO_DEG + 180) / 6) + 1;
  std::string latlon_src =
      "+proj=longlat +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::string utm_dst =
      absl::StrCat("+proj=utm +zone=", zone, " +ellps=GRS80 +units=m +no_defs");
  if (!(pj_latlon = pj_init_plus(latlon_src.c_str()))) {
    return false;
  }
  if (!(pj_utm = pj_init_plus(utm_dst.c_str()))) {
    return false;
  }
  double longitude = lon_rad;
  double latitude = lat_rad;
  pj_transform(pj_latlon, pj_utm, 1, 1, &longitude, &latitude, nullptr);
  utm_xy->x = longitude;
  utm_xy->y = latitude;
  pj_free(pj_latlon);
  pj_free(pj_utm);
  return true;
}

// Calculate the orientation of the vehicle based on the north of the Earth
// using the yaw angle in the state vector, with a range of 0-360 degrees
static void CalcYawEarthFromYaw(const double& yaw,
                                const bool& north_angle_cali_flag,
                                const double& north_angle, double* yaw_earth) {
  if (north_angle_cali_flag) {
    *yaw_earth = yaw - north_angle;
  } else {
    *yaw_earth = yaw;
  }
  if (*yaw_earth < 0) {
    *yaw_earth += 2 * M_PI;
  }
  if (*yaw_earth > 2 * M_PI) {
    *yaw_earth -= 2 * M_PI;
  }
}

static void CalcYawFromYawEarth(double yaw_earth,
                                const bool& north_angle_cali_flag,
                                const double& north_angle, double* yaw) {
  if (yaw_earth < 0) {
    yaw_earth += 2 * M_PI;
  }
  if (yaw_earth > 2 * M_PI) {
    yaw_earth -= 2 * M_PI;
  }
  if (north_angle_cali_flag) {
    *yaw = yaw_earth + north_angle;
  } else {
    *yaw = yaw_earth;
  }
}
/*
 * The yaw of rviz is based on the x-axis, and rotation towards the y-axis is a
 * positive value, while yaw_earth_ is based on due north (y-axis in rviz) and
 * rotates clockwise to be a positive value, (0-2PI)
 * Also, because Carla is a left-hand system with the x-axis pointing due west,
 * which is opposite to the setting of the right-hand system with the x-axis
 * pointing due east, it is necessary to subtract the value in the right-hand
 * system from PI when calculating the heading angle in order to correspond in
 * rviz.
 * The yaw_rviz variable below is the heading in rviz when the x-axis
 * increases in the east direction, so it is not the actual heading displayed in
 * rviz, and the direction displayed in rviz should be PI - yaw_rviz.
 * Map and rviz are in the same coordinate system, with x increasing in the
 * direction of the west side, which is the same as Carla's setting.
 */
static void TransBetXENUAndRViz(const double& yaw1, double* yaw2) {
  *yaw2 = M_PI - yaw1;
  if (*yaw2 > M_PI) {
    *yaw2 -= 2 * M_PI;
  }
  if (*yaw2 < -M_PI) {
    *yaw2 += 2 * M_PI;
  }
}

/**
 * Calculate the orientation of the vehicle in the "East-North-Up" coordinate
 * system by using the orientation of the vehicle relative to the
 * north direction, with the positive x-axis pointing east and the positive
 * y-axis pointing north.
 */
static void CalcYawENUFromYawEarth(const double& yaw_earth, double* yaw_enu) {
  if (yaw_earth >= 0 && yaw_earth <= 1.5 * M_PI) {
    *yaw_enu = 0.5 * M_PI - yaw_earth;
  } else {
    *yaw_enu = 2.5 * M_PI - yaw_earth;
  }
}
/**
 * Calculate the orientation relative to due north using the orientation
 * in the "East North Up" coordinate system
 */
static void CalcYawEarthFromYawENU(const double& yaw_enu, double* yaw_earth) {
  if (yaw_enu >= -M_PI && yaw_enu <= 0.5 * M_PI) {
    *yaw_earth = 0.5 * M_PI - yaw_enu;
  } else {
    *yaw_earth = 2.5 * M_PI - yaw_enu;
  }
}

/**
 * Calculate the orientation in RViz based on the relative north direction,
 * where the x-axis points due west and the y-axis points due north
 */
static void CalcYawRVizFromYawEarth(const double& yaw_earth, double* yaw_rviz) {
  double yaw_enu;
  CalcYawENUFromYawEarth(yaw_earth, &yaw_enu);
  TransBetXENUAndRViz(yaw_enu, yaw_rviz);
  if (*yaw_rviz > M_PI) {
    *yaw_rviz -= 2 * M_PI;
  }
  if (*yaw_rviz < -M_PI) {
    *yaw_rviz += 2 * M_PI;
  }
}
/**
 * Calculate the orientation relative to due north using the orientation in
 * RViz. The x-axis of RViz points due west and the y-axis points due north
 */
static void CalcYawEarthFromYawRViz(const double& yaw_rviz, double* yaw_earth) {
  double yaw_x_east_y_north;
  TransBetXENUAndRViz(yaw_rviz, &yaw_x_east_y_north);
  *yaw_earth = 0.5 * M_PI - yaw_x_east_y_north;
  if (*yaw_earth < 0) {
    *yaw_earth += 2 * M_PI;
  }
  if (*yaw_earth > 2 * M_PI) {
    *yaw_earth -= 2 * M_PI;
  }
}

static double random(const double& bound) {
  static bool initialized = false;
  if (!initialized) {
    srand(time(NULL));
    initialized = true;
  }
  double r =
      (double)rand() / RAND_MAX;  // Generate random numbers between 0 and 1
  return r * 2 * bound - bound;  
}

static double CalcP2PDist(const double& p1_x, const double& p1_y,
                          const double& p2_x, const double& p2_y) {
  return std::sqrt((p1_x - p2_x) * (p1_x - p2_x) +
                   (p1_y - p2_y) * (p1_y - p2_y));
}

static double AddNoise(double a, double b) {
  double delta = a * b / 100.0;                 // Calculate fluctuation range
  double rand_num =
      (double)rand() / RAND_MAX;  // Generate random numbers between 0 and 1
  double fluctuation =
      delta * (2 * rand_num - 1);  // Calculate fluctuation value
  std::cout << "fluctuation: " << delta << ' ' << rand_num << ' ' << fluctuation
            << std::endl;
  return a + fluctuation;  // Return the value after add fluctuation
}

static double AddGaussianNoise(double mean, double variance) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(mean, variance);

  return dist(gen);
}

}  // namespace common_utils

#endif  // VHDMAPSE_INCLUDE_COMMON_UTILS_H_
