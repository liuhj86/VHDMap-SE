#ifndef VHDMAPSE_INCLUDE_EARTH_PARAMETER_STATE_HPP_
#define VHDMAPSE_INCLUDE_EARTH_PARAMETER_STATE_HPP_

#include <system_parameters.h>
#include <Eigen/Dense>

class EarthParameterState {
 public:
  EarthParameterState() {
    UpdateEarthParams(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }
  EarthParameterState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vn) {
    UpdateEarthParams(pos, vn);
  }
  ~EarthParameterState() {}

  void UpdateEarthParams(const Eigen::Vector3d& pos,
                         const Eigen::Vector3d& vn) {
    double sinLat, cosLat, sinLon, cosLon, tanLat, sinLat2, sinLat4;
    double sq;

    lat = pos(0);
    lon = pos(1);
    height = pos(2);

    sinLat = sin(lat);
    cosLat = cos(lat);
    sinLon = sin(lon);
    cosLon = cos(lon);

    tanLat = sinLat / cosLat;
    sinLat2 = sinLat * sinLat;
    sinLat4 = sinLat2 * sinLat2;
    sq = 1 - EeEe * sinLat * sinLat;

    RNh = Re / sqrt(sq) + height;  // (2.5)
    // RMh = RNh * (1 - EeEe) / sq  + height; // (2.4)
    RMh = Re * (1 - EeEe) / sq / sqrt(sq) + height;
    wnie << 0.0, Wie * cosLat, Wie * sinLat;
    wnen << -vn(1) / RMh, vn(0) / RNh, vn(0) * tanLat / RNh;
    wnin = wnie + wnen;
    gn << 0.0, 0.0,
        -(g0 * (1 + 5.27094e-3 * sinLat2 + 2.32718e-5 * sinLat4) -
          3.086e-6 * pos(2));

    Eigen::Matrix3d Cne_DCM;
    Cne_DCM << -sinLon, -cosLon * sinLat, cosLon * cosLat, cosLon,
        -sinLon * sinLat, sinLon * cosLat, 0, cosLat, sinLat;
    Cne = Cne_DCM;

    // from rn(ENU) to pos(lat,lon,h)
    Mrp << 0, 1.0 / RMh, 0, 1.0 / (RNh * cos(lat)), 0, 0, 0, 0, 1;

    Mpr = Mrp.inverse();
  }
  void operator=(const EarthParameterState& other) {
    lon = other.lon;
    lat = other.lat;
    height = other.height;
    RMh = other.RMh;
    RNh = other.RNh;
    Cne = other.Cne;
    Mrp = other.Mrp;
    Mpr = other.Mpr;
    wnie = other.wnie;
    wnen = other.wnen;
    wnin = other.wnin;
    gn = other.gn;
  }

  double lat;     // latitude
  double lon;     // longitude
  double height;  // height
  double RMh;     // Earth Meridian Radius
  double RNh;     // Earth Transverse Radius
  Eigen::Quaterniond Cne;
  Eigen::Matrix3d Mrp;  // Local navigation frame (ENU) to (lat,lon,h)
  Eigen::Matrix3d Mpr;  // (lat,lon,h) to local navigation frame (ENU)
  Eigen::Vector3d
      wnie;  // Projection of Earth's rotation rate in navigation system
  Eigen::Vector3d
      wnen;  // Projection of the rotation rate of the navigation system
             // relative to the Earth system in the navigation system
  Eigen::Vector3d wnin;  // Projection of rotation rate of navigation system
                         // relative to inertial system in navigation system
  Eigen::Vector3d gn;    // gravity acceleration in navigation frame
};
#endif  // VHDMAPSE_INCLUDE_EARTH_PARAMETER_STATE_HPP_
