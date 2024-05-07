#include <mercator.h>
#include <iostream>

MercatorProj::MercatorProj()
    : __IterativeTimes(15),
      __IterativeValue(0),
      __A(0),
      __B(0),
      __B0(0),
      __L0(0) {}
void MercatorProj::SetAB(double& a,
                         double& b)  
{
  if (a <= 0 || b <= 0) {
    return;
  }
  __A = a;
  __B = b;
}
// set__B0
void MercatorProj::SetB0(double b0) {
    if (b0 < -M_PI / 2 || b0 > M_PI / 2) {
    return;
  }
  __B0 = b0;  //  Set standard latitude. The intersection of the standard
              //  latitude line and the origin longitude line forms the origin
              //  of the projected plane coordinates
}
// set__L0
void MercatorProj::SetL0(double l0) {
  if (l0 < -M_PI || l0 > M_PI) {
    return;
  }
  __L0 = l0;  // Set origin longitude
}

/*******************************************
double B: latitude,radians
double L: longitude,radians
double& X: Vertical Cartesian Coordinates
double& Y: Horizontal Cartesian Coordinates
*******************************************/
int MercatorProj::ToProj(double B, double L, double& X, double& Y) {
  double f /*Oblateness*/, e /*First eccentricity*/, e_ /*Second eccentricity*/,
      NB0 /*The curvature radius of the prime vertical circle*/, K, dtemp;

  double E = exp(1);
  if (L < -M_PI || L > M_PI || B < -M_PI / 2 || B > M_PI / 2) {
    return 1;
  }

  if (__A <= 0 || __B <= 0) {
    return 1;
  }

  f = (__A - __B) / __A;

  dtemp = 1 - (__B / __A) * (__B / __A);
  if (dtemp < 0) {
    return 1;
  }
  e = sqrt(dtemp);

  dtemp = (__A / __B) * (__A / __B) - 1;
  if (dtemp < 0) {
    return 1;
  }
  e_ = sqrt(dtemp);

  NB0 = ((__A * __A) / __B) /
        sqrt(1 + e_ * e_ * cos(__B0) *
                     cos(__B0));  // NB0=N,
                                  // Normal length (Fundamentals of Geodesy, 2nd
                                  // edition, page 103, or pages 109, 110)

  K = NB0 * cos(__B0);  // Parallel circle radius

  Y = K * (L - __L0);

  X = K * log(tan(M_PI / 4 + B / 2) *
              pow((1 - e * sin(B)) / (1 + e * sin(B)), e / 2)) -
      K * log(tan(M_PI / 4 + __B0 / 2) *
              pow((1 - e * sin(__B0)) / (1 + e * sin(__B0)), e / 2));

  return 0;
}
/*******************************************
Reverse conversion
*******************************************/
int MercatorProj::FromProj(double X, double Y, double& B, double& L) {
  double f , e , e_ ,
      NB0 , K, dtemp;
  double E = exp(1);

  if (__A <= 0 || __B <= 0) {
    return 1;
  }

  f = (__A - __B) / __A;

  dtemp = 1 - (__B / __A) * (__B / __A);
  if (dtemp < 0) {
    return 1;
  }
  e = sqrt(dtemp);

  dtemp = (__A / __B) * (__A / __B) - 1;
  if (dtemp < 0) {
    return 1;
  }
  e_ = sqrt(dtemp);

  NB0 = ((__A * __A) / __B) / sqrt(1 + e_ * e_ * cos(__B0) * cos(__B0));

  K = NB0 * cos(__B0);

  L = Y / K + __L0;
  B = __IterativeValue;
  for (int i = 0; i < __IterativeTimes; i++) {
    B = M_PI / 2 -
        2 * atan(pow(E, (-X / K)) *
                 pow(E, (e / 2) * log((1 - e * sin(B)) / (1 + e * sin(B)))));
  }

  return 0;
}

MercatorProj::~MercatorProj() {}